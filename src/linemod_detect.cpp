/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ecto/ecto.hpp>
#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif

#include <object_recognition_core/db/ModelReader.h>
#include <object_recognition_core/common/pose_result.h>

#include "db_linemod.h"

#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer3d.h>

#include "linemod_icp.h"

using ecto::tendrils;
using ecto::spore;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;
using object_recognition_core::db::ObjectDbPtr;

#if LINEMOD_VIZ_IMG
  #include <opencv2/highgui/highgui.hpp>
#endif

#if LINEMOD_VIZ_PCD
  #include "ros/ros.h"
  #include "linemod_pointcloud.h"
  LinemodPointcloud *pci_real_icpin_model;
  LinemodPointcloud *pci_real_icpin_ref;
#endif

void
drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset,
             int T)
{
  static const cv::Scalar COLORS[5] =
  { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };
  if (dst.channels() == 1)
    cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

  cv::circle(dst, cv::Point(offset.x + 20, offset.y + 20), T / 2, COLORS[4]);
  if (num_modalities > 5)
    num_modalities = 5;
  for (int m = 0; m < num_modalities; ++m)
  {
// NOTE: Original demo recalculated max response for each feature in the TxT
// box around it and chose the display color based on that response. Here
// the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int) templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}

namespace ecto_linemod
{
struct Detector: public object_recognition_core::db::bases::ModelReaderBase {
  void parameter_callback(
      const object_recognition_core::db::Documents & db_documents) {
    /*if (submethod.get_str() == "DefaultLINEMOD")
     detector_ = cv::linemod::getDefaultLINEMOD();
     else
     throw std::runtime_error("Unsupported method. Supported ones are: DefaultLINEMOD");*/

    if(!(*use_rgb_) && !(*use_depth_))
      throw std::runtime_error("Unsupported type of input data: either use_rgb or use_depth (or both) parameters shouled be true");
    if(!(*use_rgb_) && *use_depth_)
      std::cout << "WARNING:: Gradients computation will be based on depth data (but not rgb image)." << std::endl;
    detector_ = cv::linemod::getDefaultLINEMOD();

    BOOST_FOREACH(const object_recognition_core::db::Document & document, db_documents) {
      std::string object_id = document.get_field<ObjectId>("object_id");

      // Load the detector for that class
      cv::linemod::Detector detector;
      document.get_attachment<cv::linemod::Detector>("detector", detector);
      if (detector.classIds().empty())
        continue;
      std::string object_id_in_db = detector.classIds()[0];
      for (size_t template_id = 0; template_id < detector.numTemplates();
          ++template_id) {
        const std::vector<cv::linemod::Template> &templates_original = detector.getTemplates(object_id_in_db, template_id);
        detector_->addSyntheticTemplate(templates_original, object_id);
      }

      // Deal with the poses
      document.get_attachment<std::vector<cv::Mat> >("Rs", Rs_[object_id]);
      document.get_attachment<std::vector<cv::Mat> >("Ts", Ts_[object_id]);
      document.get_attachment<std::vector<float> >("distances", distances_[object_id]);
      document.get_attachment<std::vector<cv::Mat> >("Ks", Ks_[object_id]);
      renderer_n_points_ = document.get_field<int>("renderer_n_points");
      renderer_angle_step_ = document.get_field<int>("renderer_angle_step");
      renderer_radius_min_  = document.get_field<double>("renderer_radius_min");
      renderer_radius_max_ = document.get_field<double>("renderer_radius_max");
      renderer_radius_step_ = document.get_field<double>("renderer_radius_step");
      renderer_width_ = document.get_field<int>("renderer_width");
      renderer_height_ = document.get_field<int>("renderer_height");
      renderer_focal_length_x_ = document.get_field<double>("renderer_focal_length_x");
      renderer_focal_length_y_ = document.get_field<double>("renderer_focal_length_y");
      renderer_near_ = document.get_field<double>("renderer_near");
      renderer_far_ = document.get_field<double>("renderer_far");

      if (setupRenderer(object_id))
        std::cout << "Loaded " << object_id
                << " with the number of samples " << Rs_[object_id].size() << std::endl << std::endl;
    }

    //initialize the visualization
#if LINEMOD_VIZ_PCD
    ros::NodeHandle node_;
    pci_real_icpin_model = new LinemodPointcloud(node_, "real_icpin_model", *depth_frame_id_);
    pci_real_icpin_ref = new LinemodPointcloud(node_, "real_icpin_ref", *depth_frame_id_);
#endif
  }

    static void
    declare_params(tendrils& params)
    {
      object_recognition_core::db::bases::declare_params_impl(params, "LINEMOD");
      params.declare(&Detector::threshold_, "threshold", "Matching threshold, as a percentage", 93.0f);
      params.declare(&Detector::visualize_, "visualize", "If True, visualize the output.", false);
      params.declare(&Detector::use_rgb_, "use_rgb", "If True, use rgb-based detector.", true);
      params.declare(&Detector::use_depth_, "use_depth", "If True, use depth-based detector.", true);
      params.declare(&Detector::th_obj_dist_, "th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
      params.declare(&Detector::verbose_, "verbose", "If True, print.", false);
      params.declare(&Detector::depth_frame_id_, "depth_frame_id", "The depth camera frame id.", "camera_depth_optical_frame");
      params.declare(&Detector::icp_dist_min_, "icp_dist_min", "", 0.06f);
      params.declare(&Detector::px_match_min_, "px_match_min", "", 0.25f);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Detector::color_, "image", "An rgb full frame image.");
      inputs.declare(&Detector::depth_, "depth", "The 16bit depth image.");
      inputs.declare(&Detector::K_depth_, "K_depth", "The calibration matrix").required();

      outputs.declare(&Detector::pose_results_, "pose_results", "The results of object recognition");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      configure_impl();
    }

    /**
     * @brief Initializes the renderer with the parameters used in the training phase.
     * The renderer will be later used to render depth clouds for each detected object.
     * @param[in] The object id to initialize.*/
    bool
    setupRenderer(const std::string &object_id)
    {
      object_recognition_core::db::ObjectDbParameters db_params(*json_db_);
      // Get the document for the object_id_ from the DB
      object_recognition_core::db::ObjectDbPtr db = db_params.generateDb();
      object_recognition_core::db::Documents documents =
          object_recognition_core::db::ModelDocuments(db,
              std::vector<object_recognition_core::db::ObjectId>(1,
                  object_id), "mesh");
      if (documents.empty()) {
        std::cerr << "Skipping object id \"" << object_id
            << "\" : no mesh in the DB" << std::endl;
        return false;
      }

      // Get the list of _attachments and figure out the original one
      object_recognition_core::db::Document document = documents[0];
      std::vector<std::string> attachments_names = document.attachment_names();
      std::string mesh_path;
      std::vector<std::string> possible_names(2);
      possible_names[0] = "original";
      possible_names[1] = "mesh";
      for (size_t i = 0; i < possible_names.size() && mesh_path.empty(); ++i) {
        BOOST_FOREACH(const std::string& attachment_name, attachments_names){
          if (attachment_name.find(possible_names[i]) != 0)
            continue;
          std::cout << "Reading the mesh file " << attachment_name << std::endl;
          // Create a temporary file
          char mesh_path_tmp[L_tmpnam];
          mkstemp(mesh_path_tmp);
          mesh_path = std::string(mesh_path_tmp) + attachment_name.substr(possible_names[i].size());

          // Load the mesh and save it to the temporary file
          std::ofstream mesh_file;
          mesh_file.open(mesh_path.c_str());
          document.get_attachment_stream(attachment_name, mesh_file);
          mesh_file.close();
          std::string str = mesh_path.c_str();
        }
      }

      // the model name can be specified on the command line.
      Renderer3d *renderer_ = new Renderer3d(mesh_path);
      renderer_->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);

      std::remove(mesh_path.c_str());

      //initiaization of the renderer with the same parameters as used for learning
      RendererIterator *renderer_iterator_ = new RendererIterator(renderer_, renderer_n_points_);
      renderer_iterator_->angle_step_ = renderer_angle_step_;
      renderer_iterator_->radius_min_ = float(renderer_radius_min_);
      renderer_iterator_->radius_max_ = float(renderer_radius_max_);
      renderer_iterator_->radius_step_ = float(renderer_radius_step_);
      renderer_iterators_.insert(std::pair<std::string,RendererIterator*>(object_id, renderer_iterator_));
      return true;
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      PoseResult pose_result;
      pose_results_->clear();

      if (detector_->classIds().empty())
        return ecto::OK;

      std::vector<cv::Mat> sources;

      // Resize color to 640x480
      /// @todo Move resizing to separate cell, and try LINE-MOD w/ SXGA images

      cv::Mat display;
      if (*use_rgb_)
      {
        cv::Mat color;
        if (color_->rows > 960)
          cv::pyrDown(color_->rowRange(0, 960), color);
        else
          color_->copyTo(color);
        if (*visualize_)
          display = color;

        sources.push_back(color);
      }

      cv::Mat depth = *depth_;
      if (depth_->depth() == CV_32F)
        depth_->convertTo(depth, CV_16UC1, 1000.0);

      if (*use_depth_)
      {
        if (!(*use_rgb_))
        {
          //add a depth-based gray image to the list of sources for matching
          depth.convertTo(display, CV_8U, 255.0/1800.0);
          cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
          sources.push_back(display);
        }

        sources.push_back(depth);
      }

      std::vector<cv::linemod::Match> matches;
      detector_->match(sources, *threshold_, matches);

      int num_modalities = (int) detector_->getModalities().size();

      cv::Mat_<cv::Vec3f> depth_real_ref_raw;
      cv::Mat_<float> K;
      K_depth_->convertTo(K, CV_32F);
      cv::depthTo3d(depth, K, depth_real_ref_raw);

      int iter = 0;
      //clear the vector of detected objects
      objs_.clear();
      //clear the point clouds
#if LINEMOD_VIZ_PCD
      pci_real_icpin_model->clear();
      pci_real_icpin_ref->clear();
#endif

    BOOST_FOREACH(const cv::linemod::Match & match, matches) {
      const std::vector<cv::linemod::Template>& templates =
          detector_->getTemplates(match.class_id, match.template_id);
      if (*visualize_)
        drawResponse(templates, num_modalities, display,
            cv::Point(match.x, match.y), detector_->getT(0));

      // Fill the Pose object
      cv::Matx33d R_match = Rs_.at(match.class_id)[match.template_id].clone();
      cv::Vec3d T_match = Ts_.at(match.class_id)[match.template_id].clone();
      float D_match = distances_.at(match.class_id)[match.template_id];
      cv::Mat K_match = Ks_.at(match.class_id)[match.template_id];

      //get the point cloud of the rendered object model
      cv::Mat mask;
      cv::Rect rect;
      cv::Matx33d R_temp(R_match.inv());
      cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
      RendererIterator* it_r = renderer_iterators_.at(match.class_id);
      cv::Mat depth_ref_;
      it_r->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);

      cv::Mat_<cv::Vec3f> depth_real_model_raw;
      cv::depthTo3d(depth_ref_, K_match, depth_real_model_raw);

      //prepare the bounding box for the model and reference point clouds
      cv::Rect_<int> rect_model(0, 0, depth_real_model_raw.cols, depth_real_model_raw.rows);
      //prepare the bounding box for the reference point cloud: add the offset
      cv::Rect_<int> rect_ref(rect_model);
      rect_ref.x += match.x;
      rect_ref.y += match.y;

      rect_ref = rect_ref & cv::Rect(0, 0, depth_real_ref_raw.cols, depth_real_ref_raw.rows);
      if ((rect_ref.width < 5) || (rect_ref.height < 5))
        continue;
      //adjust both rectangles to be equal to the smallest among them
      if (rect_ref.width > rect_model.width)
        rect_ref.width = rect_model.width;
      if (rect_ref.height > rect_model.height)
        rect_ref.height = rect_model.height;
      if (rect_model.width > rect_ref.width)
        rect_model.width = rect_ref.width;
      if (rect_model.height > rect_ref.height)
        rect_model.height = rect_ref.height;

      //prepare the reference data: from the sensor : crop images
      cv::Mat_<cv::Vec3f> depth_real_ref = depth_real_ref_raw(rect_ref);
      //prepare the model data: from the match
      cv::Mat_<cv::Vec3f> depth_real_model = depth_real_model_raw(rect_model);

      //initialize the translation based on reference data
      cv::Vec3f T_crop = depth_real_ref(depth_real_ref.rows / 2.0f, depth_real_ref.cols / 2.0f);
      //add the object's depth
      T_crop(2) += D_match;

      if (!cv::checkRange(T_crop))
        continue;
      cv::Vec3f T_real_icp(T_crop);

      //initialize the rotation based on model data
      if (!cv::checkRange(R_match))
        continue;
      cv::Matx33f R_real_icp(R_match);

      //get the point clouds (for both reference and model)
      std::vector<cv::Vec3f> pts_real_model_temp;
      std::vector<cv::Vec3f> pts_real_ref_temp;
      float px_ratio_missing = matToVec(depth_real_ref, depth_real_model, pts_real_ref_temp, pts_real_model_temp);
      if (px_ratio_missing > (1.0f-*px_match_min_))
        continue;

      //perform the first approximate ICP
      float px_ratio_match_inliers = 0.0f;
      float icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp, T_real_icp, px_ratio_match_inliers, 1);
      //reject the match if the icp distance is too big
      if (icp_dist > *icp_dist_min_)
        continue;

      //perform a finer ICP
      icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp, T_real_icp, px_ratio_match_inliers, 2);

      //keep the object match
      objs_.push_back(object_recognition_core::db::ObjData(pts_real_ref_temp, pts_real_model_temp, match.class_id, match.similarity, icp_dist, px_ratio_match_inliers, R_real_icp, T_crop));
      ++iter;
    }

    //local non-maxima supression to find the best match at each position
    int count_pass = 0;
    std::vector <object_recognition_core::db::ObjData>::iterator it_o = objs_.begin();
    for (; it_o != objs_.end(); ++it_o)
      if (!it_o->check_done)
      {
        //initialize the object to publish
        object_recognition_core::db::ObjData *o_match = &(*it_o);
        int size_th = static_cast<int>((float)o_match->pts_model.size()*0.85);
        //find the best object match among near objects
        std::vector <object_recognition_core::db::ObjData>::iterator it_o2 = it_o;
        ++it_o2;
        for (; it_o2 != objs_.end(); ++it_o2)
          if (!it_o2->check_done)
            if (cv::norm(o_match->t, it_o2->t) < *th_obj_dist_)
            {
              it_o2->check_done = true;
              if ((it_o2->pts_model.size() > size_th) && (it_o2->icp_dist < o_match->icp_dist))
                o_match = &(*it_o2);
            }

        //perform the final precise icp
        float icp_px_match = 0.0f;
        float icp_dist = icpCloudToCloud(o_match->pts_ref, o_match->pts_model, o_match->r, o_match->t, icp_px_match, 0);

        if (*verbose_)
          std::cout << o_match->match_class << " " << o_match->match_sim << " icp " << icp_dist << ", ";

        //icp_dist in the same units as the sensor data
        //this distance is used to compute the ratio of inliers (points laying within this distance between the point clouds)
        icp_dist = 0.007f;
        float px_inliers_ratio = getL2distClouds(o_match->pts_model, o_match->pts_ref, icp_dist);
        if (*verbose_)
          std::cout << " ratio " << o_match->icp_px_match << " or " << px_inliers_ratio << std::endl;

        //add points to the clouds
#if LINEMOD_VIZ_PCD
        pci_real_icpin_model->fill(o_match->pts_model, cv::Vec3b(0,255,0));
        pci_real_icpin_ref->fill(o_match->pts_ref, cv::Vec3b(0,0,255));
#endif

        //return the outcome object pose
        pose_result.set_object_id(db_, o_match->match_class);
        pose_result.set_confidence(o_match->match_sim);
        pose_result.set_R(cv::Mat(o_match->r));
        pose_result.set_T(cv::Mat(o_match->t));
        pose_results_->push_back(pose_result);

        ++count_pass;
      }
    if (*verbose_ && (matches.size()>0))
      std::cout << "matches  " << objs_.size() << " / " << count_pass << " / " << matches.size() << std::endl;

    //publish the point clouds
#if LINEMOD_VIZ_PCD
    pci_real_icpin_model->publish();
    pci_real_icpin_ref->publish();
#endif
#if LINEMOD_VIZ_IMG
    if (*visualize_) {
      cv::namedWindow("LINEMOD");
      cv::imshow("LINEMOD", display);
      cv::waitKey(1);
    }
#endif
    return ecto::OK;
  }

  /** LINE-MOD detector */
  cv::Ptr<cv::linemod::Detector> detector_;
    // Parameters
    spore<float> threshold_;
    // Inputs
    spore<cv::Mat> color_, depth_;
    /** The calibration matrix of the camera */
    spore<cv::Mat> K_depth_;
    /** The buffer with detected objects and their info */
    std::vector <object_recognition_core::db::ObjData> objs_;

    /** True or False to output debug image */
    ecto::spore<bool> visualize_;
    /** True or False to use input rgb image */
    ecto::spore<bool> use_rgb_;
    /** True or False to use input depth image */
    ecto::spore<bool> use_depth_;
    /** Threshold on minimal distance between detected objects */
    ecto::spore<float> th_obj_dist_;
    /** True or False to output debug log */
    ecto::spore<bool> verbose_;
    /** The depth camera frame id*/
    ecto::spore<std::string> depth_frame_id_;
    /** The minimal accepted icp distance*/
    ecto::spore<float> icp_dist_min_;
    /** The minimal percetage of pixels with matching depth*/
    ecto::spore<float> px_match_min_;
    /** The object recognition results */
    ecto::spore<std::vector<PoseResult> > pose_results_;
    /** The rotations, per object and per template */
    std::map<std::string, std::vector<cv::Mat> > Rs_;
    /** The translations, per object and per template */
    std::map<std::string, std::vector<cv::Mat> > Ts_;
    /** The objects distances, per object and per template */
    std::map<std::string, std::vector<float> > distances_;
    /** The calibration matrices, per object and per template */
    std::map<std::string, std::vector<cv::Mat> > Ks_;
    /** The renderer initialized with objects meshes, per object*/
    std::map<std::string, RendererIterator*> renderer_iterators_;
    /** Renderer parameter: the number of points on the sphere */
    int renderer_n_points_;
    /** Renderer parameter: the angle step sampling in degrees*/
    int renderer_angle_step_;
    /** Renderer parameter: the minimum scale sampling*/
    double renderer_radius_min_;
    /** Renderer parameter: the maximum scale sampling*/
    double renderer_radius_max_;
    /** Renderer parameter: the step scale sampling*/
    double renderer_radius_step_;
    /** Renderer parameter: image width */
    int renderer_width_;
    /** Renderer parameter: image height */
    int renderer_height_;
    /** Renderer parameter: near distance */
    double renderer_near_;
    /** Renderer parameter: far distance */
    double renderer_far_;
    /** Renderer parameter: focal length x */
    double renderer_focal_length_x_;
    /** Renderer parameter: focal length y */
    double renderer_focal_length_y_;
  };

} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Detector, "Detector", "Use LINE-MOD for object detection.")
