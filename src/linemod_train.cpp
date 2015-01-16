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

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/model_utils.h>

#if USE_GLUT
#include <object_recognition_renderer/renderer_glut.h>
#else
#include <object_recognition_renderer/renderer_osmesa.h>
#endif
#include <object_recognition_renderer/utils.h>

#if LINEMOD_VIZ_IMG
  #include <opencv2/highgui/highgui.hpp>
#endif

using ecto::tendrils;
using ecto::spore;

namespace ecto_linemod
{
  struct Trainer
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Trainer::json_db_, "json_db", "The DB parameters", "{}").required(true);
      inputs.declare(&Trainer::object_id_, "object_id",
        "The object id, to associate this model with.").required(true);
      inputs.declare(&Trainer::visualize_, "visualize", "If True, visualize the output.", true);

      outputs.declare(&Trainer::detector_, "detector", "The LINE-MOD detector");
      outputs.declare(&Trainer::Rs_, "Rs", "The matching rotations of the templates");
      outputs.declare(&Trainer::Ts_, "Ts", "The matching translations of the templates.");
      outputs.declare(&Trainer::distances_, "distances", "The matching depth of the templates.");
      outputs.declare(&Trainer::Ks_, "Ks", "The matching calibration matrices of the templates.");
      outputs.declare(&Trainer::renderer_n_points_, "renderer_n_points", "Renderer parameter: the number of points on the sphere.");
      outputs.declare(&Trainer::renderer_angle_step_, "renderer_angle_step", "Renderer parameter: the angle step sampling in degrees.");
      outputs.declare(&Trainer::renderer_radius_min_, "renderer_radius_min", "Renderer parameter: the minimum scale sampling.");
      outputs.declare(&Trainer::renderer_radius_max_, "renderer_radius_max", "Renderer parameter: the maximum scale sampling.");
      outputs.declare(&Trainer::renderer_radius_step_, "renderer_radius_step", "Renderer parameter: the step scale sampling.");
      outputs.declare(&Trainer::renderer_width_, "renderer_width", "Renderer parameter: the image width.");
      outputs.declare(&Trainer::renderer_height_, "renderer_height", "Renderer parameter: the image height.");
      outputs.declare(&Trainer::renderer_focal_length_x_, "renderer_focal_length_x", "Renderer parameter: the focal length x.");
      outputs.declare(&Trainer::renderer_focal_length_y_, "renderer_focal_length_y", "Renderer parameter: the focal length y.");
      outputs.declare(&Trainer::renderer_near_, "renderer_near", "Renderer parameter: near distance.");
      outputs.declare(&Trainer::renderer_far_, "renderer_far", "Renderer parameter: far distance.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      //or_json::mValue submethod = object_recognition_core::to_json(*json_submethod_);
    }

  int process(const tendrils& inputs, const tendrils& outputs) {
    // Get the document for the object_id_ from the DB
    object_recognition_core::db::ObjectDbPtr db =
        object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();
    object_recognition_core::db::Documents documents =
        object_recognition_core::db::ModelDocuments(db,
            std::vector<object_recognition_core::db::ObjectId>(1, *object_id_),
            "mesh");
    if (documents.empty()) {
      std::cerr << "Skipping object id \"" << *object_id_ << "\" : no mesh in the DB" << std::endl;
      return ecto::OK;
    }

    // Get the list of _attachments and figure out the original one
    object_recognition_core::db::Document document = documents[0];
    std::vector<std::string> attachments_names = document.attachment_names();
    std::string mesh_path;
    BOOST_FOREACH(const std::string& attachment_name, attachments_names) {
      if (attachment_name.find("original") != 0)
        continue;
      // Create a temporary file
      char mesh_path_tmp[L_tmpnam];
      tmpnam(mesh_path_tmp);
      mesh_path = std::string(mesh_path_tmp) + attachment_name.substr(8);

      // Load the mesh and save it to the temporary file
      std::ofstream mesh_file;
      mesh_file.open(mesh_path.c_str());
      document.get_attachment_stream(attachment_name, mesh_file);
      mesh_file.close();
    }

    cv::Ptr<cv::linemod::Detector> detector_ptr = cv::linemod::getDefaultLINEMOD();
    *detector_ = *detector_ptr;

    // Define the display
    *renderer_width_ = 640; *renderer_height_ = 480;
    *renderer_near_ = 0.1; *renderer_far_ = 1000.0;
    *renderer_focal_length_x_ = 525.0; *renderer_focal_length_y_ = 525.0;

    // the model name can be specified on the command line.
#if USE_GLUT
    RendererGlut renderer = RendererGlut(mesh_path);
#else
    RendererOSMesa renderer = RendererOSMesa(mesh_path);
#endif
    renderer.set_parameters(*renderer_width_, *renderer_height_, *renderer_focal_length_x_,
                            *renderer_focal_length_y_, *renderer_near_, *renderer_far_);

    std::remove(mesh_path.c_str());

    *renderer_n_points_ = 150;
    *renderer_angle_step_ = 10;
    *renderer_radius_min_ = 0.6;
    *renderer_radius_max_ = 1.1;
    *renderer_radius_step_ = 0.4;
    RendererIterator renderer_iterator = RendererIterator(&renderer, *renderer_n_points_);
    //set the RendererIterator parameters
    renderer_iterator.angle_step_ = *renderer_angle_step_;
    renderer_iterator.radius_min_ = float(*renderer_radius_min_);
    renderer_iterator.radius_max_ = float(*renderer_radius_max_);
    renderer_iterator.radius_step_ = float(*renderer_radius_step_);

    cv::Mat image, depth, mask;
    cv::Matx33d R;
    cv::Vec3d T;
    cv::Matx33f K;
    for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
    {
      std::stringstream status;
      status << "Loading images " << (i+1) << "/"
          << renderer_iterator.n_templates();
      std::cout << status.str();

      cv::Rect rect;
      renderer_iterator.render(image, depth, mask, rect);

      R = renderer_iterator.R_obj();
      T = renderer_iterator.T();
      float distance = fabs(renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f)/1000.0f));
      K = cv::Matx33f(float(*renderer_focal_length_x_), 0.0f, float(rect.width)/2.0f, 0.0f, float(*renderer_focal_length_y_), float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

      std::vector<cv::Mat> sources(2);
      sources[0] = image;
      sources[1] = depth;

#if LINEMOD_VIZ_IMG
      // Display the rendered image
      if (*visualize_)
      {
        cv::namedWindow("Rendering");
        if (!image.empty()) {
          cv::imshow("Rendering", image);
          cv::waitKey(1);
        }
      }
#endif

      detector_->addTemplate(sources, "object1", mask);

      // Also store the pose of each template
      Rs_->push_back(cv::Mat(R));
      Ts_->push_back(cv::Mat(T));
      distances_->push_back(distance);
      Ks_->push_back(cv::Mat(K));

      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
    }

      return ecto::OK;
    }

    /** True or False to output debug image */
    ecto::spore<bool> visualize_;
    /** The DB parameters as a JSON string */
    ecto::spore<std::string> json_db_;
    /** The id of the object to generate a trainer for */
    ecto::spore<std::string> object_id_;
    ecto::spore<cv::linemod::Detector> detector_;
    ecto::spore<std::vector<cv::Mat> > Rs_;
    ecto::spore<std::vector<cv::Mat> > Ts_;
    ecto::spore<std::vector<float> > distances_;
    ecto::spore<std::vector<cv::Mat> > Ks_;
    ecto::spore<int> renderer_n_points_;
    ecto::spore<int> renderer_angle_step_;
    ecto::spore<double> renderer_radius_min_;
    ecto::spore<double> renderer_radius_max_;
    ecto::spore<double> renderer_radius_step_;
    ecto::spore<size_t> renderer_width_;
    ecto::spore<size_t> renderer_height_;
    ecto::spore<double> renderer_near_;
    ecto::spore<double> renderer_far_;
    ecto::spore<double> renderer_focal_length_x_;
    ecto::spore<double> renderer_focal_length_y_;
  };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
