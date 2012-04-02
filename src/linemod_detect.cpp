#include <ecto/ecto.hpp>

#include <boost/foreach.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <object_recognition_core/db/ModelReader.h>
#include <object_recognition_core/common/pose_result.h>

#include "db_linemod.h"

using ecto::tendrils;
using ecto::spore;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;
using object_recognition_core::db::ObjectDb;

namespace ecto_linemod
{
  struct LinemodDetector: public object_recognition_core::db::bases::ModelReaderImpl
  {
    void
    ParameterCallback(const object_recognition_core::db::Documents & db_documents)
    {
      BOOST_FOREACH(const object_recognition_core::db::Document & document, db_documents)
      {
        // Load the detector for that class
        document.get_attachment<cv::linemod::Detector>("detector", *detector_);

        std::string object_id = document.get_value<ObjectId>("object_id");
        printf("Loaded %s\n", object_id.c_str());
      }
    }

    static void
    declare_params(tendrils& params)
    {
      params.declare(&LinemodDetector::threshold_, "threshold", "Matching threshold, as a percentage", 90.0f);
      params.declare(&LinemodDetector::db_, "db", "The DB").required(true);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&LinemodDetector::color_, "image", "An rgb full frame image.");
      inputs.declare(&LinemodDetector::depth_, "depth", "The 16bit depth image.");

      outputs.declare(&LinemodDetector::pose_results_, "pose_results", "The results of object recognition");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      detector_ = cv::linemod::getDefaultLINEMOD();
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Resize color to 640x480
      /// @todo Move resizing to separate cell, and try LINE-MOD w/ SXGA images
      cv::Mat color;
      if (color_->rows > 960)
        cv::pyrDown(color_->rowRange(0, 960), color);
      else
        color_->copyTo(color);

      std::vector<cv::Mat> sources;
      sources.push_back(color);
      sources.push_back(*depth_);

      std::vector<cv::linemod::Match> matches;
      detector_->match(sources, *threshold_, matches);
      pose_results_->clear();
      BOOST_FOREACH(const cv::linemod::Match & match, matches)
      {
        /// @todo Where do R and T come from? Can associate with matches[0].template_id
        PoseResult pose_result;
        //pose_result.set_R(R);
        //pose_result.set_T(T);
        pose_result.set_object_id(*db_, match.class_id);
        pose_results_->push_back(pose_result);
      }

      return ecto::OK;
    }

    cv::Ptr<cv::linemod::Detector> detector_;
    // Parameters
    spore<float> threshold_;
    // Inputs
    spore<cv::Mat> color_, depth_;

    /** The object recognition results */
    ecto::spore<std::vector<PoseResult> > pose_results_;
    /** The DB parameters */
    ecto::spore<ObjectDb> db_;
  };

} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, object_recognition_core::db::bases::ModelReaderBase<ecto_linemod::LinemodDetector>, "Detector",
          "Use LINE-MOD for object detection.");
