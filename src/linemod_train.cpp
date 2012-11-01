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

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <object_recognition_core/common/json.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using ecto::tendrils;
using ecto::spore;

namespace ecto_linemod
{
  struct Trainer
  {
    static void
    declare_params(tendrils& params)
    {
      /// @todo Parameters for various LINE-MOD settings?
      params.declare(&Trainer::path_, "path", "The path to where the temporary images are.").required(true);
      //params.declare(&Trainer::json_submethod_, "json_submethod", "The submethod to use, as a JSON string.").required(
      //  true);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //inputs.declare(&Trainer::image_, "image", "An rgb full frame image.").required(true);
      //inputs.declare(&Trainer::depth_, "depth", "The 16bit depth image.").required(true);
      //inputs.declare(&Trainer::depth_mask_, "mask", "The mask for the depth.").required(true);
      //outputs.declare(&Trainer::R_, "R", "The matching rotation of the template");
      //outputs.declare(&Trainer::T_, "T", "The matching translation of the template.");

      outputs.declare(&Trainer::detector_, "detector", "The LINE-MOD detector");
      //outputs.declare(&Trainer::Rs_, "Rs", "The matching rotations of the templates");
      //outputs.declare(&Trainer::Ts_, "Ts", "The matching translations of the templates.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      //or_json::mValue submethod = object_recognition_core::to_json(*json_submethod_);
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Ptr < cv::linemod::Detector > detector_ptr = cv::linemod::getDefaultLINEMOD();
      *detector_ = *detector_ptr;

      for (unsigned int index = 1;; ++index)
      {
        std::cout << "Loading images " << index << std::endl;
        cv::Mat image, depth, mask;

        std::string depth_path = boost::str(boost::format((*path_) + "/depth_%05d.png") % (index));
        std::string image_path = boost::str(boost::format((*path_) + "/image_%05d.png") % (index));
        std::string mask_path = boost::str(boost::format((*path_) + "/mask_%05d.png") % (index));

        // Make sure the files exist
        if ((!boost::filesystem::exists(depth_path)) || (!boost::filesystem::exists(image_path))
            || (!boost::filesystem::exists(mask_path)))
          break;
        depth = cv::imread(depth_path, CV_LOAD_IMAGE_ANYDEPTH);
        image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
        mask = cv::imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);

        std::vector<cv::Mat> sources;
        sources.push_back(image);
        sources.push_back(depth);

        detector_->addTemplate(sources, "object1", mask);

        //document.get_attachment < std::vector<cv::Mat> > ("Rs", Rs_[object_id]);
        //document.get_attachment < std::vector<cv::Mat> > ("Ts", Ts_[object_id]);
      }

      // Also store the pose of each template
      //Rs_->push_back(*R_);
      //Ts_->push_back(*T_);

      return ecto::OK;
    }

    spore<std::string> path_;
    ecto::spore<cv::linemod::Detector> detector_;
    //spore<std::vector<cv::Mat> > Rs_;
    //spore<std::vector<cv::Mat> > Ts_;
    //spore<std::string> json_submethod_;
  };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
