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

#include <object_recognition_core/common/json.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

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
      params.declare(&Trainer::json_submethod_, "json_submethod", "The submethod to use, as a JSON string.").required(
          true);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Trainer::image_, "image", "An rgb full frame image.").required(true);
      inputs.declare(&Trainer::depth_, "depth", "The 16bit depth image.").required(true);
      inputs.declare(&Trainer::depth_mask_, "mask", "The mask for the depth.").required(true);
      outputs.declare(&Trainer::R_, "R", "The matching rotation of the template");
      outputs.declare(&Trainer::T_, "T", "The matching translation of the template.");

      outputs.declare(&Trainer::depths_, "depths", "The matching rotations of the templates");
      outputs.declare(&Trainer::images_, "images", "The matching rotations of the templates");
      outputs.declare(&Trainer::masks_, "masks", "The matching rotations of the templates");
      outputs.declare(&Trainer::Rs_, "Rs", "The matching rotations of the templates");
      outputs.declare(&Trainer::Ts_, "Ts", "The matching translations of the templates.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      or_json::mValue submethod = object_recognition_core::to_json(*json_submethod_);
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      CV_Assert(depth_mask_->type() == CV_8UC1);

      // Figure out the interesting bounding box in the depth mask
      int x_min = depth_->cols, x_max = 0, y_min = depth_->rows, y_max = 0;
      for (int v = 0; v != depth_->rows; ++v)
      {
        uchar *row =depth_mask_->ptr(v);
        uchar*row_end = row+depth_->cols;
        for (; row!=row_end; ++row)
        {
          if (!*row)
            continue;
          //TODO
        }
      }

      cv::Rect area(x_min, y_min, x_max - x_min, y_max - y_min);

      // Only save the interesting areas to the models
      cv::Mat depth, image, mask;
      cv::Range row_range = cv::Range(area.y, area.y + area.height), col_range = cv::Range(area.x, area.x + area.width);
      cv::Range row_range_image = cv::Range((area.y * image_->cols) / mask.cols,
                                            ((area.y + area.height) * image_->cols) / mask.cols), col_range_image =
          cv::Range((area.x * image_->rows) / mask.rows, ((area.x + area.width) * image_->rows) / mask.rows);

      (*depth_)(row_range, col_range).copyTo(depth);
      cv::resize((*image_)(row_range_image, col_range_image), image, cv::Size(area.width, area.height), 0.0, 0.0,
                 CV_INTER_NN);
      (*depth_mask_)(row_range, col_range).copyTo(mask);

      depths_->push_back(depth);
      images_->push_back(image);
      masks_->push_back(mask);

      // Also store the pose of each template
      Rs_->push_back(*R_);
      Ts_->push_back(*T_);

      return ecto::OK;
    }

    spore<cv::Mat> image_, depth_, depth_mask_;
    spore<std::vector<cv::Mat> > depths_;
    spore<std::vector<cv::Mat> > images_;
    spore<std::vector<cv::Mat> > masks_;
    spore<std::vector<cv::Mat> > Rs_;
    spore<std::vector<cv::Mat> > Ts_;
    spore<cv::Mat> R_;
    spore<cv::Mat> T_;
    spore<std::string> json_submethod_;
  };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
