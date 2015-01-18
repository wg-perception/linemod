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
#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/document.h>

#include "db_linemod.h"

using object_recognition_core::db::ObjectId;
using object_recognition_core::db::Document;

namespace linemod_ecto
{
  struct ModelFiller
  {
  public:
    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      typedef ModelFiller C;
      inputs.declare(&C::detector_, "detector", "The LINE-MOD detector").required(true);
      inputs.declare(&C::Rs_, "Rs", "The matching rotations of the templates").required(true);
      inputs.declare(&C::Ts_, "Ts", "The matching translations of the templates.").required(true);
      inputs.declare(&C::distances_, "distances", "The matching objects depths of the templates.").required(true);
      inputs.declare(&C::Ks_, "Ks", "The matching calibration matrices of the templates.").required(true);
      inputs.declare(&C::renderer_n_points_, "renderer_n_points", "The number of points on the sphere.").required(150);
      inputs.declare(&C::renderer_angle_step_, "renderer_angle_step", "The angle step sampling in degrees.").required(10);
      inputs.declare(&C::renderer_radius_min_, "renderer_radius_min", "The minimum scale sampling.").required(0.6);
      inputs.declare(&C::renderer_radius_max_, "renderer_radius_max", "The maximum scale sampling.").required(1.1);
      inputs.declare(&C::renderer_radius_step_, "renderer_radius_step", "The step scale sampling.").required(0.4);
      inputs.declare(&C::renderer_width_, "renderer_width", "Renderer parameter: the image width.").required(640);
      inputs.declare(&C::renderer_height_, "renderer_height", "Renderer parameter: the image height.").required(480);
      inputs.declare(&C::renderer_focal_length_x_, "renderer_focal_length_x", "Renderer parameter: the focal length x.").required(525.0);
      inputs.declare(&C::renderer_focal_length_y_, "renderer_focal_length_y", "Renderer parameter: the focal length y.").required(525.0);
      inputs.declare(&C::renderer_near_, "renderer_near", "Renderer parameter: near distance.").required(0.1);
      inputs.declare(&C::renderer_far_, "renderer_far", "Renderer parameter: far distance.").required(1000.0);

      outputs.declare(&C::db_document_, "db_document", "The filled document.");
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
    Document db_document;

    db_document.set_attachment<cv::linemod::Detector>("detector", *detector_);
    db_document.set_attachment<std::vector<cv::Mat> >("Rs", *Rs_);
    db_document.set_attachment<std::vector<cv::Mat> >("Ts", *Ts_);
    db_document.set_attachment<std::vector<float> >("distances", *distances_);
    db_document.set_attachment<std::vector<cv::Mat> >("Ks", *Ks_);
    db_document.set_field<int>("renderer_n_points", *renderer_n_points_);
    db_document.set_field<int>("renderer_angle_step", *renderer_angle_step_);
    db_document.set_field<double>("renderer_radius_min", *renderer_radius_min_);
    db_document.set_field<double>("renderer_radius_max", *renderer_radius_max_);
    db_document.set_field<double>("renderer_radius_step", *renderer_radius_step_);
    db_document.set_field<int>("renderer_width", *renderer_width_);
    db_document.set_field<int>("renderer_height", *renderer_height_);
    db_document.set_field<double>("renderer_focal_length_x", *renderer_focal_length_x_);
    db_document.set_field<double>("renderer_focal_length_y", *renderer_focal_length_y_);
    db_document.set_field<double>("renderer_near", *renderer_near_);
    db_document.set_field<double>("renderer_far", *renderer_far_);

    *db_document_ = db_document;

    return ecto::OK;
    }

  private:
    ecto::spore<cv::linemod::Detector> detector_;
    ecto::spore<Document> db_document_;
    ecto::spore<std::vector<cv::Mat> > Rs_;
    ecto::spore<std::vector<cv::Mat> > Ts_;
    ecto::spore<std::vector<float> > distances_;
    ecto::spore<std::vector<cv::Mat> > Ks_;
    ecto::spore<int> renderer_n_points_;
    ecto::spore<int> renderer_angle_step_;
    ecto::spore<double> renderer_radius_min_;
    ecto::spore<double> renderer_radius_max_;
    ecto::spore<double> renderer_radius_step_;
    ecto::spore<int> renderer_width_;
    ecto::spore<int> renderer_height_;
    ecto::spore<double> renderer_near_;
    ecto::spore<double> renderer_far_;
    ecto::spore<double> renderer_focal_length_x_;
    ecto::spore<double> renderer_focal_length_y_;
  };
}

ECTO_CELL(ecto_linemod, linemod_ecto::ModelFiller, "ModelFiller",
          "Populates a db document with a TOD model for persisting a later date.")
