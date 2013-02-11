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
#include <object_recognition_core/db/db.h>

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
      inputs.declare(&C::detector_, "detector", "The LINE-MOD detector");
      inputs.declare(&C::Rs_, "Rs", "The matching rotations of the templates");
      inputs.declare(&C::Ts_, "Ts", "The matching translations of the templates.");

      outputs.declare(&C::db_document_, "db_document", "The filled document.");
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
    Document db_document;

    db_document.set_attachment<cv::linemod::Detector>("detector", *detector_);
    db_document.set_attachment<std::vector<cv::Mat> >("Rs", *Rs_);
    db_document.set_attachment<std::vector<cv::Mat> >("Ts", *Ts_);

    *db_document_ = db_document;

    return ecto::OK;
    }

  private:
    ecto::spore<cv::linemod::Detector> detector_;
    ecto::spore<Document> db_document_;
    ecto::spore<std::vector<cv::Mat> > Rs_;
    ecto::spore<std::vector<cv::Mat> > Ts_;
  };
}

ECTO_CELL(ecto_linemod, linemod_ecto::ModelFiller, "ModelFiller",
          "Populates a db document with a TOD model for persisting a later date.")
