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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/model_utils.h>

#define USE_GLUT 1
#if USE_GLUT
#include <object_recognition_renderer/renderer_glut.h>
#else
#include <object_recognition_renderer/renderer_osmesa.h>
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
    inputs.declare(&Trainer::json_db_, "json_db", "The DB parameters", "{}").required(
        true);
    inputs.declare(&Trainer::object_id_, "object_id",
        "The object id, to associate this model with.").required(true);

      outputs.declare(&Trainer::detector_, "detector", "The LINE-MOD detector");
      outputs.declare(&Trainer::Rs_, "Rs", "The matching rotations of the templates");
      outputs.declare(&Trainer::Ts_, "Ts", "The matching translations of the templates.");
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
            "mesh", "");
    if (documents.empty()) {
      std::stringstream ss;
      ss << "No object with id \"" << *object_id_ << "\" with a mesh in the DB"
          << std::endl;
      throw std::runtime_error(ss.str());
    }

    // Load the mesh and save it to a temporary file
    object_recognition_core::db::Document document = documents[0];
    std::string mesh_path;
    {
      char mesh_path_tmp[L_tmpnam];
      tmpnam(mesh_path_tmp);
      mesh_path = std::string(mesh_path_tmp) + ".stl";
    }

    std::ofstream mesh_file;
    mesh_file.open(mesh_path.c_str());
    document.get_attachment_stream("mesh.stl", mesh_file);
    mesh_file.close();

      cv::Ptr<cv::linemod::Detector> detector_ptr = cv::linemod::getDefaultLINEMOD();
      *detector_ = *detector_ptr;

      // Define the display
      size_t width = 640, height = 480;
      double near = 0.1, far = 1000;
      double focal_length_x = 525, focal_length_y = 525;

    // the model name can be specified on the command line.
#if USE_GLUT
    RendererGlut renderer = RendererGlut(mesh_path);
#else
    RendererOSMesa renderer = RendererOSMesa(mesh_path);
#endif
    renderer.set_parameters(width, height, focal_length_x, focal_length_y, near,
        far);
    std::remove(mesh_path.c_str());

      RendererIterator renderer_iterator = RendererIterator(&renderer, 150);

      cv::Mat image, depth, mask;
      cv::Mat_<unsigned short> depth_short;
      cv::Matx33d R;
      cv::Vec3d T;
      for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
      {
      std::stringstream status;
      status << "Loading images " << (i+1) << "/"
          << renderer_iterator.n_templates();
      std::cout << status.str();

        renderer_iterator.render(image, depth, mask);
        R = renderer_iterator.R();
        T = renderer_iterator.T();

        depth.convertTo(depth_short, CV_16U, 1000);

        std::vector<cv::Mat> sources(2);
        sources[0] = image;
        sources[1] = depth;

        detector_->addTemplate(sources, "object1", mask);

        // Also store the pose of each template
        Rs_->push_back(cv::Mat(R));
        Ts_->push_back(cv::Mat(T));

      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
    }

      return ecto::OK;
    }

    /** The DB parameters as a JSON string */
    ecto::spore<std::string> json_db_;
    /** The id of the object to generate a trainer for */
    ecto::spore<std::string> object_id_;
    ecto::spore<cv::linemod::Detector> detector_;
    spore<std::vector<cv::Mat> > Rs_;
    spore<std::vector<cv::Mat> > Ts_;
  };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
