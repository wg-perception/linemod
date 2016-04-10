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

#ifndef DB_LINEMOD_H_
#define DB_LINEMOD_H_

#include <object_recognition_core/db/document.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
namespace cv {
using namespace cv::rgbd;
}
#else
#include <opencv2/objdetect/objdetect.hpp>
#endif

namespace object_recognition_core
{
  namespace db
  {
    // Specializations for cv::linemod::Detector
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<cv::linemod::Detector>(const AttachmentName& attachment_name,
                                                                                 cv::linemod::Detector& value) const;

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<cv::linemod::Detector>(
        const AttachmentName& attachment_name, cv::linemod::Detector& value);

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<cv::linemod::Detector>(const AttachmentName& attachment_name,
                                                                                 const cv::linemod::Detector& value);

    // Specializations for std::vector<cv::Mat>
    // Actually not needed anymore but you never know ....
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<std::vector<cv::Mat> >(const AttachmentName& attachment_name,
                                                                                 std::vector<cv::Mat>& value) const;

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<std::vector<cv::Mat> >(
        const AttachmentName& attachment_name, std::vector<cv::Mat>& value);

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<std::vector<cv::Mat> >(const AttachmentName& attachment_name,
                                                                                 const std::vector<cv::Mat>& value);


    // Specializations for std::vector<float>
    // Actually not needed anymore but you never know ....
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<std::vector<float> >(const AttachmentName& attachment_name,
                                                                                 std::vector<float>& value) const;

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<std::vector<float> >(
        const AttachmentName& attachment_name, std::vector<float>& value);

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<std::vector<float> >(const AttachmentName& attachment_name,
                                                                                 const std::vector<float>& value);

    /** Struct for detected objects info*/
    struct ObjData{
      ObjData(
              std::vector<cv::Vec3f> _pts_ref,
              std::vector<cv::Vec3f> _pts_model,
              std::string _match_class,
              const float _match_sim,
              const float _icp_dist,
              const float _icp_px_match,
              const cv::Matx33f _r,
              const cv::Vec3f _t){
        pts_ref = _pts_ref;
        pts_model = _pts_model;
        match_class = _match_class;
        match_sim = _match_sim;
        icp_dist = _icp_dist;
        icp_px_match = _icp_px_match,
        r = _r;
        t = _t;
        check_done = false;
      }
      std::vector<cv::Vec3f> pts_ref;
      std::vector<cv::Vec3f> pts_model;
      std::string match_class;
      float match_sim;
      float icp_dist;
      float icp_px_match;
      cv::Matx33f r;
      cv::Vec3f t;
      bool check_done;
    };
  }
}

#endif /* DB_LINEMOD_H_ */
