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

#include <boost/filesystem.hpp>

#include <object_recognition_core/db/opencv.h>

#include "db_linemod.h"

#include <iterator>

namespace
{
  object_recognition_core::db::MimeType MIME_TYPE = "text/x-yaml";
}
namespace object_recognition_core
{
  namespace db
  {
    // Specializations for cv::linemod::Detector
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<cv::linemod::Detector>(const AttachmentName& attachment_name,
                                                                                 cv::linemod::Detector& value) const
    {
      // Get the binary file
      std::string file_name = temporary_yml_file_name(true);
      std::stringstream ss;
      this->get_attachment_stream(attachment_name, ss, MIME_TYPE);

      // Write it to disk
      std::ofstream writer(file_name.c_str());
      writer << ss.rdbuf() << std::flush;

      // Read it
      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      value.read(fs.root());

      cv::FileNode fn = fs["classes"];
      for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
        value.readClass(*i);

      // Delete the tmp file
      boost::filesystem::remove(file_name.c_str());
    }

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<cv::linemod::Detector>(
        const AttachmentName& attachment_name, cv::linemod::Detector& value)
    {
      throw("Not implemented");
    }

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<cv::linemod::Detector>(const AttachmentName& attachment_name,
                                                                                 const cv::linemod::Detector& value)
    {
      // First write the class to a file
      std::string file_name = temporary_yml_file_name(true);
      {
        cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
        value.write(fs);
#if CV_MAJOR_VERSION == 3
        std::vector<cv::String> ids = value.classIds();
#else
        std::vector < std::string > ids = value.classIds();
#endif
        fs << "classes" << "[";
        for (int i = 0; i < (int) ids.size(); ++i)
        {
          fs << "{";
          value.writeClass(ids[i], fs);
          fs << "}"; // current class
        }
        fs << "]"; // classes
        fs.release();
      }

      // Read the file as a stream
      std::ifstream reader(file_name.c_str());
      std::stringstream out;
      out << reader.rdbuf();

      set_attachment_stream(attachment_name, out, MIME_TYPE);
      boost::filesystem::remove(file_name.c_str());
    }

    // Specializations for std::vector<cv::Mat>
    // Not used but you never know ...
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<std::vector<cv::Mat> >(const AttachmentName& attachment_name,
                                                                                 std::vector<cv::Mat> &value) const
    {
      // Get the binary file
      std::string file_name = temporary_yml_file_name(false);
      std::stringstream ss;
      this->get_attachment_stream(attachment_name, ss, MIME_TYPE);

      // Write it to disk
      std::ofstream writer(file_name.c_str());
      writer << ss.rdbuf() << std::flush;

      // Read it
      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      cv::FileNode matrices = fs["matrices"];
      matrices >> value;

      boost::filesystem::remove(file_name.c_str());
    }

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<std::vector<cv::Mat> >(
        const AttachmentName& attachment_name, std::vector<cv::Mat> &value)
    {
      throw("Not implemented");
    }

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<std::vector<cv::Mat> >(const AttachmentName& attachment_name,
                                                                                 const std::vector<cv::Mat>& value)
    {
      // First write the class to a file
      std::string file_name = temporary_yml_file_name(false);
      {
        cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
        fs << "matrices" << value;

        fs.release();
      }

      // Read the file as a stream
      std::ifstream reader(file_name.c_str());
      std::stringstream out;
      out << reader.rdbuf();

      set_attachment_stream(attachment_name, out, MIME_TYPE);
      boost::filesystem::remove(file_name.c_str());
    }

    // Specializations for std::vector<float>
    // Not used but you never know ...
    template<>
    void
    object_recognition_core::db::DummyDocument::get_attachment<std::vector<float> >(const AttachmentName& attachment_name,
                                                                                 std::vector<float> &value) const
    {
      // Get the binary file
      std::stringstream ss;
      this->get_attachment_stream(attachment_name, ss, MIME_TYPE);

      float f;
      value.clear();
      while (!ss.eof()) {
        ss >> f;
        value.push_back(f);
      }
    }

    template<>
    void
    object_recognition_core::db::Document::get_attachment_and_cache<std::vector<float> >(
        const AttachmentName& attachment_name, std::vector<float> &value)
    {
      throw("Not implemented");
    }

    template<>
    void
    object_recognition_core::db::DummyDocument::set_attachment<std::vector<float> >(const AttachmentName& attachment_name,
                                                                                 const std::vector<float>& value)
    {
      std::stringstream out;
      for(std::vector<float>::const_iterator i=value.begin(); i != value.end(); ++i)
        out << *i;

      set_attachment_stream(attachment_name, out, MIME_TYPE);
    }
  }
}
