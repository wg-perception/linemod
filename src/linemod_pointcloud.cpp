/*
 * Copyright 2014 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "linemod_pointcloud.h"

/** Initializes the point cloud visualization class */
LinemodPointcloud::LinemodPointcloud(ros::NodeHandle& nh, const std::string &topic, const std::string &frame_id):
  pc_pub_(nh.advertise<sensor_msgs::PointCloud2>(topic, 1))
{
  pc_msg.height = 1;
  pc_msg.width = 4;
  pc_msg.header.frame_id = frame_id;
  pc_msg.header.stamp = ::ros::Time::now();
  pc_msg.is_dense = false;
  pc_msg.is_bigendian = false;
  modifier  = boost::shared_ptr<sensor_msgs::PointCloud2Modifier>(new sensor_msgs::PointCloud2Modifier(pc_msg));
  modifier->setPointCloud2FieldsByString(2, "xyz", "rgb");
}

/** fill the point cloud with the 3D points */
void LinemodPointcloud::fill(const std::vector<cv::Vec3f> & pts, const cv::Vec3b &color)
{
  int size_old = modifier->size();
  modifier->resize(size_old + pts.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc_msg, "b");
  iter_x += size_old;
  iter_y += size_old;
  iter_z += size_old;
  iter_r += size_old;
  iter_g += size_old;
  iter_b += size_old;
  std::vector<cv::Vec3f>::const_iterator it_data = pts.begin();
  for(; it_data != pts.end(); ++it_data, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    *iter_x = (*it_data)(0);
    *iter_y = (*it_data)(1);
    *iter_z = (*it_data)(2);
    *iter_r = color(0);
    *iter_g = color(1);
    *iter_b = color(2);
  }
}

/** Cleans the point cloud */
void LinemodPointcloud::clear()
{
  modifier->clear();
}

/** Publishes the point cloud */
void LinemodPointcloud::publish()
{
  pc_pub_.publish(pc_msg);
}
