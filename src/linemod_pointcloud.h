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

#ifndef LINEMOD_POINTCLOUD_H
#define LINEMOD_POINTCLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core/core.hpp>

class LinemodPointcloud
{
public:
  /**
   * @brief Initializes the point cloud visualization class
   * @param nh The node handle to publish
   * @param topic The topic name to publish
   * @param frame_id The desired frame id */
  LinemodPointcloud(ros::NodeHandle& nh, const std::string &topic, const std::string &frame_id);

  /**
   * @brief Fills the point cloud with 3D points
   * @param pts[in] The vector of 3D points
   * @param color[in] The color used for visualization */
  void fill(const std::vector<cv::Vec3f> & pts, const cv::Vec3b &color);

  //! Cleans the point cloud */
  void clear();

  //! Publishes the point cloud */
  void publish();

  sensor_msgs::PointCloud2 pc_msg;

private:
  ros::Publisher pc_pub_;

  boost::shared_ptr<sensor_msgs::PointCloud2Modifier> modifier;
};

#endif // LINEMOD_POINTCLOUD_H
