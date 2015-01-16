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

#ifndef LINEMOD_ICP_H
#define LINEMOD_ICP_H

#include <opencv2/core/core.hpp>

/**
 * @brief Converts an image into a vector of 3D points
 * @param src_ref[in] The reference image
 * @param src_mod[in] The model image
 * @param pts_ref[out] The vector of 3D reference points
 * @param pts_mod[out] The vector of 3D model points
 * @return The ratio of reference points missing in the model to the total number of points*/
float matToVec(const cv::Mat_<cv::Vec3f> &src_ref, const cv::Mat_<cv::Vec3f> &src_mod, std::vector<cv::Vec3f>& pts_ref, std::vector<cv::Vec3f>& pts_mod);

/**
 * @brief Computes the centroid of 3D points
 * @param pts[in] The vector of 3D points
 * @param centroid[out] The cenroid */
void getMean(const std::vector<cv::Vec3f> &pts, cv::Vec3f& centroid);

/**
 * @brief Transforms the point cloud using the rotation and translation
 * @param src[in] The source point cloud
 * @param dst[out] The destination point cloud
 * @param R[in] The rotation matrix
 * @param T[in] The translation vector */
void transformPoints(const std::vector<cv::Vec3f> &src, std::vector<cv::Vec3f>& dst, const cv::Matx33f &R, const cv::Vec3f &T);

/**
 * @brief Computes the L2 distance between two point clouds,
 * with the distance computed for inliers only
 * (an inlier point is a point within a certain distance from the model).
 * @param model[in] The model point cloud
 * @param ref[in] The reference point cloud
 * @param dist_mean[out] The mean distance between the reference and the model point clouds
 * @param[in] mode The processing mode: 0-precise (maximum iterations), 1-fisrt approximation (few iterations), 2-better precision (more iterations)
 * @return The ratio of inlier points relative to the total number of points */
float getL2distClouds(const std::vector<cv::Vec3f> &model, const std::vector<cv::Vec3f> &ref, float &dist_mean, const float mode=0);

/**
 * @brief Iterative Closest Point algorithm that refines the object pose based on alignment of two point clouds (the reference and model).
 * @param[in] pts_ref The reference point cloud
 * @param[in] pts_model The model point cloud
 * @param[in, out] R The final rotation matrix
 * @param[in, out] T The final translation vector
 * @param[out] px_ratio_match The number of pixel with similar depth in both clouds
 * @param[in] mode The processing mode: 0-precise (maximum iterations), 1-fisrt approximation (few iterations), 2-better precision (more iterations)
 * @return Distance between the final transformed point cloud and the reference one. */
float icpCloudToCloud(const std::vector<cv::Vec3f> &pts_ref, std::vector<cv::Vec3f> &pts_model, cv::Matx33f& R, cv::Vec3f& T, float &px_ratio_match, int mode=0);

#endif // LINEMOD_ICP_H
