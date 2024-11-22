// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Koenig Johannes, Austrian Institute of Technology (AIT)

#ifndef M3T_INCLUDE_M3T_ROS_CAMERA_H_
#define M3T_INCLUDE_M3T_ROS_CAMERA_H_

#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace m3t {

/**
 * \brief \ref Camera that allows loading color images from a ROS topic.
 * @param intrinsics intrinsics of the camera that was used to record images.
 */
class RosColorCamera : public ColorCamera {
 public:
  // Constructor and setup method
  RosColorCamera(const std::string &name,
                    const Intrinsics &intrinsics);
  bool SetUp() override;

  // Setters
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_ros_color_image(cv::Mat &ros_color_image);

  // Main method
  bool UpdateImage(bool synchronized) override;

  // Getters
  cv::Mat get_ros_color_image();

  // Getters

 private:
  // Helper method
  bool LoadMetaData();

  // Data
  cv::Mat ros_color_image_{};

  // Internally used objects
  std::mutex mutex_;
};

/**
 * \brief \ref Camera that allows loading depth images from a ROS topic.
 * @param intrinsics intrinsics of the camera that was used to record images.
 * @param depth_scale scale with which pixel values have to be multiplied to get
 * depth in meter.
 */
class RosDepthCamera : public DepthCamera {
 public:
  // Constructor and setup method
  RosDepthCamera(const std::string &name,
                    const Intrinsics &intrinsics, 
                    float depth_scale);
  bool SetUp() override;

  // Setters
  void set_intrinsics(const Intrinsics &intrinsics);
  void set_depth_scale(float depth_scale);
  void set_ros_depth_image(cv::Mat &ros_depth_image);

  // Main method
  bool UpdateImage(bool synchronized) override;

  // Getters

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  cv::Mat ros_depth_image_{};

  // Internally used objects
  std::mutex mutex_;
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_ROS_CAMERA_H_
