// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/ros_camera.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace m3t {

RosColorCamera::RosColorCamera(
    const std::string &name,
    const Intrinsics &intrinsics)
    : ColorCamera{name} {
  intrinsics_ = intrinsics;
  ros_color_image_ = cv::Mat::zeros(
    intrinsics_.height, intrinsics_.width, CV_8UC3);
}

bool RosColorCamera::SetUp() {
  // const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  SaveMetaDataIfDesired();
  set_up_ = true;
  return UpdateImage(true);
}

void RosColorCamera::set_ros_color_image(cv::Mat &ros_color_image) {
  // THIS NEEDS TO BE DONE BETTER...
  const std::lock_guard<std::mutex> lock{mutex_};
  //ros_color_image_ = ros_color_image;
  image_ = ros_color_image;
}

void RosColorCamera::set_intrinsics(const Intrinsics &intrinsics) {
  // const std::lock_guard<std::mutex> lock{mutex_};
  intrinsics_ = intrinsics;
  set_up_ = false;
}

bool RosColorCamera::UpdateImage(bool synchronized) {

  const std::lock_guard<std::mutex> lock{mutex_};
  
  if (!set_up_) {
    std::cerr << "Set up loader color camera " << name_ << " first"
              << std::endl;
    return false;
  } else
    return true;
  // ros_color_image_.copyTo(image_);
  // // image_ = ros_color_image_;
  // if (image_.empty()) {
  //   std::cerr << "Could not read image from ROS Color Camera!" << std::endl;
  //   return false;
  // }
  // return true;
}

cv::Mat RosColorCamera::get_ros_color_image() {
  const std::lock_guard<std::mutex> lock{mutex_};
  return ros_color_image_; 
}


bool RosColorCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!(ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics_))) {
    std::cerr << "Could not read all required RosColorCamera parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);

  fs.release();

  // Process parameters
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

RosDepthCamera::RosDepthCamera(
    const std::string &name,
    const Intrinsics &intrinsics, 
    float depth_scale)
    : DepthCamera{name} {
  intrinsics_ = intrinsics;
  depth_scale_ = depth_scale;
}

bool RosDepthCamera::SetUp() {
  // const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  SaveMetaDataIfDesired();
  set_up_ = true;
  return UpdateImage(true);
}

void RosDepthCamera::set_intrinsics(const Intrinsics &intrinsics) {
  // const std::lock_guard<std::mutex> lock{mutex_};
  intrinsics_ = intrinsics;
  set_up_ = false;
}

void RosDepthCamera::set_depth_scale(float depth_scale) {
  // const std::lock_guard<std::mutex> lock{mutex_};
  depth_scale_ = depth_scale;
  set_up_ = false;
}

void RosDepthCamera::set_ros_depth_image(cv::Mat &ros_depth_image) {
  // THIS NEEDS TO BE DONE BETTER...
  const std::lock_guard<std::mutex> lock{mutex_};
  image_ = ros_depth_image;
}

bool RosDepthCamera::UpdateImage(bool synchronized) {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_) {
    std::cerr << "Set up loader depth camera " << name_ << " first"
              << std::endl;
    return false;
  }

  // image_ = ros_depth_image_;
  // if (image_.empty()) {
  //   std::cerr << "Could not read image from ROS Depth Camera!" << std::endl;
  //   return false;
  // }
  return true;
}

bool RosDepthCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!(ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics_) &&
        ReadRequiredValueFromYaml(fs, "depth_scale", &depth_scale_))) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  fs.release();

  // Process parameters
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

}  // namespace m3t
