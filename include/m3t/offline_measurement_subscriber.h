// SPDX-License-Identifier: MIT
// Copyright (c) 2024 König Johannes, Austrian Institute of Technology (AIT)

#ifndef M3T_INCLUDE_M3T_OFFLINE_MEASUREMENT_SUBSCIBER_H_
#define M3T_INCLUDE_M3T_OFFLINE_MEASUREMENT_SUBSCIBER_H_

#include <m3t/common.h>
#include <m3t/subscriber.h>
#include <m3t/loader_camera.h>
#include <m3t/body.h>

namespace m3t {

class OfflineMeasurementSubscriber : public Subscriber {
 public:
  // Constructor
  OfflineMeasurementSubscriber(
    const std::string &name, 
    const std::filesystem::path &csv_file_path);

  // Setup method
  bool SetUp() override;

  // Setters
  void set_color_camera(
    const std::shared_ptr<LoaderColorCamera> &loader_color_camera_ptr);
  void set_depth_camera(
    const std::shared_ptr<LoaderDepthCamera> &loader_depth_camera_ptr);
  void set_body(const std::shared_ptr<Body> &body_ptr);

  // Main methods
  bool UpdateSubscriber(int iteration) override;

 private:
  // Variables
  std::filesystem::path csv_file_path_;
  std::shared_ptr<LoaderColorCamera> loader_color_camera_ptr_{};
  std::shared_ptr<LoaderDepthCamera> loader_depth_camera_ptr_{};
  std::shared_ptr<Body> body_ptr_{};

};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_OFFLINE_MEASUREMENT_SUBSCIBER_H_
