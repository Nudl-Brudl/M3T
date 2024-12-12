#include <m3t/offline_measurement_subscriber.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>


namespace m3t {

OfflineMeasurementSubscriber::OfflineMeasurementSubscriber(
  const std::string &name, 
  const std::filesystem::path &csv_file_path)
  : Subscriber{name},
    csv_file_path_{csv_file_path} {
  std::filesystem::create_directories(csv_file_path_.parent_path());
  std::ofstream file(csv_file_path_, std::ios::trunc);
  if (file.is_open()) {
    file << "iteration,rgb_load_index,depth_load_index,r11,r12,r13,p1,r21,r22,r23,p2,r31,r32,r33,p3,e1,e2,e3,e4\n";
    file.close();
  } else {
    std::cerr << "Error: Unable to open file " << csv_file_path_ << std::endl;
  }
}

bool OfflineMeasurementSubscriber::SetUp() {
  set_up_ = true;
  return true;
}

void OfflineMeasurementSubscriber::set_color_camera(
  const std::shared_ptr<LoaderColorCamera> &loader_color_camera_ptr) {
  loader_color_camera_ptr_ = loader_color_camera_ptr;
}

void OfflineMeasurementSubscriber::set_depth_camera(
  const std::shared_ptr<LoaderDepthCamera> &loader_depth_camera_ptr) {
  loader_depth_camera_ptr_ = loader_depth_camera_ptr;    
}

void OfflineMeasurementSubscriber::set_body(const std::shared_ptr<Body> &body_ptr) {
  body_ptr_ = body_ptr;
}

bool OfflineMeasurementSubscriber::UpdateSubscriber(int iteration) {
  std::ofstream file(csv_file_path_, std::ios::app);
  if (file.is_open()) {
    file << iteration << "," << loader_color_camera_ptr_->load_index() << ",";
    file << loader_depth_camera_ptr_->load_index() << ",";
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        file << body_ptr_->body2world_pose().matrix()(i, j) << ",";
      }
    }
    file << "\n";
    file.close();
    return true;
  } else {
    std::cerr << "Error: Unable to open file " << csv_file_path_ << std::endl;
    return false;
  }  
}
} // namespace m3t