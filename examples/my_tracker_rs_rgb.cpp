/*
Tracking of multiple objects of same geometry with RGB and Realsense
*/
#include <filesystem/filesystem.h>
#include <m3t/realsense_camera.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>
#include <m3t/image_viewer.h>

#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <mutex>



int main(int argc, char *argv[]) {
  // Arguments of the main function
  int obj_id;
  std::istringstream(argv[1]) >> obj_id;

  int num_obj;
  std::istringstream(argv[2]) >> num_obj;

  std::filesystem::path data_path;
  std::istringstream(argv[3]) >> data_path;


  std::cout << "Obj_id = " << obj_id << std::endl;
  std::cout << "Num_obj = " << num_obj << std::endl;
  std::cout << "Data Folder Path = " << data_path << std::endl;


  // Create color camera
  auto color_camera_ptr{
    std::make_shared<m3t::RealSenseColorCamera>("color_camera")};
 
  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
    std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  // Create Viewer
  auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
    "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
  tracker_ptr->AddViewer(color_viewer_ptr);
  
  // Create Silhouette Renderer
  auto color_silhouette_renderer_ptr{
    std::make_shared<m3t::FocusedSilhouetteRenderer>(
      "color_silhouette_renderer", renderer_geometry_ptr, color_camera_ptr)};
  
  // Create Models
  std::shared_ptr<m3t::RegionModel> region_model_ptr;

  // TAKE CARE OF THE BODIES (i didn't kill them...)
  for (int i = 0; i < num_obj; i++) {
    std::string obj_instance_str = std::to_string(i);
    std::string body_name = "body_" + obj_instance_str;
    ///*
    std::filesystem::path metafile_path{data_path / (body_name + ".yaml")};

    // Create Bodies
    auto body_ptr{std::make_shared<m3t::Body>(body_name, metafile_path)};
    renderer_geometry_ptr->AddBody(body_ptr);
    color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);
    //*/

    // Create the model once and make all modalities use this model
    if (i == 0) {
      region_model_ptr = std::make_shared<m3t::RegionModel>(
        body_name + "region_model", 
        body_ptr, data_path / (body_name + "_region_model.bin"));
    }

    // Create Modalities
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    auto texture_modality_ptr(std::make_shared<m3t::TextureModality>(
      body_name + "texture_modality", body_ptr, color_camera_ptr, 
      color_silhouette_renderer_ptr));

    // Visualize Pose Result
    region_modality_ptr->set_visualize_pose_result(false);

    // Create Link
    auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", body_ptr)};
    link_ptr->AddModality(region_modality_ptr);
    link_ptr->AddModality(texture_modality_ptr);

    // Create Optimizer
    auto optimizer_ptr{
      std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr)};
    tracker_ptr->AddOptimizer(optimizer_ptr);

    // Create detector
    // std::filesystem::path detector_path{}
    auto detector_ptr{
      std::make_shared<m3t::StaticDetector>(
        body_name + "_detector", data_path / (body_name + "_detector.yaml"), 
        optimizer_ptr)};
    tracker_ptr->AddDetector(detector_ptr);

    std::cout << "Initialized Object " << i << std::endl;
  }

  std::cout << "Setting up Tracker..."<< std::endl;
  if (!tracker_ptr->SetUp()) return -1;
  std::cout << "Tracker is set up."<< std::endl;
  std::cout << "Start tracking process..."<< std::endl;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;

  return 0;
}