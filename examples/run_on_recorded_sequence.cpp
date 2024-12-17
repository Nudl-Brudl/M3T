// SPDX-License-Identifier: MIT
// Copyright (c) 2024 König Johannes, Austrian Institute of Technology (AIT)

#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/loader_camera.h>
#include <m3t/static_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_model.h>
#include <m3t/region_modality.h>
#include <m3t/depth_model.h>
#include <m3t/depth_modality.h>
#include <m3t/renderer_geometry.h>
#include <m3t/texture_modality.h>
#include <m3t/offline_measurement_subscriber.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <memory>

int main(int argc, char *argv[]) {
  if (argc != 11) {
    std::cerr << "Not enough arguments: Provide color metafile, depth metafile, body "
                 "metafile, detector metafile, temp directory, use texture modality"
                 "use depth modality, save_viewer_imgs, measurement save dir";
    return -1;
  }

  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path depth_camera_metafile_path{argv[2]};
  const std::filesystem::path body_metafile_path{argv[3]};
  const std::filesystem::path detector_metafile_path{argv[4]};
  const std::filesystem::path temp_directory{argv[5]};
  const bool use_texture_modality = std::string(argv[6]) == "true" ? true : false;
  const bool use_depth_modality = std::string(argv[7]) == "true" ? true : false;
  const bool use_subscriber = std::string(argv[8]) == "true" ? true : false;
  const bool save_viewer_imgs = std::string(argv[9]) == "true" ? true : false;
  const std::filesystem::path meas_save_dir{argv[10]};

  // Set up subscriber
  std::string texture_str = use_texture_modality ? "_texture" : "";
  std::string depth_str = use_depth_modality ? "_depth" : "";
  std::filesystem::path csv_file_path{
    meas_save_dir / ("offline_meas" + texture_str + depth_str + ".csv")};
  auto offline_subscriber_ptr{
    std::make_shared<m3t::OfflineMeasurementSubscriber>(
      "subscriber", csv_file_path)};
  
  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  // Set up camera
  auto color_camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};
  auto depth_camera_ptr{std::make_shared<m3t::LoaderDepthCamera>(
      "depth_camera", depth_camera_metafile_path)};

  // Set up viewers
  auto viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "viewer", color_camera_ptr, renderer_geometry_ptr)};
  tracker_ptr->AddViewer(viewer_ptr);

  if (save_viewer_imgs) 
  {
    std::filesystem::path viewer_save_path{
        meas_save_dir / "images" / "viewer"};
    viewer_ptr->StartSavingImages(viewer_save_path);
  }
  

  // Set up depth renderer
  auto color_depth_renderer_ptr{
      std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
  auto depth_depth_renderer_ptr{
      std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};

  // Set up silhouette renderer
  auto color_silhouette_renderer_ptr{
      std::make_shared<m3t::FocusedSilhouetteRenderer>(
          "color_silhouette_renderer", renderer_geometry_ptr,
          color_camera_ptr)};

  // Set up body
  std::string body_name{"body"};
  auto body_ptr{std::make_shared<m3t::Body>(body_name, body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);
  color_depth_renderer_ptr->AddReferencedBody(body_ptr);
  depth_depth_renderer_ptr->AddReferencedBody(body_ptr);
  color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);


  // Set up models
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      body_name + "_region_model", body_ptr, 
      temp_directory / (body_name + "_region_model.bin"))};
  auto depth_model_ptr{std::make_shared<m3t::DepthModel>(
        body_name + "_depth_model", body_ptr,
        temp_directory / (body_name + "_depth_model.bin"))};

  // Set up modalities
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", body_ptr, color_camera_ptr, region_model_ptr)};
  auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, color_camera_ptr,
        color_silhouette_renderer_ptr)};
  auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        depth_model_ptr)};

  // Set up link
  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  if (use_texture_modality) link_ptr->AddModality(texture_modality_ptr);
  if (use_depth_modality) link_ptr->AddModality(depth_modality_ptr);

  // Set up optimizer
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  // Set up detector
  auto detector_ptr{std::make_shared<m3t::StaticDetector>(
      "detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);


   if (use_subscriber) 
  {
    offline_subscriber_ptr->set_color_camera(color_camera_ptr);
    offline_subscriber_ptr->set_depth_camera(depth_camera_ptr);
    tracker_ptr->AddSubscriber(offline_subscriber_ptr);
    offline_subscriber_ptr->set_body(body_ptr);
  }

  // Start tracking
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, true)) return -1;
  return 0;
}
