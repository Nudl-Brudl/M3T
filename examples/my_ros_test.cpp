#include <filesystem/filesystem.h>
#include <m3t/realsense_camera.h>
#include <m3t/ros_camera.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
// #include <m3t/my_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>
#include <m3t/image_viewer.h>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>



int countImages(const std::filesystem::path &folderPath)
{
  int count = 0;
  for (const auto &entry : std::filesystem::directory_iterator(folderPath))
  {
    if (entry.is_regular_file() && entry.path().extension() == ".png")
    {
      count ++;
    }
  }
  return count;
}


int main(int argc, char *argv[]) 
{

  // Take care of arguments
  if (argc != 3) 
  {
    std::cerr << "Usage: " << argv[0] << " <add_viewer> <ros_style>" << std::endl;
    return -1;
  }

  int add_viewer;
  int ros_style;

  std::istringstream(argv[1]) >> add_viewer;
  std::istringstream(argv[2]) >> ros_style;

  // Create Color Camera
  // auto color_camera_ptr = std::make_shared<
  //   m3t::RealSenseColorCamera>("color_camera");
  auto color_camera_ptr = std::make_shared<
      m3t::RosColorCamera>("color_camera", 
        m3t::Intrinsics{921.76849365, 920.54547119, 
          654.91455078, 350.65985107, 
          1280, 720});

  std::filesystem::path img_folder{"/home/ubuntu/my_gigapose/M3T/data/video/"};
  
  // Create Tracker
  auto tracker_ptr = std::make_shared<m3t::Tracker>("tracker");

  // Create Renderer Geometry
  auto renderer_geometry_ptr = std::make_shared<
    m3t::RendererGeometry>("renderer_geometry");

  // Create Silhouette Renderer
  auto color_silhouette_renderer_ptr = std::make_shared<
    m3t::FocusedSilhouetteRenderer>(
      "color_silhouette_renderer", renderer_geometry_ptr, color_camera_ptr);

  // Create Body 
  std::string body_name = "body_0";
  std::filesystem::path geometry_path{
    "/home/ubuntu/my_gigapose/gigaPose_datasets/datasets/custom/models/000002.obj"};
  auto body_ptr = std::make_shared<m3t::Body>(
    body_name, geometry_path, 1.0f, true, true, m3t::Transform3fA::Identity());
  renderer_geometry_ptr->AddBody(body_ptr);
  color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);

  // Create Region Model
  std::filesystem::path data_path{
    "/home/ubuntu/my_gigapose/M3T/data/my_tracker"};
  auto region_model_ptr = std::make_shared<m3t::RegionModel>(
    body_name + "_region_model", 
    body_ptr, 
    data_path / (body_name + "_region_model.bin"));

  // Create Modalities
  auto region_modality_ptr = std::make_shared<m3t::RegionModality>(
    body_name + "_region_modality", 
    body_ptr, color_camera_ptr, region_model_ptr);
  auto texture_modality_ptr = std::make_shared<m3t::TextureModality>(
    body_name + "_texture_modality", 
    body_ptr, color_camera_ptr, color_silhouette_renderer_ptr);

  region_modality_ptr->set_visualize_pose_result(false);

  // Create Link
  auto link_ptr = std::make_shared<m3t::Link>(body_name + "_link", body_ptr);
  link_ptr->AddModality(region_modality_ptr);
  link_ptr->AddModality(texture_modality_ptr);

  // Create Optimizer
  auto optimizer_ptr = std::make_shared<m3t::Optimizer>(
    body_name + "_optimizer", link_ptr);
  tracker_ptr->AddOptimizer(optimizer_ptr);

  if (add_viewer)
  {
    // Create Viewer
    auto color_viewer_ptr = std::make_shared<m3t::NormalColorViewer>(
      "color_viewer", color_camera_ptr, renderer_geometry_ptr);
    tracker_ptr->AddViewer(color_viewer_ptr);
  }

  // Create Detector and initialize with pose:
  m3t::Transform3fA pose_init;
  pose_init.matrix() <<  -9.3489349e-01,  3.4826261e-01,  6.8464138e-02,  5.5691082e-02,
                    -2.3848304e-01, -4.7351161e-01, -8.4788716e-01,  7.0031846e-04,
                    -2.6286882e-01, -8.0901170e-01,  5.2573764e-01,  3.9514349e-01,
                    0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00;

  auto detector_ptr = std::make_shared<m3t::StaticDetector>(
      "body_0_detector", 
      optimizer_ptr,
      pose_init);
  tracker_ptr->AddDetector(detector_ptr);
  std::cout << "Inititialized Object" << std::endl;


  // Set up Tracker 
  tracker_ptr->set_start_tracking_after_detection(true);

  // Start Tracking
  if (!tracker_ptr->SetUp()) return -1;
  if (!ros_style)
  {
    if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  }
  else
  {

    if (tracker_ptr->ROSPrepareTrackerProcess(true, false))
      std::cout << "Tracker process PREPARED" << std::endl;

    const int rate_hz = 30;
    const auto loop_duration = std::chrono::milliseconds(1000 / rate_hz);
    int iteration = 0;
    int num_imgs = countImages(img_folder);
    int i_image = 0;
  
    while (true)
    {
      std::cout << "Iteration: " << iteration << std::endl;
      std::cout << "Image: " << i_image << std::endl;

      auto start_time = std::chrono::steady_clock::now();

      // Get the images
      std::filesystem::path img_path = img_folder / (
        "scene_" + std::to_string(i_image).insert(
          0 , 6 - std::to_string(i_image).length(), '0') + ".png");
      cv::Mat image = cv::imread(img_path.string(), cv::IMREAD_COLOR);
      if (image.empty())
      {
        std::cerr << "Could not load image: " << img_path << std::endl;
      }
      color_camera_ptr->set_ros_color_image(image);


      if (!tracker_ptr->ROSTracking(iteration)) 
      {
        std::cerr << "ERROR Tracker process FAILED!" << std::endl;
        return -1;
      }

      auto elapsed_time = std::chrono::steady_clock::now() - start_time;
      auto sleep_duration = loop_duration - elapsed_time;

      if(sleep_duration > std::chrono::milliseconds(0))
        std::this_thread::sleep_for(sleep_duration);
      // My_debug
      // if (i_image == 3)
      //   break;      
      // i_image++;
      
      iteration++;
      i_image = (i_image + 1) % num_imgs;
    }
  }
  return 0;
}