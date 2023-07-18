#ifndef LIBSYNEXENS3_ROS_DEVICE_H
#define LIBSYNEXENS3_ROS_DEVICE_H

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <thread>
// synexens
#include "libsynexens3/libsynexens3.h"
#include "synexens_package/synexens_ros_types.h"
#include "synexens_package/synexens_ros_params.h"
#include "synexens_package/synexens_calibration_transform_data.h"
// sensor_msgs/images
#include <image_transport/image_transport.hpp>
// sensor_msgs msg
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/distortion_models.hpp>



// camera
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

using namespace sensor_msgs;
using namespace image_transport;
using namespace msg;
using namespace sy3;
using namespace rclcpp;

class SynexensROSDevice
{
public:
  SynexensROSDevice(rclcpp::Node::SharedPtr nd);

  ~SynexensROSDevice();

  sy3_error startCameras();
  void stopCameras();

  // get data
  sy3_error getDepthFrame(sy3::frameset *capture, Image::SharedPtr &depth_frame);
  sy3_error getYuvRbgFrame(sy3::frameset *capture, Image::SharedPtr &rgb_frame);
  sy3_error getRbgFrame(sy3::frameset *capture, Image::SharedPtr &rgb_frame);
  sy3_error getIrFrame(sy3::frameset *capture, Image::SharedPtr &ir_image);
  sy3_error getPointCloud(sy3::frameset *capture, PointCloud2::SharedPtr &point_cloud);

private:
  // to ros
  sy3_error renderYuvRgbToROS(Image::SharedPtr &rgb_frame, sy3::frame *sy3_rgb_frame);
  sy3_error renderRgbToROS(Image::SharedPtr &rgb_frame, sy3::frame *sy3_rgb_frame);
  sy3_error renderDepthToROS(Image::SharedPtr &depth_image, sy3::frame *sy3_depth_frame);
  sy3_error renderIrToROS(Image::SharedPtr &ir_image, sy3::frame *sy3_ir_frame);
  sy3_error fillPointCloud(sy3::depth_frame *depth_image, PointCloud2::SharedPtr &point_cloud);

  // node
  rclcpp::Node::SharedPtr node_;
  // run threads
  void framePublisherThread();

  // sy3 device
  sy3::context *sy3_ctx_{nullptr};
  sy3::device *sy3_device_{nullptr};
  sy3::pipeline *sy3_pline_{nullptr};
  sy3::config *sy3_cfg_{nullptr};
  sy3::process_engine *sy3_engine_{nullptr};

  // Thread control
  std::atomic<bool> running_;

  // Threads
  std::thread frame_publisher_thread_;

  // form data
  SynexensCalibrationTransformData calibration_data_;

  //============== params start ==============//
  // setoption
  sy3_error setOptions();
  // configParams
  sy3_config_params sy3_config_params_;
  // params
  SynexensRosParams parmas_;
  //============== params end ==============//

  //============== pub start ==============//
  // image_transport::ImageTransport image_transport_;
  // rgb
  image_transport::Publisher rgb_raw_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr rgb_raw_camerainfo_publisher_;
  // depth
  image_transport::Publisher depth_raw_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr depth_raw_camerainfo_publisher_;
  // ir
  image_transport::Publisher ir_raw_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr ir_raw_camerainfo_publisher_;
  // point
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_publisher_;
  // depth to rgb
  image_transport::Publisher depth_rect_publisher_;
  // rgb to depth
  image_transport::Publisher rgb_rect_publisher_;
  //============== pub end ==============//
};

#endif // LIBSYNEXENS3_ROS_DEVICE_H