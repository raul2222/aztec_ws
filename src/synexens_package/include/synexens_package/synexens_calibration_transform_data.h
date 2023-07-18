#ifndef LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H
#define LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H

#include <vector>
#include <libsynexens3/libsynexens3.h>
#include "synexens_package/synexens_ros_params.h"
#include "synexens_package/synexens_ros_types.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/distortion_models.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <angles/angles.h>

using namespace sy3;

class SynexensCalibrationTransformData
{
public:
    void initialize(const sy3_config_params &params,rclcpp::Node::SharedPtr nd);

    void setDepthCameraCalib(const sy3_intrinsics &intrinsics);
    void setColorCameraCalib(const sy3_intrinsics &intrinsics);
    int getDepthWidth();
    int getDepthHeight();
    int getColorWidth();
    int getColorHeight();
    void getDepthCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, sy3_intrinsics *intrinsics = nullptr);
    void getRgbCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, sy3_intrinsics *intrinsics = nullptr);
    void print();

    sy3_intrinsics rgb_camera_intrinsics_;
    sy3_intrinsics depth_camera_intrinsics_;

    std::string tf_prefix_ = "";
    std::string camera_base_frame_ = "camera_base";
    std::string rgb_camera_frame_ = "rgb_camera_link";
    std::string depth_camera_frame_ = "depth_camera_link";

private:
    void printCameraCalibration(sy3_intrinsics &calibration);
    void publishDepthToBaseTf(rclcpp::Node::SharedPtr nd);

    tf2::Quaternion getDepthToBaseRotationCorrection();
    tf2::Vector3 getDepthToBaseTranslationCorrection();
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

#endif // LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H
