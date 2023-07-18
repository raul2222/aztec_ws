#ifndef LIBSYNEXENS3_ROS_PARAMS_H
#define LIBSYNEXENS3_ROS_PARAMS_H

#include "libsynexens3/libsynexens3.h"
#include "synexens_package/synexens_ros_types.h"
#include <string.h>
#include <rclcpp/rclcpp.hpp>
using namespace std;
using namespace sy3;

// set config parmas
struct sy3_config_params
{
    string tf_prefix;            // Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
    bool depth_enabled;          // Enable or disable the depth camera
    string depth_resolution;     // The resolution of the depth frame. Options are: 240P, 480P
    bool ir_enabled;             // Enable or disable the ir camera
    bool color_enabled;          // Enable or disable the color camera
    string color_resolution;     // Resolution at which to run the color camera. Valid options:  1080P
    int fps;                     // The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
    bool point_cloud_enabled;    // Generate a point cloud from depth data. Requires depth_enabled
    bool depth_to_rgb_enabled;   // True if mapped depth in color space should be enabled, only valid in depth480P rgb1080P
    bool rgb_to_depth_enabled;   // True if mapped color in depth space should be enabled, only valid in depth480P rgb1080P
    bool rescale_ir_to_mono8;    // Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied.
    int exposure;                // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=-1
    int exposure_range_min;      // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
    int exposure_range_max;      // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
    int distance_range_min;      // The Min Value of Depth Map Display Distance in mm. Use default setting if value=-1
    int distance_range_max;      // The Max Value of Depth Map Display Distance in mm. Use default setting if value=-1
    int rgb_image_flip;          // 0 to Disable Rgb image Flip, 1 to Enable. Use default setting if value=-1
    int rgb_image_mirror;        // 0 to Disable Rgb image Mirror, 1 to Enable. Use default setting if value=-1
    int depth_image_flip;        // 0 to Disable Depth image Flip, 1 to Enable. Use default setting if value=-1
    int depth_image_mirror;      // 0 to Disable Depth image Mirror, 1 to Enable. Use default setting if value=-1
    int depth_image_filter;      // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    int filter_amplititud_value; // AMPLITITUD value sett. Use default setting if value=-1
    int filter_median_value;     // MEDIAN value sett. Use default setting if value=-1
    int filter_gauss_value;      // GAUSS value sett. Use default setting if value=-1
    int filter_edge_value;       // EDGE value sett. Use default setting if value=-1
    int filter_speckle_value;    // SPECKLE value sett. Use default setting if value=-1
    int filter_sobel_value;      // SOBEL value sett. Use default setting if value=-1
    int filter_mad_value;        // EDGE_MAD value sett. Use default setting if value=-1
    int filter_okada_value;      // OKADA value sett. Use default setting if value=-1
};

class SynexensRosParams
{
public:
    // get device config
    sy3_error GetDeviceConfig(sy3_config_mode_t *configuration);
    // set parmas and default params
    void setParams(rclcpp::Node::SharedPtr node_);
    // pringf params
    void printfParams();
    sy3_config_params getConfig();

private:
    sy3_config_params config_params_;
};

#endif // LIBSYNEXENS3_ROS_PARAMS_H