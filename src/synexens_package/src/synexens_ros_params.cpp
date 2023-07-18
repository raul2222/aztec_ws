#include "synexens_package/synexens_ros_params.h"

void SynexensRosParams::setParams(rclcpp::Node::SharedPtr node_)
{
    // Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
    node_->declare_parameter<string>("tf_prefix", "sy3_");
    node_->get_parameter("tf_prefix", config_params_.tf_prefix);
    // Enable or disable the depth camera
    node_->declare_parameter<bool>("depth_enabled", true);
    node_->get_parameter("depth_enabled", config_params_.depth_enabled);
    // The resolution of the depth frame. Options are: 240P, 480P
    node_->declare_parameter<string>("depth_resolution", "240P");
    node_->get_parameter("depth_resolution", config_params_.depth_resolution);
    // Enable or disable the ir camera
    node_->declare_parameter<bool>("ir_enabled", true);
    node_->get_parameter("ir_enabled", config_params_.ir_enabled);
    // Enable or disable the color camera
    node_->declare_parameter<bool>("color_enabled", true);
    node_->get_parameter("color_enabled", config_params_.color_enabled);
    // Resolution at which to run the color camera. Valid options:  1080P
    node_->declare_parameter<string>("color_resolution", "1080P");
    node_->get_parameter("color_resolution", config_params_.color_resolution);
    // The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
    node_->declare_parameter<int>("fps", 7);
    node_->get_parameter("fps", config_params_.fps);
    node_->declare_parameter<bool>("point_cloud_enabled", true);
    node_->get_parameter("point_cloud_enabled", config_params_.point_cloud_enabled);
    // // True if mapped depth in color space should be enabled, only valid in depth480P rgb1080P
    node_->declare_parameter<bool>("depth_to_rgb_enabled", false);
    node_->get_parameter("depth_to_rgb_enabled", config_params_.depth_to_rgb_enabled);
    // // True if mapped color in depth space should be enabled, only valid in depth480P rgb1080P
    node_->declare_parameter<bool>("rgb_to_depth_enabled", false);
    node_->get_parameter("rgb_to_depth_enabled", config_params_.rgb_to_depth_enabled);
    // // Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied.
    node_->declare_parameter<bool>("rescale_ir_to_mono8", false);
    node_->get_parameter("rescale_ir_to_mono8", config_params_.rescale_ir_to_mono8);
    // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=-1
    node_->declare_parameter<int>("exposure", -1);
    node_->get_parameter("exposure", config_params_.exposure);
    // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
    node_->declare_parameter<int>("exposure_range_min", -1);
    node_->get_parameter("exposure_range_min", config_params_.exposure_range_min);
    // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
    node_->declare_parameter<int>("exposure_range_max", -1);
    node_->get_parameter("exposure_range_max", config_params_.exposure_range_max);
    // The Min Value of Depth Map Display Distance in mm. Use default setting if value=-1
    node_->declare_parameter<int>("distance_range_min", -1);
    node_->get_parameter("distance_range_min", config_params_.distance_range_min);
    // The Max Value of Depth Map Display Distance in mm. Use default setting if value=-1
    node_->declare_parameter<int>("distance_range_max", -1);
    node_->get_parameter("distance_range_max", config_params_.distance_range_max);
    // 0 to Disable Rgb image Flip, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("rgb_image_flip", -1);
    node_->get_parameter("rgb_image_flip", config_params_.rgb_image_flip);
    // 0 to Disable Rgb image Mirror, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("rgb_image_mirror", -1);
    node_->get_parameter("rgb_image_mirror", config_params_.rgb_image_mirror);
    // 0 to Disable Depth image Flip, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("depth_image_flip", -1);
    node_->get_parameter("depth_image_flip", config_params_.depth_image_flip);
    // 0 to Disable Depth image Mirror, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("depth_image_mirror", -1);
    node_->get_parameter("depth_image_mirror", config_params_.depth_image_mirror);
    // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("depth_image_filter", -1);
    node_->get_parameter("depth_image_filter", config_params_.depth_image_filter);
    // AMPLITITUD value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_amplititud_value", -1);
    node_->get_parameter("filter_amplititud_value", config_params_.filter_amplititud_value);
    // MEDIAN value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_median_value", -1);
    node_->get_parameter("filter_median_value", config_params_.filter_median_value);
    // GAUSS value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_gauss_value", -1);
    node_->get_parameter("filter_gauss_value", config_params_.filter_gauss_value);
    // EDGE value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_edge_value", -1);
    node_->get_parameter("filter_edge_value", config_params_.filter_edge_value);
    // SPECKLE value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_speckle_value", -1);
    node_->get_parameter("filter_speckle_value", config_params_.filter_speckle_value);
    // SOBEL value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_sobel_value", -1);
    node_->get_parameter("filter_sobel_value", config_params_.filter_sobel_value);
    // EDGE_MAD value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_mad_value", -1);
    node_->get_parameter("filter_mad_value", config_params_.filter_mad_value);
    // OKADA value sett. Use default setting if value=-1
    node_->declare_parameter<int>("filter_okada_value", -1);
    node_->get_parameter("filter_okada_value", config_params_.filter_okada_value);
}

// get device config
sy3_error SynexensRosParams::GetDeviceConfig(sy3_config_mode_t *configuration)
{
    if (!config_params_.color_enabled)
    {
        printf("Disabling RGB Camera \n");
        configuration->rgb_mode = SY3_COLOR_RESOLUTION_OFF;
    }
    else
    {
        printf("Setting RGB Camera Resolution: %s \n", config_params_.color_resolution.c_str());
        if (config_params_.color_resolution == "1080P")
        {
            configuration->rgb_mode = SY3_COLOR_RESOLUTION_1920x1080P;
        }
        else
        {
            printf("Invalid RGB Camera Resolution: %s", config_params_.color_resolution);
            return sy3_error::INVALID_FORMAT;
        }
    }

    if (config_params_.ir_enabled || config_params_.depth_enabled)
    {
        printf("Setting Depth Camera Resolution: %s \n", config_params_.depth_resolution.c_str());

        if (config_params_.depth_resolution == "240P")
        {
            configuration->depth_mode = SY3_DEPTH_RESOLUTION_320x240P;
        }
        else if (config_params_.depth_resolution == "480P")
        {
            configuration->depth_mode = SY3_DEPTH_RESOLUTION_640x480P;
        }
        else
        {
            printf("Invalid Depth Camera Resolution: %s \n", config_params_.depth_resolution);
            return sy3_error::INVALID_FORMAT;
        }
    }
    else
    {
        printf("Disabling Depth Camera");
        configuration->depth_mode = SY3_DEPTH_RESOLUTION_OFF;
    }

    if (!config_params_.depth_enabled && config_params_.point_cloud_enabled)
        printf("Depth Camera Disabled, PointCloud Off");
    config_params_.point_cloud_enabled = config_params_.point_cloud_enabled && config_params_.depth_enabled;

    configuration->enable_depth = config_params_.depth_enabled;
    configuration->enable_ir = config_params_.ir_enabled;
    configuration->enable_rgb = config_params_.color_enabled;

    return sy3_error::SUCCESS;
}

void SynexensRosParams::printfParams()
{
    printf("tf_prefix:%s \n", config_params_.tf_prefix.c_str());
    printf("depth_enabled:%d \n", config_params_.depth_enabled);
    printf("depth_resolution:%s \n", config_params_.depth_resolution.c_str());
    printf("ir_enabled:%d \n", config_params_.ir_enabled);
    printf("color_enabled:%d \n", config_params_.color_enabled);
    printf("color_resolution:%s \n", config_params_.color_resolution.c_str());
    printf("fps:%d \n", config_params_.fps);
    printf("exposure:%d \n", config_params_.exposure);
    printf("exposure_range_min:%d \n", config_params_.exposure_range_min);
    printf("point_cloud_enabled:%d \n", config_params_.point_cloud_enabled);
    printf("depth_to_rgb_enabled:%d \n", config_params_.depth_to_rgb_enabled);
    printf("rgb_to_depth_enabled:%d \n", config_params_.rgb_to_depth_enabled);
    printf("rescale_ir_to_mono8:%d \n", config_params_.rescale_ir_to_mono8);
    printf("exposure:%d \n", config_params_.exposure);
    printf("exposure_range_min:%d \n", config_params_.exposure_range_min);
    printf("exposure_range_max:%d \n", config_params_.exposure_range_max);
    printf("distance_range_min:%d \n", config_params_.distance_range_min);
    printf("distance_range_max:%d \n", config_params_.distance_range_max);
    printf("rgb_image_flip:%d \n", config_params_.rgb_image_flip);
    printf("rgb_image_mirror:%d \n", config_params_.rgb_image_mirror);
    printf("depth_image_flip:%d \n", config_params_.depth_image_flip);
    printf("depth_image_mirror:%d \n", config_params_.depth_image_mirror);
    printf("depth_image_filter:%d \n", config_params_.depth_image_filter);
    printf("filter_amplititud_value:%d \n", config_params_.filter_amplititud_value);
    printf("filter_median_value:%d \n", config_params_.filter_median_value);
    printf("filter_gauss_value:%d \n", config_params_.filter_gauss_value);
    printf("filter_edge_value:%d \n", config_params_.filter_edge_value);
    printf("filter_speckle_value:%d \n", config_params_.filter_speckle_value);
    printf("filter_sobel_value:%d \n", config_params_.filter_sobel_value);
    printf("filter_mad_value:%d \n", config_params_.filter_mad_value);
    printf("filter_okada_value:%d \n", config_params_.filter_okada_value);
}

sy3_config_params SynexensRosParams::getConfig()
{
    return config_params_;
}