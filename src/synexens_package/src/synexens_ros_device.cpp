#include "synexens_package/synexens_ros_device.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// init ctx dev pline cfg
SynexensROSDevice::SynexensROSDevice(rclcpp::Node::SharedPtr nd) : node_(nd), sy3_device_(nullptr), sy3_ctx_(nullptr), sy3_pline_(nullptr), sy3_cfg_(nullptr)
{
    // setParams();
    // ======================= sett steam config start ======================= //
    parmas_.setParams(node_);
    sy3_config_params_ = parmas_.getConfig();
    // ======================= sett steam config end ======================= //

    sy3_error e;
    RCLCPP_INFO(node_->get_logger(), "version:%s \n", sy3::sy3_get_version(e));

    sy3_ctx_ = sy3::sy3_create_context(e);
    sy3::device *dev = sy3_ctx_->query_device(e);
    if (e != sy3::sy3_error::SUCCESS || !dev)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to open a SY3 device. Cannot continue. Error: %s", sy3::sy3_error_to_string(e));
        return;
    }

    sy3_device_ = dev;

    sy3_pline_ = sy3::sy3_create_pipeline(sy3_ctx_, e);
    if (e != sy3_error::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to create pipline! %s", sy3::sy3_error_to_string(e));
        return;
    }

    sy3::config *sy3_config = sy3::sy3_create_config(sy3_ctx_, e);
    if (e != sy3_error::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to create config! %s", sy3::sy3_error_to_string(e));
        return;
    }

    // ======================= Register our topics start ======================= //
    // node_->create_publisher<image_transport::ImageTransport(this)>("image/s",1);
    // publisher_ = node_->create_publisher<std_msgs::msg::String>("topic", 10);
    image_transport::ImageTransport image_transport_(node_);
    // rgb
    if (sy3_config_params_.color_enabled)
    {
        rgb_raw_publisher_ = image_transport_.advertise("rgb/image_raw", 1);
        rgb_raw_camerainfo_publisher_ = node_->create_publisher<CameraInfo>("rgb/camera_info", 1);
    }
    // ir
    if (sy3_config_params_.ir_enabled)
    {
        ir_raw_publisher_ = image_transport_.advertise("ir/image_raw", 1);
        ir_raw_camerainfo_publisher_ = node_->create_publisher<CameraInfo>("ir/camera_info", 1);
    }
    // depth
    if (sy3_config_params_.depth_enabled)
    {
        depth_raw_publisher_ = image_transport_.advertise("depth/image_raw", 1);
        depth_raw_camerainfo_publisher_ = node_->create_publisher<CameraInfo>("depth/camera_info", 1);
    }
    // point_cloud
    if (sy3_config_params_.point_cloud_enabled)
    {
        pointcloud_publisher_ = node_->create_publisher<PointCloud2>("points2", 1);
    }

    bool enable_mapping = sy3_config_params_.depth_enabled && sy3_config_params_.color_enabled && sy3_config_params_.depth_resolution == "480P" && sy3_config_params_.color_resolution == "1080P";
    // depth_to_rgb
    if (enable_mapping && sy3_config_params_.depth_to_rgb_enabled)
    {
        depth_rect_publisher_ = image_transport_.advertise("depth_to_rgb/image_raw", 1);
    }
    // rgb_to_depth
    if (enable_mapping && sy3_config_params_.rgb_to_depth_enabled)
    {
        rgb_rect_publisher_ = image_transport_.advertise("rgb_to_depth/image_raw", 1);
    }

    // ======================= Register our topics end ======================= //

    // ======================= sdk to configure start ======================= //
    sy3_cfg_ = sy3_config;
    sy3_config_mode_t sy3_configuration;

    if (sy3_device_)
    {
        sy3_error result = parmas_.GetDeviceConfig(&sy3_configuration);

        if (result != sy3_error::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Failed to generate a device configuration. Not starting camera!\n");
            return;
        }

        // ======================= configStreams start ======================= //
        {
            std::vector<sy3::sy3_stream> support_stream = sy3_device_->get_support_stream(e);
            for (int i = 0; i < support_stream.size(); i++)
            {
                RCLCPP_INFO(node_->get_logger(), "support stream:%s ", sy3_stream_to_string(support_stream[i]));
                std::vector<sy3::sy3_format> support_format = sy3_device_->get_support_format(support_stream[i], e);
                for (int j = 0; j < support_format.size(); j++)
                {
                    RCLCPP_INFO(node_->get_logger(), "\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
                }
            }

            int depth_width, depth_height;

            if (sy3_configuration.enable_depth || sy3_configuration.enable_ir)
            {
                switch (sy3_configuration.depth_mode)
                {
                case SY3_DEPTH_RESOLUTION_320x240P:
                    depth_width = 320;
                    depth_height = 240;
                    break;
                case SY3_DEPTH_RESOLUTION_640x480P:
                    depth_width = 640;
                    depth_height = 480;
                    break;
                default:
                    depth_width = 640;
                    depth_height = 480;
                    break;
                }
            }
            if (sy3_configuration.enable_depth)
                sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, depth_width, depth_height, e);

            if (e != SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "Enable depth failed: %d, %d, %s", depth_width, depth_height, sy3_error_to_string(e));
                return;
            }

            if (sy3_configuration.enable_ir)
                sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_IR, depth_width, depth_height, e);

            if (e != SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "Enable ir failed: %d, %d, %s", depth_width, depth_height, sy3_error_to_string(e));
                return;
            }

            if (sy3_configuration.enable_rgb)
            {
                int rgb_width, rgb_height;
                switch (sy3_configuration.rgb_mode)
                {
                case SY3_COLOR_RESOLUTION_1920x1080P:
                    rgb_width = 1920;
                    rgb_height = 1080;
                    break;
                default:
                    rgb_width = 1920;
                    rgb_height = 1080;
                    break;
                }
                sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, rgb_width, rgb_height, e);

                if (e != SUCCESS)
                {
                    RCLCPP_INFO(node_->get_logger(), "Enable rgb failed: %d, %d, %s", rgb_width, rgb_height, sy3_error_to_string(e));
                    return;
                }
            }
        }
        // ======================= configStreams end ======================= //
        // ======================= sdk start ======================= //
        {
            RCLCPP_INFO(node_->get_logger(), "Starting Streams \n");
            sy3_pline_->start(sy3_cfg_, e);
            if (e != sy3_error::SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "Failed to starting Streams:%s \n", sy3_error_to_string(e));
                return;
            }
        }
        // ======================= sdk end ======================= //
    }
    calibration_data_.initialize(sy3_config_params_, node_);
    // ======================= sdk to configure end ======================= //
}

SynexensROSDevice::~SynexensROSDevice() {}

void SynexensROSDevice::stopCameras() {}

sy3_error SynexensROSDevice::startCameras()
{
    sy3_error e;
    e = setOptions();
    if (e != sy3_error::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to Set Options: %s \n", sy3_error_to_string(e));
        return e;
    }
    // get algorithm p
    sy3_engine_ = sy3_pline_->get_process_engin(e);
    if (e != sy3_error::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to Get Process Engin:%s \n", sy3_error_to_string(e));
        return e;
    }
    running_ = true;
    frame_publisher_thread_ = std::thread(&SynexensROSDevice::framePublisherThread, this);

    return sy3::sy3_error::SUCCESS;
}

// rgb start
sy3_error SynexensROSDevice::getYuvRbgFrame(sy3::frameset *capture, Image::SharedPtr &rgb_image)
{
    sy3::rgb_frame *sy3_rgb_frame = capture->get_rgb_frame();

    if (!sy3_rgb_frame || !sy3_rgb_frame->get_data())
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot render rgb frame: no frame");
        return sy3_error::INCONSISTENCY_RES;
    }

    return renderYuvRgbToROS(rgb_image, sy3_rgb_frame);
}

sy3_error SynexensROSDevice::renderYuvRgbToROS(Image::SharedPtr &rgb_image, sy3::frame *sy3_rgb_frame)
{
    int rgb_width = sy3_rgb_frame->get_width();
    int rgb_height = sy3_rgb_frame->get_height();
    cv::Mat yuvImg(rgb_height, rgb_width, CV_8UC2, sy3_rgb_frame->get_data());
    cv::Mat rgb_buffer_mat = cv::Mat(rgb_height, rgb_width, CV_8UC3);
    cv::cvtColor(yuvImg, rgb_buffer_mat, cv::ColorConversionCodes::COLOR_YUV2BGR_YUYV);
    rgb_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rgb_buffer_mat).toImageMsg();

    return sy3_error::SUCCESS;
}
// rgb end

// ir start
sy3_error SynexensROSDevice::getIrFrame(sy3::frameset *capture, Image::SharedPtr &ir_frame)
{
    sy3::ir_frame *sy3_ir_frame = capture->get_ir_frame();

    if (!sy3_ir_frame || !sy3_ir_frame->get_data())
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot render ir frame: no frame");
        return sy3_error::INCONSISTENCY_RES;
    }

    return renderIrToROS(ir_frame, sy3_ir_frame);
}

sy3_error SynexensROSDevice::renderIrToROS(Image::SharedPtr &ir_image, sy3::frame *sy3_ir_frame)
{
    cv::Mat ir_buffer_mat(sy3_ir_frame->get_height(), sy3_ir_frame->get_width(), CV_16UC1, sy3_ir_frame->get_data());

    // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
    if (sy3_config_params_.rescale_ir_to_mono8)
    {
        cv::Mat tmp;
        cv::Mat new_image(sy3_ir_frame->get_height(), sy3_ir_frame->get_width(), CV_8UC1);
        cv::normalize(ir_buffer_mat, tmp, 0, 255, cv::NORM_MINMAX);
        cv::convertScaleAbs(tmp, new_image);
        ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
    }
    else
    {
        ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
    }

    return sy3_error::SUCCESS;
}
// ir end

// depth start
sy3_error SynexensROSDevice::getDepthFrame(sy3::frameset *capture, Image::SharedPtr &depth_frame)
{
    sy3::depth_frame *sy3_depth_frame = capture->get_depth_frame();

    if (!sy3_depth_frame || !sy3_depth_frame->get_data())
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot render depth frame: no frame \n");
        return sy3_error::INCONSISTENCY_RES;
    }

    return renderDepthToROS(depth_frame, sy3_depth_frame);
}

sy3_error SynexensROSDevice::renderDepthToROS(Image::SharedPtr &depth_image, sy3::frame *sy3_depth_frame)
{
    cv::Mat depth_frame_buffer_mat(sy3_depth_frame->get_height(), sy3_depth_frame->get_width(), CV_16UC1, sy3_depth_frame->get_data());

    depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();

    return sy3_error::SUCCESS;
}
// depth end

// PointCloud start
sy3_error SynexensROSDevice::getPointCloud(sy3::frameset *capture, PointCloud2::SharedPtr &point_cloud)
{
    sy3::depth_frame *sy3_depth_frame = capture->get_depth_frame();

    if (!sy3_depth_frame || !sy3_depth_frame->get_data())
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot render depth frame: no frame \n");
        return sy3_error::INCONSISTENCY_RES;
    }

    point_cloud->header.frame_id = calibration_data_.tf_prefix_;
    //  + calibration_data_.depth_camera_frame_;
    point_cloud->header.stamp = rclcpp::Clock().now();

    // Tranform depth image to point cloud
    return fillPointCloud(sy3_depth_frame, point_cloud);
}
/*
sy3_error SynexensROSDevice::fillPointCloud(sy3::depth_frame *depth_image, PointCloud2::SharedPtr &point_cloud)
{
    sy3::sy3_error e;
    if (!sy3_engine_)
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot get process engin \n");
        return sy3_error::INCONSISTENCY_RES;
    }
    sy3::points *points = sy3_engine_->comptute_points(depth_image, e);

    int length = points->get_length() / 3;
    int point_count = depth_image->get_height() * depth_image->get_width();
    // check points number
    if (point_count != length)
    {
        RCLCPP_INFO(node_->get_logger(), "Point Cloud Error: invalid points number \n");
        return sy3_error::INCONSISTENCY_RES;
    }

    point_cloud->height = depth_image->get_height();
    point_cloud->width = depth_image->get_width();
    point_cloud->is_dense = false;
    point_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

    pcd_modifier.resize(point_count);

    point3f_t *point_cloud_buffer = (point3f_t *)points->get_points();

    for (int i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
    {

        if (point_cloud_buffer[i].z <= 0.0f)
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
            *iter_x = kMillimeterToMeter * point_cloud_buffer[i].x;
            *iter_y = kMillimeterToMeter * point_cloud_buffer[i].y;
            *iter_z = kMillimeterToMeter * point_cloud_buffer[i].z;
        }
    }
    delete points;

    return sy3_error::SUCCESS;
}
*/

sy3_error SynexensROSDevice::fillPointCloud(sy3::depth_frame *depth_image, PointCloud2::SharedPtr &point_cloud)
{
    sy3::sy3_error e;
    if (!sy3_engine_)
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot get process engin \n");
        return sy3_error::INCONSISTENCY_RES;
    }
    sy3::points *points = sy3_engine_->comptute_points(depth_image, e);

    int length = points->get_length() / 3;
    int point_count = depth_image->get_height() * depth_image->get_width();
    // check points number
    if (point_count != length)
    {
        RCLCPP_INFO(node_->get_logger(), "Point Cloud Error: invalid points number \n");
        return sy3_error::INCONSISTENCY_RES;
    }

    point_cloud->height = depth_image->get_height();
    point_cloud->width = depth_image->get_width();
    point_cloud->is_dense = false;
    point_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

    pcd_modifier.resize(point_count);

    point3f_t *point_cloud_buffer = (point3f_t *)points->get_points();

    for (int i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
    {
        if (point_cloud_buffer[i].z <= 0.0f)
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
            float x = kMillimeterToMeter * point_cloud_buffer[i].x;
            float y = kMillimeterToMeter * point_cloud_buffer[i].y;
            float z = kMillimeterToMeter * point_cloud_buffer[i].z;

            // 90 degree rotation downward around X-axis
            *iter_x = z;
            *iter_y = x;
            *iter_z = y;
        }
    }
    delete points;

    return sy3_error::SUCCESS;
}






// PointCloud end

// mapping start
sy3_error SynexensROSDevice::getRbgFrame(sy3::frameset *capture, Image::SharedPtr &rgb_image)
{
    sy3::rgb_frame *sy3_rgb_frame = capture->get_rgb_frame();

    if (!sy3_rgb_frame || !sy3_rgb_frame->get_data())
    {
        RCLCPP_INFO(node_->get_logger(), "Cannot render rgb frame: no frame");
        return sy3_error::INCONSISTENCY_RES;
    }

    return renderRgbToROS(rgb_image, sy3_rgb_frame);
}

sy3_error SynexensROSDevice::renderRgbToROS(Image::SharedPtr &rgb_image, sy3::frame *sy3_rgb_frame)
{
    cv::Mat rgb_buffer_mat(sy3_rgb_frame->get_height(), sy3_rgb_frame->get_width(), CV_8UC3, sy3_rgb_frame->get_data());

    rgb_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rgb_buffer_mat).toImageMsg();

    return sy3_error::SUCCESS;
}
// mapping end

// fps start
volatile bool g_is_start = false;
volatile int g_fps = 0;
double  g_last_time = 0;
volatile int g_frame_count = 0;
std::thread fpsThread;
void calculate_framerate()
{

	while (g_is_start)
	{
		double cur_time = cv::getTickCount() / cv::getTickFrequency() * 1000;

		if (cur_time - g_last_time > 1000)
		{
			//printf("===============> cur_time:%lf \n", cur_time);
			g_fps = g_frame_count;
			g_frame_count = 0;
			g_last_time = cur_time;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
// fps end

// run thread
void SynexensROSDevice::framePublisherThread()
{
    rclcpp::WallRate loop_rate(sy3_config_params_.fps);
    sy3_error result;
    int i = 0;

    CameraInfo rgb_raw_camera_info;
    CameraInfo depth_raw_camera_info;
    CameraInfo ir_raw_camera_info;

    Time capture_time;
    sy3::frameset *capture;

    // First frame needs longer to arrive, we wait up to 15 seconds for it
    const unsigned int firstFrameWaitTime = SY3_DEFAULT_TIMEOUT;
    // fail if we did non receive 5 consecutive frames in a row
    const unsigned int regularFrameWaitTime = 1000 * 5 / sy3_config_params_.fps;
    unsigned int waitTime = firstFrameWaitTime;

    int nIndex = 0;
	int switch_flag = 1;
	g_is_start = true;
	fpsThread = std::thread(calculate_framerate);

    while (rclcpp::ok())
    {

        if (sy3_pline_)
        {
            capture = sy3_pline_->wait_for_frames(waitTime, result);
            if (!capture)
            {
                RCLCPP_ERROR(node_->get_logger(), "Timeout to get frame");
                continue;
            }

            waitTime = regularFrameWaitTime;
            capture_time = node_->now();
        }

        Image::SharedPtr rgb_raw_frame(new Image);
        Image::SharedPtr depth_raw_frame(new Image);
        Image::SharedPtr ir_raw_frame(new Image);
        Image::SharedPtr rgb_rect_frame(new Image);
        Image::SharedPtr depth_rect_frame(new Image);
        // PointCloud2::SharedPtr point_cloud;
        PointCloud2::SharedPtr point_cloud(new PointCloud2);

        // rgb
        if (sy3_config_params_.color_enabled)
        {
            // Only create rgb frame when we are using a device or we have a color image.
            // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
            if ((capture && capture->get_rgb_frame() != nullptr))
            {
                result = getYuvRbgFrame(capture, rgb_raw_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get RGB frame");
                    rclcpp::shutdown();
                    return;
                }

                rgb_raw_frame->header.stamp = capture_time;
                rgb_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
                rgb_raw_publisher_.publish(rgb_raw_frame);

                rgb_raw_camera_info.header.stamp = capture_time;
                depth_raw_camera_info.header.stamp = capture_time;
                const stream_profile *profile = capture->get_rgb_frame()->get_profile();
                sy3_intrinsics intrinsics = profile->get_intrinsics();
                calibration_data_.getDepthCameraInfo(rgb_raw_camera_info, &intrinsics);
                rgb_raw_camerainfo_publisher_->publish(rgb_raw_camera_info);
            }
        }

        // ir
        if (sy3_config_params_.ir_enabled)
        {
            if ((capture && capture->get_ir_frame() != nullptr))
            {
                // IR images are available in all depth modes
                result = getIrFrame(capture, ir_raw_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get raw IR frame");
                    rclcpp::shutdown();
                    return;
                }

                // Re-sychronize the timestamps with the capture timestamp
                ir_raw_frame->header.stamp = capture_time;
                ir_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
                ir_raw_publisher_.publish(ir_raw_frame);

                ir_raw_camera_info.header.stamp = capture_time;
                const stream_profile *profile = capture->get_ir_frame()->get_profile();
                sy3_intrinsics intrinsics = profile->get_intrinsics();
                calibration_data_.getDepthCameraInfo(ir_raw_camera_info, &intrinsics);
                ir_raw_camerainfo_publisher_->publish(ir_raw_camera_info);
            }
        }
        
        // depth
        if (sy3_config_params_.depth_enabled)
        {
            if ((capture && capture->get_depth_frame() != nullptr))
            {
                result = getDepthFrame(capture, depth_raw_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get raw depth frame");
                    rclcpp::shutdown();
                    return;
                }

                // Re-sychronize the timestamps with the capture timestamp
                depth_raw_frame->header.stamp = capture_time;
                depth_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
                
                depth_raw_publisher_.publish(depth_raw_frame);


                depth_raw_camera_info.header.stamp = capture_time;
                const stream_profile *profile = capture->get_depth_frame()->get_profile();
                sy3_intrinsics intrinsics = profile->get_intrinsics();
                calibration_data_.getDepthCameraInfo(depth_raw_camera_info, &intrinsics);
                depth_raw_camerainfo_publisher_->publish(depth_raw_camera_info);
            }
        }
        /*
        // depth
        if (sy3_config_params_.depth_enabled)
        {
            if ((capture && capture->get_depth_frame() != nullptr))
            {
                result = getDepthFrame(capture, depth_raw_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get raw depth frame");
                    rclcpp::shutdown();
                    return;
                }

                // Convert Image::SharedPtr to cv::Mat
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(depth_raw_frame, sensor_msgs::image_encodings::TYPE_16UC1);
                }
                catch (cv_bridge::Exception& e)
                {
                    //ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
                cv::Mat gray16 = cv_ptr->image;

                // Normalize the depth image
                cv::Mat tmp;
                cv::Mat gray8 = cv::Mat(gray16.size(), CV_8U);
                cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
                cv::convertScaleAbs(tmp, gray8);
                
                // Apply colormap for visualization
                cv::Mat colored_depth;
                cv::applyColorMap(gray8, colored_depth, cv::COLORMAP_JET);

                // Convert the cv::Mat to sensor_msgs/Image
                cv_bridge::CvImage out_msg;
                out_msg.header   = depth_raw_frame->header; // Same timestamp and tf frame as depth image
                out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
                out_msg.image    = colored_depth; // Your cv::Mat

                // Re-sychronize the timestamps with the capture timestamp
                out_msg.header.stamp = capture_time;
                out_msg.header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
                
                depth_raw_publisher_.publish(out_msg.toImageMsg());

                depth_raw_camera_info.header.stamp = capture_time;
                const stream_profile *profile = capture->get_depth_frame()->get_profile();
                sy3_intrinsics intrinsics = profile->get_intrinsics();
                calibration_data_.getDepthCameraInfo(depth_raw_camera_info, &intrinsics);
                depth_raw_camerainfo_publisher_->publish(depth_raw_camera_info);
            }
        }
*/

        // PointCloud
        // Only create pointcloud when we are using a device or we have a synchronized image.
        if (sy3_config_params_.point_cloud_enabled)
        {
            if ((capture && capture->get_depth_frame() != nullptr))
            {
                result = getPointCloud(capture, point_cloud);
                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get Point Cloud");
                    rclcpp::shutdown();
                    return;
                }

                pointcloud_publisher_->publish(*point_cloud.get());
            }
        }

        // mapping
        bool get_depth_and_rgb = capture && capture->get_rgb_frame() != nullptr && capture->get_depth_frame() != nullptr;
        if (((rgb_rect_publisher_.getNumSubscribers() > 0) || (depth_rect_publisher_.getNumSubscribers() > 0)) && get_depth_and_rgb)
        {
            sy3::frameset *mapped_frames = sy3_engine_->align_to_rgb(capture->get_depth_frame(), capture->get_rgb_frame(), result);

            if (rgb_rect_publisher_.getNumSubscribers() > 0)
            {
                result = getRbgFrame(mapped_frames, rgb_rect_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get rect RGB frame");
                    rclcpp::shutdown();
                    return;
                }

                rgb_rect_frame->header.stamp = capture_time;
                rgb_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
                rgb_rect_publisher_.publish(rgb_rect_frame);
            }

            if (depth_rect_publisher_.getNumSubscribers() > 0)
            {
                result = getDepthFrame(mapped_frames, depth_rect_frame);

                if (result != sy3_error::SUCCESS)
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get rect Depth frame");
                    rclcpp::shutdown();
                    return;
                }

                depth_rect_frame->header.stamp = capture_time;
                depth_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
                depth_rect_publisher_.publish(depth_rect_frame);
            }
            delete mapped_frames;
            mapped_frames = nullptr;
        }
        g_frame_count++;

        delete capture;
        capture = nullptr;
        
        RCLCPP_INFO(node_->get_logger(),"FPS:%d",g_fps);

        loop_rate.sleep();
    }
}

// set options
sy3_error SynexensROSDevice::setOptions()
{
    sy3_error e = sy3_error::SUCCESS;
    const sy3::sensor *sensor = sy3_device_->get_sensor(e);

    if (e != sy3_error::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed to get a SY3 sensor. Error: %s", sy3::sy3_error_to_string(e));
        return e;
    }

    RCLCPP_INFO(node_->get_logger(), "Set Sensor Options");

    if (sy3_config_params_.exposure_range_min > 0 && sy3_config_params_.exposure_range_max > 0 && sy3_config_params_.exposure_range_min < sy3_config_params_.exposure_range_max)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE_RANGE, (uint16_t)sy3_config_params_.exposure_range_max, (uint16_t)sy3_config_params_.exposure_range_min, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.exposure > 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, (uint16_t)sy3_config_params_.exposure, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }

    if (sy3_config_params_.distance_range_min > 0 && sy3_config_params_.distance_range_max > 0 && sy3_config_params_.distance_range_min < sy3_config_params_.distance_range_max)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_DISTANCE_RANGE, (uint16_t)sy3_config_params_.distance_range_max, (uint16_t)sy3_config_params_.distance_range_min, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.rgb_image_flip >= 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_RGB_IMAGE_FLIP, (uint16_t)sy3_config_params_.rgb_image_flip, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.rgb_image_mirror >= 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_RGB_IMAGE_MIRROR, (uint16_t)sy3_config_params_.rgb_image_mirror, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.depth_image_flip >= 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_TOF_IMAGE_FLIP, (uint16_t)sy3_config_params_.depth_image_flip, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.depth_image_mirror >= 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_TOF_IMAGE_MIRROR, (uint16_t)sy3_config_params_.depth_image_mirror, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }
    if (sy3_config_params_.depth_image_filter >= 0)
    {
        sensor->set_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, (uint16_t)sy3_config_params_.depth_image_filter, e);
        RCLCPP_INFO(node_->get_logger(), sy3::sy3_error_to_string(e));
    }

    // ===== filter daiding ===== //
    if (sy3_config_params_.filter_amplititud_value >= 0)
    {
    }
    return sy3_error::SUCCESS;
}
//=========================== parmas set get ... end ===========================//