
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path
 
def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('synexens_package'), 'rviz', 'synexens_node_rviz.rviz')
    print(rviz_config_dir)
    return LaunchDescription([
        Node(
            package='synexens_package',
            executable='synexens_ros_node',
            name='synexens_ros',
            parameters=[{
                # Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
                'tf_prefix': 'sy3_',  
                # Enable or disable the depth camera
                'depth_enabled': True,
                # The resolution of the depth frame. Options are: 240P, 480P
                'depth_resolution': '240P',
                # Enable or disable the ir camera
                'ir_enabled': True,
                # Enable or disable the color camera
                'color_enabled': True,
                # Resolution at which to run the color camera. Valid options: 1080P
                'color_resolution': '1080P',
                # The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
                'fps': 30,
                # Generate a point cloud from depth data. Requires depth_enabled
                'point_cloud_enabled': True,
                # True if mapped depth in color space should be enabled, only valid in depth480P rgb1080P
                'depth_to_rgb_enabled': False,
                # True if mapped color in depth space should be enabled, only valid in depth480P rgb1080P
                'rgb_to_depth_enabled': False,
                # Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied.
                'rescale_ir_to_mono8': False,
                # The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=-1
                'exposure': -1,
                # The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
                'exposure_range_min': -1,
                # The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1
                'exposure_range_max': -1,
                # The Min Value of Depth Map Display Distance in mm. Use default setting if value=-1
                'distance_range_min': -1,
                # The Max Value of Depth Map Display Distance in mm. Use default setting if value=-1
                'distance_range_max': -1,
                # 0 to Disable Rgb image Flip, 1 to Enable. Use default setting if value=-1
                'rgb_image_flip': -1,
                # 0 to Disable Rgb image Mirror, 1 to Enable. Use default setting if value=-1
                'rgb_image_mirror': -1,
                # 0 to Disable Depth image Flip, 1 to Enable. Use default setting if value=-1
                'depth_image_flip': -1,
                # 0 to Disable Depth image Mirror, 1 to Enable. Use default setting if value=-1
                'depth_image_mirror': -1,
                # 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
                'depth_image_filter': -1,
                # AMPLITITUD value sett. Use default setting if value=-1
                'filter_amplititud_value': -1,
                # MEDIAN value sett. Use default setting if value=-1
                'filter_median_value': -1,
                # GAUSS value sett. Use default setting if value=-1
                'filter_gauss_value': -1,
                # EDGE value sett. Use default setting if value=-1
                'filter_edge_value': -1,
                # SPECKLE value sett. Use default setting if value=-1
                'filter_speckle_value': -1,
                # SOBEL value sett. Use default setting if value=-1
                'filter_sobel_value': -1,
                # EDGE_MAD value sett. Use default setting if value=-1
                'filter_mad_value': -1,
                # OKADA value sett. Use default setting if value=-1
                'filter_okada_value': -1,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir]
        )
    ])