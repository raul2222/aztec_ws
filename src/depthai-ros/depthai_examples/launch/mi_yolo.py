import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    depthai_examples_path = get_package_share_directory('depthai_examples')
    default_resources_path = os.path.join(depthai_examples_path, 'resources')

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    nnName = LaunchConfiguration('nnName', default = "yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr")
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_nnName_cmd = DeclareLaunchArgument(
        'nnName',
        default_value=nnName,
        description='Path to the object detection blob needed for detection')

    declare_resourceBaseFolder_cmd = DeclareLaunchArgument(
        'resourceBaseFolder',
        default_value=resourceBaseFolder,
        description='Path to the resources folder which contains the default blobs for the network')

    mobilenet_node = launch_ros.actions.Node(
            package='depthai_examples', executable='mobilenet_node',
            output='screen',
            parameters=[{'camera_model': camera_model},
                        {'nnName': nnName},
                        {'resourceBaseFolder': resourceBaseFolder}])

    ld = LaunchDescription()
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_nnName_cmd)
    ld.add_action(declare_resourceBaseFolder_cmd)
    ld.add_action(mobilenet_node)

    return ld
