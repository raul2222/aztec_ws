#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': 'MS200/scan'},
        {'port_name': '/dev/oradar'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''

def generate_launch_description():
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/lidar'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  ord = LaunchDescription()


  # base_link to laser_frame tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser',
    arguments=['0', '0' ,'0','0.0','0.0','0.0','base_link','odom']
  )
  
  ## tf2 - base_footprint to laser
  node_tf2_fp2laser = Node(
      name='tf2_ros_fp_laser',
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],   
  )


  ## tf2 - base_footprint to map
  node_tf2_fp2map = Node(
      name='tf2_ros_fp_map',
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'map'], 
  )


  ## tf2 - base_footprint to odom
  node_tf2_fp2odom = Node(
      name='tf2_ros_fp_odom',
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'laser_frame', 'odom'],
  )






  ord.add_action(ordlidar_node)
#  ord.add_action(base_link_to_laser_tf_node)#
#  ord.add_action(node_tf2_fp2laser)
#  ord.add_action( node_tf2_fp2map)
#  ord.add_action( node_tf2_fp2odom)
  
 

  return ord
