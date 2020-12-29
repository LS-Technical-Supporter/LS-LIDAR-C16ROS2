#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():
    decoder_dir = os.path.join(get_package_share_directory('lslidar_c16_decoder'), 'params', 'lslidar_c16.yaml')
    
    decoder_node = LifecycleNode(package='lslidar_c16_decoder',
                                executable='cloud_node',
                                name='cloud_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
    driver_node = LifecycleNode(package='lslidar_c16_driver',
                                executable='lslidar_node',
                                name='lslidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
                                
    return LaunchDescription([
        decoder_node,
        driver_node,
    ])

