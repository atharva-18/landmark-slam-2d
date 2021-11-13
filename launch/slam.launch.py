#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('landmark_slam_2d'),
        'config',
        'config.yaml'
        )

    # Launching landmar_slam_2d node
    landmark_slam_2d_node = Node(
            package="landmark_slam_2d",
            executable="landmark_slam_2d_node",
            name="landmark_slam_2d_node",
            parameters= [config]
    )

    ld.add_action(detect_leg_clusters_node)

    return ld 
