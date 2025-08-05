#!/usr/bin/env python3
"""
Onboard launch configuration for drone telemetry system.
Minimal bandwidth usage (9 KB/s) with essential telemetry only.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase1       = get_package_share_directory('cbr_fase2')
    onboard_params  = os.path.join(pkg_fase1, "config", "onboard.yaml")
    fsm_params     = os.path.join(pkg_fase1, "config", "fsm.yaml")

    # Core telemetry nodes
    system_health_node = Node(
        package='cbr_drone_lib',
        executable='system_health',
        parameters=[onboard_params],
        output='screen'
    )

    # Telemetry Recorder - saves telemetry data
    telemetry_recorder_node = Node(
        package='telemetry_handler',
        executable='telemetry_recorder',
        parameters=[onboard_params],
        output='screen'
    )

    # Camera node
    camera_node = Node(
        package='camera_publisher',
        executable='oak',
        parameters=[onboard_params],
        output='screen'
    )

    # Base detector node
    base_detector_node = Node(
        package='cbr_cv_utils',
        executable='base_detector',
        parameters=[onboard_params],
        output='screen'
    )

    fsm_node = Node(
        package='cbr_fase2',
        executable='fase2',
        parameters=[fsm_params],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        system_health_node,
        telemetry_recorder_node,        
        # camera_node,
        base_detector_node,
        delayed_fsm_node
    ])
