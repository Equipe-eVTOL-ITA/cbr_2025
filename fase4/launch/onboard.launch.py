#!/usr/bin/env python3
"""
Onboard launch configuration for drone telemetry system.
Minimal bandwidth usage (9 KB/s) with essential telemetry only.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase4       = get_package_share_directory('cbr_fase4')
    onboard_params  = os.path.join(pkg_fase4, "config", "onboard.yaml")
    fsm_params     = os.path.join(pkg_fase4, "config", "fsm.yaml")
    vision_params   = os.path.join(pkg_fase4, "config", "vision.yaml")

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase4",
        description="Executable that implements the mission FSM")
    

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

        # Vision nodes
    window_detector_node = Node(
        package='cbr_2025_cv_utils',
        executable='window_detector',
        parameters=[vision_params],
        output='screen'
    )

    qr_code_detector_node = Node(
        package='cbr_2025_cv_utils',
        executable='qr_code_detector',
        parameters=[vision_params],
        output='screen'
    )

    fsm_node = Node(
        package='cbr_fase4',
        executable=LaunchConfiguration("mission"),
        parameters=[fsm_params],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        exec_arg,
        system_health_node,
        telemetry_recorder_node,        
        # Camera not launched here; using external feed
        window_detector_node,
        qr_code_detector_node,
        delayed_fsm_node
    ])
