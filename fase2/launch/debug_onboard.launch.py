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

    pkg_fase2       = get_package_share_directory('cbr_fase2')
    onboard_params  = os.path.join(pkg_fase2, "config", "onboard.yaml")
    fsm_params     = os.path.join(pkg_fase2, "config", "fsm.yaml")
    vision_params   = os.path.join(pkg_fase2, "config", "vision.yaml")

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase2",
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

    # Camera node
    camera_node = Node(
        package='camera_publisher',
        executable='oak',
        parameters=[onboard_params],
        output='screen'
    )

    # Base detector node (using new target_detector framework)
    base_detector_node = Node(
        package='cbr_cv_utils',
        executable='target_base_detector',
        parameters=[vision_params],
        output='screen'
    )

    # Package detector node (delayed slightly to avoid conflicts)
    package_detector_node = Node(
        package='cbr_cv_utils',
        executable='target_package_detector',
        parameters=[vision_params],
        output='screen'
    )

    # Delay package detector by 1 second after base detector
    delayed_package_detector = TimerAction(period=1.0, actions=[package_detector_node])

    # FSM node with GDB debug support
    fsm_node = Node(
        package='cbr_fase2',
        executable=LaunchConfiguration("mission"),
        prefix='gdb --args',  # String simples funciona melhor
        parameters=[fsm_params],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        exec_arg,
        system_health_node,
        telemetry_recorder_node,        
        # camera_node,
        base_detector_node,
        delayed_package_detector,
        delayed_fsm_node
    ])