#!/usr/bin/env python3
"""
Simulation launch configuration for drone telemetry system.
Medium bandwidth usage (100 KB/s) with development and testing capabilities.
Includes both onboard simulation and ground station analysis tools.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase4       = get_package_share_directory('cbr_fase4')
    simulation_params  = os.path.join(pkg_fase4, "config", "simulation.yaml")
    fsm_params     = os.path.join(pkg_fase4, "config", "fsm.yaml")
    rviz_cfg = os.path.join(pkg_fase4, 'launch', 'simulation.rviz')
    vision_params   = os.path.join(pkg_fase4, "config", "vision.yaml")

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase4",
        description="Executable that implements the mission FSM"
    )
    
    # Telemetry Handler - core telemetry processing
    telemetry_handler_node = Node(
        package='telemetry_handler',
        executable='telemetry_handler',
        parameters=[simulation_params],
        output='screen'
    )

    # Telemetry Dashboard - unified GUI for comprehensive monitoring
    telemetry_dashboard_node = Node(
        package='telemetry_handler',
        executable='telemetry_dashboard',
        parameters=[simulation_params],
        output='screen'
    )

    # Telemetry Recorder - saves telemetry data
    telemetry_recorder_node = Node(
        package='telemetry_handler',
        executable='telemetry_recorder',
        parameters=[simulation_params],
        output='screen'
    )

    # RViz for visualization (delayed start to allow nodes to initialize)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    # Core telemetry nodes
    system_health_node = Node(
        package='cbr_drone_lib',
        executable='system_health',
        parameters=[simulation_params],
        output='screen'
    )

    # Gazebo-ROS2 Camera Bridge (keep only depth camera used by window detector)
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        remappings=[
            ('/depth_camera', '/depth_camera/image_raw')
        ]
    )

    fsm_node = Node(
        package='cbr_fase4',
        executable=LaunchConfiguration("mission"),
        parameters=[fsm_params],
        output='screen'
    )

    # Vision nodes (only window + QR)
    window_detector_node = Node(
        package='cbr_2025_cv_utils',
        executable='window_detector',
        parameters=[vision_params],
        output='screen'
    )

    qr_code_detector_node = Node(
        package='cbr_2025_cv_utils',
        executable='qr_code_detector',
        parameters=[
            vision_params,
            {'image_topic': '/oak/camera/image/raw', 'use_compressed': False}
        ],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        exec_arg,
        telemetry_handler_node,        
        telemetry_dashboard_node,
        telemetry_recorder_node,
        rviz_node,
        system_health_node,
        camera_bridge_node,
        delayed_fsm_node,
        window_detector_node,
        qr_code_detector_node
    ])
