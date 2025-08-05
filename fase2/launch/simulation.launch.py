#!/usr/bin/env python3
"""
Simulation launch configuration for drone telemetry system.
Medium bandwidth usage (100 KB/s) with development and testing capabilities.
Includes both onboard simulation and ground station analysis tools.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase1       = get_package_share_directory('cbr_fase2')
    simulation_params  = os.path.join(pkg_fase1, "config", "simulation.yaml")
    fsm_params     = os.path.join(pkg_fase1, "config", "fsm.yaml")
    rviz_cfg = os.path.join(pkg_fase1, 'launch', 'simulation.rviz')

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

    # Base detector node
    base_detector_node = Node(
        package='cbr_cv_utils',
        executable='base_detector',
        parameters=[simulation_params],
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
        telemetry_handler_node,        
        telemetry_dashboard_node,
        telemetry_recorder_node,
        rviz_node,
        system_health_node,        
        base_detector_node,
        delayed_fsm_node
    ])
