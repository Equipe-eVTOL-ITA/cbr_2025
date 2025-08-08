#!/usr/bin/env python3
"""
Ground station launch configuration for drone telemetry system.
Full bandwidth usage (400 KB/s) with complete telemetry reception and analysis tools.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_fase2   = get_package_share_directory('cbr_fase2')
    ground_station_params      = os.path.join(pkg_fase2, "config", "ground_station.yaml")
    rviz_cfg = os.path.join(pkg_fase2, 'launch', 'ground_station.rviz')

    # Telemetry Handler - core telemetry processing
    telemetry_handler_node = Node(
        package='telemetry_handler',
        executable='telemetry_handler',
        parameters=[ground_station_params],
        output='screen'
    )

    # Telemetry Dashboard - unified GUI for comprehensive monitoring
    telemetry_dashboard_node = Node(
        package='telemetry_handler',
        executable='telemetry_dashboard',
        parameters=[ground_station_params],
        output='screen'
    )

    # Telemetry Recorder - saves telemetry data
    telemetry_recorder_node = Node(
        package='telemetry_handler',
        executable='telemetry_recorder',
        parameters=[ground_station_params],
        output='screen'
    )

    # RViz for visualization (delayed start to allow nodes to initialize)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        telemetry_handler_node,        
        telemetry_dashboard_node,
        telemetry_recorder_node,
        rviz_node
    ])
