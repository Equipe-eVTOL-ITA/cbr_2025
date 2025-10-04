#!/usr/bin/env python3
"""
Example configuration file for VisionNode with TargetDetector integration

This example shows how to configure the VisionNode to work with both
BaseDetector and PackageDetector simultaneously.
"""

import rclpy
from rclpy.node import Node
from target_detector import BaseDetector, PackageDetector, run_detector
import threading
import time


class ExampleUsage(Node):
    """Example showing how to use VisionNode with multiple detectors"""
    
    def __init__(self):
        super().__init__('vision_example')
        
        # This would typically be your FSM or control node
        self.get_logger().info("Vision example node started")
        
        # Create timer to simulate FSM queries
        self.timer = self.create_timer(1.0, self.example_callback)
    
    def example_callback(self):
        """Example of how FSM would query VisionNode"""
        # In real application, you would have a VisionNode instance here
        # and query it like this:
        
        # Check for base detections
        # if vision_node.isThereBaseDetection():
        #     closest_base = vision_node.getClosestBaseBbox()
        #     base_position = vision_node.getClosestBasePosition(drone_pos, drone_rpy)
        #     self.get_logger().info(f"Base detected at: {base_position}")
        
        # Check for package detections  
        # if vision_node.isTherePackageDetection():
        #     closest_package = vision_node.getClosestPackageBbox()
        #     package_position = vision_node.getClosestPackagePosition(drone_pos, drone_rpy)
        #     self.get_logger().info(f"Package detected at: {package_position}")
        
        self.get_logger().info("Example callback - would query vision node here")


def run_base_detector():
    """Run base detector in separate thread"""
    detector = BaseDetector()
    run_detector(detector)


def run_package_detector():
    """Run package detector in separate thread"""
    detector = PackageDetector()
    run_detector(detector)


def main():
    """
    Example of how to run the complete vision system:
    1. Start both detectors (BaseDetector and PackageDetector)
    2. Start VisionNode (C++) which subscribes to both detection topics
    3. FSM queries VisionNode for base and package information
    """
    
    rclpy.init()
    
    print("=== CBR 2025 Vision System Example ===")
    print("This example shows how to integrate:")
    print("1. BaseDetector (Python) -> /base_detector/detections")
    print("2. PackageDetector (Python) -> /package_detector/detections") 
    print("3. VisionNode (C++) subscribes to both topics")
    print("4. FSM queries VisionNode for integrated information")
    print()
    
    # Start detectors in separate threads
    base_thread = threading.Thread(target=run_base_detector, daemon=True)
    package_thread = threading.Thread(target=run_package_detector, daemon=True)
    
    print("Starting BaseDetector...")
    base_thread.start()
    time.sleep(1)
    
    print("Starting PackageDetector...")
    package_thread.start()
    time.sleep(1)
    
    print("Now start your C++ VisionNode with these parameters:")
    print("  enable_base_detection: true")
    print("  base_detection_topic: '/base_detector/detections'")
    print("  enable_package_detection: true") 
    print("  package_detection_topic: '/package_detector/detections'")
    print()
    
    # Start example FSM node
    try:
        example_node = ExampleUsage()
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
