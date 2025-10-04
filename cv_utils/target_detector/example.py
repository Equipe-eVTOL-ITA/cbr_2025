#!/usr/bin/env python3
"""
Example usage of the Target Detection Framework

This script demonstrates how to use the different detectors
provided by the framework.
"""

import rclpy
from target_detector import BaseDetector, PackageDetector, run_detector


def main_base_detector():
    """Run the base detector for landing pads"""
    print("Starting Base Detector...")
    detector = BaseDetector()
    run_detector(detector)


def main_package_detector():
    """Run the package detector for gray packages"""
    print("Starting Package Detector...")
    detector = PackageDetector()
    run_detector(detector)


def main_combined():
    """Example of how you might run multiple detectors"""
    rclpy.init()
    
    try:
        # Create both detectors
        base_detector = BaseDetector()
        package_detector = PackageDetector()
        
        # You would need to implement your own logic here
        # to run both detectors simultaneously or switch between them
        print("Both detectors created successfully!")
        print("Base detector topics:")
        print(f"  - Input: {base_detector.image_topic}")
        print(f"  - Output: {base_detector.detection_topic}")
        print("Package detector topics:")
        print(f"  - Input: {package_detector.image_topic}")
        print(f"  - Output: {package_detector.detection_topic}")
        
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'base':
            main_base_detector()
        elif sys.argv[1] == 'package':
            main_package_detector()
        elif sys.argv[1] == 'combined':
            main_combined()
        else:
            print("Usage: python example.py [base|package|combined]")
    else:
        print("No detector specified. Available options:")
        print("  python example.py base     - Run base detector")
        print("  python example.py package  - Run package detector")
        print("  python example.py combined - Show both detectors")
