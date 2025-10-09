"""
Target Detection Framework for CBR 2025

This package provides a generalized framework for detecting different types of objects
in drone vision applications. The framework includes:

- TargetDetector: Abstract base class for all detectors
- BaseDetector: Detector for landing pads (blue/yellow colors)
- PackageDetector: Detector for gray packages
- ImageDebug: Debug visualization system

Example usage:
    from target_detector import BaseDetector, PackageDetector, run_detector
    
    # Create and run a base detector
    detector = BaseDetector()
    run_detector(detector)
    
    # Or create and run a package detector
    detector = PackageDetector()
    run_detector(detector)
"""

from .target_detector import TargetDetector, ImageDebug, run_detector
from .base_detector import BaseDetector
from .package_detector import PackageDetector

__all__ = [
    'TargetDetector',
    'ImageDebug', 
    'BaseDetector',
    'PackageDetector',
    'run_detector'
]
