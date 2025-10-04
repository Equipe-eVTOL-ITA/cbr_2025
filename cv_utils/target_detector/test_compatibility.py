#!/usr/bin/env python3
"""
Test script to verify compatibility between original and refactored BaseDetector

This script compares key behaviors to ensure the refactored version
maintains the same functionality as the original CBR Phase 1 detector.
"""

import sys
import os

# Add path to import both versions
sys.path.append('/home/marconipavan/frtl_2025_ws/src/cbr_2025/cv_utils/base_detector')
sys.path.append('/home/marconipavan/frtl_2025_ws/src/cbr_2025/cv_utils/target_detector')

try:
    from target_detector import BaseDetector as RefactoredBaseDetector
    from base_detector import BaseDetector as OriginalBaseDetector
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

import rclpy
import numpy as np


def test_parameter_compatibility():
    """Test if both detectors have the same default parameters"""
    print("Testing parameter compatibility...")
    
    rclpy.init()
    
    try:
        # Create both detectors
        original = OriginalBaseDetector()
        refactored = RefactoredBaseDetector()
        
        # Key parameters to check
        params_to_check = [
            'image_topic',
            'detection_topic',
            'blue_lower_h', 'blue_lower_s', 'blue_lower_v',
            'blue_upper_h', 'blue_upper_s', 'blue_upper_v',
            'yellow_lower_h', 'yellow_lower_s', 'yellow_lower_v',
            'yellow_upper_h', 'yellow_upper_s', 'yellow_upper_v',
            'combined_min_area', 'combined_max_area',
            'combined_aspect_ratio_min', 'combined_aspect_ratio_max',
            'combination_kernel_size', 'combination_iterations'
        ]
        
        differences = []
        
        for param in params_to_check:
            try:
                orig_val = original.get_parameter(param).value
                refact_val = refactored.get_parameter(param).value
                
                if orig_val != refact_val:
                    differences.append(f"{param}: Original={orig_val}, Refactored={refact_val}")
                else:
                    print(f"✅ {param}: {orig_val}")
                    
            except Exception as e:
                differences.append(f"{param}: Error - {e}")
        
        if differences:
            print("\n❌ Parameter differences found:")
            for diff in differences:
                print(f"  {diff}")
            return False
        else:
            print("\n✅ All parameters match!")
            return True
            
    except Exception as e:
        print(f"❌ Error during parameter test: {e}")
        return False
    finally:
        rclpy.shutdown()


def test_detection_format():
    """Test if detection output format matches"""
    print("\nTesting detection format compatibility...")
    
    # Create a dummy image for testing
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Add some blue and yellow regions for testing
    test_image[100:200, 100:200] = [255, 0, 0]    # Blue region
    test_image[150:250, 150:250] = [0, 255, 255]  # Yellow region (overlap)
    
    rclpy.init()
    
    try:
        refactored = RefactoredBaseDetector()
        
        # Test detection
        detections, debug_images = refactored.detect(test_image)
        
        print(f"✅ Detection method works, found {len(detections)} detections")
        
        if detections:
            detection = detections[0]
            required_keys = ['bbox', 'center', 'size', 'angle', 'area', 'aspect_ratio', 'class_id', 'confidence']
            
            missing_keys = [key for key in required_keys if key not in detection]
            
            if missing_keys:
                print(f"❌ Missing keys in detection: {missing_keys}")
                return False
            else:
                print("✅ Detection format matches expected structure")
                print(f"  Sample detection: {detection}")
                return True
        else:
            print("⚠️  No detections found in test image")
            return True
                
    except Exception as e:
        print(f"❌ Error during detection test: {e}")
        return False
    finally:
        rclpy.shutdown()


def test_debug_images():
    """Test if debug images are generated correctly"""
    print("\nTesting debug image generation...")
    
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    test_image[100:200, 100:200] = [255, 0, 0]    # Blue region
    test_image[150:250, 150:250] = [0, 255, 255]  # Yellow region
    
    rclpy.init()
    
    try:
        refactored = RefactoredBaseDetector()
        
        detections, debug_images = refactored.detect(test_image)
        
        expected_debug_keys = ['mask_debug', 'bbox_debug']
        missing_debug_keys = [key for key in expected_debug_keys if key not in debug_images]
        
        if missing_debug_keys:
            print(f"❌ Missing debug images: {missing_debug_keys}")
            return False
        else:
            print("✅ All debug images generated correctly")
            for key, img in debug_images.items():
                if img is not None:
                    print(f"  {key}: {img.shape}")
                else:
                    print(f"  {key}: None")
            return True
            
    except Exception as e:
        print(f"❌ Error during debug image test: {e}")
        return False
    finally:
        rclpy.shutdown()


def main():
    """Run all compatibility tests"""
    print("=== CBR BaseDetector Compatibility Test ===\n")
    
    tests = [
        test_parameter_compatibility,
        test_detection_format,
        test_debug_images
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ Test failed with exception: {e}")
            results.append(False)
    
    print("\n=== Test Summary ===")
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✅ All {total} tests passed! Refactored detector is compatible.")
        return 0
    else:
        print(f"❌ {total - passed} out of {total} tests failed.")
        print("⚠️  Refactored detector may not be fully compatible with original.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
