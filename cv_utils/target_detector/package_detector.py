#!/usr/bin/env python3
"""
Package Detection Node for CBR 2025 using TargetDetector Framework

This detector specializes in finding gray packages on landing pads.
Unlike the base detector, it only needs to detect a single color (gray)
and focuses on object orientation and shape analysis.

Inherits from TargetDetector and implements the abstract methods:
- _setup_color_parameters(): Sets up gray HSV ranges
- detect(): Finds gray package regions with orientation analysis
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple

from .target_detector import TargetDetector, run_detector


class PackageDetector(TargetDetector):
    """Detector for gray packages on landing pads"""
    
    def __init__(self):
        super().__init__('package_detector')
        
        # Initialize color ranges after parent initialization
        self._initialize_color_ranges()
        
        self.get_logger().info("Package detector initialized successfully")
    
    def _setup_color_parameters(self):
        """Setup HSV color parameters for gray detection"""
        # Default HSV ranges for gray packages
        gray_lower = {'h': 0, 's': 0, 'v': 50}
        gray_upper = {'h': 180, 's': 50, 'v': 150}
        
        # Setup parameters for gray color
        self.gray_lower, self.gray_upper, self.gray_kernel_size, self.gray_iterations = \
            self.setup_color_parameters('gray', gray_lower, gray_upper, kernel_size=5, iterations=2)
        
        # Additional parameters for package detection
        self.declare_parameter('package_min_area', 0.0005)
        self.declare_parameter('package_max_area', 0.5)
        self.declare_parameter('package_aspect_ratio_min', 0.3)
        self.declare_parameter('package_aspect_ratio_max', 3.0)
        self.declare_parameter('package_min_contour_points', 50)
    
    def _initialize_color_ranges(self):
        """Initialize package detection parameters from ROS parameters"""
        self.package_min_area = float(self.get_parameter('package_min_area').value)
        self.package_max_area = float(self.get_parameter('package_max_area').value)
        self.package_aspect_ratio_min = float(self.get_parameter('package_aspect_ratio_min').value)
        self.package_aspect_ratio_max = float(self.get_parameter('package_aspect_ratio_max').value)
        self.package_min_contour_points = int(self.get_parameter('package_min_contour_points').value)
    
    def detect(self, image: np.ndarray) -> Tuple[List[Dict], Dict]:
        """
        Detect gray packages with orientation analysis
        
        Args:
            image: Input BGR image
            
        Returns:
            Tuple of (detections_list, debug_images_dict)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Segment gray color
        gray_mask = self._segment_color(hsv, 'gray')
        
        # Create debug images
        debug_images = self._create_debug_images(image, gray_mask)
        
        # Find package regions
        detections = self._find_package_regions(gray_mask)
        
        # Draw detections on bbox debug image
        if 'bbox_debug' in debug_images:
            self._draw_detections_on_image(debug_images['bbox_debug'], detections)
        
        return detections, debug_images
    
    def _segment_color(self, hsv_image: np.ndarray, color_name: str) -> np.ndarray:
        """Segment gray color from HSV image"""
        if color_name != 'gray':
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        # Apply HSV color filtering for gray
        mask = cv2.inRange(hsv_image, self.gray_lower, self.gray_upper)
        
        # Apply morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.gray_kernel_size, self.gray_kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.gray_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return mask
    
    def _create_debug_images(self, image: np.ndarray, gray_mask: np.ndarray) -> Dict:
        """Create debug images for visualization"""
        debug_images = {}
        
        # Create mask debug image
        mask_debug = image.copy()
        
        # Create gray overlay
        gray_overlay = np.zeros_like(image)
        gray_overlay[gray_mask > 0] = (128, 128, 128)  # Gray pixels in gray
        
        # Apply overlay with transparency
        alpha = 0.4
        mask_debug = cv2.addWeighted(mask_debug, 1 - alpha, gray_overlay, alpha, 0)
        
        debug_images['mask_debug'] = mask_debug
        debug_images['bbox_debug'] = image.copy()
        
        return debug_images
    
    def _find_package_regions(self, gray_mask: np.ndarray) -> List[Dict]:
        """Find gray package regions with orientation analysis"""
        detections = []
        
        # Get image dimensions for normalization
        img_height, img_width = gray_mask.shape[:2]
        img_area = img_height * img_width
        
        # Convert normalized area thresholds to actual pixel counts
        min_area_pixels = self.package_min_area * img_area
        max_area_pixels = self.package_max_area * img_area
        
        # Find contours in the gray mask
        contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < min_area_pixels or area > max_area_pixels:
                continue
            
            # Filter by contour complexity (packages should have well-defined shapes)
            if len(contour) < self.package_min_contour_points:
                continue
            
            # Get axis-aligned bounding rect
            x, y, w, h = cv2.boundingRect(contour)
            
            # Use minAreaRect to get oriented bounding box
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rect_w, rect_h), angle_deg = rect
            
            # Ignore degenerate sizes
            if rect_w <= 0 or rect_h <= 0:
                continue
            
            # Compute aspect ratio from rotated box
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h) if min(rect_w, rect_h) > 0 else 0
            if not (self.package_aspect_ratio_min <= aspect_ratio <= self.package_aspect_ratio_max):
                continue
            
            # Calculate shape metrics for confidence scoring
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            # Calculate extent (area ratio to bounding rectangle)
            extent = area / (w * h) if (w * h) > 0 else 0
            
            # Calculate confidence based on shape metrics
            confidence = self._calculate_confidence(solidity, extent, aspect_ratio)
            
            detection = {
                'bbox': (x, y, w, h),                         # axis-aligned for quick drawing/roi
                'center': (float(cx), float(cy)),             # rotated center in pixels
                'size': (float(rect_w), float(rect_h)),       # rotated width/height in pixels
                'angle': float(np.deg2rad(angle_deg)),        # angle in radians
                'area': area,
                'aspect_ratio': aspect_ratio,
                'solidity': solidity,
                'extent': extent,
                'class_id': 'package',
                'confidence': confidence
            }
            detections.append(detection)
        
        # Sort detections by confidence (highest first)
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        
        return detections
    
    def _calculate_confidence(self, solidity: float, extent: float, aspect_ratio: float) -> float:
        """
        Calculate detection confidence based on shape metrics
        
        Args:
            solidity: Ratio of contour area to convex hull area
            extent: Ratio of contour area to bounding rectangle area
            aspect_ratio: Ratio of longer side to shorter side
            
        Returns:
            Confidence score between 0 and 1
        """
        # Ideal values for package-like objects
        ideal_solidity = 0.95      # Packages should be fairly solid/convex
        ideal_extent = 0.8         # Packages should fill most of their bounding rect
        ideal_aspect_ratio = 1.5   # Slightly rectangular is common for packages
        
        # Calculate individual scores (closer to ideal = higher score)
        solidity_score = 1.0 - abs(solidity - ideal_solidity)
        extent_score = 1.0 - abs(extent - ideal_extent)
        aspect_ratio_score = 1.0 - min(abs(aspect_ratio - ideal_aspect_ratio) / ideal_aspect_ratio, 1.0)
        
        # Weighted combination
        confidence = (0.4 * solidity_score + 0.4 * extent_score + 0.2 * aspect_ratio_score)
        
        # Ensure confidence is in [0, 1] range
        return max(0.0, min(1.0, confidence))
    
    def _draw_detections_on_image(self, image: np.ndarray, detections: List[Dict]):
        """Draw detection bounding boxes and info on image"""
        img_height, img_width = image.shape[:2]
        img_area = img_height * img_width
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            area = detection['area'] / img_area
            confidence = detection['confidence']
            
            # Choose color based on confidence (high = green, low = red)
            if confidence > 0.7:
                color = (0, 255, 0)  # Green
            elif confidence > 0.4:
                color = (0, 255, 255)  # Yellow
            else:
                color = (0, 0, 255)  # Red
            
            # Draw rotated rectangle if available
            if 'angle' in detection and 'size' in detection and 'center' in detection:
                cx, cy = detection['center']
                sz_w, sz_h = detection['size']
                angle_deg = np.rad2deg(detection['angle'])
                rect = ((cx, cy), (sz_w, sz_h), angle_deg)
                box_pts = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(image, [box_pts], 0, color, 2)
                
                # Draw orientation arrow
                arrow_length = min(sz_w, sz_h) * 0.3
                arrow_end_x = cx + arrow_length * np.cos(detection['angle'])
                arrow_end_y = cy + arrow_length * np.sin(detection['angle'])
                cv2.arrowedLine(image, (int(cx), int(cy)), (int(arrow_end_x), int(arrow_end_y)), color, 2)
            else:
                cv2.rectangle(image, (x, y), (x+w, y+h), color, 2)

            # Add text with confidence and metrics
            text = f"Conf: {confidence:.2f}, Area: {area:.3f}"
            cv2.putText(image, text, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Add statistics
        stats_text = f"Package Detections: {len(detections)}"
        cv2.putText(image, stats_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def main(args=None):
    """Main function for package detector"""
    package_detector = PackageDetector()
    run_detector(package_detector, args)


if __name__ == '__main__':
    main()
