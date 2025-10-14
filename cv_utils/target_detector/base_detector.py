#!/usr/bin/env python3
"""
Base Detection Node for CBR 2025 using TargetDetector Framework

This detector specializes in finding landing pads by detecting regions
that contain both yellow and blue pixels (adjacent within a square landing pad).

Inherits from TargetDetector and implements the abstract methods:
- _setup_color_parameters(): Sets up blue and yellow HSV ranges
- detect(): Finds combined blue/yellow regions
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple

from target_detector import TargetDetector, run_detector


class BaseDetector(TargetDetector):
    """Detector for landing pads with blue and yellow colors"""
    
    def __init__(self):
        super().__init__('base_detector')
        
        # Initialize color ranges after parent initialization
        self._initialize_color_ranges()
        
        self.get_logger().info("Base detector initialized successfully")
    
    def _declare_ros_parameters(self):
        """Override to set base_detector specific default topics"""
        # Image subscription and detection publishing
        self.declare_parameter('image_topic', 'vertical_camera/image/compressed')
        self.declare_parameter('detection_topic', '/base_detector/detections')  # Base detector specific topic
    
    def _setup_color_parameters(self):
        """Setup HSV color parameters for blue and yellow detection"""
        # Default HSV ranges for blue
        blue_lower = {'h': 100, 's': 80, 'v': 50}
        blue_upper = {'h': 130, 's': 255, 'v': 255}
        
        # Default HSV ranges for yellow  
        yellow_lower = {'h': 10, 's': 100, 'v': 100}
        yellow_upper = {'h': 40, 's': 255, 'v': 255}
        
        # Setup parameters for both colors
        self.blue_lower, self.blue_upper, self.blue_kernel_size, self.blue_iterations = \
            self.setup_color_parameters('blue', blue_lower, blue_upper, kernel_size=5, iterations=3)
            
        self.yellow_lower, self.yellow_upper, self.yellow_kernel_size, self.yellow_iterations = \
            self.setup_color_parameters('yellow', yellow_lower, yellow_upper, kernel_size=3, iterations=2)
        
        # Additional parameters for combined region detection
        self.declare_parameter('combined_min_area', 0.001)
        self.declare_parameter('combined_max_area', 1.0)
        self.declare_parameter('combined_aspect_ratio_min', 0.5)
        self.declare_parameter('combined_aspect_ratio_max', 2.0)
        self.declare_parameter('combination_kernel_size', 15)
        self.declare_parameter('combination_iterations', 2)
        
        # Non-maximum suppression for concentric detections
        self.declare_parameter('nms_center_distance_threshold', 0.1)  # Normalized distance threshold
    
    def _initialize_color_ranges(self):
        """Initialize color detection parameters from ROS parameters"""
        self.combined_min_area = float(self.get_parameter('combined_min_area').value)
        self.combined_max_area = float(self.get_parameter('combined_max_area').value)
        self.combined_aspect_ratio_min = float(self.get_parameter('combined_aspect_ratio_min').value)
        self.combined_aspect_ratio_max = float(self.get_parameter('combined_aspect_ratio_max').value)
        self.combination_kernel_size = int(self.get_parameter('combination_kernel_size').value)
        self.combination_iterations = int(self.get_parameter('combination_iterations').value)
        self.nms_center_distance_threshold = float(self.get_parameter('nms_center_distance_threshold').value)
    
    def detect(self, image: np.ndarray) -> Tuple[List[Dict], Dict]:
        """
        Detect landing pads by finding regions with both yellow and blue pixels
        
        Args:
            image: Input BGR image
            
        Returns:
            Tuple of (detections_list, debug_images_dict)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Segment colors
        blue_mask = self._segment_color(hsv, 'blue')
        yellow_mask = self._segment_color(hsv, 'yellow')
        
        # Create debug images
        debug_images = self._create_debug_images(image, blue_mask, yellow_mask)
        
        # Find regions containing both colors
        detections = self._find_combined_regions(blue_mask, yellow_mask)
        
        # Filter concentric detections (keep largest area)
        detections = self._filter_concentric_detections(detections, image.shape)
        
        # Draw detections on bbox debug image
        if 'bbox_debug' in debug_images:
            self._draw_detections_on_image(debug_images['bbox_debug'], detections)
        
        return detections, debug_images
    
    def _segment_color(self, hsv_image: np.ndarray, color_name: str) -> np.ndarray:
        """Segment a specific color from HSV image with appropriate morphology"""
        if color_name == 'blue':
            lower, upper = self.blue_lower, self.blue_upper
            kernel_size, iterations = self.blue_kernel_size, self.blue_iterations
        elif color_name == 'yellow':
            lower, upper = self.yellow_lower, self.yellow_upper
            kernel_size, iterations = self.yellow_kernel_size, self.yellow_iterations
        else:
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        # Apply HSV color filtering
        mask = cv2.inRange(hsv_image, lower, upper)
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return mask
    
    def _create_debug_images(self, image: np.ndarray, blue_mask: np.ndarray, yellow_mask: np.ndarray) -> Dict:
        """Create debug images for visualization"""
        debug_images = {}
        
        # Create mask debug image
        mask_debug = image.copy()
        
        # Create colored overlays
        blue_overlay = np.zeros_like(image)
        yellow_overlay = np.zeros_like(image)
        
        blue_overlay[blue_mask > 0] = (255, 0, 0)  # Blue pixels in blue
        yellow_overlay[yellow_mask > 0] = (0, 255, 255)  # Yellow pixels in yellow
        
        # Apply overlays with transparency
        alpha = 0.4
        mask_debug = cv2.addWeighted(mask_debug, 1 - alpha, blue_overlay, alpha, 0)
        mask_debug = cv2.addWeighted(mask_debug, 1 - alpha, yellow_overlay, alpha, 0)
        
        debug_images['mask_debug'] = mask_debug
        debug_images['bbox_debug'] = image.copy()
        
        return debug_images
    
    def _find_combined_regions(self, blue_mask: np.ndarray, yellow_mask: np.ndarray) -> List[Dict]:
        """Find regions that contain both yellow and blue pixels"""
        detections = []
        
        # Get image dimensions for normalization
        img_height, img_width = blue_mask.shape[:2]
        img_area = img_height * img_width
        
        # Convert normalized area thresholds to actual pixel counts
        min_area_pixels = self.combined_min_area * img_area
        max_area_pixels = self.combined_max_area * img_area
        
        # Dilate both masks to connect adjacent regions
        combination_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (self.combination_kernel_size, self.combination_kernel_size)
        )
        
        blue_dilated = cv2.dilate(blue_mask, combination_kernel, iterations=self.combination_iterations)
        yellow_dilated = cv2.dilate(yellow_mask, combination_kernel, iterations=self.combination_iterations)
        
        # Find regions where dilated masks overlap
        combined_mask = cv2.bitwise_and(blue_dilated, yellow_dilated)
        
        # Also add regions where original masks overlap directly
        direct_overlap = cv2.bitwise_and(blue_mask, yellow_mask)
        combined_mask = cv2.bitwise_or(combined_mask, direct_overlap)
        
        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < min_area_pixels or area > max_area_pixels:
                continue
            
            # Get axis-aligned bounding rect for ROI checks
            x, y, w, h = cv2.boundingRect(contour)
            
            # Use minAreaRect to obtain rotated box (center, size, angle)
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rect_w, rect_h), angle_deg = rect
            
            # Ignore degenerate sizes
            if rect_w <= 0 or rect_h <= 0:
                continue
            
            # Compute aspect ratio from rotated box
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h) if min(rect_w, rect_h) > 0 else 0
            if not (self.combined_aspect_ratio_min <= aspect_ratio <= self.combined_aspect_ratio_max):
                continue
            
            # Check if this region contains both colors in original masks
            roi_blue = blue_mask[y:y+h, x:x+w]
            roi_yellow = yellow_mask[y:y+h, x:x+w]
            
            blue_pixels = np.sum(roi_blue > 0)
            yellow_pixels = np.sum(roi_yellow > 0)
            
            min_pixels = 100  # Minimum pixels for each color
            if blue_pixels < min_pixels or yellow_pixels < min_pixels:
                continue
            
            detection = {
                'bbox': (x, y, w, h),                         # axis-aligned for quick drawing/roi
                'center': (float(cx), float(cy)),             # rotated center in pixels
                'size': (float(rect_w), float(rect_h)),       # rotated width/height in pixels
                'angle': float(np.deg2rad(angle_deg)),        # angle in radians
                'area': area,
                'aspect_ratio': aspect_ratio,
                'blue_pixels': int(blue_pixels),
                'yellow_pixels': int(yellow_pixels),
                'class_id': 'landing_pad',
                'confidence': 1.0
            }
            detections.append(detection)
        
        return detections
    
    def _filter_concentric_detections(self, detections: List[Dict], image_shape: Tuple[int, int, int]) -> List[Dict]:
        """
        Filter out concentric detections, keeping only the one with largest area
        
        Args:
            detections: List of detection dictionaries
            image_shape: Shape of the image (height, width, channels)
            
        Returns:
            Filtered list of detections without concentric duplicates
        """
        if len(detections) <= 1:
            return detections
        
        img_height, img_width = image_shape[:2]
        img_diagonal = np.sqrt(img_height**2 + img_width**2)
        
        # Convert threshold from normalized to pixels
        distance_threshold_pixels = self.nms_center_distance_threshold * img_diagonal
        
        # Sort by area (largest first) to prioritize larger detections
        sorted_detections = sorted(detections, key=lambda x: x['area'], reverse=True)
        
        # List to store filtered detections
        filtered_detections = []
        
        for current_det in sorted_detections:
            cx_current, cy_current = current_det['center']
            is_concentric = False
            
            # Check if current detection is concentric with any already accepted detection
            for accepted_det in filtered_detections:
                cx_accepted, cy_accepted = accepted_det['center']
                
                # Calculate Euclidean distance between centers
                distance = np.sqrt((cx_current - cx_accepted)**2 + (cy_current - cy_accepted)**2)
                
                if distance < distance_threshold_pixels:
                    # Current detection is concentric with an already accepted (larger) detection
                    is_concentric = True
                    self.get_logger().debug(
                        f"Filtered concentric detection: area={current_det['area']:.0f} "
                        f"(kept larger with area={accepted_det['area']:.0f}, distance={distance:.1f}px)"
                    )
                    break
            
            # Only add if not concentric with any larger detection
            if not is_concentric:
                filtered_detections.append(current_det)
        
        # Log filtering results
        num_filtered = len(detections) - len(filtered_detections)
        if num_filtered > 0:
            self.get_logger().info(
                f"Filtered {num_filtered} concentric detection(s). "
                f"Kept {len(filtered_detections)}/{len(detections)} detections."
            )
        
        return filtered_detections
    
    def _draw_detections_on_image(self, image: np.ndarray, detections: List[Dict]):
        """Draw detection bounding boxes and info on image"""
        img_height, img_width = image.shape[:2]
        img_area = img_height * img_width
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            area = detection['area'] / img_area
            aspect_ratio = detection['aspect_ratio']
            
            # Draw rotated rectangle if available
            if 'angle' in detection and 'size' in detection and 'center' in detection:
                cx, cy = detection['center']
                sz_w, sz_h = detection['size']
                angle_deg = np.rad2deg(detection['angle'])
                rect = ((cx, cy), (sz_w, sz_h), angle_deg)
                box_pts = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(image, [box_pts], 0, (255, 0, 255), 2)
            else:
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 255), 2)

            # Add text with area and aspect ratio
            text = f"Area: {area:.3f}, AR: {aspect_ratio:.2f}"
            cv2.putText(image, text, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # Add statistics
        stats_text = f"Detections: {len(detections)}"
        cv2.putText(image, stats_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


def main(args=None):
    """Main function for base detector"""
    import rclpy
    rclpy.init(args=args)
    
    try:
        base_detector = BaseDetector()
        rclpy.spin(base_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
