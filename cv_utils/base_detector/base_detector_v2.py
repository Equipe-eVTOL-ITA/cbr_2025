#!/usr/bin/env python3
"""
Simplified Landing Pad Detection Node for CBR 2025

This ROS 2 node detects landing pads by finding regions that contain
both yellow and blue pixels (adjacent within a square landing pad).

The detector uses HSV color segmentation and looks for areas where
yellow and blue masks have overlapping or adjacent regions.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from typing import Dict, List, Tuple


class BaseDetectorV2(Node):
    """ROS 2 Node for simplified landing pad detection using combined color regions"""
    
    def __init__(self):
        super().__init__('base_detector_v2')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Declare and get parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize color segmentation
        self._initialize_color_ranges()
        
        # ROS 2 subscribers and publishers
        self._setup_ros_interface()
        
        self.get_logger().info("Base detector v2 node initialized successfully")
    
    def _declare_parameters(self):
        """Declare all ROS 2 parameters with default values"""
        
        # General parameters
        self.declare_parameter('image_topic', '/vertical_camera/image_raw')
        self.declare_parameter('detection_topic', '/base_detector/detections')
        self.declare_parameter('mask_debug_topic', '/base_detector/mask_debug')
        self.declare_parameter('bbox_debug_topic', '/base_detector/bbox_debug')
        self.declare_parameter('enable_debug_output', True)
        
        # Morphological operations
        self.declare_parameter('yellow_morph_kernel_size', 3)
        self.declare_parameter('yellow_morph_iterations', 2)
        self.declare_parameter('blue_morph_kernel_size', 5)
        self.declare_parameter('blue_morph_iterations', 3)
        
        # HSV color ranges (reuse same HSV parameters as base_detector)
        self.declare_parameter('blue_lower_h', 100)
        self.declare_parameter('blue_lower_s', 80)
        self.declare_parameter('blue_lower_v', 50)
        self.declare_parameter('blue_upper_h', 130)
        self.declare_parameter('blue_upper_s', 255)
        self.declare_parameter('blue_upper_v', 255)
        
        self.declare_parameter('yellow_lower_h', 20)
        self.declare_parameter('yellow_lower_s', 100)
        self.declare_parameter('yellow_lower_v', 100)
        self.declare_parameter('yellow_upper_h', 30)
        self.declare_parameter('yellow_upper_s', 255)
        self.declare_parameter('yellow_upper_v', 255)
        
        # Detection constraints
        self.declare_parameter('combined_min_area', 1000)
        self.declare_parameter('combined_max_area', 50000)
        self.declare_parameter('combined_aspect_ratio_min', 0.5)
        self.declare_parameter('combined_aspect_ratio_max', 2.0)
        
        # Dilation for combining adjacent regions
        self.declare_parameter('combination_kernel_size', 15)
        self.declare_parameter('combination_iterations', 2)
        
    def _load_parameters(self):
        """Load parameters from ROS 2 parameter server"""
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)
        self.mask_debug_topic = str(self.get_parameter('mask_debug_topic').value)
        self.bbox_debug_topic = str(self.get_parameter('bbox_debug_topic').value)
        self.enable_debug_output = bool(self.get_parameter('enable_debug_output').value)
        
        # Morphology parameters
        self.yellow_morph_kernel_size = int(self.get_parameter('yellow_morph_kernel_size').value)
        self.yellow_morph_iterations = int(self.get_parameter('yellow_morph_iterations').value)
        self.blue_morph_kernel_size = int(self.get_parameter('blue_morph_kernel_size').value)
        self.blue_morph_iterations = int(self.get_parameter('blue_morph_iterations').value)
        
        # Detection constraints
        self.combined_min_area = int(self.get_parameter('combined_min_area').value)
        self.combined_max_area = int(self.get_parameter('combined_max_area').value)
        self.combined_aspect_ratio_min = float(self.get_parameter('combined_aspect_ratio_min').value)
        self.combined_aspect_ratio_max = float(self.get_parameter('combined_aspect_ratio_max').value)
        
        # Combination parameters
        self.combination_kernel_size = int(self.get_parameter('combination_kernel_size').value)
        self.combination_iterations = int(self.get_parameter('combination_iterations').value)
        
    def _initialize_color_ranges(self):
        """Initialize HSV color ranges from parameters"""
        self.color_ranges = {
            'blue': {
                'lower': np.array([
                    self.get_parameter('blue_lower_h').value,
                    self.get_parameter('blue_lower_s').value,
                    self.get_parameter('blue_lower_v').value
                ]),
                'upper': np.array([
                    self.get_parameter('blue_upper_h').value,
                    self.get_parameter('blue_upper_s').value,
                    self.get_parameter('blue_upper_v').value
                ])
            },
            'yellow': {
                'lower': np.array([
                    self.get_parameter('yellow_lower_h').value,
                    self.get_parameter('yellow_lower_s').value,
                    self.get_parameter('yellow_lower_v').value
                ]),
                'upper': np.array([
                    self.get_parameter('yellow_upper_h').value,
                    self.get_parameter('yellow_upper_s').value,
                    self.get_parameter('yellow_upper_v').value
                ])
            }
        }
    
    def _setup_ros_interface(self):
        """Setup ROS 2 subscribers and publishers"""
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            self.detection_topic,
            10
        )
        
        # Debug image publishers
        if self.enable_debug_output:
            self.mask_debug_pub = self.create_publisher(
                Image,
                self.mask_debug_topic,
                10
            )
            self.bbox_debug_pub = self.create_publisher(
                Image,
                self.bbox_debug_topic,
                10
            )
    
    def image_callback(self, msg: Image):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect landing pads
            detections, mask_debug_image, bbox_debug_image = self.detect_landing_pads(cv_image)
            
            # Publish detections
            detection_msg = self._create_detection_message(detections, msg.header)
            self.detection_pub.publish(detection_msg)
            
            # Publish debug images if enabled
            if self.enable_debug_output:
                if mask_debug_image is not None:
                    mask_msg = self.bridge.cv2_to_imgmsg(mask_debug_image, "bgr8")
                    mask_msg.header = msg.header
                    self.mask_debug_pub.publish(mask_msg)
                
                if bbox_debug_image is not None:
                    bbox_msg = self.bridge.cv2_to_imgmsg(bbox_debug_image, "bgr8")
                    bbox_msg.header = msg.header
                    self.bbox_debug_pub.publish(bbox_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def detect_landing_pads(self, image: np.ndarray) -> Tuple[List[Dict], np.ndarray, np.ndarray]:
        """
        Detect landing pads by finding regions with both yellow and blue pixels
        
        Args:
            image: Input BGR image
            
        Returns:
            Tuple of (detections_list, mask_debug_image, bbox_debug_image)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Segment colors
        blue_mask = self._segment_color(hsv, 'blue')
        yellow_mask = self._segment_color(hsv, 'yellow')
        
        # Create mask debug image
        mask_debug_image = self._create_mask_debug_image(image, blue_mask, yellow_mask)
        
        # Create bbox debug image
        bbox_debug_image = image.copy()
        
        # Find regions containing both colors
        detections = self._find_combined_regions(blue_mask, yellow_mask)
        
        # Draw detections on bbox debug image
        for detection in detections:
            x, y, w, h = detection['bbox']
            area = detection['area']
            aspect_ratio = detection['aspect_ratio']
            
            # Draw bounding box in pink
            cv2.rectangle(bbox_debug_image, (x, y), (x+w, y+h), (255, 0, 255), 2)
            
            # Add text with area and aspect ratio
            text = f"Area: {area:.0f}, AR: {aspect_ratio:.2f}"
            cv2.putText(bbox_debug_image, text, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Add statistics
        stats_text = f"Detections: {len(detections)}"
        cv2.putText(bbox_debug_image, stats_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return detections, mask_debug_image, bbox_debug_image
    
    def _segment_color(self, hsv_image: np.ndarray, color_name: str) -> np.ndarray:
        """Segment a specific color from HSV image with appropriate morphology"""
        if color_name not in self.color_ranges:
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        color_range = self.color_ranges[color_name]
        mask = cv2.inRange(hsv_image, color_range['lower'], color_range['upper'])
        
        # Apply color-specific morphological operations
        if color_name == 'yellow':
            kernel_size = self.yellow_morph_kernel_size
            iterations = self.yellow_morph_iterations
        else:  # blue
            kernel_size = self.blue_morph_kernel_size
            iterations = self.blue_morph_iterations
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return mask
    
    def _create_mask_debug_image(self, image: np.ndarray, blue_mask: np.ndarray, yellow_mask: np.ndarray) -> np.ndarray:
        """Create colored mask visualization"""
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
        
        return mask_debug
    
    def _find_combined_regions(self, blue_mask: np.ndarray, yellow_mask: np.ndarray) -> List[Dict]:
        """Find regions that contain both yellow and blue pixels"""
        detections = []
        
        # Dilate both masks to connect adjacent regions
        combination_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (self.combination_kernel_size, self.combination_kernel_size)
        )
        
        blue_dilated = cv2.dilate(blue_mask, combination_kernel, iterations=self.combination_iterations)
        yellow_dilated = cv2.dilate(yellow_mask, combination_kernel, iterations=self.combination_iterations)
        
        # Find regions where dilated masks overlap (indicating proximity)
        combined_mask = cv2.bitwise_and(blue_dilated, yellow_dilated)
        
        # Also add regions where original masks overlap directly
        direct_overlap = cv2.bitwise_and(blue_mask, yellow_mask)
        combined_mask = cv2.bitwise_or(combined_mask, direct_overlap)
        
        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Check area constraints
            if area < self.combined_min_area or area > self.combined_max_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            
            # Check aspect ratio constraints
            if not (self.combined_aspect_ratio_min <= aspect_ratio <= self.combined_aspect_ratio_max):
                continue
            
            # Check if this region actually contains both colors in original masks
            roi_blue = blue_mask[y:y+h, x:x+w]
            roi_yellow = yellow_mask[y:y+h, x:x+w]
            
            blue_pixels = np.sum(roi_blue > 0)
            yellow_pixels = np.sum(roi_yellow > 0)
            
            # Require minimum presence of both colors
            min_pixels = 50  # Minimum pixels for each color
            if blue_pixels < min_pixels or yellow_pixels < min_pixels:
                continue
            
            detection = {
                'bbox': (x, y, w, h),
                'center': (x + w/2, y + h/2),
                'area': area,
                'aspect_ratio': aspect_ratio,
                'blue_pixels': blue_pixels,
                'yellow_pixels': yellow_pixels,
                'class_id': 'landing_pad'
            }
            detections.append(detection)
        
        return detections
    
    def _create_detection_message(self, detections: List[Dict], header) -> Detection2DArray:
        """Create ROS 2 Detection2DArray message from detections"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for det in detections:
            detection = Detection2D()
            
            # Set bounding box
            x, y, w, h = det['bbox']
            detection.bbox.center.position.x = float(x + w/2)
            detection.bbox.center.position.y = float(y + h/2)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            
            # Set hypothesis (no confidence score for now)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_id']
            hypothesis.hypothesis.score = 1.0  # Fixed score since no confidence calculation
            detection.results = [hypothesis]
            
            detection_array.detections.append(detection)
        
        return detection_array


def main(args=None):
    rclpy.init(args=args)
    
    try:
        base_detector = BaseDetectorV2()
        rclpy.spin(base_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
