#!/usr/bin/env python3
"""
Landing Pad Detection Node for CBR 2025

This ROS 2 node detects landing pads which consist of:
1. Yellow cross (0.5m arms) - Primary detection
2. Yellow circle (0.8m diameter, 0.05m line width) - Secondary
3. Blue circle inside yellow circle (0.7m diameter) - Secondary  
4. Yellow square (1.0m side, 0.05m line width) - Secondary

The detector uses cross-based detection with geometric validation.
Final detection combines all shapes with weighted contributions.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from typing import Dict, List, Tuple, Optional


class BaseDetector(Node):
    """ROS 2 Node for landing pad detection using multi-shape detection"""
    
    def __init__(self):
        super().__init__('base_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Declare and get parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize color segmentation
        self._initialize_color_ranges()
        
        # ROS 2 subscribers and publishers
        self._setup_ros_interface()
        
        self.get_logger().info("Base detector node initialized successfully")
    
    def _declare_parameters(self):
        """Declare all ROS 2 parameters with default values"""
        
        # General parameters
        self.declare_parameter('image_topic', '/vertical_camera/image_raw')
        self.declare_parameter('detection_topic', '/base_detector/detections')
        self.declare_parameter('mask_debug_topic', '/base_detector/mask_debug')
        self.declare_parameter('bbox_debug_topic', '/base_detector/bbox_debug')
        self.declare_parameter('enable_debug_output', True)
        
        # Detection constraints
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Morphological operations
        self.declare_parameter('yellow_morph_kernel_size', 3)
        self.declare_parameter('yellow_morph_iterations', 2)
        self.declare_parameter('blue_morph_kernel_size', 5)
        self.declare_parameter('blue_morph_iterations', 3)
        
        # HSV color ranges
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
        
        # Cross detection parameters (0.5m arms - square cross)
        self.declare_parameter('cross_min_area', 200)
        self.declare_parameter('cross_max_area', 5000)
        self.declare_parameter('cross_aspect_ratio_min', 0.8)  # Cross is square
        self.declare_parameter('cross_aspect_ratio_max', 1.3)
        self.declare_parameter('cross_line_thickness_ratio', 0.1)  # Line thickness to total size ratio
        self.declare_parameter('cross_solidity_max', 0.8)  # Cross should not be too solid (has concave parts)
        
        # Yellow circle detection parameters (0.8m diameter, 0.05m width)
        self.declare_parameter('yellow_circle_min_area', 300)
        self.declare_parameter('yellow_circle_max_area', 8000)
        self.declare_parameter('yellow_circle_circularity_min', 0.7)
        self.declare_parameter('yellow_circle_aspect_ratio_max', 1.3)
        
        # Blue circle detection parameters (0.7m diameter)
        self.declare_parameter('blue_circle_min_area', 500)
        self.declare_parameter('blue_circle_max_area', 7000)
        self.declare_parameter('blue_circle_circularity_min', 0.5)  # Lower due to cross inside
        self.declare_parameter('blue_circle_aspect_ratio_max', 1.4)
        
        # Yellow square detection parameters (1.0m side, 0.05m width)
        self.declare_parameter('yellow_square_min_area', 400)
        self.declare_parameter('yellow_square_max_area', 10000)
        self.declare_parameter('yellow_square_aspect_ratio_min', 0.8)
        self.declare_parameter('yellow_square_aspect_ratio_max', 1.25)
        self.declare_parameter('yellow_square_rectangularity_min', 0.7)
        
        # Geometric relationship parameters
        self.declare_parameter('max_center_distance_ratio', 0.3)  # Max distance between centers as ratio of size
        
        # Weight parameters for final bounding box calculation
        self.declare_parameter('cross_weight', 0.6)  # Primary detection
        self.declare_parameter('yellow_circle_weight', 0.15)
        self.declare_parameter('blue_circle_weight', 0.15)
        self.declare_parameter('yellow_square_weight', 0.1)
        
    def _load_parameters(self):
        """Load parameters from ROS 2 parameter server"""
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)
        self.mask_debug_topic = str(self.get_parameter('mask_debug_topic').value)
        self.bbox_debug_topic = str(self.get_parameter('bbox_debug_topic').value)
        self.enable_debug_output = bool(self.get_parameter('enable_debug_output').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        
        # Morphology parameters
        self.yellow_morph_kernel_size = int(self.get_parameter('yellow_morph_kernel_size').value)
        self.yellow_morph_iterations = int(self.get_parameter('yellow_morph_iterations').value)
        self.blue_morph_kernel_size = int(self.get_parameter('blue_morph_kernel_size').value)
        self.blue_morph_iterations = int(self.get_parameter('blue_morph_iterations').value)
        
        # Cross parameters
        self.cross_min_area = int(self.get_parameter('cross_min_area').value)
        self.cross_max_area = int(self.get_parameter('cross_max_area').value)
        self.cross_aspect_ratio_min = float(self.get_parameter('cross_aspect_ratio_min').value)
        self.cross_aspect_ratio_max = float(self.get_parameter('cross_aspect_ratio_max').value)
        self.cross_line_thickness_ratio = float(self.get_parameter('cross_line_thickness_ratio').value)
        self.cross_solidity_max = float(self.get_parameter('cross_solidity_max').value)
        
        # Yellow circle parameters
        self.yellow_circle_min_area = int(self.get_parameter('yellow_circle_min_area').value)
        self.yellow_circle_max_area = int(self.get_parameter('yellow_circle_max_area').value)
        self.yellow_circle_circularity_min = float(self.get_parameter('yellow_circle_circularity_min').value)
        self.yellow_circle_aspect_ratio_max = float(self.get_parameter('yellow_circle_aspect_ratio_max').value)
        
        # Blue circle parameters
        self.blue_circle_min_area = int(self.get_parameter('blue_circle_min_area').value)
        self.blue_circle_max_area = int(self.get_parameter('blue_circle_max_area').value)
        self.blue_circle_circularity_min = float(self.get_parameter('blue_circle_circularity_min').value)
        self.blue_circle_aspect_ratio_max = float(self.get_parameter('blue_circle_aspect_ratio_max').value)
        
        # Yellow square parameters
        self.yellow_square_min_area = int(self.get_parameter('yellow_square_min_area').value)
        self.yellow_square_max_area = int(self.get_parameter('yellow_square_max_area').value)
        self.yellow_square_aspect_ratio_min = float(self.get_parameter('yellow_square_aspect_ratio_min').value)
        self.yellow_square_aspect_ratio_max = float(self.get_parameter('yellow_square_aspect_ratio_max').value)
        self.yellow_square_rectangularity_min = float(self.get_parameter('yellow_square_rectangularity_min').value)
        
        # Geometric and weight parameters
        self.max_center_distance_ratio = float(self.get_parameter('max_center_distance_ratio').value)
        self.cross_weight = float(self.get_parameter('cross_weight').value)
        self.yellow_circle_weight = float(self.get_parameter('yellow_circle_weight').value)
        self.blue_circle_weight = float(self.get_parameter('blue_circle_weight').value)
        self.yellow_square_weight = float(self.get_parameter('yellow_square_weight').value)
        
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
        Detect landing pads using multi-shape detection
        
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
        
        # Detect individual shapes
        cross_detections = self._detect_crosses(yellow_mask)
        yellow_circle_detections = self._detect_yellow_circles(yellow_mask)
        blue_circle_detections = self._detect_blue_circles(blue_mask)
        yellow_square_detections = self._detect_yellow_squares(yellow_mask)
        
        # Draw all detections on bbox debug image
        self._draw_shape_detections(bbox_debug_image, cross_detections, (0, 255, 255), "Cross")  # Yellow
        self._draw_shape_detections(bbox_debug_image, yellow_circle_detections, (0, 0, 255), "Y-Circle")  # Red
        self._draw_shape_detections(bbox_debug_image, blue_circle_detections, (255, 0, 0), "B-Circle")  # Blue
        self._draw_shape_detections(bbox_debug_image, yellow_square_detections, (0, 255, 0), "Y-Square")  # Green
        
        # Combine detections to form final landing pad detections
        final_detections = self._combine_shape_detections(
            cross_detections, yellow_circle_detections, 
            blue_circle_detections, yellow_square_detections
        )
        
        # Draw final detections on bbox debug image
        for detection in final_detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            # Draw final bounding box in pink with thicker line
            cv2.rectangle(bbox_debug_image, (x, y), (x+w, y+h), (255, 0, 255), 3)  # Pink, thick line
            cv2.putText(bbox_debug_image, f'FINAL ({confidence:.2f})', 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        
        # Add statistics
        stats_text = f"Cross: {len(cross_detections)}, Y-Circle: {len(yellow_circle_detections)}, "
        stats_text += f"B-Circle: {len(blue_circle_detections)}, Y-Square: {len(yellow_square_detections)}, Final: {len(final_detections)}"
        cv2.putText(bbox_debug_image, stats_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return final_detections, mask_debug_image, bbox_debug_image
    
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
    
    def _detect_crosses(self, yellow_mask: np.ndarray) -> List[Dict]:
        """Detect yellow cross patterns (0.5m arms, square cross)"""
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.cross_min_area or area > self.cross_max_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            
            # Check if roughly square (cross should be square overall)
            if not (self.cross_aspect_ratio_min <= aspect_ratio <= self.cross_aspect_ratio_max):
                continue
            
            # Advanced cross detection using multiple methods
            confidence = self._calculate_cross_confidence_advanced(contour, yellow_mask[y:y+h, x:x+w])
            
            if confidence > 0.3:  # Threshold for valid cross
                detection = {
                    'bbox': (x, y, w, h),
                    'center': (x + w/2, y + h/2),
                    'confidence': confidence,
                    'area': area,
                    'type': 'cross'
                }
                detections.append(detection)
        
        return detections
    
    def _detect_yellow_circles(self, yellow_mask: np.ndarray) -> List[Dict]:
        """Detect yellow circle patterns (0.8m diameter, 0.05m line width)"""
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.yellow_circle_min_area or area > self.yellow_circle_max_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            
            # Check aspect ratio (should be close to square)
            if aspect_ratio > self.yellow_circle_aspect_ratio_max:
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < self.yellow_circle_circularity_min:
                    continue
            else:
                continue
            
            confidence = min(1.0, circularity)
            
            detection = {
                'bbox': (x, y, w, h),
                'center': (x + w/2, y + h/2),
                'confidence': confidence,
                'area': area,
                'type': 'yellow_circle'
            }
            detections.append(detection)
        
        return detections
    
    def _detect_blue_circles(self, blue_mask: np.ndarray) -> List[Dict]:
        """Detect blue circle patterns (0.7m diameter, has cross inside)"""
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.blue_circle_min_area or area > self.blue_circle_max_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            
            # Check aspect ratio (should be close to square)
            if aspect_ratio > self.blue_circle_aspect_ratio_max:
                continue
            
            # Check circularity (lower threshold due to cross inside)
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < self.blue_circle_circularity_min:
                    continue
            else:
                continue
            
            confidence = min(1.0, circularity * 0.8)  # Lower confidence due to cross interference
            
            detection = {
                'bbox': (x, y, w, h),
                'center': (x + w/2, y + h/2),
                'confidence': confidence,
                'area': area,
                'type': 'blue_circle'
            }
            detections.append(detection)
        
        return detections
    
    def _detect_yellow_squares(self, yellow_mask: np.ndarray) -> List[Dict]:
        """Detect yellow square patterns (1.0m side, 0.05m line width) - rotation invariant"""
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.yellow_square_min_area or area > self.yellow_square_max_area:
                continue
            
            # Use minimum area rectangle for rotation invariance
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype=np.int32)
            
            # Get width and height of the minimum area rectangle
            rect_width, rect_height = rect[1]
            aspect_ratio = max(rect_width, rect_height) / min(rect_width, rect_height) if min(rect_width, rect_height) > 0 else 0
            
            # Check aspect ratio (should be close to square)
            if not (self.yellow_square_aspect_ratio_min <= aspect_ratio <= self.yellow_square_aspect_ratio_max):
                continue
            
            # Check rectangularity using polygon approximation
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Should have approximately 4 corners for a square
            if len(approx) < 4:
                continue
            
            # Calculate confidence based on shape properties
            confidence = self._calculate_square_confidence(contour, rect, approx)
            
            # Get axis-aligned bounding box for final detection
            x, y, w, h = cv2.boundingRect(contour)
            
            detection = {
                'bbox': (x, y, w, h),
                'center': (x + w/2, y + h/2),
                'confidence': confidence,
                'area': area,
                'type': 'yellow_square',
                'rotation_angle': rect[2]  # Store rotation angle for debugging
            }
            detections.append(detection)
        
        return detections
    
    def _calculate_square_confidence(self, contour: np.ndarray, min_area_rect: tuple, approx_poly: np.ndarray) -> float:
        """Calculate confidence for square detection"""
        confidence = 0.0
        
        # 1. Rectangularity test (how well does it fill its bounding box)
        x, y, w, h = cv2.boundingRect(contour)
        bbox_area = w * h
        contour_area = cv2.contourArea(contour)
        rectangularity = contour_area / bbox_area if bbox_area > 0 else 0
        
        if rectangularity >= self.yellow_square_rectangularity_min:
            confidence += 0.3
        else:
            confidence += rectangularity * 0.3 / self.yellow_square_rectangularity_min
        
        # 2. Aspect ratio test (square-ness of minimum area rectangle)
        rect_width, rect_height = min_area_rect[1]
        aspect_ratio = max(rect_width, rect_height) / min(rect_width, rect_height) if min(rect_width, rect_height) > 0 else 0
        
        if 0.9 <= aspect_ratio <= 1.1:
            confidence += 0.3  # Perfect square
        elif 0.8 <= aspect_ratio <= 1.25:
            confidence += 0.2  # Good square
        else:
            confidence += 0.1  # Acceptable square
        
        # 3. Corner count test (should have 4 corners)
        corner_count = len(approx_poly)
        if corner_count == 4:
            confidence += 0.2
        elif corner_count == 5 or corner_count == 3:
            confidence += 0.1
        
        # 4. Minimum area rectangle fit test
        rect_area = rect_width * rect_height
        fit_ratio = contour_area / rect_area if rect_area > 0 else 0
        
        if fit_ratio > 0.8:
            confidence += 0.2
        elif fit_ratio > 0.6:
            confidence += 0.1
        
        return min(1.0, confidence)
        
        return detections
    
    def _calculate_cross_confidence_advanced(self, contour: np.ndarray, region_mask: np.ndarray) -> float:
        """Advanced cross detection using multiple geometric tests"""
        confidence = 0.0
        
        # 1. Solidity test (crosses have concave parts, should not be too solid)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = cv2.contourArea(contour) / hull_area
            if solidity < self.cross_solidity_max:
                confidence += 0.3  # Good, has concave parts like a cross
            else:
                confidence += 0.1  # Too solid, might not be a cross
        
        # 2. Template matching approach
        h, w = region_mask.shape
        if h > 10 and w > 10:
            # Create cross template
            cross_template = self._create_cross_template(w, h)
            
            # Normalize both images
            region_norm = region_mask.astype(np.float32)
            template_norm = cross_template.astype(np.float32)
            region_norm = cv2.normalize(region_norm, region_norm, 0, 1, cv2.NORM_MINMAX)
            template_norm = cv2.normalize(template_norm, template_norm, 0, 1, cv2.NORM_MINMAX)
            
            # Calculate correlation
            correlation = cv2.matchTemplate(region_norm, template_norm, cv2.TM_CCORR_NORMED)[0, 0]
            confidence += min(0.4, correlation * 0.4)
        
        # 3. Check for cross-like pixel distribution
        if h > 5 and w > 5:
            center_y, center_x = h // 2, w // 2
            
            # Check horizontal line
            horizontal_line = region_mask[center_y, :]
            h_coverage = np.sum(horizontal_line > 0) / w if w > 0 else 0
            
            # Check vertical line  
            vertical_line = region_mask[:, center_x]
            v_coverage = np.sum(vertical_line > 0) / h if h > 0 else 0
            
            # Both lines should have good coverage
            if h_coverage > 0.7 and v_coverage > 0.7:
                confidence += 0.2
            elif h_coverage > 0.5 and v_coverage > 0.5:
                confidence += 0.1
        
        # 4. Aspect ratio bonus (square is good for cross)
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
        if 0.9 <= aspect_ratio <= 1.1:
            confidence += 0.1  # Perfect square
        elif 0.8 <= aspect_ratio <= 1.3:
            confidence += 0.05  # Close to square
        
        return min(1.0, confidence)
    
    def _create_cross_template(self, width: int, height: int) -> np.ndarray:
        """Create a cross template for matching"""
        template = np.zeros((height, width), dtype=np.uint8)
        
        # Calculate line thickness (should be proportional to size)
        line_thickness = max(1, int(min(width, height) * self.cross_line_thickness_ratio))
        
        # Draw horizontal line
        center_y = height // 2
        cv2.line(template, (0, center_y), (width-1, center_y), 255, line_thickness)
        
        # Draw vertical line
        center_x = width // 2
        cv2.line(template, (center_x, 0), (center_x, height-1), 255, line_thickness)
        
        return template
    
    def _combine_shape_detections(self, cross_detections: List[Dict], 
                                yellow_circle_detections: List[Dict],
                                blue_circle_detections: List[Dict], 
                                yellow_square_detections: List[Dict]) -> List[Dict]:
        """Combine shape detections to form final landing pad detections"""
        final_detections = []
        
        # Cross detection is mandatory
        for cross in cross_detections:
            cross_center = cross['center']
            cross_size = max(cross['bbox'][2], cross['bbox'][3])  # Use larger dimension
            
            # Find compatible shapes near this cross
            compatible_yellow_circle = self._find_compatible_shape(cross_center, cross_size, yellow_circle_detections)
            compatible_blue_circle = self._find_compatible_shape(cross_center, cross_size, blue_circle_detections)
            compatible_yellow_square = self._find_compatible_shape(cross_center, cross_size, yellow_square_detections)
            
            # Calculate weighted final bounding box
            final_bbox = self._calculate_weighted_bbox(
                cross, compatible_yellow_circle, compatible_blue_circle, compatible_yellow_square
            )
            
            # Calculate combined confidence
            final_confidence = self._calculate_combined_confidence(
                cross, compatible_yellow_circle, compatible_blue_circle, compatible_yellow_square
            )
            
            if final_confidence >= self.confidence_threshold:
                detection = {
                    'bbox': final_bbox,
                    'center': (final_bbox[0] + final_bbox[2]/2, final_bbox[1] + final_bbox[3]/2),
                    'confidence': final_confidence,
                    'class_id': 'landing_pad',
                    'components': {
                        'cross': cross,
                        'yellow_circle': compatible_yellow_circle,
                        'blue_circle': compatible_blue_circle,
                        'yellow_square': compatible_yellow_square
                    }
                }
                final_detections.append(detection)
        
        return final_detections
    
    def _find_compatible_shape(self, cross_center: Tuple[float, float], cross_size: float, 
                             shape_detections: List[Dict]) -> Optional[Dict]:
        """Find a shape detection compatible with the given cross (accounting for size differences)"""
        cross_x, cross_y = cross_center
        
        # Expected size ratios relative to cross (0.5m)
        size_ratios = {
            'cross': 0.5,
            'blue_circle': 0.7,
            'yellow_circle': 0.8,
            'yellow_square': 1.0
        }
        
        best_shape = None
        min_distance = float('inf')
        
        for shape in shape_detections:
            shape_x, shape_y = shape['center']
            shape_size = max(shape['bbox'][2], shape['bbox'][3])  # Use larger dimension
            
            # Calculate expected size ratio for this shape type
            shape_type_key = shape['type']
            if shape_type_key == 'yellow_circle':
                expected_ratio = size_ratios['yellow_circle'] / size_ratios['cross']
            elif shape_type_key == 'blue_circle':
                expected_ratio = size_ratios['blue_circle'] / size_ratios['cross']
            elif shape_type_key == 'yellow_square':
                expected_ratio = size_ratios['yellow_square'] / size_ratios['cross']
            else:
                expected_ratio = 1.0  # Default ratio
            
            expected_size = cross_size * expected_ratio
            
            # Size compatibility check (allow some tolerance)
            size_ratio = shape_size / expected_size if expected_size > 0 else 0
            if not (0.7 <= size_ratio <= 1.4):  # Allow 30% tolerance in size
                continue
            
            # Distance check with adaptive threshold based on shape size
            max_distance = max(cross_size, shape_size) * self.max_center_distance_ratio
            distance = np.sqrt((cross_x - shape_x)**2 + (cross_y - shape_y)**2)
            
            if distance <= max_distance and distance < min_distance:
                min_distance = distance
                best_shape = shape
        
        return best_shape
    
    def _calculate_weighted_bbox(self, cross: Dict, yellow_circle: Optional[Dict], 
                               blue_circle: Optional[Dict], yellow_square: Optional[Dict]) -> Tuple[int, int, int, int]:
        """Calculate final bounding box using weighted combination with size ratio scaling"""
        
        # Expected size ratios (relative to cross size of 0.5m)
        # yellow_cross : blue_circle : yellow_circle : yellow_square = 0.5 : 0.7 : 0.8 : 1.0
        size_ratios = {
            'cross': 0.5,
            'blue_circle': 0.7,
            'yellow_circle': 0.8,
            'yellow_square': 1.0
        }
        
        # Normalize all bounding boxes to the same scale (cross scale)
        cross_bbox = self._normalize_bbox_to_cross_scale(cross['bbox'], 'cross', size_ratios)
        
        total_weight = self.cross_weight
        weighted_x = cross_bbox[0] * self.cross_weight
        weighted_y = cross_bbox[1] * self.cross_weight
        weighted_w = cross_bbox[2] * self.cross_weight
        weighted_h = cross_bbox[3] * self.cross_weight
        
        if yellow_circle is not None:
            normalized_bbox = self._normalize_bbox_to_cross_scale(yellow_circle['bbox'], 'yellow_circle', size_ratios)
            total_weight += self.yellow_circle_weight
            weighted_x += normalized_bbox[0] * self.yellow_circle_weight
            weighted_y += normalized_bbox[1] * self.yellow_circle_weight
            weighted_w += normalized_bbox[2] * self.yellow_circle_weight
            weighted_h += normalized_bbox[3] * self.yellow_circle_weight
        
        if blue_circle is not None:
            normalized_bbox = self._normalize_bbox_to_cross_scale(blue_circle['bbox'], 'blue_circle', size_ratios)
            total_weight += self.blue_circle_weight
            weighted_x += normalized_bbox[0] * self.blue_circle_weight
            weighted_y += normalized_bbox[1] * self.blue_circle_weight
            weighted_w += normalized_bbox[2] * self.blue_circle_weight
            weighted_h += normalized_bbox[3] * self.blue_circle_weight
        
        if yellow_square is not None:
            normalized_bbox = self._normalize_bbox_to_cross_scale(yellow_square['bbox'], 'yellow_square', size_ratios)
            total_weight += self.yellow_square_weight
            weighted_x += normalized_bbox[0] * self.yellow_square_weight
            weighted_y += normalized_bbox[1] * self.yellow_square_weight
            weighted_w += normalized_bbox[2] * self.yellow_square_weight
            weighted_h += normalized_bbox[3] * self.yellow_square_weight
        
        final_x = int(weighted_x / total_weight)
        final_y = int(weighted_y / total_weight)
        final_w = int(weighted_w / total_weight)
        final_h = int(weighted_h / total_weight)
        
        return (final_x, final_y, final_w, final_h)
    
    def _normalize_bbox_to_cross_scale(self, bbox: Tuple[int, int, int, int], 
                                     shape_type: str, size_ratios: Dict[str, float]) -> Tuple[float, float, float, float]:
        """Normalize bounding box to cross scale based on expected size ratios"""
        x, y, w, h = bbox
        center_x = x + w/2
        center_y = y + h/2
        
        # Get the scaling factor to normalize to cross size
        current_ratio = size_ratios[shape_type]
        cross_ratio = size_ratios['cross']
        scale_factor = cross_ratio / current_ratio
        
        # Scale the size while keeping the center
        new_w = w * scale_factor
        new_h = h * scale_factor
        new_x = center_x - new_w/2
        new_y = center_y - new_h/2
        
        return (new_x, new_y, new_w, new_h)
    
    def _calculate_combined_confidence(self, cross: Dict, yellow_circle: Optional[Dict], 
                                     blue_circle: Optional[Dict], yellow_square: Optional[Dict]) -> float:
        """Calculate combined confidence score"""
        base_confidence = cross['confidence'] * self.cross_weight
        
        if yellow_circle is not None:
            base_confidence += yellow_circle['confidence'] * self.yellow_circle_weight
        
        if blue_circle is not None:
            base_confidence += blue_circle['confidence'] * self.blue_circle_weight
        
        if yellow_square is not None:
            base_confidence += yellow_square['confidence'] * self.yellow_square_weight
        
        # Bonus for having multiple components
        component_count = 1  # Cross is always present
        if yellow_circle is not None:
            component_count += 1
        if blue_circle is not None:
            component_count += 1
        if yellow_square is not None:
            component_count += 1
        
        # Small bonus for each additional component
        bonus = (component_count - 1) * 0.05
        
        return min(1.0, base_confidence + bonus)
    
    def _draw_shape_detections(self, image: np.ndarray, detections: List[Dict], 
                             color: Tuple[int, int, int], label: str):
        """Draw shape detections on debug image"""
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            cv2.rectangle(image, (x, y), (x+w, y+h), color, 1)
            cv2.putText(image, f'{label} ({confidence:.2f})', 
                       (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    
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
            
            # Set hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_id']
            hypothesis.hypothesis.score = float(det['confidence'])
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
        
        return detection_array


def main(args=None):
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
