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
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from typing import Dict, List, Tuple


class BaseDetector(Node):
    """ROS 2 Node for simplified landing pad detection using combined color regions"""
    
    def __init__(self):
        super().__init__('base_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Frame counting for debug publishing rate control
        self.frame_count = 0
        self.debug_frame_skip = 3  # Publish debug every 3rd frame (10Hz -> 3.33Hz)
        
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
        
        # Image Subscription and Detection Publisher
        self.declare_parameter('image_topic', '/vertical_camera/image_raw')
        self.declare_parameter('detection_topic', '/base_detector/detections')

        # Full debug mode
        self.declare_parameter('full_debug_mode', False)
        self.declare_parameter('mask_debug_topic', '/base_detector/mask_debug')
        self.declare_parameter('bbox_debug_topic', '/base_detector/bbox_debug')
        
        # Light debug mode
        self.declare_parameter('light_debug_mode', False)
        self.declare_parameter('light_debug_topic', '/telemetry/camera_debug/compressed')
        self.declare_parameter('light_debug_size', 400)  # Size for debug images
        self.declare_parameter('light_debug_quality', 80)  # JPEG compression quality
        
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
        
        self.declare_parameter('combined_min_area', 0.001)
        self.declare_parameter('combined_max_area', 1.0)
        self.declare_parameter('combined_aspect_ratio_min', 0.5)
        self.declare_parameter('combined_aspect_ratio_max', 2.0)
        
        # Dilation for combining adjacent regions
        self.declare_parameter('combination_kernel_size', 15)
        self.declare_parameter('combination_iterations', 2)
        
    def _load_parameters(self):
        """Load parameters from ROS 2 parameter server"""

        # Image Subscription and Detection Publisher
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)

        # Full debug mode
        self.full_debug_mode = bool(self.get_parameter('full_debug_mode').value)
        self.mask_debug_topic = str(self.get_parameter('mask_debug_topic').value)
        self.bbox_debug_topic = str(self.get_parameter('bbox_debug_topic').value)
        
        # Light debug mode
        self.light_debug_mode = bool(self.get_parameter('light_debug_mode').value)
        self.light_debug_topic = str(self.get_parameter('light_debug_topic').value)
        self.light_debug_size = int(self.get_parameter('light_debug_size').value)
        self.light_debug_quality = int(self.get_parameter('light_debug_quality').value)
        
        # Morphology parameters
        self.yellow_morph_kernel_size = int(self.get_parameter('yellow_morph_kernel_size').value)
        self.yellow_morph_iterations = int(self.get_parameter('yellow_morph_iterations').value)
        self.blue_morph_kernel_size = int(self.get_parameter('blue_morph_kernel_size').value)
        self.blue_morph_iterations = int(self.get_parameter('blue_morph_iterations').value)
        
        # Detection constraints
        self.combined_min_area = float(self.get_parameter('combined_min_area').value)
        self.combined_max_area = float(self.get_parameter('combined_max_area').value)
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
        if self.full_debug_mode:
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
            
        # Telemetry debug publisher (conditional based on debug mode)
        if self.light_debug_mode:
            self.telemetry_debug_pub = self.create_publisher(
                CompressedImage,
                self.light_debug_topic,
                10
            )
    
    def image_callback(self, msg: Image):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Store image dimensions for normalization
            self.last_image_shape = cv_image.shape
            
            # Detect landing pads
            detections, mask_debug_image, bbox_debug_image = self.detect_landing_pads(cv_image)
            
            # Publish detections
            detection_msg = self._create_detection_message(detections, msg.header)
            self.detection_pub.publish(detection_msg)
            
            # Publish debug images if enabled
            if self.full_debug_mode:
                if mask_debug_image is not None:
                    mask_msg = self.bridge.cv2_to_imgmsg(mask_debug_image, "bgr8")
                    mask_msg.header = msg.header
                    self.mask_debug_pub.publish(mask_msg)
                
                if bbox_debug_image is not None:
                    bbox_msg = self.bridge.cv2_to_imgmsg(bbox_debug_image, "bgr8")
                    bbox_msg.header = msg.header
                    self.bbox_debug_pub.publish(bbox_msg)
            
            # Publish telemetry debug images at reduced rate (3Hz instead of 10Hz)
            if self.light_debug_mode and hasattr(self, 'telemetry_debug_pub'):
                self.frame_count += 1
                if self.frame_count % self.debug_frame_skip == 0:
                    # Create combined debug image with masks and bounding boxes
                    telemetry_debug = self._create_telemetry_debug_image(
                        cv_image, mask_debug_image, bbox_debug_image, detections, msg.header
                    )
                    
                    if telemetry_debug is not None:
                        # Resize to target size for bandwidth efficiency
                        telemetry_debug = cv2.resize(telemetry_debug, 
                                                   (self.light_debug_size, self.light_debug_size))
                        
                        # Compress to JPEG and publish as CompressedImage
                        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.light_debug_quality]
                        success, encoded_image = cv2.imencode('.jpg', telemetry_debug, encode_param)
                        
                        if success:
                            compressed_msg = CompressedImage()
                            compressed_msg.header = msg.header
                            compressed_msg.format = "jpeg"
                            compressed_msg.data = encoded_image.tobytes()
                            self.telemetry_debug_pub.publish(compressed_msg)
                
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

        img_height, img_width = blue_mask.shape[:2]
        img_area = img_height * img_width

        # Find regions containing both colors
        detections = self._find_combined_regions(blue_mask, yellow_mask)
        
        # Draw detections on bbox debug image
        for detection in detections:
            x, y, w, h = detection['bbox']
            area = detection['area'] / img_area
            aspect_ratio = detection['aspect_ratio']
            # If we have rotated box info, draw rotated rectangle
            if 'angle' in detection and 'size' in detection and 'center' in detection:
                cx, cy = detection['center']
                sz_w, sz_h = detection['size']
                angle_deg = np.rad2deg(detection['angle'])
                rect = ((cx, cy), (sz_w, sz_h), angle_deg)
                box_pts = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(bbox_debug_image, [box_pts], 0, (255, 0, 255), 2)
            else:
                cv2.rectangle(bbox_debug_image, (x, y), (x+w, y+h), (255, 0, 255), 2)

            # Add text with area and aspect ratio
            text = f"Area: {area:.3f}, AR: {aspect_ratio:.2f}"
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
        """Find regions that contain both yellow and blue pixels (include rotation)"""
        detections = []
        
        # Get image dimensions for normalization calculations
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
        
        # Find regions where dilated masks overlap (indicating proximity)
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
            
            # Axis-aligned bounding rect for ROI checks
            x, y, w, h = cv2.boundingRect(contour)
            
            # Use minAreaRect to obtain rotated box (center, size, angle)
            rect = cv2.minAreaRect(contour)  # ((cx, cy), (width, height), angle_degrees)
            (cx, cy), (rect_w, rect_h), angle_deg = rect
            # ignore degenerate sizes
            if rect_w <= 0 or rect_h <= 0:
                continue
            
            # Compute aspect ratio from rotated box
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h) if min(rect_w, rect_h) > 0 else 0
            if not (self.combined_aspect_ratio_min <= aspect_ratio <= self.combined_aspect_ratio_max):
                continue
            
            # Check if this region actually contains both colors in original masks (use axis-aligned ROI)
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
                'class_id': 'landing_pad'
            }
            detections.append(detection)
        
        return detections
    
    def _create_telemetry_debug_image(self, original_image: np.ndarray, mask_debug_image: np.ndarray, 
                                    bbox_debug_image: np.ndarray, detections: List[Dict], header) -> np.ndarray:
        """
        Create combined debug image for telemetry with masks and bounding boxes.
        Optimized to avoid duplicate operations.
        
        Args:
            original_image: Original BGR image
            mask_debug_image: Color mask visualization (if available)
            bbox_debug_image: Bounding box visualization (if available)
            detections: List of detection dictionaries
            header: ROS header for timestamp info
            
        Returns:
            Combined debug image for telemetry
        """
        if bbox_debug_image is not None:
            # Use bbox_debug_image as base (it already has detections drawn)
            telemetry_debug = bbox_debug_image.copy()
        else:
            # Fallback to original image
            telemetry_debug = original_image.copy()
        
        # If we have mask debug image, overlay it with transparency
        if mask_debug_image is not None:
            # Create a mask overlay with 30% transparency
            alpha = 0.3
            beta = 1.0 - alpha
            cv2.addWeighted(mask_debug_image, alpha, telemetry_debug, beta, 0, telemetry_debug)
        
        # Add telemetry-specific information
        height, width = telemetry_debug.shape[:2]
        
        # Add timestamp and frame info
        timestamp_text = f"BaseDetector - {header.stamp.sec}.{header.stamp.nanosec//1000000:03d}"
        cv2.putText(telemetry_debug, timestamp_text, (10, height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Add detection count
        detection_count_text = f"Detections: {len(detections)}"
        cv2.putText(telemetry_debug, detection_count_text, (10, height - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Add processing resolution info
        res_text = f"Src: {width}x{height} -> {self.light_debug_size}x{self.light_debug_size}"
        cv2.putText(telemetry_debug, res_text, (10, height - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        return telemetry_debug
    
    def _create_detection_message(self, detections: List[Dict], header) -> Detection2DArray:
        """Create ROS 2 Detection2DArray message from detections"""
        detection_array = Detection2DArray()
        detection_array.header = header

        img_height, img_width = self.last_image_shape[:2]
        
        for det in detections:
            detection = Detection2D()
            
            # Use rotated box center/size/angle when available
            if 'center' in det and 'size' in det:
                cx, cy = det['center']
                sz_w, sz_h = det['size']
                detection.bbox.center.position.x = float(cx) / img_width
                detection.bbox.center.position.y = float(cy) / img_height
                detection.bbox.size_x = float(sz_w) / img_width
                detection.bbox.size_y = float(sz_h) / img_height
            else:
                # Fallback to axis-aligned bbox center/size
                x, y, w, h = det['bbox']
                detection.bbox.center.position.x = float(x + w/2) / img_width
                detection.bbox.center.position.y = float(y + h/2) / img_height
                detection.bbox.size_x = float(w) / img_width
                detection.bbox.size_y = float(h) / img_height

            # Publish angle (expect radians)
            detection.bbox.center.theta = float(det.get('angle', 0.0))
            
            # Set hypothesis (no confidence score for now)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_id']
            hypothesis.hypothesis.score = 1.0
            detection.results = [hypothesis]
            
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