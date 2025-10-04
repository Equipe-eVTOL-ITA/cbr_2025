#!/usr/bin/env python3

"""
Generalized Target Detection Framework for CBR 2025

TargetDetector é uma classe abstrata base que implementa funcionalidades comuns
para detecção de diferentes tipos de objetos (bases, pacotes, etc.).

Classes filhas devem implementar:
- detect(): método principal de detecção
- _setup_color_parameters(): configuração específica de cores HSV
- _create_debug_images(): criação de imagens de debug específicas

A classe ImageDebug é responsável por todo o sistema de debug e visualização.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge

from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Optional

class TargetDetector(Node, ABC):
    def __init__(self, detector_name: str):
        super().__init__(detector_name)
        
        self.bridge = CvBridge()
        
        # Frame counting for debug publishing rate control
        self.frame_count = 0
        
        # Declare and load common ROS parameters
        self._declare_ros_parameters()
        self._load_ros_parameters()
        
        # Initialize debug system
        self.image_debugger = ImageDebug(self)
        
        # Setup color parameters (implemented by child classes)
        self._setup_color_parameters()
        
        # Setup ROS interface
        self._setup_ros_interface()
        
        self.get_logger().info(f"{detector_name} initialized.")
    
    def _declare_ros_parameters(self):
        """Declare common ROS parameters for all detectors"""
        # Image subscription and detection publishing
        self.declare_parameter('image_topic', '/vertical_camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')
    
    def _load_ros_parameters(self):
        """Load common ROS parameters"""
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)

    def _setup_ros_interface(self):
        """Setup ROS subscribers and publishers"""
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

    def setup_color_parameters(self, color_name: str, lower_hsv: dict[str, int], upper_hsv: dict[str, int], kernel_size: int = 3, iterations: int = 2):
        """Setup color detection parameters for HSV filtering"""
        # Declare parameters for color ranges
        self.declare_parameter(f'{color_name}_lower_h', lower_hsv['h'])
        self.declare_parameter(f'{color_name}_lower_s', lower_hsv['s'])
        self.declare_parameter(f'{color_name}_lower_v', lower_hsv['v'])
        
        self.declare_parameter(f'{color_name}_upper_h', upper_hsv['h'])
        self.declare_parameter(f'{color_name}_upper_s', upper_hsv['s'])
        self.declare_parameter(f'{color_name}_upper_v', upper_hsv['v'])
        
        # Morphological operation parameters
        self.declare_parameter(f'{color_name}_morph_kernel_size', kernel_size)
        self.declare_parameter(f'{color_name}_morph_iterations', iterations)
        
        # Load the color ranges
        lower = np.array([
            int(self.get_parameter(f'{color_name}_lower_h').value),
            int(self.get_parameter(f'{color_name}_lower_s').value),
            int(self.get_parameter(f'{color_name}_lower_v').value)
        ])
        
        upper = np.array([
            int(self.get_parameter(f'{color_name}_upper_h').value),
            int(self.get_parameter(f'{color_name}_upper_s').value),
            int(self.get_parameter(f'{color_name}_upper_v').value)
        ])
        
        kernel_size_loaded = int(self.get_parameter(f'{color_name}_morph_kernel_size').value)
        iterations_loaded = int(self.get_parameter(f'{color_name}_morph_iterations').value)
        
        return lower, upper, kernel_size_loaded, iterations_loaded

    def image_callback(self, msg: Image):
        try:
            # converte a imagem ROS para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.last_image_shape = cv_image.shape # dimensoes

            # publicando as deteccoes
            detections, debug_images = self.detect(cv_image)
            detections_msg = self._create_detection_msg(detections, msg.header)
            self.detection_pub.publish(detections_msg)

            # publish debug images if enables
            self.image_debugger.publish(
                msg=msg,
                cv_image=cv_image,
                detections=detections,
                debug_images=debug_images
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _create_detection_msg(self, detections: List[Dict], header) -> Detection2DArray:
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
            hypothesis.hypothesis.score = float(det.get('confidence', 1.0))
            detection.results = [hypothesis]
            
            detection_array.detections.append(detection)
        
        return detection_array

    @abstractmethod
    def _setup_color_parameters(self):
        """Setup specific color parameters for the detector"""
        pass

    @abstractmethod
    def detect(self, image: np.ndarray) -> Tuple[List[Dict], Dict]:
        """
        Detect targets in the image
        
        Args:
            image: Input BGR image
            
        Returns:
            Tuple of (detections_list, debug_images_dict)
        """
        pass

# Utility function for running any detector (polymorphism)
def run_detector(detector: TargetDetector, args=None):
    """
    Run any detector that inherits from TargetDetector
    
    Args:
        detector: Instance of a class that inherits from TargetDetector
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()    


class ImageDebug():
    def __init__(self, detector_node: TargetDetector):
        self.frame_count = 0
        self.debug_frame_skip = 3  # 3rd frame (10 Hz -> 3.33 Hz)
        self.detector_node = detector_node
        
        self._declare_ros_parameters()
        self._load_parameters()
        self._setup_publishers()
    
    def _declare_ros_parameters(self):
        """Declare debug-related ROS parameters"""
        # Full debug mode
        self.detector_node.declare_parameter('full_debug_mode', False)
        self.detector_node.declare_parameter('mask_debug_topic', '/base_detector/mask_debug')
        self.detector_node.declare_parameter('bbox_debug_topic', '/base_detector/bbox_debug')

        # Light debug mode
        self.detector_node.declare_parameter('light_debug_mode', False)
        self.detector_node.declare_parameter('light_debug_topic', '/telemetry/camera_debug/compressed')
        self.detector_node.declare_parameter('light_debug_size', 400)
        self.detector_node.declare_parameter('light_debug_quality', 80)
    
    def _load_parameters(self):
        """Load debug parameters"""
        # Full debug mode
        self.full_debug_mode = bool(self.detector_node.get_parameter('full_debug_mode').value)
        self.mask_debug_topic = str(self.detector_node.get_parameter('mask_debug_topic').value)
        self.bbox_debug_topic = str(self.detector_node.get_parameter('bbox_debug_topic').value)

        # Light debug mode
        self.light_debug_mode = bool(self.detector_node.get_parameter('light_debug_mode').value)
        self.light_debug_topic = str(self.detector_node.get_parameter('light_debug_topic').value)
        self.light_debug_size = int(self.detector_node.get_parameter('light_debug_size').value)
        self.light_debug_quality = int(self.detector_node.get_parameter('light_debug_quality').value)

    def _setup_publishers(self):
        """Setup ROS publishers for debug images"""
        if self.full_debug_mode:
            self.detector_node.get_logger().info("Full debug mode enabled.")
            self.mask_debug_pub = self.detector_node.create_publisher(Image, self.mask_debug_topic, 10)
            self.bbox_debug_pub = self.detector_node.create_publisher(Image, self.bbox_debug_topic, 10)

        if self.light_debug_mode:
            self.detector_node.get_logger().info("Light debug mode enabled.")
            self.telemetry_debug_pub = self.detector_node.create_publisher(CompressedImage, self.light_debug_topic, 10)
    
    def _create_telemetry_debug_image(self, original_image: np.ndarray, debug_images: Dict, 
                                     detections: List[Dict], header) -> np.ndarray:
        """
        Create combined debug image for telemetry with masks and bounding boxes.
        
        Args:
            original_image: Original BGR image
            debug_images: Dictionary with 'mask_debug' and 'bbox_debug' keys
            detections: List of detection dictionaries
            header: ROS header for timestamp info
            
        Returns:
            Combined debug image for telemetry
        """
        # Use bbox_debug as base if available, otherwise original image
        if 'bbox_debug' in debug_images and debug_images['bbox_debug'] is not None:
            telemetry_debug = debug_images['bbox_debug'].copy()
        else:
            telemetry_debug = original_image.copy()
        
        # Overlay mask debug with transparency if available
        if 'mask_debug' in debug_images and debug_images['mask_debug'] is not None:
            alpha = 0.3
            beta = 1.0 - alpha
            cv2.addWeighted(debug_images['mask_debug'], alpha, telemetry_debug, beta, 0, telemetry_debug)
        
        # Add telemetry information
        height, width = telemetry_debug.shape[:2]
        
        # Add timestamp
        detector_name = self.detector_node.get_name()
        timestamp_text = f"{detector_name} - {header.stamp.sec}.{header.stamp.nanosec//1000000:03d}"
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
    
    def publish(self, msg: Image, cv_image: np.ndarray, detections: List[Dict], debug_images: Dict):
        """
        Publish debug images based on configuration
        
        Args:
            msg: Original ROS Image message
            cv_image: OpenCV image
            detections: List of detections
            debug_images: Dictionary containing debug images from detector
        """
        # Publish full debug images
        if self.full_debug_mode:
            if 'mask_debug' in debug_images and debug_images['mask_debug'] is not None:
                mask_msg = self.detector_node.bridge.cv2_to_imgmsg(debug_images['mask_debug'], "bgr8")
                mask_msg.header = msg.header
                self.mask_debug_pub.publish(mask_msg)
            
            if 'bbox_debug' in debug_images and debug_images['bbox_debug'] is not None:
                bbox_msg = self.detector_node.bridge.cv2_to_imgmsg(debug_images['bbox_debug'], "bgr8")
                bbox_msg.header = msg.header
                self.bbox_debug_pub.publish(bbox_msg)
        
        # Publish telemetry debug at reduced rate
        if self.light_debug_mode and hasattr(self, 'telemetry_debug_pub'):
            self.frame_count += 1
            if self.frame_count % self.debug_frame_skip == 0:
                telemetry_debug = self._create_telemetry_debug_image(
                    cv_image, debug_images, detections, msg.header
                )
                
                if telemetry_debug is not None:
                    # Resize for bandwidth efficiency
                    telemetry_debug = cv2.resize(telemetry_debug, 
                                               (self.light_debug_size, self.light_debug_size))
                    
                    # Compress to JPEG and publish
                    encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.light_debug_quality]
                    success, encoded_image = cv2.imencode('.jpg', telemetry_debug, encode_param)
                    
                    if success:
                        compressed_msg = CompressedImage()
                        compressed_msg.header = msg.header
                        compressed_msg.format = "jpeg"
                        compressed_msg.data = encoded_image.tobytes()
                        self.telemetry_debug_pub.publish(compressed_msg)