#!/usr/bin/env python3
"""
Main telemetry handler node for processing and managing drone telemetry data.
Handles bandwidth management, topic aggregation, and telemetry distribution.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth, BaseDetection, Position


class TelemetryHandler(Node):
    """
    Main telemetry handler node that manages telemetry data flow,
    bandwidth allocation, and provides unified telemetry interface.
    """
    
    def __init__(self):
        super().__init__('telemetry_handler')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bandwidth.max_total', 9000),
                ('bandwidth.priority_allocation.position', 30),
                ('bandwidth.priority_allocation.drone_status', 25),
                ('bandwidth.priority_allocation.logs', 20),
                ('bandwidth.priority_allocation.system_health', 15),
                ('bandwidth.priority_allocation.bases', 10),
                ('bandwidth.priority_allocation.camera_debug', 0),
                ('topics.position', '/telemetry/position'),
                ('topics.bases', '/telemetry/bases'),
                ('topics.logs', '/telemetry/logs'),
                ('topics.drone_status', '/telemetry/drone_status'),
                ('topics.system_health', '/telemetry/system_health'),
                ('topics.camera_debug', '/telemetry/camera_debug'),
                ('log_level', 2),
                ('enable_logging', True),
            ]
        )
        
        # Get parameters
        self.max_bandwidth = self.get_parameter('bandwidth.max_total').value
        self.priority_allocation = {
            'position': self.get_parameter('bandwidth.priority_allocation.position').value,
            'drone_status': self.get_parameter('bandwidth.priority_allocation.drone_status').value,
            'logs': self.get_parameter('bandwidth.priority_allocation.logs').value,
            'system_health': self.get_parameter('bandwidth.priority_allocation.system_health').value,
            'bases': self.get_parameter('bandwidth.priority_allocation.bases').value,
            'camera_debug': self.get_parameter('bandwidth.priority_allocation.camera_debug').value,
        }
        
        # Topic names
        self.topics = {
            'position': self.get_parameter('topics.position').value,
            'bases': self.get_parameter('topics.bases').value,
            'logs': self.get_parameter('topics.logs').value,
            'drone_status': self.get_parameter('topics.drone_status').value,
            'system_health': self.get_parameter('topics.system_health').value,
            'camera_debug': self.get_parameter('topics.camera_debug').value,
        }
        
        # Visualization state (always enabled)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.marker_id_counter = 0
        
        # Bandwidth monitoring
        self.bandwidth_usage = {key: 0.0 for key in self.priority_allocation.keys()}
        self.message_counts = {key: 0 for key in self.priority_allocation.keys()}
        self.last_reset_time = time.time()
        
        # QoS profiles for different telemetry types
        self.position_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.system_health_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.base_detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.drone_status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.log_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Initialize subscribers
        self._setup_subscribers()
        
        # Initialize publishers (for processed/aggregated data)
        self._setup_publishers()
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        # Bandwidth reset timer
        self.bandwidth_timer = self.create_timer(1.0, self._reset_bandwidth_counters)
        
        self.get_logger().info(f"TelemetryHandler initialized with {self.max_bandwidth} B/s max bandwidth")
        
    def _setup_subscribers(self):
        """Setup subscribers for all telemetry topics."""
        
        # Position telemetry
        self.position_sub = self.create_subscription(
            Position,
            self.topics['position'],
            self._position_callback,
            self.position_qos
        )
        
        # Base detection telemetry
        self.bases_sub = self.create_subscription(
            BaseDetection,
            self.topics['bases'],
            self._bases_callback,
            self.base_detection_qos
        )
        
        # Log messages
        self.logs_sub = self.create_subscription(
            LogMessage,
            self.topics['logs'],
            self._logs_callback,
            self.log_qos
        )
        
        # Drone status
        self.drone_status_sub = self.create_subscription(
            DroneStatus,
            self.topics['drone_status'],
            self._drone_status_callback,
            self.drone_status_qos
        )
        
        # System health
        self.system_health_sub = self.create_subscription(
            SystemHealth,
            self.topics['system_health'],
            self._system_health_callback,
            self.system_health_qos
        )
        
    def _setup_publishers(self):
        """Setup publishers for processed telemetry data."""
        
        # Aggregated telemetry status  
        self.telemetry_status_pub = self.create_publisher(
            LogMessage,
            '/telemetry/handler_status',
            self.log_qos  # Use log_qos for status messages
        )
        
        viz_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone/pose', 
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/drone/path',
            viz_qos
        )
        
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/drone/twist',
            10
        )
        
        # Base marker publisher (like vision_fase1 functionality)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/base_markers',
            10
        )
        
    def _position_callback(self, msg):
        """Handle position telemetry."""
        self._track_bandwidth('position', msg)
        
        # Validate finite values (like pos_to_rviz did)
        if not (math.isfinite(msg.x_frd) and math.isfinite(msg.y_frd) and 
                math.isfinite(msg.z_frd) and math.isfinite(msg.yaw_frd) and
                math.isfinite(msg.vx_frd) and math.isfinite(msg.vy_frd) and 
                math.isfinite(msg.vz_frd)):
            self.get_logger().warn("Ignoring Position with NaN/inf values")
            return
        
        # Create visualization data (always enabled)
        current_time = self.get_clock().now()
        
        # Create and publish pose (FRD to ENU conversion like pos_to_rviz)
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "map"
        
        # FRD to ENU coordinate transformation
        pose.pose.position.x = msg.y_frd
        pose.pose.position.y = msg.x_frd  
        pose.pose.position.z = -msg.z_frd
        
        # Convert yaw to quaternion (ENU frame)
        yaw_enu = msg.yaw_frd + math.pi / 2
        half = yaw_enu * 0.5
        pose.pose.orientation.w = math.cos(half)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(half)
        
        self.pose_pub.publish(pose)
        
        # Update and publish path
        self.path.header = pose.header
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        
        # Create and publish twist
        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = msg.vy_frd
        twist.twist.linear.y = msg.vx_frd
        twist.twist.linear.z = -msg.vz_frd
        self.twist_pub.publish(twist)
        
    def _bases_callback(self, msg):
        """Handle base detection telemetry and create markers."""
        self._track_bandwidth('bases', msg)
        
        marker_array = MarkerArray()
        
        # Determine marker properties based on base_type
        if msg.base_type == "confirmed_base":
            # Green for confirmed bases (Confirmed base when drone is landed on it)
            color = (0.0, 1.0, 0.0)
            persistent = True
        elif msg.base_type == "first_estimate_base":
            # Yellow for new base discoveries (estimate based on vision)
            color = (1.0, 1.0, 0.0)
            persistent = True
        elif msg.base_type == "detected_base":
            # Red for estimates (temporary)
            color = (1.0, 0.0, 0.0)
            persistent = False
        else:
            # Default blue for unknown types
            color = (0.0, 0.0, 1.0)
            persistent = False
        
        # Create base marker
        marker = self._create_base_marker(
            (msg.position.x, msg.position.y, msg.position.z),
            f"{msg.base_type}_{msg.detection_id}",
            color[0], color[1], color[2],
            persistent
        )
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
    def _logs_callback(self, msg):
        """Handle log messages."""
        self._track_bandwidth('logs', msg)
        
        # Log level filtering
        log_level = self.get_parameter('log_level').value
        if msg.level >= log_level:
            # Forward important logs or process them
            if msg.level <= 2:  # ERROR=1, WARN=2 - high priority messages
                self.get_logger().warn(f"[{msg.node_name}] {msg.message}")
                
    def _drone_status_callback(self, msg):
        """Handle drone status telemetry."""
        self._track_bandwidth('drone_status', msg)
        
        # Could add status validation, alerts for critical states, etc.
        if msg.battery_voltage < 14.0:  # Low battery threshold
            self._publish_alert("Low battery voltage detected", "WARN")
            
    def _system_health_callback(self, msg):
        """Handle system health telemetry."""
        self._track_bandwidth('system_health', msg)
        
        # Health monitoring and alerts
        if msg.cpu_percent > 90.0:
            self._publish_alert("High CPU usage detected", "WARN")
        if msg.memory_percent > 90.0:
            self._publish_alert("High memory usage detected", "WARN")
        if msg.temperature > 80.0:
            self._publish_alert("High temperature detected", "ERROR")
            
    def _track_bandwidth(self, category: str, msg):
        """Track bandwidth usage for a message category."""
        # Rough estimate of message size (could be more accurate)
        estimated_size = len(str(msg)) * 1.5  # Rough serialization overhead
        self.bandwidth_usage[category] += estimated_size
        self.message_counts[category] += 1
        
    def _reset_bandwidth_counters(self):
        """Reset bandwidth counters every second."""
        current_time = time.time()
        if current_time - self.last_reset_time >= 1.0:
            # Log bandwidth usage
            total_usage = sum(self.bandwidth_usage.values())
            if total_usage > self.max_bandwidth * 1.1:  # 10% tolerance
                self.get_logger().warn(f"Bandwidth exceeded: {total_usage:.0f} B/s > {self.max_bandwidth} B/s")
            
            # Reset counters
            self.bandwidth_usage = {key: 0.0 for key in self.priority_allocation.keys()}
            self.last_reset_time = current_time
            
    def _publish_status(self):
        """Publish telemetry handler status."""
        total_messages = sum(self.message_counts.values())
        total_bandwidth = sum(self.bandwidth_usage.values())
        
        status_msg = LogMessage()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.node_name = "telemetry_handler"
        status_msg.level = 4  # DEBUG=4 according to LogMessage.msg
        status_msg.message = f"Processed {total_messages} messages, using {total_bandwidth:.0f}/{self.max_bandwidth} B/s"
        
        self.telemetry_status_pub.publish(status_msg)
        
        # Reset message counts
        self.message_counts = {key: 0 for key in self.priority_allocation.keys()}
        
    def _publish_alert(self, message: str, level: str):
        """Publish an alert message."""
        alert_msg = LogMessage()
        alert_msg.header.stamp = self.get_clock().now().to_msg()
        alert_msg.node_name = "telemetry_handler"
        alert_msg.level = 2 if level == "WARN" else 1  # WARN=2, ERROR=1 according to LogMessage.msg
        alert_msg.message = f"ALERT: {message}"
        
        self.telemetry_status_pub.publish(alert_msg)
    
    def _create_base_marker(self, position: tuple, namespace: str, r: float, g: float, b: float, persistent: bool = True):
        """Create a base marker for visualization (like vision_fase1 createBaseMarker)."""
        marker = Marker()
        
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position (FRD to ENU conversion like vision_fase1)
        marker.pose.position.x = position[1]  # y_frd -> x_enu
        marker.pose.position.y = position[0]  # x_frd -> y_enu
        marker.pose.position.z = -position[2] # z_frd -> z_enu
        
        # Orientation (no rotation)
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale (1x1x0.1m like vision_fase1)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1
        
        # Color
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        
        # Different transparency for red estimated bases vs others (like vision_fase1)
        if r == 1.0 and g == 0.0 and b == 0.0:
            marker.color.a = 0.4  # Red estimated bases - more transparent
        else:
            marker.color.a = 0.8  # Semi-transparent for others
        
        # Lifetime
        if persistent:
            marker.lifetime.nanosec = 0  # Persistent
        else:
            marker.lifetime.sec = 15  # 15 seconds
            marker.lifetime.nanosec = 0
        
        return marker
    
    def _create_circle_marker(self, position: tuple, namespace: str, radius: float, r: float, g: float, b: float, persistent: bool = True):
        """Create a circle marker for base radius visualization (like vision_fase1 createCircleMarker)."""
        marker = Marker()
        
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position (FRD to ENU conversion)
        marker.pose.position.x = position[1]  # y_frd -> x_enu
        marker.pose.position.y = position[0]  # x_frd -> y_enu
        marker.pose.position.z = -position[2] # z_frd -> z_enu
        
        # Orientation (no rotation)
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale (radius x radius x very thin height like vision_fase1)
        marker.scale.x = radius * 2.0  # Diameter
        marker.scale.y = radius * 2.0  # Diameter  
        marker.scale.z = 0.01          # Very thin cylinder (like a circle)
        
        # Color (more transparent for the circle like vision_fase1)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.3  # More transparent than the square
        
        # Lifetime
        if persistent:
            marker.lifetime.nanosec = 0  # Persistent
        else:
            marker.lifetime.sec = 15  # 15 seconds
            marker.lifetime.nanosec = 0
        
        return marker
    
    def publish_known_bases(self, bases_positions: list, known_base_radius: float = 1.0):
        """
        Publish known base markers (like vision_fase1 publishKnownBases).
        
        Args:
            bases_positions: List of (x, y, z) tuples representing base positions
            known_base_radius: Radius for the base circle marker
        """
        marker_array = MarkerArray()
        
        for i, position in enumerate(bases_positions):
            # Green square for the base
            base_marker = self._create_base_marker(
                position,
                f"known_base_{i}",
                0.0, 1.0, 0.0,  # Green
                True  # Persistent
            )
            marker_array.markers.append(base_marker)
            
            # Green circle for the radius
            circle_marker = self._create_circle_marker(
                position,
                f"known_base_circle_{i}",
                known_base_radius,
                0.0, 1.0, 0.0,  # Green
                True  # Persistent
            )
            marker_array.markers.append(circle_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = TelemetryHandler()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
