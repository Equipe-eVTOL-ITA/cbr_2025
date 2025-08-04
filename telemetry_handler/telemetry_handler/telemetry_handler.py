#!/usr/bin/env python3
"""
Main telemetry handler node for RVIZ visualization.
Handles position and base detection telemetry for visualization purposes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import BaseDetection, Position


class TelemetryHandler(Node):
    """
    Telemetry handler node for RVIZ visualization.
    Processes position and base detection data for visualization in RVIZ.
    """
    
    def __init__(self):
        super().__init__('telemetry_handler')
        
        # Declare parameters
        self.declare_parameter('topics.position', '/telemetry/position')
        self.declare_parameter('topics.bases', '/telemetry/bases')
        self.declare_parameter('visualization.known_base_radius', 1.5)
        
        # Topic names with proper type handling
        self.topics = {
            'position': self.get_parameter('topics.position').value or '/telemetry/position',
            'bases': self.get_parameter('topics.bases').value or '/telemetry/position',
            'known_base_radius' : self.get_parameter('visualization.known_base_radius').value or 1.5
        }
        
        # Visualization state
        self.path = Path()
        self.path.header.frame_id = "map"
        self.marker_id_counter = 0
        
        # Message tracking for status
        self.message_counts = {'position': 0, 'bases': 0}
        
        # QoS profiles for telemetry types
        self.position_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.base_detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Initialize subscribers
        self._setup_subscribers()
        
        # Initialize publishers (for visualization)
        self._setup_publishers()
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self._publish_status)
        
        self.get_logger().info(f"TelemetryHandler initialized for RVIZ visualization")
        
    def _setup_subscribers(self):
        """Setup subscribers for position and base detection telemetry."""
        
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
        
    def _setup_publishers(self):
        """Setup publishers for visualization data."""
        
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
        
        # Base marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/base_markers',
            10
        )
        
    def _position_callback(self, msg):
        """Handle position telemetry."""
        self.message_counts['position'] += 1
        
        # Validate finite values (like pos_to_rviz did)
        if not (math.isfinite(msg.x_frd) and math.isfinite(msg.y_frd) and 
                math.isfinite(msg.z_frd) and math.isfinite(msg.yaw_frd) and
                math.isfinite(msg.vx_frd) and math.isfinite(msg.vy_frd) and 
                math.isfinite(msg.vz_frd)):
            self.get_logger().warn("Ignoring Position with NaN/inf values")
            return
        
        # Create visualization data
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
        self.message_counts['bases'] += 1
        
        marker_array = MarkerArray()
        
        circle = False
        # Determine marker properties based on base_type
        if msg.base_type == "confirmed_base":
            # Green for confirmed bases (Confirmed base when drone is landed on it)
            color = (0.0, 1.0, 0.0)
            persistent = True
            circle = True
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

        if circle:
            circle_marker = self._create_circle_marker(
                (msg.position.x, msg.position.y, msg.position.z),
                f"{msg.base_type}_{msg.detection_id}_circle",
                self.topics['known_base_radius'],
                color[0], color[1], color[2],
                persistent
            )
            marker_array.markers.append(circle_marker)
        
        self.marker_pub.publish(marker_array)
        
    def _publish_status(self):
        """Publish simple telemetry handler status."""
        total_messages = sum(self.message_counts.values())
        
        self.get_logger().info(f"TelemetryHandler processed {total_messages} messages (Position: {self.message_counts['position']}, Bases: {self.message_counts['bases']})")
        
        # Reset message counts
        self.message_counts = {'position': 0, 'bases': 0}
    
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
