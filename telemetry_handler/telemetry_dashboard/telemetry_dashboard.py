"""
Main entry point and ROS2 node for telemetry dashboard.
Coordinates between ROS2 subscriptions and GUI display.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
from typing import Optional

from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth, Position
from .dashboard_gui import TelemetryDashboardGUI
from network_metrics.network_metrics import NetworkMetrics

class TelemetryDashboardNode(Node):
    """ROS2 node for telemetry dashboard with network monitoring."""
    
    # Topic configuration for easy maintenance
    TOPIC_CONFIG = {
        'logs': {
            'topic': '/telemetry/logs',
            'msg_type': LogMessage,
            'qos_depth': 10,
            'expected_rate': 10.0
        },
        'drone_status': {
            'topic': '/telemetry/drone_status', 
            'msg_type': DroneStatus,
            'qos_depth': 10,
            'expected_rate': 2.0
        },
        'system_health': {
            'topic': '/telemetry/system_health',
            'msg_type': SystemHealth, 
            'qos_depth': 5,
            'expected_rate': 1.0
        },
        'position': {  # NEW: Adding position monitoring
            'topic': '/telemetry/position',
            'msg_type': Position,
            'qos_depth': 5, 
            'expected_rate': 20.0
        }
    }
    
    def __init__(self):
        super().__init__('telemetry_dashboard')
        
        # Declare parameters for network monitoring
        self.declare_parameter('network.ping_target', '192.168.0.152')
        self.declare_parameter('network.ground_station_ip', '192.168.0.100')
        
        # Get network parameters with proper type handling
        ping_target = str(self.get_parameter('network.ping_target').value or '192.168.0.152')
        ground_station_ip = str(self.get_parameter('network.ground_station_ip').value or '192.168.0.100')
        
        # Initialize network monitoring with configurable targets
        self.network_metrics = NetworkMetrics(ping_target, ground_station_ip)
        self.network_metrics.start_monitoring()
        
        # Initialize GUI
        self.gui = TelemetryDashboardGUI(self.network_metrics)
        
        # Setup subscribers based on configuration
        self.subscribers = {}
        self._setup_subscribers()
        
        self.get_logger().info(f"Telemetry Dashboard initialized - pinging {ping_target}")
        
    def _setup_subscribers(self):
        """Setup ROS2 subscribers based on topic configuration."""
        for topic_key, config in self.TOPIC_CONFIG.items():
            qos = QoSProfile(
                depth=config['qos_depth'],
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
            
            callback = getattr(self, f'_{topic_key}_callback')
            
            self.subscribers[topic_key] = self.create_subscription(
                config['msg_type'],
                config['topic'], 
                callback,
                qos
            )
            
    def _logs_callback(self, msg: LogMessage):
        """Handle log messages."""
        self.gui.add_log_message(msg)
        
    def _drone_status_callback(self, msg: DroneStatus):
        """Handle drone status messages."""
        self.network_metrics.record_topic_message('/telemetry/drone_status')
        self.gui.update_drone_status(msg)
        
    def _system_health_callback(self, msg: SystemHealth):
        """Handle system health messages."""
        self.network_metrics.record_topic_message('/telemetry/system_health')
        self.gui.update_system_health(msg)
        
    def _position_callback(self, msg: Position):  # NEW
        """Handle position messages for velocity monitoring."""
        self.network_metrics.record_topic_message('/telemetry/position')
        self.gui.update_velocity(msg)

    def run_gui(self):
        """Run the GUI main loop."""
        self.gui.run()


def main(args=None):
    """Main entry point."""
    import rclpy
    import threading
    
    rclpy.init(args=args)
    
    node = None
    try:
        node = TelemetryDashboardNode()
        
        # Start ROS2 spinning in a separate thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        
        # Run GUI in main thread
        node.run_gui()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Error in telemetry dashboard: {e}")
        else:
            print(f"Error in telemetry dashboard: {e}")
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main())