#!/usr/bin/env python3
"""
Telemetry recorder for saving drone telemetry data to files.
Supports multiple output formats including rosbag, CSV, and JSON.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import csv
import os
import signal
import sys
from datetime import datetime
from typing import Dict, Any, Optional
import threading
import queue

# Message imports
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth, BaseDetection, Position

try:
    import rosbag2_py
    ROSBAG_AVAILABLE = True
except ImportError:
    ROSBAG_AVAILABLE = False
    

class TelemetryRecorder(Node):
    """
    Records telemetry data to various formats for analysis and playback.
    """
    
    def __init__(self):
        super().__init__('telemetry_recorder')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('recording.enable', False),
                ('recording.max_duration_minutes', 30),
                ('recording.max_file_size_mb', 500),
                ('recording.output_directory', '~/frtl_2025_ws/flight_logs'),
                ('recording.recording_formats', ['rosbag', 'csv']),
                ('recording.include_images', False),
                ('recording.autosave_time', 300.0),  # Auto-save every 5 minutes by default
                ('topics.position', '/telemetry/position'),
                ('topics.bases', '/telemetry/bases'),
                ('topics.logs', '/telemetry/logs'),
                ('topics.drone_status', '/telemetry/drone_status'),
                ('topics.system_health', '/telemetry/system_health'),
                ('topics.camera_debug', '/telemetry/camera_debug'),
            ]
        )
        
        # Get parameters
        self.recording_enabled = self.get_parameter('recording.enable').value
        self.max_duration = self.get_parameter('recording.max_duration_minutes').value * 60  # Convert to seconds
        self.max_file_size = self.get_parameter('recording.max_file_size_mb').value * 1024 * 1024  # Convert to bytes
        output_dir_param = self.get_parameter('recording.output_directory').value
        self.output_dir = os.path.expanduser(str(output_dir_param)) if output_dir_param else '/tmp/telemetry_logs'
        self.formats = self.get_parameter('recording.recording_formats').value
        self.include_images = self.get_parameter('recording.include_images').value
        self.autosave_time = self.get_parameter('recording.autosave_time').value
        
        # Topic names
        self.topics = {
            'position': self.get_parameter('topics.position').value,
            'bases': self.get_parameter('topics.bases').value,
            'logs': self.get_parameter('topics.logs').value,
            'drone_status': self.get_parameter('topics.drone_status').value,
            'system_health': self.get_parameter('topics.system_health').value,
            'camera_debug': self.get_parameter('topics.camera_debug').value,
        }
        
        # Recording state
        self.is_recording = False
        self.recording_start_time = None
        self.current_session_id = None
        self.recorded_messages = 0
        self.current_file_size = 0
        self.shutdown_requested = False
        
        # Data storage for CSV/JSON export
        self.recorded_data: Dict[str, list] = {
            'position': [],
            'bases': [],
            'logs': [],
            'drone_status': [],
            'system_health': [],
        }
        
        # Thread-safe queue for data writing
        self.write_queue = queue.Queue()
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Setup QoS
        self.position_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.system_health_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.base_detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.drone_status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.log_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Initialize rosbag writer if available
        self.rosbag_writer = None
        if ROSBAG_AVAILABLE and 'rosbag' in self.formats:
            self._setup_rosbag()
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Start background writer thread
        self.writer_thread = threading.Thread(target=self._data_writer_loop, daemon=True)
        self.writer_thread.start()
        
        # Recording timer for duration limits
        self.recording_timer = None
        
        # Auto-save timer for periodic saves
        self.autosave_timer = None
        
        # Start recording if auto-start is enabled
        if self.recording_enabled:
            self.start_recording()
            
        self.get_logger().info(f"TelemetryRecorder initialized, output dir: {self.output_dir}")
        
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f"Received signal {signum}, initiating graceful shutdown...")
        self.shutdown_requested = True
        
        # Save data immediately if recording
        if self.is_recording:
            self.get_logger().info("Saving recording data before shutdown...")
            self.stop_recording()
        
        # Signal the writer thread to stop
        self.write_queue.put(("SHUTDOWN", None))
        
    def _autosave_callback(self):
        """Periodic auto-save of data while recording."""
        if not self.is_recording:
            return
            
        try:
            # Save current data without stopping recording
            self._save_csv_data()
            self._save_json_data()
            
            self.get_logger().info(
                f"Auto-saved telemetry data - Session: {self.current_session_id}, "
                f"Messages: {self.recorded_messages}"
            )
        except Exception as e:
            self.get_logger().error(f"Error during auto-save: {e}")
            
    def cleanup(self):
        """Cleanup resources before shutdown."""
        self.get_logger().info("Cleaning up telemetry recorder...")
        
        # Stop recording if active
        if self.is_recording:
            self.stop_recording()
            
        # Cancel timers
        if self.recording_timer:
            self.recording_timer.cancel()
            self.recording_timer = None
            
        if self.autosave_timer:
            self.autosave_timer.cancel()
            self.autosave_timer = None
            
        # Signal writer thread to stop
        self.write_queue.put(("SHUTDOWN", None))
        
        # Wait for writer thread to finish
        if hasattr(self, 'writer_thread') and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=2.0)
            
        self.get_logger().info("Telemetry recorder cleanup complete")
        
    def _setup_rosbag(self):
        """Setup rosbag writer if available."""
        if not ROSBAG_AVAILABLE:
            self.get_logger().warn("rosbag2_py not available, rosbag recording disabled")
            return
            
        # Will be initialized when recording starts
        
    def _setup_subscribers(self):
        """Setup subscribers for all telemetry topics."""
        
        # Position telemetry
        self.position_sub = self.create_subscription(
            Position,
            self.topics['position'],
            lambda msg: self._record_message('position', msg),
            self.position_qos
        )
        
        # Base detection telemetry
        self.bases_sub = self.create_subscription(
            BaseDetection,
            self.topics['bases'],
            lambda msg: self._record_message('bases', msg),
            self.base_detection_qos
        )
        
        # Log messages
        self.logs_sub = self.create_subscription(
            LogMessage,
            self.topics['logs'],
            lambda msg: self._record_message('logs', msg),
            self.log_qos
        )
        
        # Drone status
        self.drone_status_sub = self.create_subscription(
            DroneStatus,
            self.topics['drone_status'],
            lambda msg: self._record_message('drone_status', msg),
            self.drone_status_qos
        )
        
        # System health
        self.system_health_sub = self.create_subscription(
            SystemHealth,
            self.topics['system_health'],
            lambda msg: self._record_message('system_health', msg),
            self.system_health_qos
        )
        
        # Camera debug (if images are enabled)
        if self.include_images:
            self.camera_sub = self.create_subscription(
                Image,
                self.topics['camera_debug'],
                lambda msg: self._record_message('camera_debug', msg),
                self.base_detection_qos  # Use base_detection_qos for images
            )
            
    def start_recording(self, session_id: Optional[str] = None):
        """Start recording telemetry data."""
        if self.is_recording:
            self.get_logger().warn("Recording already in progress")
            return False
            
        if not self.recording_enabled:
            self.get_logger().warn("Recording is disabled")
            return False
            
        # Generate session ID if not provided
        if session_id is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_id = f"telemetry_{timestamp}"
            
        self.current_session_id = session_id
        self.recording_start_time = self.get_clock().now()
        self.is_recording = True
        self.recorded_messages = 0
        self.current_file_size = 0
        
        # Clear previous data
        for key in self.recorded_data:
            self.recorded_data[key].clear()
            
        # Initialize rosbag writer
        if ROSBAG_AVAILABLE and 'rosbag' in self.formats:
            bag_path = os.path.join(self.output_dir, f"{session_id}.bag")
            self.rosbag_writer = rosbag2_py.SequentialWriter()
            
            storage_options = rosbag2_py.StorageOptions(
                uri=bag_path,
                storage_id='sqlite3'
            )
            
            converter_options = rosbag2_py.ConverterOptions('', '')
            
            self.rosbag_writer.open(storage_options, converter_options)
            
            # Create topics in bag
            self._create_rosbag_topics()
            
        # Set recording duration timer
        if self.max_duration > 0:
            self.recording_timer = self.create_timer(
                self.max_duration, 
                self._stop_recording_timeout
            )
            
        # Set auto-save timer if enabled
        if self.autosave_time > 0:
            self.autosave_timer = self.create_timer(
                self.autosave_time,
                self._autosave_callback
            )
            
        self.get_logger().info(f"Started recording session: {session_id}")
        return True
        
    def stop_recording(self):
        """Stop recording telemetry data."""
        if not self.is_recording:
            self.get_logger().warn("No recording in progress")
            return False
            
        self.is_recording = False
        
        # Cancel timers
        if self.recording_timer:
            self.recording_timer.cancel()
            self.recording_timer = None
            
        if self.autosave_timer:
            self.autosave_timer.cancel()
            self.autosave_timer = None
            
        # Close rosbag
        if self.rosbag_writer:
            self.rosbag_writer.close()
            self.rosbag_writer = None
            
        # Save CSV/JSON data
        if 'csv' in self.formats:
            self._save_csv_data()
            
        if 'json' in self.formats:
            self._save_json_data()
            
        duration = (self.get_clock().now() - self.recording_start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"Recording stopped. Session: {self.current_session_id}, "
            f"Duration: {duration:.1f}s, Messages: {self.recorded_messages}"
        )
        
        return True
        
    def _record_message(self, topic_type: str, msg):
        """Record a message to the appropriate storage."""
        if not self.is_recording:
            return
            
        self.recorded_messages += 1
        
        # Estimate message size for file size tracking
        estimated_size = len(str(msg)) * 2  # Rough estimate
        self.current_file_size += estimated_size
        
        # Check file size limit
        if self.current_file_size > self.max_file_size:
            self.get_logger().warn("File size limit reached, stopping recording")
            self.stop_recording()
            return
            
        # Add to write queue for thread-safe processing
        self.write_queue.put((topic_type, msg))
        
    def _data_writer_loop(self):
        """Background thread for writing data."""
        while not self.shutdown_requested:
            try:
                # Get data from queue (blocking)
                topic_type, msg = self.write_queue.get(timeout=1.0)
                
                # Check for shutdown signal
                if topic_type == "SHUTDOWN":
                    self.get_logger().info("Writer thread received shutdown signal")
                    break
                
                if not self.is_recording:
                    continue
                    
                # Write to rosbag
                if self.rosbag_writer and topic_type != 'camera_debug':
                    # Convert message for rosbag
                    self._write_to_rosbag(topic_type, msg)
                    
                # Store for CSV/JSON export
                if topic_type in self.recorded_data:
                    msg_dict = self._message_to_dict(msg)
                    msg_dict['timestamp'] = self.get_clock().now().nanoseconds / 1e9
                    self.recorded_data[topic_type].append(msg_dict)
                    
            except queue.Empty:
                continue  # Normal timeout, keep checking
            except Exception as e:
                self.get_logger().error(f"Error in data writer: {e}")
                
        self.get_logger().info("Data writer thread shutting down")
                
    def _create_rosbag_topics(self):
        """Create topics in rosbag."""
        if not self.rosbag_writer:
            return
            
        topic_info_map = {
            'position': ('geometry_msgs/msg/PoseStamped', self.topics['position']),
            'bases': ('custom_msgs/msg/BaseDetection', self.topics['bases']),
            'logs': ('custom_msgs/msg/LogMessage', self.topics['logs']),
            'drone_status': ('custom_msgs/msg/DroneStatus', self.topics['drone_status']),
            'system_health': ('custom_msgs/msg/SystemHealth', self.topics['system_health']),
        }
        
        for topic_type, (msg_type, topic_name) in topic_info_map.items():
            topic_info = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=msg_type,
                serialization_format='cdr'
            )
            self.rosbag_writer.create_topic(topic_info)
            
    def _write_to_rosbag(self, topic_type: str, msg):
        """Write message to rosbag."""
        if not self.rosbag_writer:
            return
            
        try:
            from rclpy.serialization import serialize_message
            
            topic_name = self.topics[topic_type]
            timestamp = self.get_clock().now().nanoseconds
            
            # Serialize the message to CDR format
            serialized_msg = serialize_message(msg)
            
            # Write serialized message to rosbag
            self.rosbag_writer.write(topic_name, serialized_msg, timestamp)
        except Exception as e:
            self.get_logger().error(f"Error writing to rosbag: {e}")
            
    def _message_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS message to dictionary."""
        result = {}
        
        for field_name, field_type in msg.get_fields_and_field_types().items():
            value = getattr(msg, field_name)
            
            if hasattr(value, 'get_fields_and_field_types'):
                # Nested message
                result[field_name] = self._message_to_dict(value)
            elif isinstance(value, list):
                # Array of messages or primitives
                result[field_name] = [
                    self._message_to_dict(item) if hasattr(item, 'get_fields_and_field_types') else item
                    for item in value
                ]
            else:
                # Primitive type
                result[field_name] = value
                
        return result
        
    def _save_csv_data(self):
        """Save recorded data as CSV files."""
        for topic_type, messages in self.recorded_data.items():
            if not messages:
                continue
                
            csv_path = os.path.join(self.output_dir, f"{self.current_session_id}_{topic_type}.csv")
            
            try:
                with open(csv_path, 'w', newline='') as csvfile:
                    if messages:
                        # Flatten nested dictionaries for CSV
                        flattened = []
                        for msg in messages:
                            flat_msg = self._flatten_dict(msg)
                            flattened.append(flat_msg)
                            
                        writer = csv.DictWriter(csvfile, fieldnames=flattened[0].keys())
                        writer.writeheader()
                        writer.writerows(flattened)
                        
                self.get_logger().info(f"Saved {len(messages)} {topic_type} messages to {csv_path}")
            except Exception as e:
                self.get_logger().error(f"Error saving CSV for {topic_type}: {e}")
                
    def _save_json_data(self):
        """Save recorded data as JSON file."""
        json_path = os.path.join(self.output_dir, f"{self.current_session_id}.json")
        
        try:
            with open(json_path, 'w') as jsonfile:
                json.dump(self.recorded_data, jsonfile, indent=2, default=str)
                
            total_messages = sum(len(messages) for messages in self.recorded_data.values())
            self.get_logger().info(f"Saved {total_messages} total messages to {json_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving JSON: {e}")
            
    def _flatten_dict(self, d: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """Flatten nested dictionary for CSV export."""
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            elif isinstance(v, list) and v and isinstance(v[0], dict):
                # Handle array of objects by indexing
                for i, item in enumerate(v):
                    items.extend(self._flatten_dict(item, f"{new_key}[{i}]", sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)
        
    def _stop_recording_timeout(self):
        """Called when recording duration limit is reached."""
        self.get_logger().info("Recording duration limit reached")
        self.stop_recording()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = TelemetryRecorder()
        
        # Handle shutdown gracefully
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}")
        else:
            print(f"Error creating TelemetryRecorder node: {e}")
            
    finally:
        # Always cleanup, regardless of how we got here
        if node:
            try:
                node.cleanup()
            except Exception as e:
                print(f"Error during cleanup: {e}")
            finally:
                node.destroy_node()
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")


if __name__ == '__main__':
    main()
