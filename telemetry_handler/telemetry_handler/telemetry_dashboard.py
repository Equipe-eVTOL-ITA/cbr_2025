#!/usr/bin/env python3
"""
Unified telemetry dashboard for comprehensive drone monitoring.
Combines drone status, system health, logs, and network performance monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.topic_endpoint_info import TopicEndpointInfo
from rclpy.subscription import Subscription
import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime
import threading
import queue
import time
import psutil
import sys
from typing import Dict, Optional, List, TYPE_CHECKING

if TYPE_CHECKING:
    from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth
else:
    try:
        from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth
    except ImportError:
        # Handle case where custom_msgs is not available during static analysis
        LogMessage = None
        DroneStatus = None
        SystemHealth = None


class TelemetryDashboard:
    """Unified GUI for comprehensive telemetry monitoring."""
    
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Drone Telemetry Dashboard")
        self.root.geometry("1400x900")
        
        # Message queues for thread-safe GUI updates
        self.log_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.health_queue = queue.Queue()
        
        # Current status data
        self.current_drone_status: Optional[DroneStatus] = None
        self.current_health_status: Optional[SystemHealth] = None
        
        # Log storage
        self.log_messages: List[LogMessage] = []
        self.max_displayed_logs = 1000
        
        # Network performance tracking with ROS2 metrics
        self.network_stats = {
            'total_messages': 0,
            'messages_per_second': 0.0,
            'last_message_time': 0.0,
            'connection_quality': 'No Data',
            'message_times': [],  # Keep track of recent message times
            # ROS2 Performance metrics
            'bandwidth_usage': 0.0,  # Bytes per second
            'message_sizes': [],     # Track message sizes for bandwidth calculation
            'latency_stats': {
                'min': float('inf'),
                'max': 0.0,
                'avg': 0.0,
                'recent_latencies': []
            },
            'packet_loss': 0.0,
            'topic_health': {},      # Per-topic health status
            'subscriber_count': 0,
            'publisher_count': 0,
            'network_io': {
                'bytes_sent': 0,
                'bytes_recv': 0,
                'packets_sent': 0,
                'packets_recv': 0,
                'errors': 0,
                'drops': 0
            }
        }
        
        # Setup GUI
        self._setup_gui()
        
        # Start message processing
        self._start_message_processor()
        
    def _setup_gui(self):
        """Setup the main GUI layout."""
        # Create main paned window (horizontal split)
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Status displays (400px)
        left_frame = ttk.Frame(main_paned, width=400)
        main_paned.add(left_frame, weight=1)
        
        # Right panel - Logs (1000px)
        right_frame = ttk.Frame(main_paned, width=1000)
        main_paned.add(right_frame, weight=2)
        
        # Setup status panels
        self._setup_status_panels(left_frame)
        
        # Setup log panel
        self._setup_log_panel(right_frame)
        
    def _setup_status_panels(self, parent):
        """Setup drone status, health monitoring, and network panels."""
        
        # Create scrollable frame for status panels
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # === DRONE STATUS PANEL ===
        drone_frame = ttk.LabelFrame(scrollable_frame, text="🚁 Drone Status", padding=10)
        drone_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Drone status variables
        self.arming_state_var = tk.StringVar(value="Unknown")
        self.flight_mode_var = tk.StringVar(value="Unknown")
        self.battery_voltage_var = tk.StringVar(value="N/A")
        self.battery_status_var = tk.StringVar(value="Unknown")
        self.last_drone_update_var = tk.StringVar(value="Never")
        
        # Drone status display
        row = 0
        ttk.Label(drone_frame, text="Arming State:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.arming_label = ttk.Label(drone_frame, textvariable=self.arming_state_var, font=('TkDefaultFont', 10, 'bold'))
        self.arming_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(drone_frame, text="Flight Mode:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.flight_mode_label = ttk.Label(drone_frame, textvariable=self.flight_mode_var, font=('TkDefaultFont', 10, 'bold'))
        self.flight_mode_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(drone_frame, text="Battery Voltage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.battery_label = ttk.Label(drone_frame, textvariable=self.battery_voltage_var, font=('TkDefaultFont', 10, 'bold'))
        self.battery_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(drone_frame, text="Battery Status:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.battery_status_label = ttk.Label(drone_frame, textvariable=self.battery_status_var, font=('TkDefaultFont', 10, 'bold'))
        self.battery_status_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(drone_frame, text="Last Update:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(drone_frame, textvariable=self.last_drone_update_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        # === SYSTEM HEALTH PANEL ===
        health_frame = ttk.LabelFrame(scrollable_frame, text="💻 System Health", padding=10)
        health_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Health status variables
        self.cpu_percent_var = tk.StringVar(value="N/A")
        self.memory_percent_var = tk.StringVar(value="N/A")
        self.disk_percent_var = tk.StringVar(value="N/A")
        self.temperature_var = tk.StringVar(value="N/A")
        self.last_health_update_var = tk.StringVar(value="Never")
        
        # Health status display
        row = 0
        ttk.Label(health_frame, text="CPU Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.cpu_label = ttk.Label(health_frame, textvariable=self.cpu_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.cpu_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(health_frame, text="Memory Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.memory_label = ttk.Label(health_frame, textvariable=self.memory_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.memory_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(health_frame, text="Disk Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.disk_label = ttk.Label(health_frame, textvariable=self.disk_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.disk_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(health_frame, text="Temperature:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.temperature_label = ttk.Label(health_frame, textvariable=self.temperature_var, font=('TkDefaultFont', 10, 'bold'))
        self.temperature_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(health_frame, text="Last Update:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(health_frame, textvariable=self.last_health_update_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        # === NETWORK PERFORMANCE PANEL ===
        network_frame = ttk.LabelFrame(scrollable_frame, text="📡 Network Performance", padding=10)
        network_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Network status variables
        self.messages_per_sec_var = tk.StringVar(value="0.0")
        self.total_messages_var = tk.StringVar(value="0")
        self.bandwidth_usage_var = tk.StringVar(value="0.0 KB/s")
        self.connection_quality_var = tk.StringVar(value="Unknown")
        self.latency_avg_var = tk.StringVar(value="N/A")
        self.packet_loss_var = tk.StringVar(value="0.0%")
        self.topic_count_var = tk.StringVar(value="0/0")
        self.network_io_var = tk.StringVar(value="0/0 KB/s")
        self.last_message_time_var = tk.StringVar(value="Never")
        
        # Network status display (2-column layout for more metrics)
        row = 0
        ttk.Label(network_frame, text="Messages/sec:").grid(row=row, column=0, sticky=tk.W, pady=1)
        self.msg_rate_label = ttk.Label(network_frame, textvariable=self.messages_per_sec_var, font=('TkDefaultFont', 9, 'bold'))
        self.msg_rate_label.grid(row=row, column=1, sticky=tk.W, pady=1, padx=(5, 15))
        
        ttk.Label(network_frame, text="Bandwidth:").grid(row=row, column=2, sticky=tk.W, pady=1)
        self.bandwidth_label = ttk.Label(network_frame, textvariable=self.bandwidth_usage_var, font=('TkDefaultFont', 9, 'bold'))
        self.bandwidth_label.grid(row=row, column=3, sticky=tk.W, pady=1, padx=(5, 0))
        
        row += 1
        ttk.Label(network_frame, text="Total Messages:").grid(row=row, column=0, sticky=tk.W, pady=1)
        ttk.Label(network_frame, textvariable=self.total_messages_var, font=('TkDefaultFont', 9, 'bold')).grid(row=row, column=1, sticky=tk.W, pady=1, padx=(5, 15))
        
        ttk.Label(network_frame, text="Avg Latency:").grid(row=row, column=2, sticky=tk.W, pady=1)
        self.latency_label = ttk.Label(network_frame, textvariable=self.latency_avg_var, font=('TkDefaultFont', 9, 'bold'))
        self.latency_label.grid(row=row, column=3, sticky=tk.W, pady=1, padx=(5, 0))
        
        row += 1
        ttk.Label(network_frame, text="Connection Quality:").grid(row=row, column=0, sticky=tk.W, pady=1)
        self.connection_label = ttk.Label(network_frame, textvariable=self.connection_quality_var, font=('TkDefaultFont', 9, 'bold'))
        self.connection_label.grid(row=row, column=1, sticky=tk.W, pady=1, padx=(5, 15))
        
        ttk.Label(network_frame, text="Packet Loss:").grid(row=row, column=2, sticky=tk.W, pady=1)
        self.packet_loss_label = ttk.Label(network_frame, textvariable=self.packet_loss_var, font=('TkDefaultFont', 9, 'bold'))
        self.packet_loss_label.grid(row=row, column=3, sticky=tk.W, pady=1, padx=(5, 0))
        
        row += 1
        ttk.Label(network_frame, text="Topics (Sub/Pub):").grid(row=row, column=0, sticky=tk.W, pady=1)
        ttk.Label(network_frame, textvariable=self.topic_count_var, font=('TkDefaultFont', 9, 'bold')).grid(row=row, column=1, sticky=tk.W, pady=1, padx=(5, 15))
        
        ttk.Label(network_frame, text="Network I/O:").grid(row=row, column=2, sticky=tk.W, pady=1)
        self.network_io_label = ttk.Label(network_frame, textvariable=self.network_io_var, font=('TkDefaultFont', 9, 'bold'))
        self.network_io_label.grid(row=row, column=3, sticky=tk.W, pady=1, padx=(5, 0))
        
        row += 1
        ttk.Label(network_frame, text="Last Message:").grid(row=row, column=0, sticky=tk.W, pady=1)
        ttk.Label(network_frame, textvariable=self.last_message_time_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, columnspan=3, sticky=tk.W, pady=1, padx=(5, 0))
        
    def _setup_log_panel(self, parent):
        """Setup log viewing panel."""
        
        # Log control panel
        control_frame = ttk.LabelFrame(parent, text="📋 Log Controls", padding="5")
        control_frame.pack(fill=tk.X, padx=5, pady=(5, 2))
        
        # Log level filter
        ttk.Label(control_frame, text="Show Level:").grid(row=0, column=0, padx=(0, 5))
        self.level_var = tk.StringVar(value="ALL")
        level_combo = ttk.Combobox(control_frame, textvariable=self.level_var, 
                                  values=["ERROR", "WARN", "INFO", "DEBUG", "ALL"],
                                  state="readonly", width=8)
        level_combo.grid(row=0, column=1, padx=(0, 10))
        level_combo.bind('<<ComboboxSelected>>', self._on_filter_change)
        
        # Node filter
        ttk.Label(control_frame, text="Node:").grid(row=0, column=2, padx=(0, 5))
        self.node_var = tk.StringVar()
        self.node_entry = ttk.Entry(control_frame, textvariable=self.node_var, width=15)
        self.node_entry.grid(row=0, column=3, padx=(0, 10))
        self.node_entry.bind('<KeyRelease>', self._on_filter_change)
        
        # Search
        ttk.Label(control_frame, text="Search:").grid(row=0, column=4, padx=(0, 5))
        self.search_var = tk.StringVar()
        self.search_entry = ttk.Entry(control_frame, textvariable=self.search_var, width=20)
        self.search_entry.grid(row=0, column=5, padx=(0, 10))
        self.search_entry.bind('<KeyRelease>', self._on_filter_change)
        
        # Auto-scroll checkbox
        self.auto_scroll_var = tk.BooleanVar(value=True)
        auto_scroll_check = ttk.Checkbutton(control_frame, text="Auto Scroll", 
                                           variable=self.auto_scroll_var)
        auto_scroll_check.grid(row=0, column=6, padx=(0, 10))
        
        # Clear button
        clear_btn = ttk.Button(control_frame, text="Clear", command=self._clear_logs)
        clear_btn.grid(row=0, column=7)
        
        # Log display frame
        log_frame = ttk.LabelFrame(parent, text="📜 Telemetry Logs", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=(2, 5))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Text widget with scrollbar
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            wrap=tk.WORD, 
            width=120, 
            height=25,
            font=("Consolas", 9)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure text tags for different log levels
        self.log_text.tag_configure("DEBUG", foreground="gray")
        self.log_text.tag_configure("INFO", foreground="black")
        self.log_text.tag_configure("WARN", foreground="orange")
        self.log_text.tag_configure("ERROR", foreground="red")
        
    def _start_message_processor(self):
        """Start the message processing thread."""
        self.processing_thread = threading.Thread(target=self._process_messages, daemon=True)
        self.processing_thread.start()
        
        # Schedule GUI updates
        self.root.after(100, self._update_gui)
        
    def _process_messages(self):
        """Process incoming messages in a separate thread."""
        while True:
            try:
                # Process log messages
                while not self.log_queue.empty():
                    msg = self.log_queue.get_nowait()
                    self.log_messages.append(msg)
                    self._update_network_stats()
                    
                    # Limit stored messages
                    if len(self.log_messages) > self.max_displayed_logs:
                        self.log_messages = self.log_messages[-self.max_displayed_logs:]
                
                # Process status messages
                while not self.status_queue.empty():
                    self.current_drone_status = self.status_queue.get_nowait()
                    self._update_network_stats()
                
                # Process health messages
                while not self.health_queue.empty():
                    self.current_health_status = self.health_queue.get_nowait()
                    self._update_network_stats()
                
                threading.Event().wait(0.1)  # 100ms delay
                
            except Exception as e:
                print(f"Error processing messages: {e}")
                
    def _update_network_stats(self):
        """Update network performance statistics with ROS2 metrics."""
        current_time = time.time()
        self.network_stats['total_messages'] += 1
        self.network_stats['last_message_time'] = current_time
        
        # Keep track of message times for rate calculation (last 10 seconds)
        self.network_stats['message_times'].append(current_time)
        
        # Remove old message times (older than 10 seconds)
        cutoff_time = current_time - 10.0
        self.network_stats['message_times'] = [
            t for t in self.network_stats['message_times'] if t > cutoff_time
        ]
        
        # Calculate messages per second over the last 10 seconds
        if len(self.network_stats['message_times']) > 1:
            time_span = self.network_stats['message_times'][-1] - self.network_stats['message_times'][0]
            if time_span > 0:
                self.network_stats['messages_per_second'] = (len(self.network_stats['message_times']) - 1) / time_span
            else:
                self.network_stats['messages_per_second'] = 0.0
        else:
            self.network_stats['messages_per_second'] = 0.0
        
        # Update network I/O statistics using psutil
        try:
            net_io = psutil.net_io_counters()
            if hasattr(self, '_last_net_io'):
                time_diff = current_time - self._last_net_io_time
                if time_diff > 0:
                    bytes_sent_rate = (net_io.bytes_sent - self._last_net_io.bytes_sent) / time_diff
                    bytes_recv_rate = (net_io.bytes_recv - self._last_net_io.bytes_recv) / time_diff
                    self.network_stats['network_io']['bytes_sent'] = bytes_sent_rate
                    self.network_stats['network_io']['bytes_recv'] = bytes_recv_rate
                    
                    # Calculate packet rates
                    packets_sent_rate = (net_io.packets_sent - self._last_net_io.packets_sent) / time_diff
                    packets_recv_rate = (net_io.packets_recv - self._last_net_io.packets_recv) / time_diff
                    self.network_stats['network_io']['packets_sent'] = packets_sent_rate
                    self.network_stats['network_io']['packets_recv'] = packets_recv_rate
                    
                    # Track errors and drops
                    self.network_stats['network_io']['errors'] = net_io.errin + net_io.errout
                    self.network_stats['network_io']['drops'] = net_io.dropin + net_io.dropout
            
            self._last_net_io = net_io
            self._last_net_io_time = current_time
        except Exception as e:
            print(f"Error getting network I/O stats: {e}")
        
        # Estimate message size (rough calculation)
        estimated_msg_size = 100  # Average bytes per telemetry message
        self.network_stats['message_sizes'].append(estimated_msg_size)
        
        # Keep only recent message sizes (last 100 messages)
        if len(self.network_stats['message_sizes']) > 100:
            self.network_stats['message_sizes'] = self.network_stats['message_sizes'][-100:]
        
        # Calculate bandwidth usage
        if len(self.network_stats['message_sizes']) > 0:
            avg_msg_size = sum(self.network_stats['message_sizes']) / len(self.network_stats['message_sizes'])
            self.network_stats['bandwidth_usage'] = self.network_stats['messages_per_second'] * avg_msg_size
        
        # Calculate latency (simplified - time since message creation)
        # This is a rough estimate - for precise latency, we'd need message timestamps
        estimated_latency = (current_time - self.network_stats['last_message_time']) * 1000  # ms
        if estimated_latency < 1000:  # Only consider reasonable latencies
            self.network_stats['latency_stats']['recent_latencies'].append(estimated_latency)
            
            # Keep only recent latencies (last 50 measurements)
            if len(self.network_stats['latency_stats']['recent_latencies']) > 50:
                self.network_stats['latency_stats']['recent_latencies'] = self.network_stats['latency_stats']['recent_latencies'][-50:]
            
            # Update latency statistics
            recent = self.network_stats['latency_stats']['recent_latencies']
            if recent:
                self.network_stats['latency_stats']['min'] = min(recent)
                self.network_stats['latency_stats']['max'] = max(recent)
                self.network_stats['latency_stats']['avg'] = sum(recent) / len(recent)
        
        # Calculate packet loss (simplified estimation based on expected vs actual message rate)
        expected_rate = 10.0  # Expected messages per second for healthy telemetry
        if self.network_stats['messages_per_second'] > 0 and expected_rate > 0:
            loss_estimate = max(0, (expected_rate - self.network_stats['messages_per_second']) / expected_rate)
            self.network_stats['packet_loss'] = min(loss_estimate * 100, 100.0)  # Percentage
        
        # Update ROS2 topic information
        if hasattr(self, 'node'):
            try:
                # Get subscriber and publisher counts
                topics = self.node.get_topic_names_and_types()
                self.network_stats['subscriber_count'] = len([t for t in topics if any('telemetry' in t[0] for t in topics)])
                self.network_stats['publisher_count'] = len([t for t in topics if any('telemetry' in t[0] for t in topics)])
            except Exception as e:
                print(f"Error getting ROS2 topic info: {e}")
        
        # Determine connection quality based on multiple factors
        msg_rate = self.network_stats['messages_per_second']
        latency = self.network_stats['latency_stats']['avg']
        packet_loss = self.network_stats['packet_loss']
        
        # Quality scoring algorithm
        rate_score = min(msg_rate / 10.0, 1.0) * 100  # 10 msg/s = 100%
        latency_score = max(0, (1000 - latency) / 1000) * 100 if latency > 0 else 100  # <1000ms = good
        loss_score = max(0, (100 - packet_loss))  # No loss = 100%
        
        overall_score = (rate_score + latency_score + loss_score) / 3
        
        if overall_score >= 80:
            self.network_stats['connection_quality'] = "Excellent"
        elif overall_score >= 60:
            self.network_stats['connection_quality'] = "Good"
        elif overall_score >= 40:
            self.network_stats['connection_quality'] = "Fair"
        elif overall_score >= 20:
            self.network_stats['connection_quality'] = "Poor"
        else:
            self.network_stats['connection_quality'] = "Critical"
                
    def _update_gui(self):
        """Update GUI with new data."""
        try:
            # Update drone status
            if self.current_drone_status:
                self._update_drone_status_display()
            
            # Update health status  
            if self.current_health_status:
                self._update_health_status_display()
                
            # Update network stats
            self._update_network_display()
            
            # Update log display
            self._refresh_log_display()
            
        except Exception as e:
            print(f"Error updating GUI: {e}")
        
        # Schedule next update
        self.root.after(1000, self._update_gui)  # Update every second
        
    def _update_drone_status_display(self):
        """Update drone status display."""
        msg = self.current_drone_status
        if msg is None:
            return
        
        # Map arming state (matches DronePX4::ARMING_STATE enum)
        arming_states = {
            1: "DISARMED",
            2: "ARMED"
        }
        arming_state = arming_states.get(getattr(msg, 'arming_state', 0), f"UNKNOWN({getattr(msg, 'arming_state', 0)})")
        self.arming_state_var.set(arming_state)
        
        # Update label color based on arming state
        arming_state_val = getattr(msg, 'arming_state', 0)
        if arming_state_val == 2:  # ARMED
            self.arming_label.configure(foreground="red")
        elif arming_state_val == 1:  # DISARMED
            self.arming_label.configure(foreground="green")
        else:
            self.arming_label.configure(foreground="orange")
        
        # Map flight mode (matches DronePX4::FLIGHT_MODE enum)
        flight_modes = {
            0: "MANUAL",
            1: "ALTCTL", 
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER",
            5: "AUTO_RTL",
            6: "ACRO",
            7: "DESCEND",
            8: "TERMINATION",
            9: "OFFBOARD",
            10: "STAB",
            11: "AUTO_TAKEOFF",
            12: "AUTO_LAND",
            13: "AUTO_FOLLOW_TARGET",
            14: "AUTO_PRECLAND",
            15: "ORBIT",
            16: "AUTO_VTOL_TAKEOFF",
            17: "UNKNOWN_MODE"
        }
        flight_mode = flight_modes.get(getattr(msg, 'flight_mode', 0), f"UNKNOWN({getattr(msg, 'flight_mode', 0)})")
        self.flight_mode_var.set(flight_mode)
        
        # Battery voltage and status
        battery_voltage = getattr(msg, 'battery_voltage', 0.0)
        self.battery_voltage_var.set(f"{battery_voltage:.2f} V")
        
        # Battery status based on voltage
        if battery_voltage > 15.5:
            battery_status = "Excellent"
            self.battery_label.configure(foreground="green")
            self.battery_status_label.configure(foreground="green")
        elif battery_voltage > 14.8:
            battery_status = "Good"
            self.battery_label.configure(foreground="green")
            self.battery_status_label.configure(foreground="green")
        elif battery_voltage > 14.0:
            battery_status = "Fair"
            self.battery_label.configure(foreground="orange")
            self.battery_status_label.configure(foreground="orange")
        elif battery_voltage > 13.0:
            battery_status = "Low"
            self.battery_label.configure(foreground="red")
            self.battery_status_label.configure(foreground="red")
        else:
            battery_status = "Critical"
            self.battery_label.configure(foreground="red")
            self.battery_status_label.configure(foreground="red")
            
        self.battery_status_var.set(battery_status)
        
        # Last update time
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S")
            self.last_drone_update_var.set(timestamp)
        else:
            self.last_drone_update_var.set("N/A")
        
    def _update_health_status_display(self):
        """Update system health display."""
        msg = self.current_health_status
        if msg is None:
            return
        
        # CPU usage
        cpu_percent = getattr(msg, 'cpu_percent', 0.0)
        cpu_text = f"{cpu_percent:.1f}%"
        self.cpu_percent_var.set(cpu_text)
        if cpu_percent > 90:
            self.cpu_label.configure(foreground="red")
        elif cpu_percent > 70:
            self.cpu_label.configure(foreground="orange")
        else:
            self.cpu_label.configure(foreground="green")
            
        # Memory usage
        memory_percent = getattr(msg, 'memory_percent', 0.0)
        memory_text = f"{memory_percent:.1f}%"
        self.memory_percent_var.set(memory_text)
        if memory_percent > 90:
            self.memory_label.configure(foreground="red")
        elif memory_percent > 70:
            self.memory_label.configure(foreground="orange")
        else:
            self.memory_label.configure(foreground="green")
            
        # Disk usage
        disk_usage_percent = getattr(msg, 'disk_usage_percent', 0.0)
        disk_text = f"{disk_usage_percent:.1f}%"
        self.disk_percent_var.set(disk_text)
        if disk_usage_percent > 90:
            self.disk_label.configure(foreground="red")
        elif disk_usage_percent > 80:
            self.disk_label.configure(foreground="orange")
        else:
            self.disk_label.configure(foreground="green")
            
        # Temperature
        temperature = getattr(msg, 'temperature', 0.0)
        temp_text = f"{temperature:.1f}°C"
        self.temperature_var.set(temp_text)
        if temperature > 80:
            self.temperature_label.configure(foreground="red")
        elif temperature > 70:
            self.temperature_label.configure(foreground="orange")
        else:
            self.temperature_label.configure(foreground="green")
            
        # Last update time
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S")
            self.last_health_update_var.set(timestamp)
        else:
            self.last_health_update_var.set("N/A")
        
    def _update_network_display(self):
        """Update network performance display with enhanced ROS2 metrics."""
        # Messages per second
        msg_rate = self.network_stats['messages_per_second']
        self.messages_per_sec_var.set(f"{msg_rate:.1f}")
        
        if msg_rate > 10:
            self.msg_rate_label.configure(foreground="green")
        elif msg_rate > 1:
            self.msg_rate_label.configure(foreground="orange")
        else:
            self.msg_rate_label.configure(foreground="red")
            
        # Total messages
        self.total_messages_var.set(str(self.network_stats['total_messages']))
        
        # Bandwidth usage
        bandwidth = self.network_stats['bandwidth_usage']
        if bandwidth < 1024:
            bandwidth_str = f"{bandwidth:.1f} B/s"
        elif bandwidth < 1024*1024:
            bandwidth_str = f"{bandwidth/1024:.1f} KB/s"
        else:
            bandwidth_str = f"{bandwidth/(1024*1024):.1f} MB/s"
        self.bandwidth_usage_var.set(bandwidth_str)
        
        # Set bandwidth color based on usage
        if bandwidth > 1024*1024:  # > 1 MB/s
            self.bandwidth_label.configure(foreground="red")
        elif bandwidth > 100*1024:  # > 100 KB/s
            self.bandwidth_label.configure(foreground="orange")
        else:
            self.bandwidth_label.configure(foreground="green")
        
        # Average latency
        avg_latency = self.network_stats['latency_stats']['avg']
        if avg_latency > 0:
            self.latency_avg_var.set(f"{avg_latency:.1f} ms")
            # Set latency color
            if avg_latency > 1000:  # > 1 second
                self.latency_label.configure(foreground="red")
            elif avg_latency > 100:  # > 100 ms
                self.latency_label.configure(foreground="orange")
            else:
                self.latency_label.configure(foreground="green")
        else:
            self.latency_avg_var.set("N/A")
            self.latency_label.configure(foreground="gray")
        
        # Packet loss
        packet_loss = self.network_stats['packet_loss']
        self.packet_loss_var.set(f"{packet_loss:.1f}%")
        
        if packet_loss > 10:
            self.packet_loss_label.configure(foreground="red")
        elif packet_loss > 5:
            self.packet_loss_label.configure(foreground="orange")
        else:
            self.packet_loss_label.configure(foreground="green")
        
        # Topic counts (Sub/Pub)
        sub_count = self.network_stats['subscriber_count']
        pub_count = self.network_stats['publisher_count']
        self.topic_count_var.set(f"{sub_count}/{pub_count}")
        
        # Network I/O
        net_io = self.network_stats['network_io']
        bytes_sent = net_io['bytes_sent']
        bytes_recv = net_io['bytes_recv']
        
        # Format network I/O rates
        def format_bytes(bytes_val):
            if bytes_val < 1024:
                return f"{bytes_val:.0f} B/s"
            elif bytes_val < 1024*1024:
                return f"{bytes_val/1024:.1f} KB/s"
            else:
                return f"{bytes_val/(1024*1024):.1f} MB/s"
        
        sent_str = format_bytes(bytes_sent)
        recv_str = format_bytes(bytes_recv)
        self.network_io_var.set(f"↑{sent_str} ↓{recv_str}")
        
        # Set network I/O color based on total traffic
        total_traffic = bytes_sent + bytes_recv
        if total_traffic > 10*1024*1024:  # > 10 MB/s
            self.network_io_label.configure(foreground="red")
        elif total_traffic > 1024*1024:  # > 1 MB/s
            self.network_io_label.configure(foreground="orange")
        else:
            self.network_io_label.configure(foreground="green")
        
        # Connection quality
        quality = self.network_stats['connection_quality']
        self.connection_quality_var.set(quality)
        
        if quality in ["Excellent", "Good"]:
            self.connection_label.configure(foreground="green")
        elif quality == "Fair":
            self.connection_label.configure(foreground="orange")
        else:
            self.connection_label.configure(foreground="red")
            
        # Last message time
        if self.network_stats['last_message_time'] > 0:
            last_time = datetime.fromtimestamp(self.network_stats['last_message_time']).strftime("%H:%M:%S")
            self.last_message_time_var.set(last_time)
        
    def _refresh_log_display(self):
        """Refresh the log display with filtered messages."""
        # Only refresh if we have new messages
        filtered_messages = self._filter_messages()
        
        # Clear and repopulate (simple approach)
        current_content = self.log_text.get('1.0', tk.END)
        new_content = ""
        
        for msg in filtered_messages[-100:]:  # Show last 100 filtered messages
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                timestamp = datetime.fromtimestamp(
                    msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                ).strftime("%H:%M:%S.%f")[:-3]
            else:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level = getattr(msg, 'level', 0)
            level_name = level_names[level] if 0 < level < len(level_names) else "UNKNOWN"
            
            node_name = getattr(msg, 'node_name', 'unknown')
            message = getattr(msg, 'message', 'no message')
            
            log_line = f"[{timestamp}] [{level_name:5}] [{node_name:15}] {message}\n"
            new_content += log_line
        
        # Only update if content changed
        if new_content.strip() != current_content.strip():
            self.log_text.delete('1.0', tk.END)
            
            # Insert with color coding
            for msg in filtered_messages[-100:]:
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    timestamp = datetime.fromtimestamp(
                        msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                    ).strftime("%H:%M:%S.%f")[:-3]
                else:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
                level = getattr(msg, 'level', 0)
                level_name = level_names[level] if 0 < level < len(level_names) else "UNKNOWN"
                
                node_name = getattr(msg, 'node_name', 'unknown')
                message = getattr(msg, 'message', 'no message')
                
                log_line = f"[{timestamp}] [{level_name:5}] [{node_name:15}] {message}\n"
                self.log_text.insert(tk.END, log_line, level_name)
            
            # Auto-scroll to bottom if enabled
            if self.auto_scroll_var.get():
                self.log_text.see(tk.END)
        
    def _filter_messages(self):
        """Filter messages based on current filter settings."""
        filtered = []
        
        # Level mapping - according to LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
        level_map = {"ERROR": 1, "WARN": 2, "INFO": 3, "DEBUG": 4, "ALL": 4}
        min_level = level_map.get(self.level_var.get(), 4)
        
        for msg in self.log_messages:
            if msg is None:
                continue
                
            # Level filter - show messages at min_level and above (lower numbers = higher priority)
            # If "ALL" is selected, show all messages
            msg_level = getattr(msg, 'level', 0)
            if self.level_var.get() != "ALL" and msg_level > min_level:
                continue
                
            # Node filter
            node_name = getattr(msg, 'node_name', '')
            if self.node_var.get() and self.node_var.get().lower() not in node_name.lower():
                continue
                
            # Search filter
            message = getattr(msg, 'message', '')
            if self.search_var.get() and self.search_var.get().lower() not in message.lower():
                continue
                
            filtered.append(msg)
            
        return filtered
        
    def _on_filter_change(self, event=None):
        """Handle filter changes."""
        self._refresh_log_display()
        
    def _clear_logs(self):
        """Clear all log messages."""
        self.log_messages.clear()
        self.log_text.delete('1.0', tk.END)
        
    def add_log_message(self, msg):
        """Add a log message to the queue (thread-safe)."""
        if msg is not None:
            self.log_queue.put(msg)
        
    def add_drone_status(self, msg):
        """Add a drone status message to the queue (thread-safe)."""
        if msg is not None:
            self.status_queue.put(msg)
        
    def add_health_status(self, msg):
        """Add a health status message to the queue (thread-safe)."""
        if msg is not None:
            self.health_queue.put(msg)
        
    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


class TelemetryDashboardNode(Node):
    """ROS2 node for the telemetry dashboard."""
    
    def __init__(self):
        super().__init__('telemetry_dashboard')
        
        # Declare parameters
        self.declare_parameter('topics.logs', '/telemetry/logs')
        self.declare_parameter('topics.drone_status', '/telemetry/drone_status')
        self.declare_parameter('topics.system_health', '/telemetry/system_health')
        self.declare_parameter('enable_gui', True)
        
        # Get parameters with proper type handling
        self.log_topic: str = self.get_parameter('topics.logs').value or '/telemetry/logs'
        self.drone_status_topic: str = self.get_parameter('topics.drone_status').value or '/telemetry/drone_status'
        self.health_topic: str = self.get_parameter('topics.system_health').value or '/telemetry/system_health'
        self.enable_gui: bool = self.get_parameter('enable_gui').value or True
        
        # Setup QoS - Match telemetry_handler.py QoS profiles exactly
        self.system_health_qos = QoSProfile(
            depth=5,
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
        
        # Setup GUI if enabled
        if self.enable_gui:
            self.gui = TelemetryDashboard(self)
        else:
            self.gui = None
        
        # Setup subscribers
        self.log_subscriber = self.create_subscription(
            LogMessage,
            self.log_topic,
            self._log_callback,
            self.log_qos
        )
        
        self.drone_status_subscriber = self.create_subscription(
            DroneStatus,
            self.drone_status_topic,
            self._drone_status_callback,
            self.drone_status_qos
        )
        
        self.health_subscriber = self.create_subscription(
            SystemHealth,
            self.health_topic,
            self._health_callback,
            self.system_health_qos
        )
        
        self.get_logger().info(f"TelemetryDashboard initialized")
        self.get_logger().info(f"Subscribed to: {self.log_topic}, {self.drone_status_topic}, {self.health_topic}")
        
    def _log_callback(self, msg):
        """Handle incoming log messages."""
        if self.gui and msg is not None:
            self.gui.add_log_message(msg)
        elif msg is not None:
            # Console output if no GUI
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S.%f")[:-3]
            
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
            
            print(f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}")
            
    def _drone_status_callback(self, msg):
        """Handle incoming drone status messages."""
        if self.gui and msg is not None:
            self.gui.add_drone_status(msg)
        elif msg is not None:
            print(f"Drone Status - Arming: {msg.arming_state}, Mode: {msg.flight_mode}, Battery: {msg.battery_voltage:.2f}V")
            
    def _health_callback(self, msg):
        """Handle incoming system health messages.""" 
        if self.gui and msg is not None:
            self.gui.add_health_status(msg)
        elif msg is not None:
            print(f"System Health - CPU: {msg.cpu_percent:.1f}%, Memory: {msg.memory_percent:.1f}%, Temp: {msg.temperature:.1f}°C")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = TelemetryDashboardNode()
        
        if node.gui:
            # Run ROS2 in separate thread
            ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
            ros_thread.start()
            
            # Run GUI in main thread
            node.gui.run()
        else:
            # Run ROS2 in main thread
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
