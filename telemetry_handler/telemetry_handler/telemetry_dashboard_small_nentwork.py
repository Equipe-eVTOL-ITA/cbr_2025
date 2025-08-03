#!/usr/bin/env python3
"""
Unified telemetry dashboard for comprehensive drone monitoring.
Combines drone status, system health, logs, and network performance monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime
import threading
import queue
import time
from typing import Dict, Optional, List

from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth


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
        
        # Network performance tracking
        self.network_stats = {
            'total_messages': 0,
            'messages_per_second': 0.0,
            'last_message_time': 0.0,
            'connection_quality': 'No Data',
            'message_times': []  # Keep track of recent message times
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
        self.messages_per_sec_var = tk.StringVar(value="0")
        self.total_messages_var = tk.StringVar(value="0")
        self.connection_quality_var = tk.StringVar(value="Unknown")
        self.last_message_time_var = tk.StringVar(value="Never")
        
        # Network status display
        row = 0
        ttk.Label(network_frame, text="Messages/sec:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.msg_rate_label = ttk.Label(network_frame, textvariable=self.messages_per_sec_var, font=('TkDefaultFont', 10, 'bold'))
        self.msg_rate_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(network_frame, text="Total Messages:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(network_frame, textvariable=self.total_messages_var, font=('TkDefaultFont', 10, 'bold')).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(network_frame, text="Connection Quality:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.connection_label = ttk.Label(network_frame, textvariable=self.connection_quality_var, font=('TkDefaultFont', 10, 'bold'))
        self.connection_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(network_frame, text="Last Message:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(network_frame, textvariable=self.last_message_time_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
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
        """Update network performance statistics."""
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
        
        # Determine connection quality
        msg_rate = self.network_stats['messages_per_second']
        if msg_rate > 10:
            self.network_stats['connection_quality'] = "Excellent"
        elif msg_rate > 5:
            self.network_stats['connection_quality'] = "Good"
        elif msg_rate > 1:
            self.network_stats['connection_quality'] = "Fair"
        elif msg_rate > 0.1:
            self.network_stats['connection_quality'] = "Poor"
        else:
            self.network_stats['connection_quality'] = "No Data"
                
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
        
        # Map arming state (matches DronePX4::ARMING_STATE enum)
        arming_states = {
            1: "DISARMED",
            2: "ARMED"
        }
        arming_state = arming_states.get(msg.arming_state, f"UNKNOWN({msg.arming_state})")
        self.arming_state_var.set(arming_state)
        
        # Update label color based on arming state
        if msg.arming_state == 2:  # ARMED
            self.arming_label.configure(foreground="red")
        elif msg.arming_state == 1:  # DISARMED
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
        flight_mode = flight_modes.get(msg.flight_mode, f"UNKNOWN({msg.flight_mode})")
        self.flight_mode_var.set(flight_mode)
        
        # Battery voltage and status
        self.battery_voltage_var.set(f"{msg.battery_voltage:.2f} V")
        
        # Battery status based on voltage
        if msg.battery_voltage > 15.5:
            battery_status = "Excellent"
            self.battery_label.configure(foreground="green")
            self.battery_status_label.configure(foreground="green")
        elif msg.battery_voltage > 14.8:
            battery_status = "Good"
            self.battery_label.configure(foreground="green")
            self.battery_status_label.configure(foreground="green")
        elif msg.battery_voltage > 14.0:
            battery_status = "Fair"
            self.battery_label.configure(foreground="orange")
            self.battery_status_label.configure(foreground="orange")
        elif msg.battery_voltage > 13.0:
            battery_status = "Low"
            self.battery_label.configure(foreground="red")
            self.battery_status_label.configure(foreground="red")
        else:
            battery_status = "Critical"
            self.battery_label.configure(foreground="red")
            self.battery_status_label.configure(foreground="red")
            
        self.battery_status_var.set(battery_status)
        
        # Last update time
        timestamp = datetime.fromtimestamp(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        ).strftime("%H:%M:%S")
        self.last_drone_update_var.set(timestamp)
        
    def _update_health_status_display(self):
        """Update system health display."""
        msg = self.current_health_status
        
        # CPU usage
        cpu_text = f"{msg.cpu_percent:.1f}%"
        self.cpu_percent_var.set(cpu_text)
        if msg.cpu_percent > 90:
            self.cpu_label.configure(foreground="red")
        elif msg.cpu_percent > 70:
            self.cpu_label.configure(foreground="orange")
        else:
            self.cpu_label.configure(foreground="green")
            
        # Memory usage
        memory_text = f"{msg.memory_percent:.1f}%"
        self.memory_percent_var.set(memory_text)
        if msg.memory_percent > 90:
            self.memory_label.configure(foreground="red")
        elif msg.memory_percent > 70:
            self.memory_label.configure(foreground="orange")
        else:
            self.memory_label.configure(foreground="green")
            
        # Disk usage
        disk_text = f"{msg.disk_usage_percent:.1f}%"
        self.disk_percent_var.set(disk_text)
        if msg.disk_usage_percent > 90:
            self.disk_label.configure(foreground="red")
        elif msg.disk_usage_percent > 80:
            self.disk_label.configure(foreground="orange")
        else:
            self.disk_label.configure(foreground="green")
            
        # Temperature
        temp_text = f"{msg.temperature:.1f}°C"
        self.temperature_var.set(temp_text)
        if msg.temperature > 80:
            self.temperature_label.configure(foreground="red")
        elif msg.temperature > 70:
            self.temperature_label.configure(foreground="orange")
        else:
            self.temperature_label.configure(foreground="green")
            
        # Last update time
        timestamp = datetime.fromtimestamp(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        ).strftime("%H:%M:%S")
        self.last_health_update_var.set(timestamp)
        
    def _update_network_display(self):
        """Update network performance display."""
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
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S.%f")[:-3]
            
            # LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
            
            log_line = f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}\n"
            new_content += log_line
        
        # Only update if content changed
        if new_content.strip() != current_content.strip():
            self.log_text.delete('1.0', tk.END)
            
            # Insert with color coding
            for msg in filtered_messages[-100:]:
                timestamp = datetime.fromtimestamp(
                    msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                ).strftime("%H:%M:%S.%f")[:-3]
                
                level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
                level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
                
                log_line = f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}\n"
                self.log_text.insert(tk.END, log_line, level_name)
            
            # Auto-scroll to bottom if enabled
            if self.auto_scroll_var.get():
                self.log_text.see(tk.END)
        
    def _filter_messages(self) -> List[LogMessage]:
        """Filter messages based on current filter settings."""
        filtered = []
        
        # Level mapping - according to LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
        level_map = {"ERROR": 1, "WARN": 2, "INFO": 3, "DEBUG": 4, "ALL": 4}
        min_level = level_map.get(self.level_var.get(), 4)
        
        for msg in self.log_messages:
            # Level filter - show messages at min_level and above (lower numbers = higher priority)
            # If "ALL" is selected, show all messages
            if self.level_var.get() != "ALL" and msg.level > min_level:
                continue
                
            # Node filter
            if self.node_var.get() and self.node_var.get().lower() not in msg.node_name.lower():
                continue
                
            # Search filter
            if self.search_var.get() and self.search_var.get().lower() not in msg.message.lower():
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
        
    def add_log_message(self, msg: LogMessage):
        """Add a log message to the queue (thread-safe)."""
        self.log_queue.put(msg)
        
    def add_drone_status(self, msg: DroneStatus):
        """Add a drone status message to the queue (thread-safe)."""
        self.status_queue.put(msg)
        
    def add_health_status(self, msg: SystemHealth):
        """Add a health status message to the queue (thread-safe)."""
        self.health_queue.put(msg)
        
    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


class TelemetryDashboardNode(Node):
    """ROS2 node for the telemetry dashboard."""
    
    def __init__(self):
        super().__init__('telemetry_dashboard')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topics.logs', '/telemetry/logs'),
                ('topics.drone_status', '/telemetry/drone_status'),
                ('topics.system_health', '/telemetry/system_health'),
                ('enable_gui', True),
            ]
        )
        
        # Get parameters
        self.log_topic = self.get_parameter('topics.logs').value
        self.drone_status_topic = self.get_parameter('topics.drone_status').value
        self.health_topic = self.get_parameter('topics.system_health').value
        self.enable_gui = self.get_parameter('enable_gui').value
        
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
        
    def _log_callback(self, msg: LogMessage):
        """Handle incoming log messages."""
        if self.gui:
            self.gui.add_log_message(msg)
        else:
            # Console output if no GUI
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S.%f")[:-3]
            
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
            
            print(f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}")
            
    def _drone_status_callback(self, msg: DroneStatus):
        """Handle incoming drone status messages."""
        if self.gui:
            self.gui.add_drone_status(msg)
        else:
            print(f"Drone Status - Arming: {msg.arming_state}, Mode: {msg.flight_mode}, Battery: {msg.battery_voltage:.2f}V")
            
    def _health_callback(self, msg: SystemHealth):
        """Handle incoming system health messages.""" 
        if self.gui:
            self.gui.add_health_status(msg)
        else:
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
