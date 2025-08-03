#!/usr/bin/env python3
"""
Log viewer GUI for drone telemetry system.
Provides real-time log viewing with filtering and search capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
from datetime import datetime
from typing import List, Dict
import queue

from custom_msgs.msg import LogMessage


class LogViewerGUI:
    """GUI for viewing telemetry logs in real-time."""
    
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Drone Telemetry Log Viewer")
        self.root.geometry("1000x700")
        
        # Message queue for thread-safe GUI updates
        self.message_queue = queue.Queue()
        
        # Log storage
        self.log_messages: List[LogMessage] = []
        self.max_displayed_logs = 1000
        self.auto_scroll = True
        
        # Filter settings
        self.current_filter_level = 0  # Show all levels
        self.current_filter_node = ""  # Show all nodes
        self.search_text = ""
        
        self._setup_gui()
        self._start_message_processor()
        
    def _setup_gui(self):
        """Setup the GUI components."""
        
        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="5")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # Log level filter
        ttk.Label(control_frame, text="Min Level:").grid(row=0, column=0, padx=(0, 5))
        self.level_var = tk.StringVar(value="DEBUG")
        level_combo = ttk.Combobox(control_frame, textvariable=self.level_var, 
                                  values=["ERROR", "WARN", "INFO", "DEBUG"],
                                  state="readonly", width=10)
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
                                           variable=self.auto_scroll_var,
                                           command=self._on_auto_scroll_change)
        auto_scroll_check.grid(row=0, column=6, padx=(0, 10))
        
        # Clear button
        clear_btn = ttk.Button(control_frame, text="Clear", command=self._clear_logs)
        clear_btn.grid(row=0, column=7)
        
        # Status bar
        status_frame = ttk.Frame(main_frame)
        status_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))
        
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(status_frame, textvariable=self.status_var)
        status_label.grid(row=0, column=0, sticky=tk.W)
        
        self.count_var = tk.StringVar(value="Messages: 0")
        count_label = ttk.Label(status_frame, textvariable=self.count_var)
        count_label.grid(row=0, column=1, sticky=tk.E)
        status_frame.columnconfigure(0, weight=1)
        
        # Log display
        log_frame = ttk.LabelFrame(main_frame, text="Logs", padding="5")
        log_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Text widget with scrollbar
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            wrap=tk.WORD, 
            width=100, 
            height=30,
            font=("Consolas", 9)
        )
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure text tags for different log levels
        self.log_text.tag_configure("DEBUG", foreground="gray")
        self.log_text.tag_configure("INFO", foreground="black")
        self.log_text.tag_configure("WARN", foreground="orange")
        self.log_text.tag_configure("ERROR", foreground="red")
        self.log_text.tag_configure("FATAL", foreground="red", background="yellow")
        
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
                # Process messages from queue
                while not self.message_queue.empty():
                    msg = self.message_queue.get_nowait()
                    self.log_messages.append(msg)
                    
                    # Limit stored messages
                    if len(self.log_messages) > self.max_displayed_logs:
                        self.log_messages = self.log_messages[-self.max_displayed_logs:]
                
                threading.Event().wait(0.1)  # 100ms delay
                
            except Exception as e:
                print(f"Error processing messages: {e}")
                
    def _update_gui(self):
        """Update GUI with new messages."""
        try:
            # Update log display if needed
            self._refresh_log_display()
            
            # Update status
            total_logs = len(self.log_messages)
            self.count_var.set(f"Messages: {total_logs}")
            
            if total_logs > 0:
                last_msg = self.log_messages[-1]
                timestamp = datetime.fromtimestamp(
                    last_msg.header.stamp.sec + last_msg.header.stamp.nanosec / 1e9
                ).strftime("%H:%M:%S")
                self.status_var.set(f"Last message: {timestamp}")
            
        except Exception as e:
            self.status_var.set(f"Error: {e}")
        
        # Schedule next update
        self.root.after(100, self._update_gui)
        
    def _refresh_log_display(self):
        """Refresh the log display with filtered messages."""
        # Clear current display
        self.log_text.delete('1.0', tk.END)
        
        # Filter messages
        filtered_messages = self._filter_messages()
        
        # Display messages
        for msg in filtered_messages[-500:]:  # Show last 500 filtered messages
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S.%f")[:-3]
            
            # LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
            
            log_line = f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}\n"
            
            # Insert with appropriate tag
            self.log_text.insert(tk.END, log_line, level_name)
        
        # Auto-scroll to bottom if enabled
        if self.auto_scroll_var.get():
            self.log_text.see(tk.END)
            
    def _filter_messages(self) -> List[LogMessage]:
        """Filter messages based on current filter settings."""
        filtered = []
        
        # Level mapping - according to LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
        level_map = {"ERROR": 1, "WARN": 2, "INFO": 3, "DEBUG": 4}
        min_level = level_map.get(self.level_var.get(), 4)
        
        for msg in self.log_messages:
            # Level filter - show messages at min_level and above (lower numbers = higher priority)
            if msg.level > min_level:
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
        
    def _on_auto_scroll_change(self):
        """Handle auto-scroll toggle."""
        self.auto_scroll = self.auto_scroll_var.get()
        
    def _clear_logs(self):
        """Clear all log messages."""
        self.log_messages.clear()
        self.log_text.delete('1.0', tk.END)
        self.status_var.set("Logs cleared")
        
    def add_message(self, msg: LogMessage):
        """Add a message to the queue (thread-safe)."""
        self.message_queue.put(msg)
        
    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


class LogViewer(Node):
    """ROS2 node for log viewing."""
    
    def __init__(self):
        super().__init__('log_viewer')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topics.logs', '/telemetry/logs'),
                ('max_displayed_logs', 1000),
                ('auto_scroll', True),
                ('enable_gui', True),
            ]
        )
        
        # Get parameters
        self.log_topic = self.get_parameter('topics.logs').value
        self.enable_gui = self.get_parameter('enable_gui').value
        
        # Setup QoS - BEST_EFFORT for logs to avoid network congestion
        self.log_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Setup GUI if enabled
        if self.enable_gui:
            self.gui = LogViewerGUI(self)
        else:
            self.gui = None
        
        # Setup subscriber
        self.log_subscriber = self.create_subscription(
            LogMessage,
            self.log_topic,
            self._log_callback,
            self.log_qos
        )
        
        self.get_logger().info(f"LogViewer initialized, subscribing to {self.log_topic}")
        
    def _log_callback(self, msg: LogMessage):
        """Handle incoming log messages."""
        if self.gui:
            self.gui.add_message(msg)
        else:
            # Console output if no GUI
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S.%f")[:-3]
            
            # LogMessage.msg: ERROR=1, WARN=2, INFO=3, DEBUG=4
            level_names = ["UNKNOWN", "ERROR", "WARN", "INFO", "DEBUG"]
            level_name = level_names[msg.level] if 0 < msg.level < len(level_names) else "UNKNOWN"
            
            print(f"[{timestamp}] [{level_name:5}] [{msg.node_name:15}] {msg.message}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = LogViewer()
        
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
