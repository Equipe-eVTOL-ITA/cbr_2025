"""
All status monitoring widgets: drone, health, velocity, network.
Designed for easy extension when new topics are added.
"""

import tkinter as tk
from tkinter import ttk
import math
from datetime import datetime
from typing import Optional

from custom_msgs.msg import DroneStatus, SystemHealth, Position
from network_metrics.network_metrics import NetworkMetrics, NetworkHealth

class StatusPanel:
    """Container for all status monitoring widgets."""
    
    def __init__(self, parent, network_metrics: NetworkMetrics):
        self.parent = parent
        self.network_metrics = network_metrics
        
        # Current data storage
        self.current_drone_status: Optional[DroneStatus] = None
        self.current_health_status: Optional[SystemHealth] = None
        self.current_position: Optional[Position] = None
        
        self._setup_scrollable_container()
        self._setup_status_widgets()
        
    def _setup_scrollable_container(self):
        """Setup scrollable container for status panels."""
        self.canvas = tk.Canvas(self.parent)
        self.scrollbar = ttk.Scrollbar(self.parent, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        
    def _setup_status_widgets(self):
        """Setup all status monitoring widgets."""
        self._setup_drone_status_widget()
        self._setup_velocity_widget()     # NEW
        self._setup_health_widget()
        self._setup_network_widget()
        
    def _setup_drone_status_widget(self):
        """Setup drone status monitoring widget."""
        frame = ttk.LabelFrame(self.scrollable_frame, text="🚁 Drone Status", padding=10)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Status variables
        self.arming_state_var = tk.StringVar(value="Unknown")
        self.flight_mode_var = tk.StringVar(value="Unknown") 
        self.battery_voltage_var = tk.StringVar(value="N/A")
        self.battery_status_var = tk.StringVar(value="Unknown")
        self.last_drone_update_var = tk.StringVar(value="Never")
        
        # Layout
        row = 0
        ttk.Label(frame, text="Arming State:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.arming_label = ttk.Label(frame, textvariable=self.arming_state_var, font=('TkDefaultFont', 10, 'bold'))
        self.arming_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Flight Mode:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.flight_mode_label = ttk.Label(frame, textvariable=self.flight_mode_var, font=('TkDefaultFont', 10, 'bold'))
        self.flight_mode_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Battery Voltage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.battery_label = ttk.Label(frame, textvariable=self.battery_voltage_var, font=('TkDefaultFont', 10, 'bold'))
        self.battery_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Battery Status:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.battery_status_label = ttk.Label(frame, textvariable=self.battery_status_var, font=('TkDefaultFont', 10, 'bold'))
        self.battery_status_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Last Update:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(frame, textvariable=self.last_drone_update_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
    def _setup_velocity_widget(self):  # NEW
        """Setup velocity monitoring widget."""
        frame = ttk.LabelFrame(self.scrollable_frame, text="🏃 Velocity (FRD)", padding=10)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Velocity variables
        self.vx_frd_var = tk.StringVar(value="N/A")
        self.vy_frd_var = tk.StringVar(value="N/A")
        self.vz_frd_var = tk.StringVar(value="N/A")
        self.v_frd_var = tk.StringVar(value="N/A")
        self.last_velocity_update_var = tk.StringVar(value="Never")
        
        # Layout
        row = 0
        ttk.Label(frame, text="Vx (Forward):").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.vx_label = ttk.Label(frame, textvariable=self.vx_frd_var, font=('TkDefaultFont', 10, 'bold'))
        self.vx_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Vy (Right):").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.vy_label = ttk.Label(frame, textvariable=self.vy_frd_var, font=('TkDefaultFont', 10, 'bold'))
        self.vy_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Vz (Down):").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.vz_label = ttk.Label(frame, textvariable=self.vz_frd_var, font=('TkDefaultFont', 10, 'bold'))
        self.vz_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="V Total:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.v_total_label = ttk.Label(frame, textvariable=self.v_frd_var, font=('TkDefaultFont', 10, 'bold'))
        self.v_total_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Last Update:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(frame, textvariable=self.last_velocity_update_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
    def _setup_health_widget(self):
        """Setup system health monitoring widget.""" 
        frame = ttk.LabelFrame(self.scrollable_frame, text="💻 System Health", padding=10)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Health variables
        self.cpu_percent_var = tk.StringVar(value="N/A")
        self.memory_percent_var = tk.StringVar(value="N/A")
        self.disk_percent_var = tk.StringVar(value="N/A")
        self.gpu_percent_var = tk.StringVar(value="N/A")
        self.temperature_var = tk.StringVar(value="N/A")
        self.last_health_update_var = tk.StringVar(value="Never")
        
        # Layout
        row = 0
        ttk.Label(frame, text="CPU Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.cpu_label = ttk.Label(frame, textvariable=self.cpu_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.cpu_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="GPU Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.gpu_label = ttk.Label(frame, textvariable=self.gpu_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.gpu_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Memory Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.memory_label = ttk.Label(frame, textvariable=self.memory_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.memory_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Disk Usage:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.disk_label = ttk.Label(frame, textvariable=self.disk_percent_var, font=('TkDefaultFont', 10, 'bold'))
        self.disk_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Temperature:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.temperature_label = ttk.Label(frame, textvariable=self.temperature_var, font=('TkDefaultFont', 10, 'bold'))
        self.temperature_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Last Update:").grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(frame, textvariable=self.last_health_update_var, font=('TkDefaultFont', 8)).grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
    def _setup_network_widget(self):
        """Setup network performance monitoring widget."""
        frame = ttk.LabelFrame(self.scrollable_frame, text="📡 Network Performance", padding=10)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Network variables
        self.network_status_var = tk.StringVar(value="Unknown")
        self.ping_latency_var = tk.StringVar(value="N/A")
        self.packet_loss_var = tk.StringVar(value="N/A")
        self.ros2_health_var = tk.StringVar(value="N/A")
        self.overall_score_var = tk.StringVar(value="N/A")
        
        # Layout
        row = 0
        ttk.Label(frame, text="Status:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.network_status_label = ttk.Label(frame, textvariable=self.network_status_var, font=('TkDefaultFont', 10, 'bold'))
        self.network_status_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Ping Latency:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.ping_label = ttk.Label(frame, textvariable=self.ping_latency_var, font=('TkDefaultFont', 10, 'bold'))
        self.ping_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Packet Loss:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.packet_loss_label = ttk.Label(frame, textvariable=self.packet_loss_var, font=('TkDefaultFont', 10, 'bold'))
        self.packet_loss_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="ROS2 Health:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.ros2_health_label = ttk.Label(frame, textvariable=self.ros2_health_var, font=('TkDefaultFont', 10, 'bold'))
        self.ros2_health_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
        row += 1
        ttk.Label(frame, text="Overall Score:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.overall_score_label = ttk.Label(frame, textvariable=self.overall_score_var, font=('TkDefaultFont', 10, 'bold'))
        self.overall_score_label.grid(row=row, column=1, sticky=tk.W, pady=2, padx=(10, 0))
        
    # Update methods for each data type
    def update_drone_status(self, msg: DroneStatus):
        """Update drone status display."""
        self.current_drone_status = msg
        
        # Map arming state
        arming_states = {1: "DISARMED", 2: "ARMED"}
        arming_state = arming_states.get(msg.arming_state, f"UNKNOWN({msg.arming_state})")
        self.arming_state_var.set(arming_state)
        
        # Color coding
        if msg.arming_state == 2:  # ARMED
            self.arming_label.configure(foreground="red")
        elif msg.arming_state == 1:  # DISARMED 
            self.arming_label.configure(foreground="green")
        else:
            self.arming_label.configure(foreground="orange")
            
        # Map flight mode
        flight_modes = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
            4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "ACRO", 7: "DESCEND",
            8: "TERMINATION", 9: "OFFBOARD", 10: "STAB", 11: "AUTO_TAKEOFF",
            12: "AUTO_LAND", 13: "AUTO_FOLLOW_TARGET", 14: "AUTO_PRECLAND",
            15: "ORBIT", 16: "AUTO_VTOL_TAKEOFF", 17: "UNKNOWN_MODE"
        }
        flight_mode = flight_modes.get(msg.flight_mode, f"UNKNOWN({msg.flight_mode})")
        self.flight_mode_var.set(flight_mode)
        
        # Battery
        self.battery_voltage_var.set(f"{msg.battery_voltage:.2f} V")
        
        # Battery status and color
        if msg.battery_voltage > 15.5:
            status, color = "Excellent", "green"
        elif msg.battery_voltage > 14.8:
            status, color = "Good", "green"
        elif msg.battery_voltage > 14.0:
            status, color = "Fair", "orange"
        elif msg.battery_voltage > 13.0:
            status, color = "Low", "red"
        else:
            status, color = "Critical", "red"
            
        self.battery_status_var.set(status)
        self.battery_label.configure(foreground=color)
        self.battery_status_label.configure(foreground=color)
        
        # Timestamp
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S")
            self.last_drone_update_var.set(timestamp)
            
    def update_velocity(self, msg: Position):  # NEW
        """Update velocity display from position message."""
        self.current_position = msg
        
        # Extract FRD velocities
        vx = msg.vx_frd
        vy = msg.vy_frd  
        vz = msg.vz_frd
        
        # Calculate vector magnitude
        v_total = math.sqrt(vx*vx + vy*vy + vz*vz)
        
        # Update display
        self.vx_frd_var.set(f"{vx:.2f} m/s")
        self.vy_frd_var.set(f"{vy:.2f} m/s")
        self.vz_frd_var.set(f"{vz:.2f} m/s")
        self.v_frd_var.set(f"{v_total:.2f} m/s")
        
        # Color coding based on velocity magnitude
        if v_total > 10.0:
            color = "red"    # High velocity
        elif v_total > 5.0:
            color = "orange" # Medium velocity
        else:
            color = "green"  # Low/safe velocity
            
        self.v_total_label.configure(foreground=color)
        
        # Update timestamp
        self.last_velocity_update_var.set(datetime.now().strftime("%H:%M:%S"))
        
    def update_system_health(self, msg: SystemHealth):
        """Update system health display."""
        self.current_health_status = msg
        
        # CPU
        self.cpu_percent_var.set(f"{msg.cpu_percent:.1f}%")
        cpu_color = "red" if msg.cpu_percent > 90 else "orange" if msg.cpu_percent > 70 else "green"
        self.cpu_label.configure(foreground=cpu_color)
        
        # GPU
        if hasattr(msg, 'gpu_percent'):
            self.gpu_percent_var.set(f"{msg.gpu_percent:.1f}%")
            gpu_color = "red" if msg.gpu_percent > 90 else "orange" if msg.gpu_percent > 70 else "green"
            self.gpu_label.configure(foreground=gpu_color)
        else:
            self.gpu_percent_var.set("N/A")
            self.gpu_label.configure(foreground="gray")
        
        # Memory
        self.memory_percent_var.set(f"{msg.memory_percent:.1f}%")
        mem_color = "red" if msg.memory_percent > 90 else "orange" if msg.memory_percent > 70 else "green"
        self.memory_label.configure(foreground=mem_color)
        
        # Disk
        self.disk_percent_var.set(f"{msg.disk_usage_percent:.1f}%")
        disk_color = "red" if msg.disk_usage_percent > 90 else "orange" if msg.disk_usage_percent > 80 else "green"
        self.disk_label.configure(foreground=disk_color)
        
        # Temperature
        self.temperature_var.set(f"{msg.temperature:.1f}°C")
        temp_color = "red" if msg.temperature > 80 else "orange" if msg.temperature > 70 else "green"
        self.temperature_label.configure(foreground=temp_color)
        
        # Timestamp
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            timestamp = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ).strftime("%H:%M:%S")
            self.last_health_update_var.set(timestamp)
            
    def update_network_status(self, network_health: NetworkHealth):
        """Update network performance display."""
        # Status
        self.network_status_var.set(network_health.status.title())
        
        # Color coding
        status_colors = {
            "excellent": "green", "good": "green", "fair": "orange", 
            "poor": "red", "critical": "red"
        }
        color = status_colors.get(network_health.status, "gray")
        self.network_status_label.configure(foreground=color)
        
        # Metrics
        self.ping_latency_var.set(f"{network_health.ping_latency:.1f} ms")
        self.packet_loss_var.set(f"{network_health.packet_loss:.1f}%")
        self.ros2_health_var.set(f"{network_health.ros2_health:.1f}%")
        self.overall_score_var.set(f"{network_health.overall_score:.1f}%")
        
        # Color labels based on values
        ping_color = "green" if network_health.ping_latency < 100 else "orange" if network_health.ping_latency < 500 else "red"
        self.ping_label.configure(foreground=ping_color)
        
        loss_color = "green" if network_health.packet_loss < 5 else "orange" if network_health.packet_loss < 15 else "red"
        self.packet_loss_label.configure(foreground=loss_color)
        
        ros2_color = "green" if network_health.ros2_health > 80 else "orange" if network_health.ros2_health > 60 else "red"
        self.ros2_health_label.configure(foreground=ros2_color)
        
        score_color = "green" if network_health.overall_score > 80 else "orange" if network_health.overall_score > 60 else "red"
        self.overall_score_label.configure(foreground=score_color)