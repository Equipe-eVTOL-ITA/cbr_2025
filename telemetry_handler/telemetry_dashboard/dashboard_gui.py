"""
Main GUI coordinator - handles layout and coordinates between widgets.
"""

import tkinter as tk
from tkinter import ttk
import threading
from typing import Optional

from custom_msgs.msg import LogMessage, DroneStatus, SystemHealth, Position
from network_metrics.network_metrics import NetworkMetrics
from .status_widgets import StatusPanel
from .log_widgets import LogPanel

class TelemetryDashboardGUI:
    """Main GUI coordinator for telemetry dashboard."""
    
    def __init__(self, network_metrics: NetworkMetrics):
        self.network_metrics = network_metrics
        self.root = tk.Tk()
        self.root.title("Drone Telemetry Dashboard")
        self.root.geometry("1000x900")  # Wider for velocity panel
        
        # Initialize panels
        self.status_panel: Optional[StatusPanel] = None
        self.log_panel: Optional[LogPanel] = None
        
        self._setup_layout()
        self._start_updates()
        
    def _setup_layout(self):
        """Setup main GUI layout."""
        # Horizontal split: status panels (left) | logs (right)
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Status displays (150px for velocity addition)
        left_frame = ttk.Frame(main_paned, width=250)
        main_paned.add(left_frame, weight=3)
        
        # Right panel - Logs (750px)  
        right_frame = ttk.Frame(main_paned, width=750)
        main_paned.add(right_frame, weight=2)
        
        # Initialize panels
        self.status_panel = StatusPanel(left_frame, self.network_metrics)
        self.log_panel = LogPanel(right_frame)
        
    def _start_updates(self):
        """Start periodic GUI updates."""
        self._update_network_display()
        self.root.after(1000, self._start_updates)  # Update every second
        
    def _update_network_display(self):
        """Update network performance display."""
        if self.status_panel:
            network_health = self.network_metrics.get_network_health()
            self.status_panel.update_network_status(network_health)
            
    # Message forwarding methods
    def add_log_message(self, msg: LogMessage):
        """Forward log message to log panel."""
        if self.log_panel:
            self.log_panel.add_message(msg)
            
    def update_drone_status(self, msg: DroneStatus):
        """Forward drone status to status panel."""
        if self.status_panel:
            self.status_panel.update_drone_status(msg)
            
    def update_system_health(self, msg: SystemHealth):
        """Forward system health to status panel."""
        if self.status_panel:
            self.status_panel.update_system_health(msg)
            
    def update_velocity(self, msg: Position):  # NEW
        """Forward velocity data to status panel."""
        if self.status_panel:
            self.status_panel.update_velocity(msg)
            
    def run(self):
        """Run GUI main loop."""
        self.root.mainloop()