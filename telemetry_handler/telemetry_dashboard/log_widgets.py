"""
Log display widgets for the telemetry dashboard.
Handles log message display, filtering, and search functionality.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime
from collections import deque
from typing import Optional, List
import re

from custom_msgs.msg import LogMessage

class LogPanel:
    """Log display panel with filtering and search capabilities."""
    
    # Log level configuration
    LOG_LEVELS = {
        1: {"name": "ERROR", "color": "red"},
        2: {"name": "WARN", "color": "orange"}, 
        3: {"name": "INFO", "color": "blue"},
        4: {"name": "DEBUG", "color": "gray"}
    }
    
    def __init__(self, parent):
        self.parent = parent
        
        # Log storage
        self.log_messages = deque(maxlen=1000)  # Keep last 1000 messages
        self.filtered_messages = []
        
        # Filter state
        self.current_min_level = 1  # Show all levels by default
        self.current_search_text = ""
        self.current_node_filter = ""
        self.auto_scroll_enabled = True
        
        # Thread safety and display coordination
        self._updating_display = False
        self._refresh_scheduled = False
        self._pending_scroll = False    
        self._last_scroll_position = None
        
        self._setup_log_panel()
        
    def _setup_log_panel(self):
        """Set up the log panel UI components."""
        # Create main log frame
        self.log_frame = ttk.Frame(self.parent)
        self.log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control panel
        control_frame = ttk.Frame(self.log_frame)
        control_frame.pack(fill=tk.X, pady=(0, 5))
        
        # Level filter
        ttk.Label(control_frame, text="Min Level:").pack(side=tk.LEFT, padx=(0, 5))
        self.level_var = tk.StringVar(value="DEBUG")
        level_combo = ttk.Combobox(control_frame, textvariable=self.level_var, 
                                 values=["ERROR", "WARN", "INFO", "DEBUG"], 
                                 width=10, state="readonly")
        level_combo.pack(side=tk.LEFT, padx=(0, 10))
        level_combo.bind("<<ComboboxSelected>>", self._on_level_filter_changed)
        
        # Node filter
        ttk.Label(control_frame, text="Node:").pack(side=tk.LEFT, padx=(0, 5))
        self.node_var = tk.StringVar()
        node_entry = ttk.Entry(control_frame, textvariable=self.node_var, width=15)
        node_entry.pack(side=tk.LEFT, padx=(0, 10))
        node_entry.bind("<KeyRelease>", self._on_node_filter_changed)
        
        # Search filter
        ttk.Label(control_frame, text="Search:").pack(side=tk.LEFT, padx=(0, 5))
        self.search_var = tk.StringVar()
        search_entry = ttk.Entry(control_frame, textvariable=self.search_var, width=20)
        search_entry.pack(side=tk.LEFT, padx=(0, 10))
        search_entry.bind("<KeyRelease>", self._on_search_changed)
        
        # Auto-scroll checkbox
        self.auto_scroll_var = tk.BooleanVar(value=True)
        auto_scroll_cb = ttk.Checkbutton(control_frame, text="Auto-scroll", 
                                       variable=self.auto_scroll_var,
                                       command=self._on_auto_scroll_changed)
        auto_scroll_cb.pack(side=tk.LEFT, padx=(10, 0))
        
        # Button frame
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(side=tk.RIGHT)
        
        # Clear button
        clear_btn = ttk.Button(button_frame, text="Clear", command=self._clear_logs, width=8)
        clear_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # Save button
        save_btn = ttk.Button(button_frame, text="Save", command=self._save_logs, width=8)
        save_btn.pack(side=tk.LEFT)
        
        # Log text area with scrollbar
        text_frame = ttk.Frame(self.log_frame)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(
            text_frame,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=("Consolas", 9),
            height=15
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure text tags for colored output
        self.log_text.tag_configure("timestamp", foreground="gray")
        self.log_text.tag_configure("node", foreground="purple")
        self.log_text.tag_configure("ERROR", foreground="red")
        self.log_text.tag_configure("WARN", foreground="orange")
        self.log_text.tag_configure("INFO", foreground="blue")
        self.log_text.tag_configure("DEBUG", foreground="gray")
        
        # Status bar
        status_frame = ttk.Frame(self.log_frame)
        status_frame.pack(fill=tk.X, pady=(5, 0))
        
        self.stats_var = tk.StringVar(value="Messages: 0")
        stats_label = ttk.Label(status_frame, textvariable=self.stats_var)
        stats_label.pack(side=tk.LEFT)
        
    def _message_passes_filters(self, log_entry: dict) -> bool:
        """Check if a log message passes all current filters."""
        # Level filter
        level_map = {"ERROR": 1, "WARN": 2, "INFO": 3, "DEBUG": 4}
        min_level = level_map.get(self.level_var.get(), 1)
        if log_entry['level'] > min_level:
            return False
            
        # Node filter
        if self.current_node_filter and self.current_node_filter.lower() not in log_entry['node_name'].lower():
            return False
            
        # Search filter
        if self.current_search_text:
            search_text = self.current_search_text.lower()
            if (search_text not in log_entry['message'].lower() and 
                search_text not in log_entry['node_name'].lower()):
                return False
                
        return True
    
    def add_message(self, msg: LogMessage):
        """Add a new log message to the display."""
        if msg is None:
            return
            
        # Store the message
        log_entry = {
            'timestamp': datetime.now(),
            'level': msg.level,
            'node_name': getattr(msg, 'node_name', 'Unknown'),
            'message': getattr(msg, 'message', ''),
            'raw_msg': msg
        }
        
        self.log_messages.append(log_entry)
        
        # Only display if message passes current filters and we're not updating
        if not self._updating_display and self._message_passes_filters(log_entry):
            self._display_message(log_entry)
            
        self._update_statistics()
        
    def _display_message(self, log_entry: dict):
        """Display a single log message in the text widget."""
        try:
            # Store current scroll position before any updates
            if not self.auto_scroll_enabled:
                self._last_scroll_position = self.log_text.yview()
            
            self.log_text.configure(state=tk.NORMAL)
            
            # Format timestamp
            timestamp_str = log_entry['timestamp'].strftime("[%H:%M:%S.%f")[:-3] + "]"
            
            # Format level
            level_info = self.LOG_LEVELS.get(log_entry['level'], {"name": "UNKNOWN", "color": "black"})
            level_str = f"[{level_info['name']:5}]"
            
            # Format node name
            node_str = f"[{log_entry['node_name']}]"
            
            # Insert timestamp
            self.log_text.insert(tk.END, timestamp_str, "timestamp")
            self.log_text.insert(tk.END, " ")
            
            # Insert level with color
            self.log_text.insert(tk.END, level_str, level_info["name"])
            self.log_text.insert(tk.END, " ")
            
            # Insert node name
            self.log_text.insert(tk.END, node_str, "node")
            self.log_text.insert(tk.END, " ")
            
            # Insert message
            self.log_text.insert(tk.END, log_entry['message'])
            self.log_text.insert(tk.END, "\n")
            
            self.log_text.configure(state=tk.DISABLED)
            
            # Handle scrolling after text is properly configured
            if self.auto_scroll_enabled:
                self._schedule_auto_scroll()
            elif self._last_scroll_position:
                # Restore previous scroll position
                self.log_text.yview_moveto(self._last_scroll_position[0])
                
        except tk.TclError:
            # Widget might be destroyed, ignore
            pass

    def _schedule_auto_scroll(self):
        """Schedule auto-scroll with proper timing to prevent flickering."""
        if self._pending_scroll:
            return
            
        self._pending_scroll = True
        
        def do_scroll():
            self._pending_scroll = False
            try:
                if self.auto_scroll_enabled and not self._updating_display:
                    self.log_text.see(tk.END)
                    self.log_text.update_idletasks()
            except tk.TclError:
                pass
                
        # Use a minimal delay to ensure text widget is ready
        self.parent.after(10, do_scroll)
        
    def _refresh_display(self):
        """Refresh the entire log display based on current filters."""
        if self._updating_display:
            return
            
        self._updating_display = True
        
        try:
            # Store scroll position if auto-scroll is disabled
            scroll_position = None
            if not self.auto_scroll_enabled:
                scroll_position = self.log_text.yview()
            
            # Clear current display
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.delete(1.0, tk.END)
            
            # Re-display filtered messages
            for log_entry in self.log_messages:
                if self._message_passes_filters(log_entry):
                    self._display_message_batch(log_entry)
            
            self.log_text.configure(state=tk.DISABLED)
            
            # Handle scrolling after all content is loaded
            if self.auto_scroll_enabled:
                self._schedule_auto_scroll()
            elif scroll_position:
                # Restore previous scroll position
                self.log_text.yview_moveto(scroll_position[0])
                
        except tk.TclError:
            # Widget might be destroyed, ignore
            pass
        finally:
            self._updating_display = False
            
        self._update_statistics()
    
    def _display_message_batch(self, log_entry: dict):
        """Display a message during batch refresh (no state changes)."""
        # Format timestamp
        timestamp_str = log_entry['timestamp'].strftime("[%H:%M:%S.%f")[:-3] + "]"
        
        # Format level
        level_info = self.LOG_LEVELS.get(log_entry['level'], {"name": "UNKNOWN", "color": "black"})
        level_str = f"[{level_info['name']:5}]"
        
        # Format node name
        node_str = f"[{log_entry['node_name']:15}]"
        
        # Insert timestamp
        self.log_text.insert(tk.END, timestamp_str, "timestamp")
        self.log_text.insert(tk.END, " ")
        
        # Insert level with color
        self.log_text.insert(tk.END, level_str, level_info["name"])
        self.log_text.insert(tk.END, " ")
        
        # Insert node name
        self.log_text.insert(tk.END, node_str, "node")
        self.log_text.insert(tk.END, " ")
        
        # Insert message
        self.log_text.insert(tk.END, log_entry['message'])
        self.log_text.insert(tk.END, "\n")
    
    # Event handlers with debouncing
    def _on_level_filter_changed(self, event=None):
        """Handle level filter change."""
        self._schedule_refresh()
        
    def _on_node_filter_changed(self, event=None):
        """Handle node filter change."""
        self.current_node_filter = self.node_var.get()
        self._schedule_refresh()
        
    def _on_search_changed(self, event=None):
        """Handle search text change."""
        self.current_search_text = self.search_var.get()
        self._schedule_refresh()
    
    def _schedule_refresh(self):
        """Schedule a refresh with debouncing to prevent flickering."""
        if self._refresh_scheduled:
            return
            
        self._refresh_scheduled = True
        
        def do_refresh():
            self._refresh_scheduled = False
            self._refresh_display()
            
        # Delay refresh by 100ms to debounce rapid filter changes
        self.parent.after(100, do_refresh)
    
    def _on_auto_scroll_changed(self):
        """Handle auto scroll toggle."""
        self.auto_scroll_enabled = self.auto_scroll_var.get()
        if self.auto_scroll_enabled:
            # Clear any pending scroll operations
            self._pending_scroll = False
            # Schedule immediate scroll to end
            self._schedule_auto_scroll()
            
    def _clear_logs(self):
        """Clear all log messages."""
        self._updating_display = True
        try:
            self.log_messages.clear()
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.delete(1.0, tk.END)
            self.log_text.configure(state=tk.DISABLED)
        except tk.TclError:
            pass
        finally:
            self._updating_display = False
        self._update_statistics()
        
    def _update_statistics(self):
        """Update the statistics display."""
        try:
            total_messages = len(self.log_messages)
            filtered_count = sum(1 for msg in self.log_messages if self._message_passes_filters(msg))
            
            if total_messages != filtered_count:
                stats_text = f"Messages: {filtered_count}/{total_messages}"
            else:
                stats_text = f"Messages: {total_messages}"
                
            self.stats_var.set(stats_text)
        except tk.TclError:
            # Widget might be destroyed, ignore
            pass
        
    def _save_logs(self):
        """Save current filtered logs to a file."""
        from tkinter import filedialog
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save Log Messages"
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    for log_entry in self.log_messages:
                        if self._message_passes_filters(log_entry):
                            timestamp_str = log_entry['timestamp'].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            level_info = self.LOG_LEVELS.get(log_entry['level'], {"name": "UNKNOWN"})
                            
                            line = f"[{timestamp_str}] [{level_info['name']:5}] [{log_entry['node_name']:15}] {log_entry['message']}\n"
                            f.write(line)
                            
                # Show success message briefly
                original_text = self.stats_var.get()
                self.stats_var.set(f"Saved to {filename}")
                self.parent.after(3000, lambda: self.stats_var.set(original_text))
                
            except Exception as e:
                # Show error message briefly
                original_text = self.stats_var.get()
                self.stats_var.set(f"Save failed: {str(e)}")
                self.parent.after(3000, lambda: self.stats_var.set(original_text))

class LogMessageFormatter:
    """Utility class for formatting log messages."""
    
    @staticmethod
    def format_for_console(msg: LogMessage) -> str:
        """Format log message for console output."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        level_names = {1: "ERROR", 2: "WARN", 3: "INFO", 4: "DEBUG"}
        level_name = level_names.get(getattr(msg, 'level', 0), "UNKNOWN")
        
        node_name = getattr(msg, 'node_name', 'Unknown')
        message = getattr(msg, 'message', '')
        
        return f"[{timestamp}] [{level_name:5}] [{node_name:15}] {message}"
    
    @staticmethod
    def format_for_file(msg: LogMessage) -> str:
        """Format log message for file output."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        level_names = {1: "ERROR", 2: "WARN", 3: "INFO", 4: "DEBUG"}
        level_name = level_names.get(getattr(msg, 'level', 0), "UNKNOWN")
        
        node_name = getattr(msg, 'node_name', 'Unknown')
        message = getattr(msg, 'message', '')
        
        return f"[{timestamp}] [{level_name:5}] [{node_name:15}] {message}"