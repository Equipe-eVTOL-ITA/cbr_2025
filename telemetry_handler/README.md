# Telemetry Handler Package

The `telemetry_handler` processes and displays telemetry data from various drone subsystems, providing both RVIZ visualization for spatial data and a comprehensive GUI dashboard for system monitoring.

## Dependencies

- ROS2 (Humble/Iron)
- `custom_msgs` package
- `network_metrics` package (included)
- Python packages: `tkinter`, `psutil`
- Optional: `rosbag2_py` for bag recording


## Telemetry Topics

### Subscribed topics

| Topic | Message Type | Rate | Description | Publisher |
|-------|-------------|------|-------------|-------------|
| `/telemetry/position` | `custom_msgs/Position` | 20Hz | Drone position and velocity in FRD and NED coordinates | `Drone` |
| `/telemetry/drone_status` | `custom_msgs/DroneStatus` | 2Hz | Flight mode, arming state, and battery information | `Drone` |
| `/telemetry/system_health` | `custom_msgs/SystemHealth` | 1Hz | CPU, memory, disk usage, and temperature monitoring | `SystemHealth` |
| `/telemetry/logs` | `custom_msgs/LogMessage` | ~10Hz | System log messages with levels and node information | `FSM` |
| `/telemetry/bases` | `custom_msgs/BaseDetection` | Variable | Detected landing base positions and classifications | `BaseDetector` |
| `/telemetry/camera_debug` | `/telemetry/camera_debug/compressed` | ~3Hz | Compressed image + bbox annotations | `BaseDetector` | 




### Published topics (Visualization)

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/drone/pose` | `geometry_msgs/PoseStamped` | Current drone pose for RVIZ |
| `/drone/path` | `nav_msgs/Path` | Flight path visualization |
| `/drone/twist` | `geometry_msgs/TwistStamped` | Velocity visualization |
| `/base_markers` | `visualization_msgs/MarkerArray` | Landing base markers |

## Modules

### 1. Telemetry Handler (`telemetry_handler.py`)

**Purpose**: RVIZ visualization backend
- Processes position and base detection data
- Converts FRD coordinates to ENU for RVIZ
- Publishes visualization markers for bases and flight path
- Creates colored markers based on base detection confidence

### 2. Telemetry Dashboard (`telemetry_dashboard/`)

**Purpose**: Real-time monitoring GUI
- **Main Node** (`telemetry_dashboard.py`): Coordinates ROS2 subscriptions and GUI
- **GUI Coordinator** (`dashboard_gui.py`): Manages layout and widget coordination  
- **Status Widgets** (`status_widgets.py`): Displays drone status, velocity, system health, and network metrics
- **Log Widgets** (`log_widgets.py`): Advanced log viewer with filtering and search

**Features**:
- Real-time status monitoring (arming, flight mode, battery)
- Velocity monitoring (Vx, Vy, Vz in FRD coordinates + magnitude)
- System health monitoring (CPU, memory, temperature)
- Network performance assessment (ping latency, packet loss, topic health)
- Advanced log filtering (by level, node, search text)
- Color-coded status indicators

### 3. Telemetry Recorder (`telemetry_recorder.py`)

**Purpose**: Data logging and analysis
- Records telemetry data to multiple formats (RosBag, CSV, JSON)
- Configurable recording duration and file size limits
- Thread-safe data writing
- Supports session-based recording with timestamps

**Supported Formats**:
- **RosBag**: Native ROS2 format for playback
- **CSV**: Flattened data for spreadsheet analysis
- **JSON**: Structured data for custom analysis tools

### 4. Network Metrics (`network_metrics/`)

**Purpose**: Network performance monitoring
- Real ping-based latency measurement (default target: 8.8.8.8)
- ROS2 topic health assessment based on expected rates
- Packet loss calculation
- Overall network quality scoring (0-100)
- Independent module for reusability

**Health Assessment**:
- **Excellent** (80-100): Low latency, no packet loss, healthy topics
- **Good** (60-79): Acceptable performance
- **Fair** (40-59): Degraded performance  
- **Poor** (20-39): Significant issues
- **Critical** (0-19): Severe network problems


## Configuration

### Dashboard Configuration
Key parameters in `ground_station.yaml`:
```yaml
telemetry_dashboard:
  ros__parameters:
    network:
      ping_target: "8.8.8.8"  # Network monitoring target
    topics:
      position: "/telemetry/position"
      drone_status: "/telemetry/drone_status"
      system_health: "/telemetry/system_health" 
      logs: "/telemetry/logs"
```

### Recorder Configuration
```yaml
telemetry_recorder:
  ros__parameters:
    recording:
      enable: true
      auto_start_recording: false
      max_duration_minutes: 30
      output_directory: "/tmp/telemetry_logs"
      recording_formats: ["rosbag", "csv"]
```
