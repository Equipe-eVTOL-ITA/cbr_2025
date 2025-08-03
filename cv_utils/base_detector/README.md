# Base Detector - CBR 2025 Landing Pad Detection

Two ROS 2 detectors for identifying landing pads in drone camera feeds.

## ROS 2 Interface (Both Detectors)

### Topics
- **Input**: `image_topic` (sensor_msgs/Image)
- **Output**: `detection_topic` (vision_msgs/Detection2DArray)
- **Debug**: `mask_debug_topic`, `bbox_debug_topic` (sensor_msgs/Image)

### Launch
```bash
ros2 launch cbr_fase1 fase1.launch.py
```

If you want only computer vision,  don't run the MicroXRCE-DDS Agent, so px4 can't  communicate with ROS 2. Debug can be done in RVIZ, which opens with the launch file.


## Base Detector

### What it does
Detects landing pads by finding regions containing both yellow and blue pixels (indicating the yellow square with inner blue circle). Uses color segmentation and dilation to connect adjacent colored regions.

### Key Features
- Simple single-detection approach
- Looks for yellow+blue pixel combinations
- Fixed confidence score (1.0)
- Area and aspect ratio validation
- Fast and lightweight


### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `yellow_*` | HSV ranges | Yellow color detection |
| `blue_*` | HSV ranges | Blue color detection |
| `min_area` | 500 | Minimum detection area |
| `max_area` | 50000 | Maximum detection area |
| `min_aspect_ratio` | 0.5 | Minimum width/height ratio |
| `max_aspect_ratio` | 2.0 | Maximum width/height ratio |
| `dilation_kernel_size` | 5 | Kernel for connecting regions |


---


