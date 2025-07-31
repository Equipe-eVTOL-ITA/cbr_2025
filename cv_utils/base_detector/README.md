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


## V2 Detector (Simplified) - `base_detector_v2`

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

## V1 Detector (Multi-Shape) - `base_detector`

### What it does
Advanced detector that identifies 4 distinct shapes on landing pads:
- Yellow cross (0.5m arms) - **Required**
- Yellow circle (0.8m diameter)
- Blue circle (0.7m diameter) 
- Yellow square (1.0m side)

Combines all detected shapes using weighted fusion to create final cross-centered bounding box.

### Key Features
- Multi-shape detection with geometric validation
- Cross detection mandatory for output
- Weighted bounding box fusion
- Rotation-invariant detection
- Template matching for cross validation
- Dual debug visualization

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `yellow_*` / `blue_*` | HSV ranges | Color detection ranges |
| `confidence_threshold` | 0.5 | Minimum detection confidence |
| `cross_*` | Various | Cross detection parameters |
| `circle_*` | Various | Circle detection parameters |
| `square_*` | Various | Square detection parameters |
| `fusion_*` | Various | Weighted combination parameters |

### Weight Parameters
- `fusion_cross_weight`: 0.4 (Cross influence)
- `fusion_yellow_circle_weight`: 0.2 (Yellow circle)
- `fusion_blue_circle_weight`: 0.2 (Blue circle)
- `fusion_yellow_square_weight`: 0.2 (Yellow square)

### Morphological Operations (Shape-Specific)
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yellow_morph_kernel_size` | int | `3` | Kernel size for yellow morphology |
| `yellow_morph_iterations` | int | `2` | Iterations for yellow morphology |
| `blue_morph_kernel_size` | int | `5` | Kernel size for blue morphology |

---



