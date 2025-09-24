# Software Architecture Guide

This document describes the software architecture and components of the IGVC autonomous robot project.

## System Overview

The software system is built on ROS2 (Robot Operating System 2) with the following key components:

- **ROS2 Workspace**: Core robotics framework
- **Computer Vision**: DepthAI OAK-D camera processing
- **Inference Engine**: Roboflow-based object detection
- **Navigation**: LiDAR and GPS integration
- **Web Interface**: Real-time monitoring and control

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    IGVC Robot Software Stack                │
├─────────────────────────────────────────────────────────────┤
│  Web Interface (Flask)                                      │
│  ├── Camera Stream (/stream)                                │
│  ├── Inference Results (/results)                           │
│  └── System Status (/status)                                │
├─────────────────────────────────────────────────────────────┤
│  Application Layer                                          │
│  ├── oakd_web_stream.py (Main camera processing)          │
│  ├── oakd_lane_segmentation.py (Lane detection)             │
│  ├── oakd_roboflow.py (Object detection)                    │
│  └── oakd_local_inference.py (Local inference)              │
├─────────────────────────────────────────────────────────────┤
│  Inference Layer                                            │
│  ├── Roboflow Inference Server (Docker)                    │
│  ├── Local Model Inference                                  │
│  └── Model Management                                       │
├─────────────────────────────────────────────────────────────┤
│  ROS2 Framework                                             │
│  ├── sllidar_ros2 (LiDAR driver)                           │
│  ├── ublox_gps (GPS driver)                                │
│  ├── Custom nodes                                           │
│  └── Message passing                                        │
├─────────────────────────────────────────────────────────────┤
│  Hardware Abstraction Layer                                │
│  ├── DepthAI SDK (Camera)                                  │
│  ├── CH341SER (USB-Serial)                                 │
│  └── Device drivers                                         │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. ROS2 Workspace (`ros2_ws/`)

#### Package Structure:
```
ros2_ws/src/
├── sllidar_ros2/          # LiDAR ROS2 package
│   ├── src/               # Source files
│   ├── launch/             # Launch files
│   ├── config/             # Configuration files
│   └── package.xml          # Package dependencies
└── ublox/                  # GPS packages
    ├── ublox_gps/          # GPS driver
    ├── ublox_msgs/         # GPS message definitions
    └── ublox_serialization/ # GPS serialization
```

#### Key ROS2 Topics:
- `/scan` - LiDAR point cloud data
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/gps/vel` - GPS velocity (geometry_msgs/TwistWithCovarianceStamped)
- `/camera/image_raw` - Camera feed (sensor_msgs/Image)

### 2. Computer Vision Pipeline

#### DepthAI OAK-D Processing:
```python
# Camera pipeline setup
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
```

#### Processing Scripts:
- `oakd_web_stream.py` - Main camera processing with web interface
- `oakd_lane_segmentation.py` - Lane detection and segmentation
- `oakd_roboflow.py` - Object detection using Roboflow
- `oakd_local_inference.py` - Local model inference

### 3. Inference Engine

#### Roboflow Integration:
```python
# Model configuration
MODEL_ID = "barrels-potholes-markers-test"
VERSION = 5
API_KEY = "your-api-key"
INFERENCE_SERVER_URL = f"http://localhost:9001/{MODEL_ID}/{VERSION}"
```

#### Docker Container:
```yaml
# docker-compose.yaml
services:
  offline_tiles:
    image: mapproxy/mapproxy
    network_mode: host
    volumes:
      - ./mapproxy.yaml:/mapproxy/config/mapproxy.yaml:ro
```

### 4. Web Interface

#### Flask Application:
```python
app = Flask(__name__)

@app.route('/stream')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace',
                   headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'})
```

#### Features:
- Real-time camera stream
- Inference results overlay
- System status monitoring
- Configuration interface

## Data Flow

### 1. Camera Data Flow:
```
OAK-D Camera → DepthAI SDK → Python Processing → Flask Web Interface
                    ↓
              Inference Server → Object Detection → Results Overlay
```

### 2. LiDAR Data Flow:
```
RPLidar → CH341SER Driver → sllidar_ros2 → ROS2 Topics → Navigation Stack
```

### 3. GPS Data Flow:
```
u-blox GPS → CH341SER Driver → ublox_gps → ROS2 Topics → Navigation Stack
```

## Configuration Management

### 1. ROS2 Configuration
```yaml
# GPS configuration (neo_f10n.yaml)
device: /dev/ttyNEO_F10N
uart1:
  baudrate: 38400
frame_id: gps
rate: 4
```

### 2. Camera Configuration
```python
# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30
CONFIDENCE_THRESHOLD = 0.5
```

### 3. Inference Configuration
```python
# Model settings
MODEL_ID = "your-model-id"
VERSION = 1
API_KEY = "your-api-key"
INFERENCE_SERVER_URL = "http://localhost:9001"
```

## Development Workflow

### 1. Local Development
```bash
# Setup environment
source setup_env.sh

# Start ROS2 nodes
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
ros2 launch ublox_gps neo_f10n_nmea.launch.py

# Start camera processing
python3 oakd_web_stream.py
```

### 2. Testing
```bash
# Test individual components
python3 test_oakd.py
python3 test_roboflow_api.py
python3 test_local_roboflow.py

# Test ROS2 packages
colcon test --packages-select sllidar_ros2
```

### 3. Deployment
```bash
# Build Docker containers
docker-compose build

# Start services
docker-compose up -d

# Monitor logs
docker-compose logs -f
```

## Performance Optimization

### 1. ROS2 Optimization
```bash
# Set ROS2 parameters
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

# Optimize message passing
# Use shared memory for large messages
# Reduce topic publishing rates
```

### 2. Camera Optimization
```python
# Optimize camera settings
cam_rgb.setPreviewSize(640, 480)  # Reduce resolution
cam_rgb.setFps(30)                # Limit frame rate
cam_rgb.setInterleaved(False)      # Optimize data format
```

### 3. Inference Optimization
```python
# Batch processing
# Model quantization
# GPU acceleration (if available)
```

## Security Considerations

### 1. API Key Management
```python
# Use environment variables
import os
API_KEY = os.getenv('ROBOFLOW_API_KEY')
```

### 2. Network Security
```bash
# Firewall configuration
sudo ufw allow 5000  # Flask web interface
sudo ufw allow 9001  # Inference server
```

### 3. Device Security
```bash
# Secure device permissions
sudo chmod 600 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0
```

## Monitoring and Debugging

### 1. ROS2 Monitoring
```bash
# List topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /scan
ros2 topic hz /camera/image_raw

# Check node status
ros2 node list
ros2 node info /sllidar_node
```

### 2. System Monitoring
```bash
# CPU and memory usage
htop
nvidia-smi  # For Jetson

# Network monitoring
netstat -tulpn
```

### 3. Logging
```python
# Python logging
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
```

## Troubleshooting

### Common Issues:

1. **ROS2 nodes not starting**
   - Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`
   - Source setup files: `source install/setup.bash`

2. **Camera not detected**
   - Check USB connection: `lsusb`
   - Install DepthAI drivers: `pip3 install depthai`

3. **Inference server not responding**
   - Check Docker status: `docker ps`
   - Check logs: `docker-compose logs`

4. **Web interface not accessible**
   - Check Flask process: `ps aux | grep python`
   - Check port availability: `netstat -tulpn | grep 5000`

## Future Enhancements

### Planned Features:
- SLAM integration
- Path planning algorithms
- Advanced sensor fusion
- Machine learning model optimization
- Real-time performance monitoring
- Automated testing framework

### Architecture Improvements:
- Microservices architecture
- Container orchestration (Kubernetes)
- Distributed processing
- Cloud integration
- Edge computing optimization
