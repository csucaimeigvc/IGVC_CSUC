# IGVC Autonomous Robot Project

An autonomous ground vehicle project for the Intelligent Ground Vehicle Competition (IGVC) featuring computer vision, LiDAR navigation, GPS positioning, and real-time object detection.

## 🚀 Features

- **Computer Vision**: DepthAI OAK-D camera with lane segmentation and object detection
- **LiDAR Navigation**: RPLidar integration for obstacle detection and mapping
- **GPS Positioning**: u-blox GPS with RTK support for precise navigation
- **Real-time Inference**: Roboflow-based object detection and lane segmentation
- **ROS2 Integration**: Complete ROS2 workspace with custom packages
- **Docker Support**: Containerized inference server for scalable deployment

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   DepthAI OAK-D │    │   RPLidar A1    │    │  u-blox GPS     │
│   (Vision)      │    │   (LiDAR)       │    │  (Navigation)   │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌─────────────┴─────────────┐
                    │      ROS2 Workspace       │
                    │  - sllidar_ros2           │
                    │  - ublox_gps              │
                    │  - Custom nodes           │
                    └─────────────┬─────────────┘
                                 │
                    ┌─────────────┴─────────────┐
                    │   Inference Server        │
                    │   (Roboflow/Docker)       │
                    └───────────────────────────┘
```

## 📋 Prerequisites

### Hardware Requirements
- NVIDIA Jetson (recommended) or x86_64 system
- DepthAI OAK-D camera
- RPLidar A1/A2/A3
- u-blox GPS receiver (NEO-F10N or similar)
- USB-serial adapters (CH341SER compatible)

### Software Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.8+
- Docker and Docker Compose
- NVIDIA drivers (for Jetson)

## 🛠️ Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-username/igvc-robot.git
cd igvc-robot
```

### 2. Install System Dependencies
```bash
# Install ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool -y

# Install Python dependencies
sudo apt install python3-pip python3-opencv python3-numpy python3-flask -y

# Install Docker
sudo apt install docker.io docker-compose -y
sudo usermod -aG docker $USER
```

### 3. Install Hardware Drivers

#### CH341SER Driver (USB-Serial)
```bash
cd CH341SER
make
sudo make load
sudo cp ch34x.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo depmod -a
```

#### DepthAI SDK
```bash
# For Jetson
pip3 install depthai

# For x86_64
pip3 install depthai[examples]
```

### 4. Setup ROS2 Workspace
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Install Python Dependencies
```bash
pip3 install -r requirements.txt
```

## 🚀 Quick Start

### 1. Start the Inference Server
```bash
# Start Roboflow inference server
docker-compose up -d
```

### 2. Launch ROS2 Nodes
```bash
# Terminal 1: Start LiDAR
./start_lidar.sh

# Terminal 2: Start GPS
ros2 launch ublox_gps neo_f10n_nmea.launch.py

# Terminal 3: Start camera processing
python3 oakd_web_stream.py
```

### 3. View Data
```bash
# List available topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /scan
ros2 topic echo /gps/fix
```

## 📁 Project Structure

```
igvc-robot/
├── ros2_ws/                    # ROS2 workspace
│   ├── src/
│   │   ├── sllidar_ros2/       # LiDAR ROS2 package
│   │   └── ublox/              # GPS ROS2 packages
│   ├── build/                  # Build directory
│   ├── install/                # Install directory
│   └── log/                    # Build logs
├── CH341SER/                   # USB-serial driver
├── depthai-python/             # DepthAI SDK
├── oakd_*.py                   # Camera processing scripts
├── test_*.py                   # Test scripts
├── docker-compose.yaml         # Docker configuration
├── Dockerfile.roboflow         # Roboflow inference container
├── requirements.txt            # Python dependencies
├── start_lidar.sh             # LiDAR startup script
└── README.md                   # This file
```

## 🔧 Configuration

### GPS Configuration
Edit `ros2_ws/src/ublox/ublox_gps/config/neo_f10n.yaml`:
```yaml
device: /dev/ttyNEO_F10N
uart1:
  baudrate: 38400
frame_id: gps
rate: 4
```

### LiDAR Configuration
Edit `ros2_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py`:
```python
serial_port: '/dev/ttyUSB0'
serial_baudrate: 115200
frame_id: 'laser'
```

### Camera Configuration
Edit camera scripts to adjust:
- Model IDs and API keys
- Confidence thresholds
- Output directories
- Stream settings

## 🧪 Testing

### Test Individual Components
```bash
# Test camera
python3 test_oakd.py

# Test Roboflow API
python3 test_roboflow_api.py

# Test local inference
python3 test_local_roboflow.py
```

### Test ROS2 Packages
```bash
# Build and test
cd ros2_ws
colcon build --packages-select sllidar_ros2
colcon test --packages-select sllidar_ros2
```

## 📊 Performance Monitoring

### ROS2 Topics
- `/scan` - LiDAR data
- `/gps/fix` - GPS position
- `/gps/vel` - GPS velocity
- `/camera/image_raw` - Camera feed

### Web Interfaces
- Camera stream: `http://localhost:5000`
- Inference results: `http://localhost:5000/stream`

## 🐛 Troubleshooting

### Common Issues

1. **Permission denied on /dev/ttyUSB0**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # Or add user to dialout group
   sudo usermod -a -G dialout $USER
   ```

2. **GPS not connecting**
   - Check baud rate settings
   - Verify device path: `ls /dev/tty*`
   - Test with: `cat /dev/ttyNEO_F10N`

3. **Camera not detected**
   - Check USB connection
   - Install DepthAI drivers
   - Test with: `python3 test_oakd.py`

4. **ROS2 nodes not starting**
   - Source setup files: `source install/setup.bash`
   - Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`

### Debug Commands
```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /scan

# Check device permissions
ls -la /dev/tty*

# Monitor system resources
htop
nvidia-smi  # For Jetson
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [DepthAI](https://docs.luxonis.com/) for OAK-D camera support
- [Roboflow](https://roboflow.com/) for inference infrastructure
- [Slamtec](https://www.slamtec.com/) for RPLidar drivers
- [u-blox](https://www.u-blox.com/) for GPS solutions
- ROS2 community for excellent robotics framework

## 📞 Support

For questions and support:
- Create an issue on GitHub
- Check the troubleshooting section
- Review ROS2 and hardware documentation

---

**Note**: This project is designed for educational and competition purposes. Always follow safety guidelines when operating autonomous vehicles.
