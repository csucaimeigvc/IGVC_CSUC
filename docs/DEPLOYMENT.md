# Deployment Guide

This guide covers deploying the IGVC autonomous robot system in various environments.

## Deployment Options

### 1. Local Development
- Single machine setup
- All components on one system
- Direct hardware access

### 2. Distributed System
- Separate inference server
- Remote monitoring
- Network-based communication

### 3. Containerized Deployment
- Docker-based services
- Scalable inference
- Easy deployment

## Prerequisites

### System Requirements
- Ubuntu 20.04/22.04 LTS
- 8GB+ RAM
- 32GB+ storage
- USB 3.0 ports
- Network connectivity

### Hardware Requirements
- NVIDIA Jetson (recommended) or x86_64
- DepthAI OAK-D camera
- RPLidar A1/A2/A3
- u-blox GPS receiver
- USB-serial adapters

## Local Deployment

### 1. System Setup
```bash
# Clone repository
git clone https://github.com/your-username/igvc-robot.git
cd igvc-robot

# Run setup script
chmod +x setup.sh
./setup.sh

# Source environment
source setup_env.sh
```

### 2. Hardware Configuration
```bash
# Install drivers
cd CH341SER
make && sudo make load

# Set device permissions
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyNEO_F10N

# Add user to groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
```

### 3. ROS2 Workspace Setup
```bash
# Build workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Start Services
```bash
# Terminal 1: LiDAR
./start_lidar.sh

# Terminal 2: GPS
ros2 launch ublox_gps neo_f10n_nmea.launch.py

# Terminal 3: Camera
python3 oakd_web_stream.py
```

## Distributed Deployment

### 1. Inference Server Setup
```bash
# On inference server
git clone https://github.com/your-username/igvc-robot.git
cd igvc-robot

# Start inference services
docker-compose up -d

# Check services
docker ps
```

### 2. Robot System Setup
```bash
# On robot system
git clone https://github.com/your-username/igvc-robot.git
cd igvc-robot

# Configure for remote inference
# Edit camera scripts to use remote inference server
INFERENCE_SERVER_URL = "http://inference-server:9001"
```

### 3. Network Configuration
```bash
# Configure network
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Set inference server IP
export INFERENCE_SERVER_IP="192.168.1.100"
```

## Containerized Deployment

### 1. Docker Setup
```bash
# Build containers
docker-compose build

# Start services
docker-compose up -d

# Check status
docker-compose ps
```

### 2. Service Configuration
```yaml
# docker-compose.yaml
version: "3.8"
services:
  inference-server:
    build: .
    ports:
      - "9001:9001"
    volumes:
      - ./models:/models
    environment:
      - ROBOFLOW_API_KEY=${ROBOFLOW_API_KEY}
  
  web-interface:
    build: .
    ports:
      - "5000:5000"
    depends_on:
      - inference-server
    volumes:
      - ./output_frames:/output_frames
```

### 3. Production Deployment
```bash
# Use production compose file
docker-compose -f docker-compose.prod.yml up -d

# Enable auto-restart
docker-compose up -d --restart unless-stopped
```

## Systemd Service Deployment

### 1. Create Service File
```bash
# Copy service template
sudo cp igvc-robot.service /etc/systemd/system/

# Edit service file
sudo nano /etc/systemd/system/igvc-robot.service
```

### 2. Configure Service
```ini
[Unit]
Description=IGVC Robot Service
After=network.target

[Service]
Type=simple
User=igvc
WorkingDirectory=/home/igvc
ExecStart=/bin/bash -c 'source setup_env.sh && python3 oakd_web_stream.py'
Restart=always
RestartSec=10
Environment=ROS_DOMAIN_ID=0

[Install]
WantedBy=multi-user.target
```

### 3. Enable Service
```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service
sudo systemctl enable igvc-robot

# Start service
sudo systemctl start igvc-robot

# Check status
sudo systemctl status igvc-robot
```

## Monitoring and Logging

### 1. System Monitoring
```bash
# Install monitoring tools
sudo apt install htop iotop nethogs

# Monitor system resources
htop
nvidia-smi  # For Jetson
```

### 2. Application Logging
```bash
# View service logs
sudo journalctl -u igvc-robot -f

# View Docker logs
docker-compose logs -f

# View ROS2 logs
ros2 topic echo /rosout
```

### 3. Performance Monitoring
```bash
# Monitor ROS2 topics
ros2 topic hz /scan
ros2 topic hz /camera/image_raw

# Monitor system performance
sudo tegrastats  # For Jetson
```

## Backup and Recovery

### 1. Configuration Backup
```bash
# Backup configuration files
tar -czf igvc-config-backup.tar.gz \
  ros2_ws/src/ \
  *.yaml \
  *.py \
  requirements.txt
```

### 2. System Backup
```bash
# Create system image
sudo dd if=/dev/mmcblk0 of=igvc-system-backup.img bs=4M
```

### 3. Recovery Process
```bash
# Restore from backup
tar -xzf igvc-config-backup.tar.gz

# Rebuild system
./setup.sh
```

## Security Considerations

### 1. Network Security
```bash
# Configure firewall
sudo ufw enable
sudo ufw allow 22    # SSH
sudo ufw allow 5000  # Web interface
sudo ufw allow 9001  # Inference server
```

### 2. Device Security
```bash
# Secure device permissions
sudo chmod 600 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable cups
```

### 3. API Security
```bash
# Use environment variables for API keys
export ROBOFLOW_API_KEY="your-secure-key"

# Rotate API keys regularly
# Use HTTPS for web interfaces
```

## Troubleshooting

### Common Deployment Issues

1. **Services not starting**
   ```bash
   # Check service status
   sudo systemctl status igvc-robot
   
   # Check logs
   sudo journalctl -u igvc-robot -f
   ```

2. **Hardware not detected**
   ```bash
   # Check device connections
   lsusb
   ls -la /dev/tty*
   
   # Check driver status
   lsmod | grep ch34x
   ```

3. **Network connectivity issues**
   ```bash
   # Test network connectivity
   ping inference-server
   curl http://inference-server:9001/health
   ```

4. **Performance issues**
   ```bash
   # Monitor system resources
   htop
   nvidia-smi
   
   # Check ROS2 performance
   ros2 topic hz /scan
   ```

## Maintenance

### 1. Regular Maintenance
```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Update Python packages
pip3 install --upgrade -r requirements.txt

# Clean up logs
sudo journalctl --vacuum-time=7d
```

### 2. Hardware Maintenance
- Clean camera lenses
- Check LiDAR mounting
- Verify GPS antenna connection
- Monitor power consumption

### 3. Software Maintenance
- Update ROS2 packages
- Update inference models
- Monitor system performance
- Backup configurations

## Scaling Considerations

### 1. Horizontal Scaling
- Multiple inference servers
- Load balancing
- Distributed processing

### 2. Vertical Scaling
- More powerful hardware
- GPU acceleration
- Memory optimization

### 3. Cloud Integration
- Cloud-based inference
- Remote monitoring
- Data synchronization

## Production Checklist

- [ ] Hardware properly installed and tested
- [ ] All drivers installed and working
- [ ] ROS2 workspace built successfully
- [ ] All services starting correctly
- [ ] Network connectivity verified
- [ ] Security measures implemented
- [ ] Monitoring and logging configured
- [ ] Backup procedures in place
- [ ] Documentation updated
- [ ] Team trained on system operation
