# Troubleshooting Guide

This guide helps diagnose and resolve common issues with the IGVC autonomous robot system.

## Quick Diagnostics

### System Health Check
```bash
# Check system status
./scripts/health_check.sh

# Check all services
systemctl status igvc-robot
docker-compose ps
ros2 node list
```

### Hardware Detection
```bash
# Check USB devices
lsusb

# Check serial devices
ls -la /dev/tty*

# Check camera devices
ls -la /dev/video*
```

## Common Issues and Solutions

### 1. Hardware Issues

#### Camera Not Detected
**Symptoms:**
- Camera not appearing in device list
- DepthAI initialization fails
- No video feed

**Diagnosis:**
```bash
# Check USB connection
lsusb | grep Luxonis

# Check device permissions
ls -la /dev/video*

# Test camera
python3 test_oakd.py
```

**Solutions:**
```bash
# Install DepthAI drivers
pip3 install depthai

# Check USB 3.0 connection
# Ensure adequate power supply
# Try different USB port

# Check permissions
sudo chmod 666 /dev/video0
```

#### LiDAR Not Responding
**Symptoms:**
- No LiDAR data on `/scan` topic
- Serial port errors
- Device not detected

**Diagnosis:**
```bash
# Check serial port
dmesg | grep ttyUSB

# Test serial communication
sudo minicom -D /dev/ttyUSB0

# Check ROS2 topics
ros2 topic list | grep scan
```

**Solutions:**
```bash
# Install CH341SER driver
cd CH341SER
make && sudo make load

# Set device permissions
sudo chmod 666 /dev/ttyUSB0

# Check baud rate settings
# Verify device path in launch files
```

#### GPS No Signal
**Symptoms:**
- No GPS data on `/gps/fix` topic
- Invalid fix status
- No satellite signals

**Diagnosis:**
```bash
# Check GPS device
ls -la /dev/ttyNEO_F10N

# Test GPS data
cat /dev/ttyNEO_F10N

# Check ROS2 topics
ros2 topic echo /gps/fix
```

**Solutions:**
```bash
# Check antenna connection
# Ensure clear sky view
# Verify baud rate settings
# Check device path in config files

# Test with different baud rates
stty -F /dev/ttyNEO_F10N 38400
```

### 2. Software Issues

#### ROS2 Nodes Not Starting
**Symptoms:**
- Nodes not appearing in `ros2 node list`
- Launch files failing
- Package build errors

**Diagnosis:**
```bash
# Check ROS2 environment
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

# Check workspace
cd ros2_ws
colcon build --packages-select sllidar_ros2

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

**Solutions:**
```bash
# Rebuild workspace
cd ros2_ws
rm -rf build install log
colcon build --symlink-install

# Source setup files
source install/setup.bash

# Check package dependencies
rosdep check --from-paths src
```

#### Python Import Errors
**Symptoms:**
- ModuleNotFoundError
- Import failures
- Package conflicts

**Diagnosis:**
```bash
# Check Python version
python3 --version

# Check installed packages
pip3 list

# Test imports
python3 -c "import depthai; import cv2; import numpy"
```

**Solutions:**
```bash
# Install missing packages
pip3 install -r requirements.txt

# Update packages
pip3 install --upgrade depthai opencv-python

# Check virtual environment
# Use system Python if needed
```

#### Docker Issues
**Symptoms:**
- Containers not starting
- Port conflicts
- Permission errors

**Diagnosis:**
```bash
# Check Docker status
docker ps
docker-compose ps

# Check logs
docker-compose logs

# Check port usage
netstat -tulpn | grep 9001
```

**Solutions:**
```bash
# Restart Docker
sudo systemctl restart docker

# Rebuild containers
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# Check permissions
sudo usermod -a -G docker $USER
```

### 3. Performance Issues

#### High CPU Usage
**Symptoms:**
- System lag
- Slow response
- High temperature

**Diagnosis:**
```bash
# Monitor CPU usage
htop
top

# Check specific processes
ps aux | grep python
ps aux | grep ros2
```

**Solutions:**
```bash
# Optimize camera settings
# Reduce frame rate
# Lower resolution

# Optimize ROS2
export ROS_LOCALHOST_ONLY=1

# Use Jetson optimization
sudo nvpmodel -m 0
sudo jetson_clocks
```

#### Memory Issues
**Symptoms:**
- Out of memory errors
- System crashes
- Slow performance

**Diagnosis:**
```bash
# Check memory usage
free -h
htop

# Check swap usage
swapon -s
```

**Solutions:**
```bash
# Increase swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Optimize memory usage
# Close unnecessary processes
# Reduce buffer sizes
```

#### Network Issues
**Symptoms:**
- Inference server not responding
- Slow network communication
- Connection timeouts

**Diagnosis:**
```bash
# Test network connectivity
ping inference-server
curl http://localhost:9001/health

# Check port usage
netstat -tulpn | grep 9001
```

**Solutions:**
```bash
# Check firewall settings
sudo ufw status

# Restart network services
sudo systemctl restart networking

# Check DNS resolution
nslookup inference-server
```

## Advanced Troubleshooting

### 1. System Logs Analysis
```bash
# Check system logs
sudo journalctl -f

# Check ROS2 logs
ros2 topic echo /rosout

# Check application logs
tail -f /var/log/igvc-robot.log
```

### 2. Performance Profiling
```bash
# Profile Python applications
python3 -m cProfile oakd_web_stream.py

# Monitor ROS2 performance
ros2 topic hz /scan
ros2 topic hz /camera/image_raw

# Check system resources
sudo tegrastats  # For Jetson
```

### 3. Hardware Diagnostics
```bash
# Check USB bandwidth
lsusb -t

# Check serial port settings
stty -F /dev/ttyUSB0 -a

# Check camera capabilities
v4l2-ctl --list-devices
```

## Debugging Tools

### 1. ROS2 Debugging
```bash
# Enable debug logging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Check node information
ros2 node info /sllidar_node

# Monitor topic rates
ros2 topic hz /scan
ros2 topic bw /camera/image_raw
```

### 2. Python Debugging
```python
# Add debug logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Use pdb for debugging
import pdb; pdb.set_trace()

# Check memory usage
import psutil
print(psutil.virtual_memory())
```

### 3. System Monitoring
```bash
# Install monitoring tools
sudo apt install htop iotop nethogs

# Monitor system resources
htop
iotop
nethogs
```

## Recovery Procedures

### 1. System Recovery
```bash
# Restore from backup
tar -xzf igvc-config-backup.tar.gz

# Rebuild system
./setup.sh

# Restart services
sudo systemctl restart igvc-robot
```

### 2. Hardware Recovery
```bash
# Reinstall drivers
cd CH341SER
make clean && make && sudo make load

# Reset device permissions
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyNEO_F10N
```

### 3. Software Recovery
```bash
# Rebuild ROS2 workspace
cd ros2_ws
rm -rf build install log
colcon build --symlink-install

# Reinstall Python packages
pip3 install --force-reinstall -r requirements.txt
```

## Prevention Strategies

### 1. Regular Maintenance
```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Clean up logs
sudo journalctl --vacuum-time=7d

# Check disk space
df -h
```

### 2. Monitoring Setup
```bash
# Set up system monitoring
sudo apt install htop iotop nethogs

# Create monitoring script
cat > monitor.sh << 'EOF'
#!/bin/bash
while true; do
    echo "$(date): CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)"
    echo "$(date): Memory: $(free | grep Mem | awk '{printf "%.2f%%", $3/$2 * 100.0}')"
    sleep 60
done
EOF
chmod +x monitor.sh
```

### 3. Backup Procedures
```bash
# Create backup script
cat > backup.sh << 'EOF'
#!/bin/bash
DATE=$(date +%Y%m%d_%H%M%S)
tar -czf "igvc-backup-$DATE.tar.gz" \
    ros2_ws/src/ \
    *.py \
    *.yaml \
    requirements.txt \
    setup.sh
EOF
chmod +x backup.sh
```

## Getting Help

### 1. Documentation
- Check README.md for basic setup
- Review HARDWARE.md for hardware issues
- Consult SOFTWARE.md for software problems

### 2. Community Support
- Create GitHub issues for bugs
- Check ROS2 community forums
- Consult DepthAI documentation

### 3. Professional Support
- Contact hardware manufacturers
- Consult robotics experts
- Use professional debugging services

## Emergency Procedures

### 1. System Shutdown
```bash
# Graceful shutdown
sudo systemctl stop igvc-robot
docker-compose down
sudo shutdown -h now
```

### 2. Hardware Reset
```bash
# Disconnect all USB devices
# Power cycle system
# Reconnect devices one by one
```

### 3. Software Reset
```bash
# Remove all packages
pip3 uninstall -r requirements.txt

# Rebuild from scratch
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
./setup.sh
```

Remember: Always document issues and solutions for future reference!
