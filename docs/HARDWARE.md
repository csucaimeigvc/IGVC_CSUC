# Hardware Setup Guide

This document provides detailed information about the hardware components used in the IGVC autonomous robot project.

## Required Hardware

### 1. Computing Platform
- **Recommended**: NVIDIA Jetson Xavier NX or AGX Orin
- **Alternative**: x86_64 system with NVIDIA GPU
- **Minimum**: 8GB RAM, 32GB storage

### 2. DepthAI OAK-D Camera
- **Model**: OAK-D or OAK-D-Lite
- **Interface**: USB 3.0
- **Features**: RGB + Stereo depth + IMU
- **Power**: 5V via USB

#### Setup:
```bash
# Test camera detection
python3 test_oakd.py

# Check USB connection
lsusb | grep Luxonis
```

### 3. RPLidar
- **Model**: RPLidar A1/A2/A3
- **Interface**: USB-Serial (CH341SER)
- **Range**: 12m (A1), 18m (A2), 25m (A3)
- **Scan Rate**: 5.5Hz (A1), 10Hz (A2), 15Hz (A3)

#### Setup:
```bash
# Check device
ls -la /dev/ttyUSB*

# Test connection
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

### 4. u-blox GPS Receiver
- **Model**: NEO-F10N or ZED-F9P
- **Interface**: USB-Serial
- **Features**: Multi-GNSS, RTK support
- **Accuracy**: 2.5m (standard), 1cm (RTK)

#### Setup:
```bash
# Check GPS device
ls -la /dev/ttyNEO_F10N

# Test GPS data
cat /dev/ttyNEO_F10N
```

### 5. USB-Serial Adapters
- **Model**: CH341SER compatible
- **Purpose**: LiDAR and GPS communication
- **Driver**: CH341SER (included)

## Hardware Connections

### Power Distribution
```
Battery (12V) → Power Distribution Board
├── Jetson (12V → 5V converter)
├── OAK-D Camera (5V via USB)
├── RPLidar (5V via USB)
└── GPS (5V via USB)
```

### USB Connections
```
Jetson USB Ports:
├── USB 3.0 → OAK-D Camera
├── USB 2.0 → RPLidar (via CH341SER)
└── USB 2.0 → GPS (via CH341SER)
```

## Device Configuration

### 1. OAK-D Camera Configuration
```python
# In camera scripts
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
```

### 2. RPLidar Configuration
```yaml
# In launch files
serial_port: '/dev/ttyUSB0'
serial_baudrate: 115200
frame_id: 'laser'
inverted: false
angle_compensate: true
```

### 3. GPS Configuration
```yaml
# In ublox config files
device: /dev/ttyNEO_F10N
uart1:
  baudrate: 38400
frame_id: gps
rate: 4
```

## Troubleshooting

### Common Hardware Issues

1. **Camera not detected**
   ```bash
   # Check USB connection
   lsusb
   # Check device permissions
   ls -la /dev/video*
   ```

2. **LiDAR not responding**
   ```bash
   # Check serial port
   dmesg | grep ttyUSB
   # Test with minicom
   sudo minicom -D /dev/ttyUSB0
   ```

3. **GPS no signal**
   ```bash
   # Check antenna connection
   # Ensure clear sky view
   # Check baud rate settings
   ```

4. **Power issues**
   ```bash
   # Check power supply
   # Monitor voltage levels
   # Check for loose connections
   ```

### Device Permissions
```bash
# Add user to required groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
sudo usermod -a -G plugdev $USER

# Set device permissions
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyNEO_F10N
```

## Performance Optimization

### Jetson Optimization
```bash
# Set maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor performance
sudo tegrastats
```

### Camera Optimization
- Use USB 3.0 ports for OAK-D
- Ensure adequate power supply
- Check for USB bandwidth issues

### LiDAR Optimization
- Use dedicated USB port
- Check for electromagnetic interference
- Ensure stable mounting

## Safety Considerations

1. **Power Safety**
   - Use appropriate fuses
   - Check voltage levels
   - Ensure proper grounding

2. **Mechanical Safety**
   - Secure all components
   - Use appropriate mounting hardware
   - Check for loose connections

3. **Environmental Safety**
   - Protect from moisture
   - Ensure adequate ventilation
   - Use appropriate enclosures

## Maintenance

### Regular Checks
- Clean camera lenses
- Check LiDAR mounting
- Verify GPS antenna connection
- Monitor power consumption

### Calibration
- Camera calibration (if needed)
- LiDAR mounting alignment
- GPS antenna positioning

## Specifications Summary

| Component | Power | Interface | Range/Accuracy |
|-----------|-------|-----------|----------------|
| Jetson | 12V, 20W | USB, Ethernet | - |
| OAK-D | 5V, 1.5W | USB 3.0 | 0.1-3m depth |
| RPLidar A1 | 5V, 1.2W | USB-Serial | 12m, 5.5Hz |
| GPS | 5V, 0.1W | USB-Serial | 2.5m accuracy |
