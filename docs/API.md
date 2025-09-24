# API Documentation

This document describes the APIs and interfaces used in the IGVC autonomous robot system.

## ROS2 Topics

### LiDAR Data
- **Topic**: `/scan`
- **Type**: `sensor_msgs/LaserScan`
- **Description**: LiDAR point cloud data from RPLidar
- **Frequency**: 5.5Hz (A1), 10Hz (A2), 15Hz (A3)

```python
# Subscribe to LiDAR data
import rclpy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    ranges = msg.ranges
    angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]
    # Process scan data
```

### GPS Data
- **Topic**: `/gps/fix`
- **Type**: `sensor_msgs/NavSatFix`
- **Description**: GPS position data
- **Frequency**: 4Hz

```python
# Subscribe to GPS data
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude
    # Process GPS data
```

- **Topic**: `/gps/vel`
- **Type**: `geometry_msgs/TwistWithCovarianceStamped`
- **Description**: GPS velocity data
- **Frequency**: 4Hz

### Camera Data
- **Topic**: `/camera/image_raw`
- **Type**: `sensor_msgs/Image`
- **Description**: Camera image data
- **Frequency**: 30Hz

```python
# Subscribe to camera data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Process image data
```

## Web API Endpoints

### Flask Web Interface

#### Camera Stream
- **Endpoint**: `/stream`
- **Method**: GET
- **Description**: Real-time camera stream with inference overlay
- **Response**: Multipart HTTP stream

```python
@app.route('/stream')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace',
                   headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'})
```

#### Inference Results
- **Endpoint**: `/results`
- **Method**: GET
- **Description**: Latest inference results
- **Response**: JSON

```json
{
    "detections": [
        {
            "class": "barrel",
            "confidence": 0.95,
            "bbox": [100, 150, 200, 300]
        }
    ],
    "timestamp": "2024-01-01T12:00:00Z"
}
```

#### System Status
- **Endpoint**: `/status`
- **Method**: GET
- **Description**: System health and status
- **Response**: JSON

```json
{
    "camera": "connected",
    "lidar": "connected",
    "gps": "connected",
    "inference": "running",
    "timestamp": "2024-01-01T12:00:00Z"
}
```

## Roboflow Inference API

### Local Inference Server
- **Base URL**: `http://localhost:9001`
- **Authentication**: API key in headers

#### Object Detection
- **Endpoint**: `/{model_id}/{version}`
- **Method**: POST
- **Content-Type**: `application/x-www-form-urlencoded`

```python
# Request format
data = {
    'api_key': 'your-api-key',
    'image': base64_encoded_image
}

response = requests.post(
    f"{INFERENCE_SERVER_URL}",
    data=data,
    headers={'Content-Type': 'application/x-www-form-urlencoded'}
)
```

#### Response Format
```json
{
    "predictions": [
        {
            "class": "barrel",
            "confidence": 0.95,
            "x": 100,
            "y": 150,
            "width": 100,
            "height": 150
        }
    ],
    "image": {
        "width": 640,
        "height": 480
    }
}
```

### Lane Segmentation
- **Endpoint**: `/{model_id}/{version}`
- **Method**: POST
- **Description**: Lane detection and segmentation

```json
{
    "predictions": [
        {
            "class": "lane",
            "confidence": 0.98,
            "segmentation_mask": "base64_encoded_mask"
        }
    ]
}
```

## DepthAI Camera API

### Camera Configuration
```python
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Configure camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
cam_rgb.setFps(30)
```

### Camera Controls
```python
# Set camera properties
cam_rgb.setBrightness(0)
cam_rgb.setContrast(0)
cam_rgb.setSaturation(0)
cam_rgb.setSharpness(0)
cam_rgb.setLumaDenoise(0)
cam_rgb.setChromaDenoise(0)
```

### Depth Data
```python
# Configure depth
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.StereoDepthProperties.Preset.HighAccuracy)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
```

## ROS2 Service APIs

### LiDAR Services
- **Service**: `/sllidar/start_motor`
- **Type**: `std_srvs/Empty`
- **Description**: Start LiDAR motor

- **Service**: `/sllidar/stop_motor`
- **Type**: `std_srvs/Empty`
- **Description**: Stop LiDAR motor

```python
# Call LiDAR service
from std_srvs.srv import Empty

def start_lidar():
    client = node.create_client(Empty, '/sllidar/start_motor')
    request = Empty.Request()
    future = client.call_async(request)
```

### GPS Services
- **Service**: `/gps/configure`
- **Type**: `ublox_msgs/CfgPRT`
- **Description**: Configure GPS settings

```python
# Configure GPS
from ublox_msgs.srv import CfgPRT

def configure_gps():
    client = node.create_client(CfgPRT, '/gps/configure')
    request = CfgPRT.Request()
    request.portID = 1
    request.baudrate = 38400
    future = client.call_async(request)
```

## Configuration APIs

### ROS2 Parameters
```python
# Set parameters
node.declare_parameter('camera_width', 640)
node.declare_parameter('camera_height', 480)
node.declare_parameter('confidence_threshold', 0.5)

# Get parameters
width = node.get_parameter('camera_width').get_parameter_value().integer_value
height = node.get_parameter('camera_height').get_parameter_value().integer_value
```

### YAML Configuration
```yaml
# GPS configuration
device: /dev/ttyNEO_F10N
uart1:
  baudrate: 38400
frame_id: gps
rate: 4

# LiDAR configuration
serial_port: '/dev/ttyUSB0'
serial_baudrate: 115200
frame_id: 'laser'
```

## Data Processing APIs

### Image Processing
```python
import cv2
import numpy as np

def process_image(image):
    # Resize image
    resized = cv2.resize(image, (640, 480))
    
    # Apply filters
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    
    # Convert to different color spaces
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    return processed_image
```

### Inference Processing
```python
def process_inference(image, results):
    # Draw bounding boxes
    for detection in results['predictions']:
        x, y, w, h = detection['bbox']
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(image, f"{detection['class']}: {detection['confidence']:.2f}",
                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return image
```

## Error Handling

### ROS2 Error Handling
```python
try:
    # ROS2 operations
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
except Exception as e:
    node.get_logger().error(f"Service call failed: {e}")
```

### HTTP Error Handling
```python
try:
    response = requests.post(url, data=data, timeout=10)
    response.raise_for_status()
    return response.json()
except requests.exceptions.RequestException as e:
    print(f"Request failed: {e}")
    return None
```

### Camera Error Handling
```python
try:
    with dai.Device(pipeline) as device:
        # Camera operations
        pass
except RuntimeError as e:
    print(f"Camera error: {e}")
```

## Performance Monitoring

### Topic Monitoring
```python
# Monitor topic rates
ros2 topic hz /scan
ros2 topic hz /camera/image_raw

# Monitor topic bandwidth
ros2 topic bw /camera/image_raw
```

### System Monitoring
```python
import psutil

def monitor_system():
    cpu_percent = psutil.cpu_percent()
    memory_percent = psutil.virtual_memory().percent
    disk_percent = psutil.disk_usage('/').percent
    
    return {
        'cpu': cpu_percent,
        'memory': memory_percent,
        'disk': disk_percent
    }
```

## Security Considerations

### API Key Management
```python
import os

# Use environment variables
API_KEY = os.getenv('ROBOFLOW_API_KEY')
if not API_KEY:
    raise ValueError("ROBOFLOW_API_KEY not set")
```

### Input Validation
```python
def validate_image(image):
    if image is None:
        raise ValueError("Image is None")
    if image.shape[0] == 0 or image.shape[1] == 0:
        raise ValueError("Image has zero dimensions")
    return True
```

### Rate Limiting
```python
import time

class RateLimiter:
    def __init__(self, max_calls, time_window):
        self.max_calls = max_calls
        self.time_window = time_window
        self.calls = []
    
    def can_call(self):
        now = time.time()
        self.calls = [call for call in self.calls if now - call < self.time_window]
        return len(self.calls) < self.max_calls
    
    def record_call(self):
        self.calls.append(time.time())
```

## Testing APIs

### Unit Testing
```python
import unittest
from unittest.mock import Mock, patch

class TestCameraAPI(unittest.TestCase):
    def test_camera_initialization(self):
        with patch('depthai.Device') as mock_device:
            # Test camera initialization
            pass
    
    def test_image_processing(self):
        # Test image processing functions
        pass
```

### Integration Testing
```python
def test_ros2_integration():
    # Test ROS2 topic publishing
    # Test service calls
    # Test parameter setting
    pass

def test_web_api_integration():
    # Test web endpoints
    # Test inference API
    # Test error handling
    pass
```

## Documentation Standards

### API Documentation
- Use clear, descriptive names
- Provide examples for all endpoints
- Document error conditions
- Include performance characteristics

### Code Documentation
```python
def process_inference(image, model_id, version):
    """
    Process image through inference model.
    
    Args:
        image (numpy.ndarray): Input image
        model_id (str): Model identifier
        version (int): Model version
    
    Returns:
        dict: Inference results
    
    Raises:
        ValueError: If image is invalid
        requests.RequestException: If API call fails
    """
    pass
```

This API documentation provides comprehensive information about all interfaces and endpoints used in the IGVC autonomous robot system.
