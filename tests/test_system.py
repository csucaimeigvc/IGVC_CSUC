#!/usr/bin/env python3

"""
IGVC Robot System Test Suite
This module contains comprehensive tests for the IGVC autonomous robot system.
"""

import unittest
import sys
import os
import time
import subprocess
import requests
import json
from unittest.mock import Mock, patch, MagicMock

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class TestSystemDependencies(unittest.TestCase):
    """Test system dependencies and requirements."""
    
    def test_python_version(self):
        """Test Python version compatibility."""
        self.assertGreaterEqual(sys.version_info, (3, 8), "Python 3.8+ required")
    
    def test_required_modules(self):
        """Test required Python modules are available."""
        required_modules = [
            'cv2', 'numpy', 'flask', 'requests', 
            'depthai', 'roboflow', 'PIL'
        ]
        
        for module in required_modules:
            with self.subTest(module=module):
                try:
                    __import__(module)
                except ImportError:
                    self.fail(f"Required module {module} not found")
    
    def test_system_commands(self):
        """Test required system commands are available."""
        required_commands = ['python3', 'pip3', 'docker', 'ros2']
        
        for command in required_commands:
            with self.subTest(command=command):
                result = subprocess.run(['which', command], 
                                     capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, 
                               f"Command {command} not found")

class TestHardwareDetection(unittest.TestCase):
    """Test hardware detection and connectivity."""
    
    def test_camera_detection(self):
        """Test camera hardware detection."""
        try:
            import depthai as dai
            devices = dai.Device.getAllAvailableDevices()
            self.assertGreater(len(devices), 0, "No DepthAI devices found")
        except ImportError:
            self.skipTest("DepthAI not available")
        except Exception as e:
            self.fail(f"Camera detection failed: {e}")
    
    def test_serial_devices(self):
        """Test serial device detection."""
        import glob
        serial_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyNEO_F10N')
        # Note: This test may fail if hardware is not connected
        # This is expected in CI/CD environments
        if len(serial_devices) == 0:
            self.skipTest("No serial devices found (hardware not connected)")
    
    def test_usb_devices(self):
        """Test USB device detection."""
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, "lsusb command failed")
        
        # Check for specific hardware
        usb_output = result.stdout.lower()
        hardware_found = any(keyword in usb_output for keyword in [
            'luxonis', 'ch341', 'u-blox'
        ])
        
        if not hardware_found:
            self.skipTest("IGVC hardware not detected")

class TestROS2Integration(unittest.TestCase):
    """Test ROS2 integration and functionality."""
    
    def setUp(self):
        """Set up ROS2 environment for testing."""
        try:
            import rclpy
            rclpy.init()
            self.node = rclpy.create_node('test_node')
        except ImportError:
            self.skipTest("ROS2 not available")
        except Exception as e:
            self.fail(f"ROS2 initialization failed: {e}")
    
    def tearDown(self):
        """Clean up ROS2 resources."""
        if hasattr(self, 'node'):
            self.node.destroy_node()
        try:
            import rclpy
            rclpy.shutdown()
        except:
            pass
    
    def test_ros2_environment(self):
        """Test ROS2 environment setup."""
        self.assertIsNotNone(self.node, "ROS2 node creation failed")
    
    def test_ros2_topics(self):
        """Test ROS2 topic functionality."""
        from sensor_msgs.msg import LaserScan, Image, NavSatFix
        
        # Test message creation
        scan_msg = LaserScan()
        self.assertIsNotNone(scan_msg)
        
        image_msg = Image()
        self.assertIsNotNone(image_msg)
        
        gps_msg = NavSatFix()
        self.assertIsNotNone(gps_msg)

class TestCameraProcessing(unittest.TestCase):
    """Test camera processing functionality."""
    
    def test_depthai_initialization(self):
        """Test DepthAI camera initialization."""
        try:
            import depthai as dai
            
            # Create a simple pipeline
            pipeline = dai.Pipeline()
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(640, 480)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            
            self.assertIsNotNone(pipeline, "Pipeline creation failed")
            self.assertIsNotNone(cam_rgb, "Camera node creation failed")
            
        except ImportError:
            self.skipTest("DepthAI not available")
        except Exception as e:
            self.fail(f"DepthAI initialization failed: {e}")
    
    def test_image_processing(self):
        """Test image processing functions."""
        import cv2
        import numpy as np
        
        # Create test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Test basic operations
        resized = cv2.resize(test_image, (320, 240))
        self.assertEqual(resized.shape, (240, 320, 3))
        
        # Test color space conversion
        hsv = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
        self.assertEqual(hsv.shape, test_image.shape)

class TestInferenceAPI(unittest.TestCase):
    """Test inference API functionality."""
    
    def test_roboflow_import(self):
        """Test Roboflow module import."""
        try:
            import roboflow
            self.assertTrue(True, "Roboflow import successful")
        except ImportError:
            self.skipTest("Roboflow not available")
    
    def test_inference_request_format(self):
        """Test inference request format."""
        import base64
        import cv2
        import numpy as np
        
        # Create test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Encode image
        success, buffer = cv2.imencode('.jpg', test_image)
        self.assertTrue(success, "Image encoding failed")
        
        img_str = base64.b64encode(buffer).decode('ascii')
        self.assertIsInstance(img_str, str, "Base64 encoding failed")
    
    @patch('requests.post')
    def test_inference_api_call(self, mock_post):
        """Test inference API call."""
        # Mock successful response
        mock_response = Mock()
        mock_response.json.return_value = {
            'predictions': [
                {
                    'class': 'barrel',
                    'confidence': 0.95,
                    'x': 100,
                    'y': 150,
                    'width': 100,
                    'height': 150
                }
            ]
        }
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response
        
        # Test API call
        response = requests.post(
            'http://localhost:9001/test-model/1',
            data={'api_key': 'test-key', 'image': 'test-image'},
            headers={'Content-Type': 'application/x-www-form-urlencoded'}
        )
        
        self.assertIsNotNone(response.json())
        mock_post.assert_called_once()

class TestWebInterface(unittest.TestCase):
    """Test web interface functionality."""
    
    def test_flask_import(self):
        """Test Flask import."""
        try:
            from flask import Flask
            self.assertTrue(True, "Flask import successful")
        except ImportError:
            self.fail("Flask not available")
    
    def test_flask_app_creation(self):
        """Test Flask app creation."""
        from flask import Flask
        
        app = Flask(__name__)
        self.assertIsNotNone(app, "Flask app creation failed")
    
    def test_web_endpoints(self):
        """Test web endpoint definitions."""
        from flask import Flask
        
        app = Flask(__name__)
        
        @app.route('/test')
        def test_endpoint():
            return {'status': 'ok'}
        
        # Test endpoint registration
        with app.test_client() as client:
            response = client.get('/test')
            self.assertEqual(response.status_code, 200)

class TestDataProcessing(unittest.TestCase):
    """Test data processing functionality."""
    
    def test_numpy_operations(self):
        """Test NumPy operations."""
        import numpy as np
        
        # Test array operations
        arr = np.array([1, 2, 3, 4, 5])
        self.assertEqual(arr.sum(), 15)
        self.assertEqual(arr.mean(), 3.0)
    
    def test_opencv_operations(self):
        """Test OpenCV operations."""
        import cv2
        import numpy as np
        
        # Test image operations
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        
        # Test blur
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        self.assertEqual(blurred.shape, img.shape)
        
        # Test edge detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        self.assertEqual(edges.shape, (100, 100))

class TestConfiguration(unittest.TestCase):
    """Test configuration management."""
    
    def test_yaml_parsing(self):
        """Test YAML configuration parsing."""
        try:
            import yaml
            
            # Test YAML parsing
            config = {
                'device': '/dev/ttyUSB0',
                'baudrate': 115200,
                'frame_id': 'laser'
            }
            
            yaml_str = yaml.dump(config)
            parsed_config = yaml.safe_load(yaml_str)
            
            self.assertEqual(parsed_config['device'], '/dev/ttyUSB0')
            self.assertEqual(parsed_config['baudrate'], 115200)
            
        except ImportError:
            self.skipTest("PyYAML not available")
    
    def test_environment_variables(self):
        """Test environment variable handling."""
        import os
        
        # Test environment variable access
        test_var = os.getenv('TEST_VAR', 'default')
        self.assertIsInstance(test_var, str)

class TestPerformance(unittest.TestCase):
    """Test system performance characteristics."""
    
    def test_memory_usage(self):
        """Test memory usage monitoring."""
        try:
            import psutil
            
            # Get memory usage
            memory = psutil.virtual_memory()
            self.assertGreater(memory.total, 0, "Invalid memory total")
            self.assertGreaterEqual(memory.percent, 0, "Invalid memory percent")
            
        except ImportError:
            self.skipTest("psutil not available")
    
    def test_cpu_usage(self):
        """Test CPU usage monitoring."""
        try:
            import psutil
            
            # Get CPU usage
            cpu_percent = psutil.cpu_percent(interval=1)
            self.assertGreaterEqual(cpu_percent, 0, "Invalid CPU usage")
            self.assertLessEqual(cpu_percent, 100, "CPU usage over 100%")
            
        except ImportError:
            self.skipTest("psutil not available")

class TestIntegration(unittest.TestCase):
    """Test system integration."""
    
    def test_project_structure(self):
        """Test project directory structure."""
        required_files = [
            'README.md',
            'requirements.txt',
            'setup.sh',
            'docker-compose.yaml'
        ]
        
        for file in required_files:
            with self.subTest(file=file):
                self.assertTrue(os.path.exists(file), f"Required file {file} not found")
    
    def test_ros2_workspace(self):
        """Test ROS2 workspace structure."""
        if os.path.exists('ros2_ws'):
            required_dirs = ['src', 'build', 'install']
            for dir_name in required_dirs:
                with self.subTest(directory=dir_name):
                    self.assertTrue(os.path.exists(f'ros2_ws/{dir_name}'), 
                                  f"ROS2 workspace missing {dir_name} directory")

def run_tests():
    """Run all tests with appropriate verbosity."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestSystemDependencies,
        TestHardwareDetection,
        TestROS2Integration,
        TestCameraProcessing,
        TestInferenceAPI,
        TestWebInterface,
        TestDataProcessing,
        TestConfiguration,
        TestPerformance,
        TestIntegration
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*50}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped) if hasattr(result, 'skipped') else 0}")
    
    if result.failures:
        print("\nFailures:")
        for test, traceback in result.failures:
            print(f"  {test}: {traceback}")
    
    if result.errors:
        print("\nErrors:")
        for test, traceback in result.errors:
            print(f"  {test}: {traceback}")
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
