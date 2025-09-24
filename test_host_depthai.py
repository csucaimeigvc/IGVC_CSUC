#!/usr/bin/env python3
import depthai as dai
import time

print(f"DepthAI version: {dai.__version__}")

print("Checking available devices:")
devices = dai.Device.getAllAvailableDevices()
print(f"Available devices: {devices}")

try:
    print("\nTrying to initialize device:")
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("preview")
    cam.preview.link(xout.input)
    
    with dai.Device(pipeline) as device:
        print(f"Connected to {device.getDeviceName()}")
        print(f"MxId: {device.getMxId()}")
        print(f"Connected cameras: {device.getConnectedCameras()}")
        print(f"Device is working!")
except Exception as e:
    print(f"Failed to initialize: {e}")
