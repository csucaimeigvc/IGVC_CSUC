#!/usr/bin/env python3

import cv2
import depthai as dai
import time

def main():
    # Check available devices
    print("Checking for available devices...")
    device_infos = dai.Device.getAllAvailableDevices()
    if len(device_infos) == 0:
        print("No OAK-D devices found!")
        return
    
    print(f"Found {len(device_infos)} devices:")
    for device_info in device_infos:
        print(f" - {device_info.getMxId()} - State: {device_info.state}")
    
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    
    xoutRgb.setStreamName("rgb")
    
    # Properties
    camRgb.setPreviewSize(640, 480)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(30)
    
    # Linking
    camRgb.preview.link(xoutRgb.input)
    
    try:
        # Connect to device
        with dai.Device(pipeline) as device:
            print(f"Connected to device: {device.getDeviceInfo().getMxId()}")
            
            # Output queue
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            start_time = time.time()
            frame_count = 0
            
            print("Press 'q' to quit")
            
            while True:
                in_rgb = q_rgb.get()
                if in_rgb is None:
                    continue
                
                # Get BGR frame
                frame = in_rgb.getCvFrame()
                frame_count += 1
                
                # Calculate FPS
                elapsed_time = time.time() - start_time
                if elapsed_time >= 1.0:
                    fps = frame_count / elapsed_time
                    print(f"FPS: {fps:.2f}")
                    start_time = time.time()
                    frame_count = 0
                
                # Add some information to the frame
                cv2.putText(frame, "OAK-D Pro Test", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Show the frame
                cv2.imshow("OAK-D Pro", frame)
                
                if cv2.waitKey(1) == ord('q'):
                    break
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        print("Test complete")

if __name__ == "__main__":
    main() 