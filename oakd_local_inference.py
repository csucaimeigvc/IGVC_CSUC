#!/usr/bin/env python3

import os
import time
import cv2
import depthai as dai
import numpy as np
import requests
import base64
from io import BytesIO

# Model information
MODEL_ID = "barrels-potholes-markers-test"
VERSION = 5
API_KEY = "LtFU6JM5SYefg3nVtg63"  # Your Roboflow API key
CONFIDENCE = 0.5  # Minimum confidence threshold

# Local Roboflow Inference Server URL
INFERENCE_SERVER_URL = f"http://localhost:9001/{MODEL_ID}/{VERSION}"

# Create output directory
OUTPUT_DIR = "output_frames"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def process_frame_with_inference(frame, frame_id):
    """Send frame to Roboflow Inference Server and process results"""
    # Convert frame to base64 for API request
    success, buffer = cv2.imencode('.jpg', frame)
    if not success:
        print("Failed to encode image")
        return frame
    
    img_str = base64.b64encode(buffer).decode('ascii')
    
    # Prepare headers
    headers = {
        "Content-Type": "application/x-www-form-urlencoded"
    }
    
    try:
        # Send request to local inference server
        response = requests.post(
            f"{INFERENCE_SERVER_URL}?api_key={API_KEY}",
            data=img_str,
            headers=headers,
            timeout=5
        )
        
        # Check if the request was successful
        if response.status_code == 200:
            results = response.json()
            print(f"Frame {frame_id} - Predictions received: {len(results.get('predictions', []))}")
            
            # Draw predictions on the frame
            predictions = results.get('predictions', [])
            for pred in predictions:
                if pred['confidence'] >= CONFIDENCE:
                    # Get prediction details
                    x = pred['x']
                    y = pred['y']
                    width = pred['width']
                    height = pred['height']
                    class_name = pred['class']
                    confidence = pred['confidence']
                    
                    # Calculate bounding box coordinates
                    x1 = int(x - width/2)
                    y1 = int(y - height/2)
                    x2 = int(x + width/2)
                    y2 = int(y + height/2)
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label
                    label = f"{class_name}: {confidence:.2f}"
                    cv2.putText(frame, label, (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Draw segmentation mask if available
                    if 'points' in pred:
                        points = np.array([[p['x'], p['y']] for p in pred['points']], np.int32)
                        points = points.reshape((-1, 1, 2))
                        cv2.polylines(frame, [points], True, (255, 0, 0), 2)
            
        else:
            print(f"Error from inference server: {response.status_code}, {response.text}")
    
    except Exception as e:
        print(f"Error processing frame with inference: {e}")
    
    # Add frame number and timestamp
    cv2.putText(
        frame,
        f"Frame: {frame_id} | Time: {time.strftime('%H:%M:%S')}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2
    )
    
    # Save the frame periodically
    if frame_id % 10 == 0:  # Save every 10th frame
        output_path = os.path.join(OUTPUT_DIR, f"frame_{frame_id:06d}.jpg")
        cv2.imwrite(output_path, frame)
        print(f"Saved: {output_path}")
    
    return frame

def setup_oakd():
    """Set up the OAK-D Pro camera"""
    # Create OAK-D pipeline
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
    
    return pipeline

def main():
    print("Starting OAK-D Pro with local Roboflow Inference Server...")
    print(f"Model: {MODEL_ID}/{VERSION}")
    print(f"Frames will be saved to: {OUTPUT_DIR}")
    print("Press Ctrl+C to stop")
    
    # Initialize the OAK-D Pro camera
    oak_pipeline = setup_oakd()
    
    try:
        # Connect to device
        with dai.Device(oak_pipeline) as device:
            print(f"Connected to OAK-D Pro camera: {device.getDeviceInfo().getMxId()}")
            
            # Output queue for RGB frames
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            # Create a video writer for saving the output
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
            
            frame_id = 0
            start_time = time.time()
            fps_counter = 0
            
            try:
                while True:
                    in_rgb = q_rgb.get()
                    if in_rgb is None:
                        continue
                    
                    # Get the frame from the camera
                    frame = in_rgb.getCvFrame()
                    frame_id += 1
                    fps_counter += 1
                    
                    # Calculate and print FPS every second
                    if time.time() - start_time >= 1.0:
                        fps = fps_counter / (time.time() - start_time)
                        print(f"FPS: {fps:.2f}")
                        start_time = time.time()
                        fps_counter = 0
                    
                    # Process the frame with Roboflow Inference Server
                    processed_frame = process_frame_with_inference(frame, frame_id)
                    
                    # Save the frame to video file
                    out.write(processed_frame)
                    
                    # Sleep a tiny bit to reduce CPU usage
                    time.sleep(0.001)
                    
            except KeyboardInterrupt:
                print("\nKeyboard interrupt received, stopping...")
            finally:
                # Release resources
                print("Cleaning up resources...")
                out.release()
                print("Resources released")
    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 