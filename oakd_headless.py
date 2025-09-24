#!/usr/bin/env python3

import os
import time
import cv2
import depthai as dai
import numpy as np
from inference import InferencePipeline
from inference.core.interfaces.stream.sinks import render_boxes
import threading

# Set your Roboflow API key
os.environ["ROBOFLOW_API_KEY"] = "LtFU6JM5SYefg3nVtg63"

# Model information
MODEL_ID = "barrels-potholes-markers-test/5"
CONFIDENCE = 0.5  # Minimum confidence threshold

# Create output directory
OUTPUT_DIR = "output_frames"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def process_frame(prediction, frame_data):
    """Custom processing function for predictions"""
    # Get the frame and metadata
    frame = frame_data["frame"]
    frame_id = frame_data.get("frame_id", 0)
    
    # Log detection counts
    if prediction and prediction.predictions:
        detection_counts = {}
        for pred in prediction.predictions:
            class_name = pred.class_name
            confidence = pred.confidence
            
            if class_name not in detection_counts:
                detection_counts[class_name] = 0
            detection_counts[class_name] += 1
        
        # Print detection summary
        print(f"Frame {frame_id} - Detections: {sum(detection_counts.values())}")
        for class_name, count in detection_counts.items():
            print(f"  {class_name}: {count}")
    
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
    
    # Save the frame with detections to a file
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
    print("Starting OAK-D Pro with Roboflow in headless mode...")
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
            
            # Initialize the Roboflow InferencePipeline
            print(f"Initializing Roboflow model: {MODEL_ID}")
            inference_pipeline = InferencePipeline.init(
                model_id=MODEL_ID,
                video_reference=None,  # We're handling the video feed manually
                confidence=CONFIDENCE,
                on_prediction=[render_boxes, process_frame]
            )
            
            # Start the inference pipeline
            print("Starting inference pipeline...")
            inference_pipeline.start()
            
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
                    
                    # Process the frame with Roboflow
                    inference_pipeline.predict_frame(
                        frame,
                        frame_id=frame_id
                    )
                    
                    # Save the frame to video file
                    out.write(frame)
                    
                    # Sleep a tiny bit to reduce CPU usage
                    time.sleep(0.001)
                    
            except KeyboardInterrupt:
                print("\nKeyboard interrupt received, stopping...")
            finally:
                # Stop the inference pipeline and release resources
                print("Cleaning up resources...")
                inference_pipeline.join()
                out.release()
                print("Pipeline stopped and resources released")
    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 