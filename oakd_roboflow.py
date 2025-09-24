#!/usr/bin/env python3

import os
import time
import cv2
import depthai as dai
import numpy as np
from inference import InferencePipeline
from inference.core.interfaces.stream.sinks import render_boxes, save_predictions

# Set your Roboflow API key
os.environ["ROBOFLOW_API_KEY"] = "LtFU6JM5SYefg3nVtg63"

# Model information
MODEL_ID = "barrels-potholes-markers-test/5"
CONFIDENCE = 0.5  # Minimum confidence threshold

def process_frame(prediction, frame_data):
    """Custom processing function for predictions"""
    # Get the frame and metadata
    frame = frame_data["frame"]
    
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
        print(f"Detections: {sum(detection_counts.values())}")
        for class_name, count in detection_counts.items():
            print(f"  {class_name}: {count}")
    
    # Add frame number and timestamp
    cv2.putText(
        frame,
        f"Frame: {frame_data.get('frame_id', 0)} | Time: {time.strftime('%H:%M:%S')}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2
    )
    
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
    # Initialize the OAK-D Pro camera
    oak_pipeline = setup_oakd()
    
    try:
        # Connect to device
        with dai.Device(oak_pipeline) as device:
            print("Connected to OAK-D Pro camera")
            
            # Output queue for RGB frames
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            # Create a video writer for saving the output
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
            
            # Initialize the Roboflow InferencePipeline
            inference_pipeline = InferencePipeline.init(
                model_id=MODEL_ID,
                confidence=CONFIDENCE,
                on_prediction=[render_boxes, process_frame]
            )
            
            # Start the inference pipeline
            inference_pipeline.start()
            
            frame_id = 0
            try:
                while True:
                    in_rgb = q_rgb.get()
                    if in_rgb is None:
                        continue
                    
                    # Get the frame from the camera
                    frame = in_rgb.getCvFrame()
                    frame_id += 1
                    
                    # Process the frame with Roboflow
                    inference_pipeline.predict_frame(
                        frame,
                        frame_id=frame_id
                    )
                    
                    # Save the frame to video file
                    out.write(frame)
                    
                    # Display the frame
                    cv2.imshow("OAK-D Pro with Roboflow", frame)
                    
                    # Break the loop if 'q' is pressed
                    if cv2.waitKey(1) == ord('q'):
                        break
                    
            except KeyboardInterrupt:
                print("Keyboard interrupt received, stopping...")
            finally:
                # Stop the inference pipeline and release resources
                inference_pipeline.join()
                out.release()
                cv2.destroyAllWindows()
                print("Pipeline stopped and resources released")
    
    except Exception as e:
        print(f"Error connecting to OAK-D Pro: {e}")

if __name__ == "__main__":
    main() 