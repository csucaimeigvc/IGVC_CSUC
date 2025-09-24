#!/usr/bin/env python3

import os
import time
import cv2
import depthai as dai
import numpy as np
import requests
import base64
from io import BytesIO
import threading
from flask import Flask, Response, render_template

# Model information
MODEL_ID = "barrels-potholes"
VERSION = 1
API_KEY = "LtFU6JM5SYefg3nVtg63"  # Your Roboflow API key
CONFIDENCE = 0.5  # Minimum confidence threshold

# Local Roboflow Inference Server URL
INFERENCE_SERVER_URL = f"http://localhost:9001/{MODEL_ID}/{VERSION}"

# Create output directory
OUTPUT_DIR = "output_frames"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Global variables for the latest frame
latest_frame = None
frame_lock = threading.Lock()

# Flask app for streaming
app = Flask(__name__)

def process_frame_with_inference(frame, frame_id):
    """Send frame to Roboflow Inference Server and process results"""
    global latest_frame
    
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
            prediction_count = len(results.get('predictions', []))
            print(f"Frame {frame_id} - Predictions received: {prediction_count}")
            
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
    if frame_id % 30 == 0:  # Save every 30th frame
        output_path = os.path.join(OUTPUT_DIR, f"frame_{frame_id:06d}.jpg")
        cv2.imwrite(output_path, frame)
        print(f"Saved: {output_path}")
    
    # Update the latest frame
    with frame_lock:
        latest_frame = frame.copy()
    
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

def camera_loop():
    """Main camera processing loop"""
    print("Starting OAK-D Pro with local Roboflow Inference Server...")
    print(f"Model: {MODEL_ID}/{VERSION}")
    print(f"Frames will be saved to: {OUTPUT_DIR}")
    
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
                    
                    # Small delay to reduce CPU usage
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

def generate_frames():
    """Generate frames for the web stream"""
    while True:
        with frame_lock:
            if latest_frame is not None:
                # Encode the frame to JPEG
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                if not ret:
                    continue
                
                # Convert to bytes and yield for streaming
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        # Sleep to control the frame rate of the stream
        time.sleep(0.03)  # ~30 FPS

@app.route('/')
def index():
    """Render the home page"""
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>OAK-D Pro Object Detection</title>
        <style>
            body {{
                font-family: Arial, sans-serif;
                margin: 0;
                padding: 20px;
                text-align: center;
                background-color: #f0f0f0;
            }}
            h1 {{
                color: #333;
            }}
            .video-container {{
                margin: 20px auto;
                max-width: 800px;
                border: 1px solid #ccc;
                box-shadow: 0 0 10px rgba(0,0,0,0.1);
            }}
            img {{
                width: 100%;
                height: auto;
            }}
        </style>
    </head>
    <body>
        <h1>OAK-D Pro with Roboflow Object Detection</h1>
        <div class="video-container">
            <img src="/video_feed" alt="Video Feed">
        </div>
        <p>Model: {MODEL_ID}</p>
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    """Route for the video feed"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # Start the camera processing in a separate thread
    threading.Thread(target=camera_loop, daemon=True).start()
    
    # Start the Flask web server
    print("Starting web server at http://0.0.0.0:5000")
    print("You can access the stream from any device on the same network")
    print("Press Ctrl+C to stop")
    app.run(host='0.0.0.0', port=5000, threaded=True) 