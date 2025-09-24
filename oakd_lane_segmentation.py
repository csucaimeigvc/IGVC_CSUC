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
from flask import Flask, Response, render_template_string

# Lane Segmentation Model information
MODEL_ID = "lane-segmentation-2.0-xgzqs"
VERSION = 1
API_KEY = "LtFU6JM5SYefg3nVtg63"

# Local Roboflow Inference Server URL
INFERENCE_SERVER_URL = f"http://localhost:9001/{MODEL_ID}/{VERSION}"

# Create output directory
OUTPUT_DIR = "lane_segmentation_frames"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Global variables for the latest frame
latest_frame = None
frame_lock = threading.Lock()

# Flask app for streaming
app = Flask(__name__)

def process_segmentation(frame, results):
    """Process segmentation results and draw them on frame"""
    if not results or 'segmentation_mask' not in results:
        return frame

    # Decode the segmentation mask
    mask_data = base64.b64decode(results['segmentation_mask'])
    mask_array = np.frombuffer(mask_data, dtype=np.uint8)
    
    try:
        # Convert mask to image
        mask = cv2.imdecode(mask_array, cv2.IMREAD_GRAYSCALE)
        
        # Resize the mask to match frame dimensions
        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
        
        # Create a color mask (yellow for lane markings)
        color_mask = np.zeros_like(frame)
        color_mask[mask > 128] = [255, 200, 0]  # Yellow for lanes
        
        # Blend with original frame
        alpha = 0.4  # Transparency factor
        frame = cv2.addWeighted(frame, 1, color_mask, alpha, 0)
        
        # Add segmentation indicator
        cv2.putText(
            frame,
            "Lane Segmentation Active",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 200, 0),
            2
        )
    except Exception as e:
        print(f"Error processing segmentation mask: {e}")
    
    return frame

def make_inference_request(frame):
    """Make a request to the Roboflow Inference Server"""
    # Convert frame to base64 for API request
    success, buffer = cv2.imencode('.jpg', frame)
    if not success:
        print("Failed to encode image")
        return None
    
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
            return response.json()
        else:
            print(f"Error from inference server: {response.status_code}, {response.text}")
            return None
    
    except Exception as e:
        print(f"Error making inference request: {e}")
        return None

def process_frame_with_inference(frame, frame_id):
    """Send frame to Roboflow Inference Server and process results"""
    global latest_frame
    
    # Make lane segmentation request
    segmentation_results = make_inference_request(frame)
    
    # Process lane segmentation results
    if segmentation_results:
        print(f"Frame {frame_id} - Lane segmentation processed")
        frame = process_segmentation(frame, segmentation_results)
    
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
    print("Starting OAK-D Pro with Lane Segmentation...")
    print(f"Lane Segmentation Model: {MODEL_ID}/{VERSION}")
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
            out = cv2.VideoWriter('lane_segmentation_output.avi', fourcc, 20.0, (640, 480))
            
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
                    
                    # Process the frame with Lane Segmentation
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
    template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>OAK-D Pro Lane Segmentation</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                margin: 0;
                padding: 20px;
                text-align: center;
                background-color: #f0f0f0;
            }
            h1 {
                color: #333;
            }
            .video-container {
                margin: 20px auto;
                max-width: 800px;
                border: 1px solid #ccc;
                background-color: #fff;
                box-shadow: 0 0 10px rgba(0,0,0,0.1);
                padding: 10px;
            }
            img {
                width: 100%;
                height: auto;
            }
            .model-info {
                margin: 20px auto;
                max-width: 800px;
                text-align: left;
                background-color: #fff;
                padding: 15px;
                border-radius: 5px;
                box-shadow: 0 0 10px rgba(0,0,0,0.1);
            }
        </style>
    </head>
    <body>
        <h1>OAK-D Pro Lane Segmentation</h1>
        <div class="model-info">
            <h3>Active Model:</h3>
            <p><strong>Lane Segmentation:</strong> lane-segmentation-2.0-xgzqs (v1)</p>
        </div>
        <div class="video-container">
            <img src="{{ url_for('video_feed') }}" alt="Lane Segmentation Stream">
        </div>
    </body>
    </html>
    """
    return render_template_string(template)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # Start camera processing in a separate thread
    camera_thread = threading.Thread(target=camera_loop)
    camera_thread.daemon = True
    camera_thread.start()
    
    # Start the Flask web server
    print("Starting web server at http://0.0.0.0:5002")
    app.run(host='0.0.0.0', port=5002, debug=False, threaded=True) 