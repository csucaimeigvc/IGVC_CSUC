import requests
import base64
from PIL import Image
from io import BytesIO
import numpy as np
import cv2
import time

# IMPORTANT: Replace with your actual API key from https://app.roboflow.com/settings/api
api_key="YOUR_API_KEY"

# Replace with your custom model information
model="YOUR_MODEL_ID"  # Example: "my-project-abc123"
version=1  # Replace with your model version number

# Video source (0 for webcam, or provide a file path)
VIDEO_SOURCE = 0  # Use 0 for default camera or file path like "video.mp4"

# Initialize video capture
cap = cv2.VideoCapture(VIDEO_SOURCE)
if not cap.isOpened():
    print(f"Error: Could not open video source {VIDEO_SOURCE}")
    exit(1)

print("Press 'q' to quit")

# Function to process a frame
def process_frame(frame):
    # Convert OpenCV BGR to RGB for PIL
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb_frame)
    
    # Convert image to base64 string
    buffered = BytesIO()
    pil_image.save(buffered, quality=85, format="JPEG")
    img_str = base64.b64encode(buffered.getvalue())
    img_str = img_str.decode("ascii")
    
    # Send POST request to local inference server
    headers = {"Content-Type": "application/x-www-form-urlencoded"}
    try:
        res = requests.post(
            f"http://localhost:9001/{model}/{version}?api_key={api_key}",
            data=img_str,
            headers=headers,
            timeout=5
        )
        
        if res.status_code == 200:
            predictions = res.json()["predictions"]
            
            # Draw bounding boxes and masks
            for pred in predictions:
                # Get prediction details
                x, y, width, height = pred["x"], pred["y"], pred["width"], pred["height"]
                conf = pred["confidence"]
                cls = pred["class"]
                
                # Draw bounding box
                x1, y1 = int(x - width/2), int(y - height/2)
                x2, y2 = int(x + width/2), int(y + height/2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label
                label = f"{cls}: {conf:.2f}"
                cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw segmentation if available
                if "points" in pred:
                    points = np.array([[p["x"], p["y"]] for p in pred["points"]], np.int32)
                    points = points.reshape((-1, 1, 2))
                    cv2.polylines(frame, [points], True, (255, 0, 0), 2)
        else:
            print(f"Request failed with status code {res.status_code}")
            
    except Exception as e:
        print(f"Error processing frame: {e}")
    
    return frame

# Process video
frame_count = 0
start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        print("End of video stream")
        break
    
    # Process every 3rd frame to improve performance
    if frame_count % 3 == 0:
        processed_frame = process_frame(frame)
    else:
        processed_frame = frame
    
    # Display FPS
    current_time = time.time()
    fps = frame_count / (current_time - start_time)
    cv2.putText(processed_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Display the resulting frame
    cv2.imshow('Roboflow Object Detection', processed_frame)
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    frame_count += 1

# Release resources
cap.release()
cv2.destroyAllWindows()
print("Video processing stopped") 