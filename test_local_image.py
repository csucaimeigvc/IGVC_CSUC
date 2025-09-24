import requests
import base64
from PIL import Image
from io import BytesIO
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os

# IMPORTANT: Replace with your actual API key from https://app.roboflow.com/settings/api
api_key="YOUR_API_KEY"

# Replace with your custom model information
model="YOUR_MODEL_ID"  # Example: "my-project-abc123"
version=1  # Replace with your model version number

# Path to your local image
LOCAL_IMAGE_PATH = "your_image.jpg"  # Replace with the path to your image

# Check if image exists
if not os.path.exists(LOCAL_IMAGE_PATH):
    print(f"Error: Image not found at {LOCAL_IMAGE_PATH}")
    exit(1)

# Load local image
image = Image.open(LOCAL_IMAGE_PATH)

# Convert image to base64 string
buffered = BytesIO()
image.save(buffered, quality=100, format="JPEG")
img_str = base64.b64encode(buffered.getvalue())
img_str = img_str.decode("ascii")

# Send POST request to local inference server
headers = {"Content-Type": "application/x-www-form-urlencoded"}
res = requests.post(
    f"http://localhost:9001/{model}/{version}?api_key={api_key}",
    data=img_str,
    headers=headers,
)

# Check if the request was successful
if res.status_code == 200:
    print("Request successful!")
    predictions = res.json()["predictions"]
    print(f"Found {len(predictions)} predictions")
    
    # Convert PIL Image to OpenCV format
    img_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    
    # Draw bounding boxes and masks
    for pred in predictions:
        # Get prediction details
        x, y, width, height = pred["x"], pred["y"], pred["width"], pred["height"]
        conf = pred["confidence"]
        cls = pred["class"]
        
        # Draw bounding box
        x1, y1 = int(x - width/2), int(y - height/2)
        x2, y2 = int(x + width/2), int(y + height/2)
        cv2.rectangle(img_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label
        label = f"{cls}: {conf:.2f}"
        cv2.putText(img_cv, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw segmentation if available
        if "points" in pred:
            points = np.array([[p["x"], p["y"]] for p in pred["points"]], np.int32)
            points = points.reshape((-1, 1, 2))
            cv2.polylines(img_cv, [points], True, (255, 0, 0), 2)
    
    # Save and show the image
    output_filename = "result_" + os.path.basename(LOCAL_IMAGE_PATH)
    cv2.imwrite(output_filename, img_cv)
    print(f"Saved result to {output_filename}")
else:
    print(f"Request failed with status code {res.status_code}")
    print(res.text) 