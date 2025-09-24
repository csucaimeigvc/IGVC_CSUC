import requests
import base64
from PIL import Image
from io import BytesIO
import matplotlib.pyplot as plt
import numpy as np
import cv2

# IMPORTANT: Replace with your actual API key from https://app.roboflow.com/settings/api
# If you don't have a Roboflow account, sign up for free at https://roboflow.com
api_key = "LtFU6JM5SYefg3nVtg63"

# Replace with your custom model information
model = "barrels-potholes-markers-test"
version = 5  # Replace with your model version number

# URL of image to test
image_url = "https://source.roboflow.com/zD7y6XOoQnh7WC160Ae7/yA6pCzno5RW5tc3LjgSR/original.jpg"
image = Image.open(requests.get(image_url, stream=True).raw)

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
    cv2.imwrite("result.jpg", img_cv)
    print("Saved result to result.jpg")
else:
    print(f"Request failed with status code {res.status_code}")
    print(res.text) 