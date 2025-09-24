import requests
import base64
from PIL import Image
from io import BytesIO
import numpy as np
import cv2
import os
import json
import time
import glob

# Try to import tqdm, install if not available
try:
    from tqdm import tqdm
except ImportError:
    print("Installing tqdm...")
    os.system("pip install tqdm")
    from tqdm import tqdm

# IMPORTANT: Replace with your actual API key and model information
api_key = "LtFU6JM5SYefg3nVtg63"
model = "barrels-potholes-markers-test"
version = 5

# Folder containing your images
DATASET_FOLDER = "/home/igvc/dataset"  # Replace with path to your dataset folder
OUTPUT_FOLDER = "results"  # Folder to save processed images
RESULTS_FILE = "inference_results.json"  # File to save all prediction results

# Create output folder if it doesn't exist
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# Get all image files from the folder
image_extensions = ["*.jpg", "*.jpeg", "*.png", "*.bmp"]
all_images = []
for ext in image_extensions:
    all_images.extend(glob.glob(os.path.join(DATASET_FOLDER, ext)))
    all_images.extend(glob.glob(os.path.join(DATASET_FOLDER, ext.upper())))

print(f"Found {len(all_images)} images in {DATASET_FOLDER}")

if len(all_images) == 0:
    print(f"No images found in {DATASET_FOLDER}")
    exit(1)

# Function to process a single image
def process_image(image_path):
    try:
        # Load image
        image = Image.open(image_path)
        
        # Convert image to base64 string
        buffered = BytesIO()
        image.save(buffered, quality=95, format="JPEG")
        img_str = base64.b64encode(buffered.getvalue())
        img_str = img_str.decode("ascii")
        
        # Send POST request to local inference server
        headers = {"Content-Type": "application/x-www-form-urlencoded"}
        res = requests.post(
            f"http://localhost:9001/{model}/{version}?api_key={api_key}",
            data=img_str,
            headers=headers,
            timeout=10
        )
        
        if res.status_code == 200:
            result = res.json()
            predictions = result["predictions"]
            
            # Save image with annotations
            if predictions and len(predictions) > 0:
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
                
                # Save result image
                output_path = os.path.join(OUTPUT_FOLDER, f"result_{os.path.basename(image_path)}")
                cv2.imwrite(output_path, img_cv)
            
            # Add filename to result
            result["file"] = os.path.basename(image_path)
            return result
        else:
            print(f"Error processing {os.path.basename(image_path)}: HTTP {res.status_code}")
            return {"file": os.path.basename(image_path), "error": f"HTTP {res.status_code}", "message": res.text}
    
    except Exception as e:
        print(f"Error processing {os.path.basename(image_path)}: {str(e)}")
        return {"file": os.path.basename(image_path), "error": str(e)}

# Process all images and collect results
results = []
start_time = time.time()

for image_path in tqdm(all_images, desc="Processing images"):
    result = process_image(image_path)
    if result:
        results.append(result)
    # Small delay to avoid overwhelming the server
    time.sleep(0.1)

# Save all results to JSON file
with open(os.path.join(OUTPUT_FOLDER, RESULTS_FILE), 'w') as f:
    json.dump(results, f, indent=2)

# Print summary
total_time = time.time() - start_time
print(f"\nProcessed {len(results)} images in {total_time:.1f} seconds")
print(f"Average time per image: {total_time/len(results):.2f} seconds")
print(f"Results saved to {os.path.join(OUTPUT_FOLDER, RESULTS_FILE)}")
print(f"Annotated images saved to {OUTPUT_FOLDER}")

# Generate class statistics
if results:
    class_counts = {}
    total_objects = 0
    
    for result in results:
        if "predictions" in result:
            for pred in result["predictions"]:
                cls = pred["class"]
                if cls not in class_counts:
                    class_counts[cls] = 0
                class_counts[cls] += 1
                total_objects += 1
    
    print(f"\nDetected {total_objects} objects across {len(results)} images")
    print("Class distribution:")
    for cls, count in sorted(class_counts.items(), key=lambda x: x[1], reverse=True):
        print(f"  {cls}: {count} ({count/total_objects*100:.1f}%)") 