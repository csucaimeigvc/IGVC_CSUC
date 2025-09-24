#!/usr/bin/env python3

import os
import glob

# Create a simple HTML gallery
html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>Lane Segmentation Images</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h1 {
            text-align: center;
            color: #333;
        }
        .gallery {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            grid-gap: 15px;
            margin-top: 20px;
        }
        .image-container {
            background: white;
            border: 1px solid #ddd;
            border-radius: 5px;
            padding: 10px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .image-container img {
            width: 100%;
            height: auto;
            border-radius: 3px;
        }
        .image-container p {
            margin: 5px 0;
            font-size: 14px;
            color: #555;
        }
    </style>
</head>
<body>
    <h1>Lane Segmentation Images</h1>
    <div class="gallery">
"""

# Get all jpg files in the directory
image_files = sorted(glob.glob("lane_segmentation_frames/*.jpg"))

# Add each image to the gallery
for img_file in image_files:
    filename = os.path.basename(img_file)
    frame_number = filename.split("_")[1].split(".")[0]
    html_content += f"""
        <div class="image-container">
            <img src="{img_file}" alt="Frame {frame_number}">
            <p>Frame {int(frame_number)}</p>
        </div>
    """

# Close the HTML
html_content += """
    </div>
</body>
</html>
"""

# Write the HTML file
with open("lane_segmentation_gallery.html", "w") as f:
    f.write(html_content)

print("Gallery created! Open lane_segmentation_gallery.html in your browser to view images.") 