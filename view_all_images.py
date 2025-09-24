#!/usr/bin/env python3

import os
import glob
import subprocess

# Create a simple HTML gallery
html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>All Segmentation Images</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h1, h2 {
            text-align: center;
            color: #333;
        }
        .gallery {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            grid-gap: 15px;
            margin-top: 20px;
            margin-bottom: 40px;
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
        .section {
            margin-bottom: 30px;
        }
    </style>
</head>
<body>
    <h1>All Segmentation Images</h1>
    
    <div class="section">
        <h2>Lane Segmentation Images</h2>
        <div class="gallery">
"""

# Get all jpg files from lane_segmentation_frames directory
lane_images = sorted(glob.glob("lane_segmentation_frames/*.jpg"))

# Add each lane segmentation image to the gallery
for img_file in lane_images:
    filename = os.path.basename(img_file)
    frame_number = filename.split("_")[1].split(".")[0]
    html_content += f"""
        <div class="image-container">
            <img src="{img_file}" alt="Lane Frame {frame_number}">
            <p>Lane Frame {int(frame_number)}</p>
        </div>
    """

# Add dual model images section
html_content += """
        </div>
    </div>
    
    <div class="section">
        <h2>Dual Model Images</h2>
        <div class="gallery">
"""

# Get all jpg files from output_frames directory
output_images = sorted(glob.glob("output_frames/*.jpg"))

# Add each output frame image to the gallery
for img_file in output_images:
    filename = os.path.basename(img_file)
    frame_number = filename.split("_")[1].split(".")[0]
    html_content += f"""
        <div class="image-container">
            <img src="{img_file}" alt="Dual Model Frame {frame_number}">
            <p>Dual Model Frame {int(frame_number)}</p>
        </div>
    """

# Close the HTML
html_content += """
        </div>
    </div>
</body>
</html>
"""

# Write the HTML file
with open("all_segmentation_gallery.html", "w") as f:
    f.write(html_content)

# Get the IP address for SCP instructions
ip_cmd = "hostname -I | awk '{print $1}'"
jetson_ip = subprocess.check_output(ip_cmd, shell=True).decode('utf-8').strip()

print("Complete gallery created! Open all_segmentation_gallery.html in your browser.")
print("\nYou can view this gallery in several ways:")
print("1. On the Jetson directly with: firefox all_segmentation_gallery.html")
print("2. From another computer by copying it with SCP:")
print(f"   scp igvc@{jetson_ip}:~/all_segmentation_gallery.html .") 