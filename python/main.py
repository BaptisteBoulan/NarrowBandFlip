import cv2
import os
import re

# Directory containing the frames
path = "../frames"

# Get all frame files and sort them numerically
frame_files = sorted(
    [f for f in os.listdir(path) if f.startswith("frame_") and f.endswith(".png")],
    key=lambda x: int(re.findall(r'\d+', x)[0])
)

# Get the dimensions of the first frame to set the video writer
first_frame = cv2.imread(os.path.join(path, frame_files[0]))
height, width, layers = first_frame.shape

# Define the output video file
output_video = "output.mp4"

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
fps = 50  # Frames per second (adjust as needed)
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# Write each frame to the video
for frame_file in frame_files:
    frame = cv2.imread(os.path.join(path, frame_file))
    video.write(frame)

# Release the video writer
video.release()

print(f"Video saved as {output_video}")
