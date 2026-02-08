import cv2
import os
import re

path = "../frames"

frame_files = sorted(
    [f for f in os.listdir(path) if f.startswith("frame_") and f.endswith(".png")],
    key=lambda x: int(re.findall(r'\d+', x)[0])
)

first_frame = cv2.imread(os.path.join(path, frame_files[0]))
height, width, layers = first_frame.shape

output_video = "output6.mp4"

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 50
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

for frame_file in frame_files:
    frame = cv2.imread(os.path.join(path, frame_file))
    video.write(frame)

video.release()

print(f"Video saved as {output_video}")
