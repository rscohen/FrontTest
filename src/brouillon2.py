import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        plt.figure(1); plt.clf()
        plt.imshow(depth_image)
        plt.pause(.01)
        plt.figure(2); plt.clf()
        plt.imshow(color_image)
        plt.pause(.01)
finally:

    # Stop streaming
    pipeline.stop()
