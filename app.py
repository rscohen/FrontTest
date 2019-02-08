import pyrealsense2 as rs
import numpy as np
from src.visualization import visualizer_2D
import open3d as pn
# Configure depth and color streams

pipeline = rs.pipeline()
pc = rs.pointcloud()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

depth_vis = visualizer_2D(1)
color_vis = visualizer_2D(2)

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_vis.update(depth_image)
    color_image = np.asanyarray(color_frame.get_data())
    color_vis.update(color_image)
    
    #convert depth frame to pcl
    points = pc.calculate(depth_frame)
    vtx = np.asarray(points.get_vertices(),dtype=np.ndarray)
    vtx = np.array(list(vtx))
    source = pn.PointCloud() 
    source.points = pn.Vector3dVector(vtx)
    source.paint_uniform_color([1, 0.706, 0])
    pn.draw_geometries([source])