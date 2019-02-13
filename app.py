from src.camera import camera
from src.visualization import visualizer_2D
import open3d as pn
import json

cam = camera([640, 480], 30)


cam.settings(json.load(file('custom_params.json')))

depth_vis = visualizer_2D(1)
color_vis = visualizer_2D(2)
while True:
    frame = cam.new_frame()
    depth_vis.update(frame.depth_image)
    color_vis.update(frame.color_image)
    pn.draw_geometries([frame.point_cloud])

cam.stop_recording()
