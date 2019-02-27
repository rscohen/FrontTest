from src.camera import camera
from src.visualization import visualizer_2D
import time

import json

cam = camera([640, 480], 30)


cam.settings(json.load(file('custom_params.json')))

depth_vis = visualizer_2D(1)
color_vis = visualizer_2D(2)
times = []

for i in range(100):
    start_time  = time.time()
    frame = cam.new_frame()
    print frame.point_cloud
    print (time.time()-start_time)
    depth_vis.update(frame.depth_image)
    color_vis.update(frame.color_image)

cam.stop_recording()

