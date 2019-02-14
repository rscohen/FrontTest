from src.camera import camera
from src.visualization import visualizer_2D, visualizer_3D
from src.localizer import localizer
import time
import numpy as np
import json

cam = camera([640, 480], 30)


cam.settings(json.load(file('custom_params.json')))

depth_vis = visualizer_2D(1)
color_vis = visualizer_2D(2)

#%%
loc = localizer()
loc.source_to_target_transformation = np.eye(4)
#vis_3d = visualizer_3D(loc.source_pcl)
cam_point_clouds = []
cam_depth_images = []
cam_color_images = []
for i in range(30):
    start_time  = time.time()
    frame = cam.new_frame()
    depth_vis.update(frame.depth_image)
    color_vis.update(frame.color_image)
#    loc.update_source(frame.point_cloud)
    cam_point_clouds.append(np.asarray(frame.point_cloud.points))
    cam_depth_images.append(frame.depth_image)
    cam_color_images.append(frame.color_image)
#    vis_3d.update_camera_coordinates(loc.camera_coordinates())
#    vis_3d.update_trace(loc.camera_coordinates_history())
    print time.time() - start_time
cam.stop_recording()
#%%
#np.save("simulation/cam_point_clouds_real", cam_point_clouds)
#np.save("simulation/cam_depth_images_real", cam_depth_images)
#np.save("simulation/cam_color_images_real", cam_color_images)


#%%


