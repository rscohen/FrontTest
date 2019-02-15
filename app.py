from src.camera import camera
from src.visualization import visualizer_2D, visualizer_3D
from src.fileIO import read_mesh_file
import matplotlib.pyplot as plt

import time
import numpy as np
import json
import open3d as pn
import copy

target_mesh = read_mesh_file('3d_model/InBolt_Proto.stl')
target_mesh.vertices =  pn.Vector3dVector(np.asarray(target_mesh.vertices)*1000)
target_mesh.compute_vertex_normals()

#%%
cam = camera([640, 480], 30)


cam.settings(json.load(file('custom_params.json')))

depth_vis = visualizer_2D(1)
color_vis = visualizer_2D(2)

#%%


#vis_3d = visualizer_3D(loc.source_pcl)
cam_depth_images = []
cam_color_images = []
frame = cam.new_frame()

cam_depth_images = np.array([frame.depth_image])
cam_color_images = np.array([frame.color_image])
#cam_point_clouds = np.array([np.asarray(frame.point_cloud.points)],dtype = np.ndarray)
cam_point_clouds = [np.asarray(frame.point_cloud.points)]
for i in range(30):
    start_time  = time.time()
    frame = cam.new_frame()
    depth_vis.update(frame.depth_image)
    color_vis.update(frame.color_image)
#    loc.update_source(frame.point_cloud)

#    cam_point_clouds = np.append(cam_point_clouds, np.array([np.asarray(frame.point_cloud.points)]), axis=0)
    cam_point_clouds.append(np.asarray(frame.point_cloud.points))
    cam_color_images = np.append(cam_color_images, np.array([frame.color_image]), axis=0)
    cam_depth_images = np.append(cam_depth_images, np.array([frame.depth_image]), axis=0)

#    cam_depth_images.append(frame.depth_image)
#    cam_color_images.append(frame.color_image)
#    vis_3d.update_camera_coordinates(loc.camera_coordinates())
#    vis_3d.update_trace(loc.camera_coordinates_history())
    print time.time() - start_time
cam_point_clouds = np.array(cam_point_clouds)
#cam.stop_recording()

#%% PLOTING RECORDED POINTCLOUD

source = pn.PointCloud() 

vis = pn.Visualizer()
vis.create_window()
for i, point_cloud in enumerate(cam_point_clouds):
    time.sleep(0.1)
    source.points = pn.Vector3dVector(point_cloud)
    source.paint_uniform_color([1, 0.706, 0])
    vis.add_geometry(source)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()

#%% SAVING 

np.save("simulation/cam_point_clouds_real", cam_point_clouds)
np.save("simulation/cam_depth_images_real", cam_depth_images)
np.save("simulation/cam_color_images_real", cam_color_images)


#%% CONVERTING NUMPY PCL LIST TO open3d PCL LIST
        
cam_pcl_list = []
for point_cloud in cam_point_clouds:
    cam_pcl = pn.PointCloud() 
    cam_pcl.paint_uniform_color([1, 0.706, 0])
    cam_pcl.points = pn.Vector3dVector(point_cloud)
    cam_pcl_list.append(cam_pcl)
    
#%% LOCALIZER
from src.localizer import localizer

loc = localizer(target_mesh) 

for cam_pcl in cam_pcl_list:
    start_time = time.time()
    loc.update_source(cam_pcl)
    print start_time - time.time()


#%%

loc.update_source_to_target_transformation()

source = copy.deepcopy(loc.source_pcl)
source.transform(loc.source_to_target_transformation)
origines = [x[0] for x in loc.camera_coordinates_history()]

camera_trace = pn.LineSet()
camera_trace.points = pn.Vector3dVector(origines)
camera_trace.lines = pn.Vector2iVector([[i,i+1] for i in range(len(origines)-1)])

pn.draw_geometries([target_mesh, source, camera_trace])



#%% SAVING COLOR IMAGE

for i, color_image in enumerate(cam_color_images):
    plt.figure(1)
    fig = plt.imshow(color_image)
    fig.axes.get_xaxis().set_visible(False)
    fig.axes.get_yaxis().set_visible(False)
    plt.savefig("simulation/cam_color_images/%s" %i)

#%% SAVING DEPTH IMAGE

for i, depth_image in enumerate(cam_depth_images):
    plt.figure(1)
    fig =plt.imshow(depth_image*10)
    fig.axes.get_xaxis().set_visible(False)
    fig.axes.get_yaxis().set_visible(False)
    plt.savefig("simulation/cam_depth_images/%s" %i)

#%% LOCALIZER ANNIMATION



