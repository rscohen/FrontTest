"""
This script is testing the localizer class
"""

import open3d as pn
import numpy as np
from src.util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh
import pymesh as pym
from src.fileIO import read_mesh_file
from src.util import pn_mesh2tr_mesh, tr_mesh2pn_mesh,fragment_pcl
from src.camera_simulation import camera
import matplotlib.pyplot as plt
import pickle
import trimesh as tr
import time
#%%
target_mesh = read_mesh_file('3d_model/Motor.stl')
target_mesh.compute_vertex_normals()

tr_target_mesh = pn_mesh2tr_mesh(target_mesh)
mesh_1, mesh_2 = tr_target_mesh.split()[[2,8]]
tr_target_mesh = mesh_1 + mesh_2

target_mesh = tr_mesh2pn_mesh(tr_target_mesh)

target_mesh.compute_vertex_normals()
#%% camera circular trajectory settings
V = 2 # m/s
fps = 5
simulation_time = 10 # s
start_angle = 0 # degree
Z = 0
radius = 800 # mmm

dist = V * simulation_time * 100 # mm
nb_of_frames = fps * simulation_time
rotation_angle =  dist / (2 * np.pi * radius)
#%% CAMERA SIMULATION

save_sim = []
cam_ray_directions = []
cam_positions = []
cam_point_clouds = []
cam_depth_images = []
for i, theta in enumerate(np.linspace(0, rotation_angle, nb_of_frames)):
    print i 
    theta = theta + start_angle
    camera_position = np.array([radius*np.cos(theta),
                                radius*np.sin(theta),
                                Z])
    cam_positions.append(camera_position)
    vect = tr_target_mesh.centroid - camera_position
    vect  = vect * -1. / np.sqrt(vect.dot(vect.T))
    orientation = [0,
                   +np.arcsin(vect[2]),
                   np.pi + theta]
    
    cam = camera(resolution = [224,171],
                 camera_position = camera_position,
                 orientation=orientation)
    ray_directions, cam_ray_direction = cam.ray_directions()
    cam_ray_directions.append(cam_ray_direction)
    
    cam_point_cloud, depth_image = cam.simulate_camera_mesh(target_mesh,real=True, err=0)
    cam_point_clouds.append(cam_point_cloud)
    cam_depth_images.append(depth_image)

cam_point_clouds = np.array(cam_point_clouds)
cam_depth_images = np.array(cam_depth_images)
file_name = "V_%s_fps_%s_simulation_time_%s_start_angle_%s_Z_%s_radius_%s" %(V, fps, simulation_time, start_angle, Z, radius)

np.save("simulation/cam_point_clouds_" + file_name, cam_point_clouds)
np.save("simulation/cam_depth_images_" + file_name, cam_depth_images)

#%% PLOTING CAM POINT CLOUD

source = pn.PointCloud() 

vis = pn.Visualizer()
vis.create_window()
for i in range(10):
    for i, point_cloud in enumerate(cam_point_clouds):
        time.sleep(1/fps)
        source.points = pn.Vector3dVector(point_cloud)
        source.paint_uniform_color([1, 0.706, 0])
        vis.add_geometry(source)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()

vis.destroy_window()

#%% PLOTING CAM DEPTH MAP

for i in range(50):
    for i, depth_image in enumerate(cam_depth_images):
        plt.figure(1); plt.clf()
        plt.imshow(depth_image)
        plt.pause(1/fps)

#%% CONVERTING NUMPY PCL LIST TO open3d PCL LIST
        
cam_pcl_list = []
for point_cloud in cam_point_clouds:
    cam_pcl = pn.PointCloud() 
    cam_pcl.paint_uniform_color([1, 0.706, 0])
    cam_pcl.points = pn.Vector3dVector(point_cloud)
    cam_pcl_list.append(cam_pcl)

#%% INITIALISING LOCALIZER
    
from src.localizer import localizer

loc = localizer(target_mesh) 

#%% TESTINT LOCALIZER
import time
save_transfomation = []
cam_positions = []
times = []

for cam_pcl in cam_pcl_list:
    start_time = time.time()
    loc.update_source(cam_pcl)
    print start_time - time.time()
# PI
pn.draw_geometries([loc.source_pcl])

loc.update_source_to_target_transformation()
loc.camera_coordinates()
loc.camera_coordinates_history()