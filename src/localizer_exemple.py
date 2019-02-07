import open3d as pn
import numpy as np
from util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh
import pymesh as pym
from fileIO import read_mesh_file
from util import pn_mesh2tr_mesh, tr_mesh2pn_mesh,fragment_pcl
from camera_simulation import camera
import copy
import time


#%% CREATE TARGET MODEL
target_mesh = read_mesh_file('../3d_model/Motor.stl')

tr_target_mesh = pn_mesh2tr_mesh(target_mesh)
mesh_1, mesh_2 = tr_target_mesh.split()[[2,8]]
tr_target_mesh = mesh_1 + mesh_2

target_mesh = tr_mesh2pn_mesh(tr_target_mesh)
target_mesh.compute_vertex_normals()

#%% SIMULATE CAMERA
theta = np.pi / 2
camera_position = np.array([800*np.cos(theta),
                                800* np.sin(theta),
                                0])
vect = tr_target_mesh.centroid - camera_position
vect  = vect * -1. / np.sqrt(vect.dot(vect.T))
orientation = [0,
               +np.arcsin(vect[2]),
               np.pi + theta]
cam = camera(resolution = [224,171], camera_position = camera_position,orientation=orientation)
ray_directions, cam_ray_directions = cam.ray_directions()

pcl_sim, depth_map = cam.simulate_camera_mesh(target_mesh,real=True)

#%% CREATE SOURCE MODEL
source = pn.PointCloud() 
source.points = pn.Vector3dVector(pcl_sim)
source.paint_uniform_color([1, 0.706, 0])

#%% TEST THE LOCALIZER
from localizer import localizer

loc = localizer(target_mesh)

cam_pos, result_ransac, result_icp = loc.camera_position(source)

print cam_pos,
print camera_position