"""
This script is testint the localizer class
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

target_mesh = read_mesh_file('3d_model/Motor.stl')
target_mesh.compute_vertex_normals()

tr_target_mesh = pn_mesh2tr_mesh(target_mesh)
mesh_1, mesh_2 = tr_target_mesh.split()[[2,8]]
tr_target_mesh = mesh_1 + mesh_2

target_mesh = tr_mesh2pn_mesh(tr_target_mesh)

target_mesh.compute_vertex_normals()
#%% camera annimation

save_sim = []
save_cam_ray_directions = []
for i, theta in enumerate(np.linspace(0,2*np.pi,100)):
    print i 
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
    save_cam_ray_directions.append(cam_ray_directions)
    
    pcl_sim, depth_map = cam.simulate_camera_mesh(target_mesh,real=True)
    save_sim.append(pcl_sim)
    plt.imsave('simulation/depthmap/im%s.png' %i, depth_map)

pickle.dump(cam_ray_directions, open('simulation/cam_ray_directions.save', 'wb'))
pickle.dump(save_sim, open('simulation/save_sim.save', 'wb'))
#%%  
#cam_ray_directions = save_cam_ray_directions
cam_ray_directions = np.concatenate([cam_ray_directions,np.array([[0,0,0]])], axis=0)
pn_cam = pn.PointCloud()
pn_cam.points = pn.Vector3dVector(cam_ray_directions*100)
pn_cam.paint_uniform_color([0, 0.651, 0.929])

source = pn.PointCloud() 

vis = pn.Visualizer()

vis.create_window()
vis.add_geometry(pn_cam)


for i in range(len(save_sim)):
    pcl_sim = save_sim[i]
    source.points = pn.Vector3dVector(pcl_sim)
    source.paint_uniform_color([1, 0.706, 0])
    vis.add_geometry(source)

    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image("simulation/pointcloud2/temp_%04d.jpg" % (i))

vis.destroy_window()

#%%

for i in range(len(save_sim)):
    pcl_sim = save_sim[i]
    source = tr.PointCloud(vertices =pcl_sim )
    source.colors  = np.array([[1, 0.706, 0, 1]]* len(pcl_sim))

    tr_cam = tr.PointCloud(vertices=cam_ray_directions * 20)
    tr_cam.colors  = np.array([[0, 0.651, 0.92, 1]]* len(cam_ray_directions))
    
    scene = tr.Scene([source,tr_cam])
    center =np.array([100,100,0]) + scene.centroid

    scene.set_camera(angles=[np.pi/2,-np.pi/2,0],
                     center= center)
#    scene.show()
#    print scene.scale
    f = file('simulation/pointcloud/cam%s.png' %i,'wr')
    f.write(scene.save_image())
    f.close()

#%%
pn_sim_pcl = []
for pcl_sim in save_sim:
    source = pn.PointCloud() 
    source.points = pn.Vector3dVector(pcl_sim)
    pn_sim_pcl.append(source)
    
positions = [np.array([800*np.cos(theta),
                       800* np.sin(theta),
                       0]) for theta in np.linspace(0,2*np.pi,100)]
#%%
from src.localizer import localizer

loc = localizer(target_mesh)
#%%
import time
save_result_ransac = []
save_result_icp = []
cam_positions = []
times = []
for i in range(len(pn_sim_pcl)):
    start_time = time.time()
    cam_pos, result_ransac, result_icp = loc.camera_position(pn_sim_pcl[i])
    save_result_ransac.append([result_ransac.fitness,result_ransac.transformation])
    save_result_icp.append([result_icp.fitness,result_icp.transformation])
    cam_positions.append(cam_pos)
    print i ,start_time - time.time()
    times.append(start_time - time.time())

#%%
pickle.dump(save_result_ransac, open('simulation/save_result_ransac.save', 'wb'))
pickle.dump(save_result_icp, open('simulation/save_result_icp.save', 'wb'))
pickle.dump(cam_positions, open('simulation/cam_positions.save', 'wb'))
pickle.dump(times, open('simulation/times.save', 'wb'))

#%%
import pickle

save_sim = pickle.load(open('simulation/save_sim.save', 'rb'))
save_result_ransac = pickle.load(open('simulation/save_result_ransac.save', 'rb'))
save_result_icp = pickle.load(open('simulation/save_result_icp.save', 'rb'))
cam_positions = pickle.load(open('simulation/cam_positions.save', 'rb'))
times = pickle.load(file('simulation/times.save', 'rb'))
#%%
vis = pn.Visualizer()

vis.create_window()
vis.add_geometry(target_mesh)
vis.run()
source = pn.PointCloud() 


for i in range(len(save_result_ransac)):

    
    pn_cam = pn.PointCloud()
    pn_cam.points = pn.Vector3dVector(cam_ray_directions*100)
    pn_cam.paint_uniform_color([0, 0.651, 0.929])
    
    source.points = pn.Vector3dVector(save_sim[i])
    source.paint_uniform_color([1, 0.706, 0])
    
    source.transform(save_result_ransac[i][1])
    pn_cam.transform(save_result_ransac[i][1])
    
    vis.add_geometry(source)
    vis.add_geometry(pn_cam)
    
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    
    vis.capture_screen_image("simulation/ransac_registration/temp_%04d.jpg" % (i))

vis.destroy_window()

 #%%
vis = pn.Visualizer()

vis.create_window()
vis.add_geometry(target_mesh)
vis.run()

source = pn.PointCloud() 

for i in range(len(save_result_ransac)):

    
    pn_cam = pn.PointCloud()
    pn_cam.points = pn.Vector3dVector(cam_ray_directions*100)
    pn_cam.paint_uniform_color([0, 0.651, 0.929])
    
    source.points = pn.Vector3dVector(save_sim[i])
    source.paint_uniform_color([1, 0.706, 0])
    
    source.transform(save_result_ransac[i][1])
    pn_cam.transform(save_result_ransac[i][1])
    source.transform(save_result_icp[i][1])
    pn_cam.transform(save_result_icp[i][1])
    
    vis.add_geometry(source)
    vis.add_geometry(pn_cam)
    
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    
    vis.capture_screen_image("simulation/icp_registration/temp_%04d.jpg" % (i))

vis.destroy_window()

    