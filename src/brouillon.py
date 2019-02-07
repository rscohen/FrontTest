import open3d as pn
import numpy as np
from util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh
import pymesh as pym
from fileIO import read_mesh_file
from util import pn_mesh2tr_mesh, tr_mesh2pn_mesh,fragment_pcl
from camera_simulation import camera
import copy
import time

target_mesh = read_mesh_file('3d_model/Motor.stl')
#%%
tr_target_mesh = pn_mesh2tr_mesh(target_mesh)
mesh_1, mesh_2 = tr_target_mesh.split()[[2,8]]
tr_target_mesh = mesh_1 + mesh_2

target_mesh = tr_mesh2pn_mesh(tr_target_mesh)
target_mesh.compute_vertex_normals()

#%%
pym_tagert_mesh = pn_mesh2pym_mesh(target_mesh)
pym_target_outer_hull = pym.outerhull.compute_outer_hull(pym_tagert_mesh)
pn_target_outer_hull = pym_mesh2pn_mesh(pym_target_outer_hull)

pn_target_outer_hull_pcl = mesh2pcl(pn_target_outer_hull,.5)
fragments = fragment_pcl(pn_target_outer_hull_pcl,150)

#%%
theta = np.pi 
D = 600
camera_position = D * np.array([np.cos(theta),
                                np.sin(theta),
                                0])

vect = tr_target_mesh.centroid - camera_position
vect  = vect * -1. / np.sqrt(vect.dot(vect.T))

orientation = [0,
               np.arcsin(vect[2]),
               np.pi + theta]

cam = camera(resolution = [640,360],
             FOV = [74,62],
             camera_position=camera_position,
             orientation=orientation)

pcl_sim, depth_map = cam.simulate_camera_mesh(pn_target_outer_hull,err = 0.02)

source_save = pn.PointCloud()
source_save.points = pn.Vector3dVector(pcl_sim)

pn.estimate_normals(source_save, search_param = pn.KDTreeSearchParamHybrid(
        radius =10 , max_nn = 10))

#%%
target_voxel_size = 1
voxel_size = 5
distance_threshold_icp = 7 * voxel_size

def preprocess_point_cloud(pcd, voxel_size):
    pn.estimate_normals(pcd)
    
    pcd_down = pn.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    pn.estimate_normals(pcd_down, pn.KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 10))

    radius_feature = voxel_size * 5
    pcd_fpfh = pn.compute_fpfh_feature(pcd_down,
                                       pn.KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 50))
    return pcd_down, pcd_fpfh

cam_pose_pcl = pn.PointCloud()
cam_pose_pcl.points = pn.Vector3dVector(np.array([[0,0,0]]))  

source = copy.deepcopy(source_save)
target = pn.voxel_down_sample(pn_target_outer_hull_pcl,
                              target_voxel_size)

target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)

source_down.paint_uniform_color([0, 0.651, 0.929])
target_down.paint_uniform_color([1, 0.706, 0])
pn.draw_geometries([source_down, target_down])

distance_threshold_ransac = 6 * voxel_size
start_time = time.time()


result_ransac = pn.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        distance_threshold_ransac,
        pn.TransformationEstimationPointToPoint(False), 4,
        [
            pn.CorrespondenceCheckerBasedOnDistance(distance_threshold_ransac),
        ],
        pn.RANSACConvergenceCriteria(10000000, 10000))

print result_ransac
print time.time() -start_time
cam_pose_pcl.transform(result_ransac.transformation)
source.transform(result_ransac.transformation)
source_down.transform(result_ransac.transformation)

pn.draw_geometries([source_down, target_down])
print cam_pose_pcl.points[0]
#%%
target_voxel_size = 1
voxel_size = 5
distance_threshold_icp = 7 * voxel_size

def preprocess_point_cloud(pcd, voxel_size):
    pn.estimate_normals(pcd)
    
    pcd_down = pn.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    pn.estimate_normals(pcd_down, pn.KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 10))

    radius_feature = voxel_size * 5
    pcd_fpfh = pn.compute_fpfh_feature(pcd_down,
                                       pn.KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 50))
    return pcd_down, pcd_fpfh

cam_pose_pcl = pn.PointCloud()
cam_pose_pcl.points = pn.Vector3dVector(np.array([[0,0,0]]))  

source = copy.deepcopy(source_save)
target = pn.voxel_down_sample(pn_target_outer_hull_pcl,
                              target_voxel_size)

target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
source_down.paint_uniform_color([0, 0.651, 0.929])
target_down.paint_uniform_color([1, 0.706, 0])
pn.draw_geometries([source_down, target_down])

distance_threshold = 3 * voxel_size
start_time = time.time()

result = pn.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        pn.FastGlobalRegistrationOption(
        maximum_correspondence_distance = distance_threshold))
print result
print time.time() -start_time
cam_pose_pcl.transform(result.transformation)
source.transform(result.transformation)
source_down.transform(result.transformation)

pn.draw_geometries([source_down, target_down])
print cam_pose_pcl.points[0]
#%%
result_icp = pn.registration_icp(source, target,
                                 distance_threshold_icp,
                                 np.eye(4),
                                 pn.TransformationEstimationPointToPlane())
print result_icp
cam_pose_pcl.transform(result_icp.transformation)
source.transform(result_icp.transformation)
pn.draw_geometries([target_mesh, source])
print cam_pose_pcl.points[0]
err = camera_position - cam_pose_pcl.points[0]
err = np.sqrt(err.dot(err))
print err
#%%
import pyrealsense2 as rs
pipeline = rs.pipeline()
pipeline.start()
pc = rs.pointcloud()

for i in range(5):
    frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
points = pc.calculate(depth)
vtx = np.asanyarray(points.get_vertices())
pipeline.stop()

vtx = np.array([list(x) for x in vtx ])
vtx = np.array([list(x) for x in vtx if x.dot(x) >0])
scene = pn.PointCloud()
scene.points = pn.Vector3dVector(vtx)
tmp = vtx[vtx[:,2]<0.4]
scene = pn.PointCloud()
scene.points = pn.Vector3dVector(tmp)
pn.draw_geometries([scene])