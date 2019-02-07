import numpy as np
import copy
from open3d import *
import time
from fileIO import read_mesh_file
from camera_simulation import camera, rotate_vectors
import matplotlib.pyplot as plt
from util import mesh2pcl
#%%
target_mesh = read_mesh_file('3d_model/Motor.stl')
target_mesh.compute_vertex_normals()
camera_position = np.array([170,470,0])
vect  = camera_position * -1. / np.sqrt(camera_position.dot(camera_position.T))
orientation = [0,
               -np.arcsin(vect[2]),
               np.pi + np.arctan(vect[1]/vect[0])]
cam = camera(resolution = [120,70], camera_position = camera_position,orientation=orientation)
pcl_sim = cam.simulate_camera_mesh_fast(target_mesh)
pcl_sim = rotate_vectors(pcl_sim,0,0,np.pi)
source = PointCloud()
source.points = Vector3dVector(pcl_sim)
estimate_normals(source, search_param = KDTreeSearchParamHybrid(
        radius =10 , max_nn = 10))

target = mesh2pcl(target_mesh,1)
estimate_normals(target, search_param = KDTreeSearchParamHybrid(
        radius = 5 , max_nn = 30))
cam_pcl = PointCloud()
cam_pcl.points = Vector3dVector(np.array([[0,0,0]]))
#%%
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 150))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size,source,target):
    estimate_normals(source, search_param = KDTreeSearchParamHybrid(
        radius =3 , max_nn = 5))
    estimate_normals(target, search_param = KDTreeSearchParamHybrid(
        radius =3 , max_nn = 5))
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(100000000, 500))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
    return result

#%%
voxel_size = 20 # means 5cm for the dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = \
        prepare_dataset(voxel_size, source, target)

result_ransac = execute_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down,
        result_ransac.transformation)

result_icp = refine_registration(source, target,
        source_fpfh, target_fpfh, voxel_size)
print(result_icp)
draw_registration_result(source, target, result_icp.transformation)

cam_pcl1 = copy.deepcopy(cam_pcl)
cam_pcl1.transform(result_ransac.transformation)
print np.asarray(cam_pcl1.points)
cam_pcl1 = copy.deepcopy(cam_pcl)
cam_pcl1.transform(result_icp.transformation)
print np.asarray(cam_pcl1.points)