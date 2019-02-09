"""
This script implement the class that does the localisation 
"""

import open3d as pn
import numpy as np
from src.util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh, fragment_pcl
import pymesh as pym
import copy 

target_voxel_size = 1
voxel_size = 5
distance_threshold_ransac = 6 * voxel_size
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

class localizer(object):
    def __init__(self, pn_tagert_mesh):
        self.pn_target_mesh = pn_tagert_mesh  
        self.pym_target_mesh = pn_mesh2pym_mesh(self.pn_target_mesh)
        print 'OUTER HULL ----------------------'
        self.pym_target_outer_hull = pym.outerhull.compute_outer_hull(self.pym_target_mesh)
        self.pn_target_outer_hull = pym_mesh2pn_mesh(self.pym_target_outer_hull)
        print 'OUTER HULL --------------------OK'
        print 'POINT CLOUD CONVERTION-----------'
        self.pn_target_outer_hull_pcl = mesh2pcl(self.pn_target_outer_hull, .5)
        print 'POINT CLOUD CONVERTION---------OK'
        self.previous_transformation = np.array([])
        
    def camera_position(self,pn_cam_pcl):
        cam_pose_pcl = pn.PointCloud()
        cam_pose_pcl.points = pn.Vector3dVector(np.array([[0,0,0]]))  
        source = copy.deepcopy(pn_cam_pcl)
        target = pn.voxel_down_sample(self.pn_target_outer_hull_pcl,
                                      target_voxel_size)
        print 'PREPROCESS POINT CLOUD-----------'
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
        print 'PREPROCESS POINT CLOUD---------OK'
        print 'GLOBAL REGISTRATION--------------'
        if self.previous_transformation.any():
            cam_pose_pcl.transform(self.previous_transformation)
            source.transform(self.previous_transformation)
    
            result_icp = pn.registration_icp(source, target,
                                             distance_threshold_icp,
                                             np.eye(4),
                                             pn.TransformationEstimationPointToPlane())
            self.previous_transformation = result_icp.transformation.dot(self.previous_transformation)
            cam_pose_pcl.transform(result_icp.transformation)
            
            return cam_pose_pcl.points[0]
        
        cam_pose_pcl = pn.PointCloud()
        cam_pose_pcl.points = pn.Vector3dVector(np.array([[0,0,0]]))  
        source = copy.deepcopy(pn_cam_pcl)
        
        result_ransac = pn.registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh,
                distance_threshold_ransac,
                pn.TransformationEstimationPointToPoint(False), 4,
                [
                        pn.CorrespondenceCheckerBasedOnDistance(distance_threshold_ransac)
                ],
                pn.RANSACConvergenceCriteria(10000000, 10000))
        
#        if result_ransac.fitness < 0.7:
#            print 'GLOBAL REGISTRATION----------FAIL'
#            return np.array([0,0,0])

        print 'GLOBAL REGISTRATION------------OK'
        print 'ICP REGISTRATION-----------------'

        cam_pose_pcl.transform(result_ransac.transformation)
        source.transform(result_ransac.transformation)

        result_icp = pn.registration_icp(source, target,
                                         distance_threshold_icp,
                                         np.eye(4),
                                         pn.TransformationEstimationPointToPlane())
        
#        if result_icp.fitness < 0.8:
#            print 'ICP REGISTRATION-----------------'
#            return np.array([0,0,0])
 
        print 'ICP REGISTRATION---------------OK'
        cam_pose_pcl.transform(result_icp.transformation)
        self.previous_transformation = result_icp.transformation.dot(result_ransac.transformation)
        return cam_pose_pcl.points[0]