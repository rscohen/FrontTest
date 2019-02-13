"""
This script implement the class that does the localisation 
"""

import open3d as pn
import numpy as np
from src.util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh, fragment_pcl, list_apply
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



def icp_registration(source, target, distance_threshold):
    pn.estimate_normals(source)
    pn.estimate_normals(target)
    icp_result = pn.registration_icp(source, target,
                                     distance_threshold,
                                     np.eye(4),
                                     pn.TransformationEstimationPointToPlane())
    return icp_result


class localizer(object):
    """ 
    Class for computing the camera position. 
      
    Attributes: 
        model_mesh (open3d.TriangleMesh) :
        target_pcl (opend3d.PointCloud) : 
        previous_transformation (numpy.ndarray) : 
        source_pcl (open3d.PointCloud) : 
        source_to_target_transformation (numpy.ndarray) : 
        previous_transformations (numpy.ndarray) :
    """
    
    def __init__(self, model_mesh):
        """ 
        Constructor for localizer class. 
  
        Parameters: 
            model_mesh (open3d.TriangleMesh) :
        """
        self.model_mesh = model_mesh  
        pym_target_mesh = pn_mesh2pym_mesh(self.model_mesh)
        
        # Computing model outer hull 
        pym_target_outer_hull = pym.outerhull.compute_outer_hull(pym_target_mesh)
        pn_target_outer_hull = pym_mesh2pn_mesh(pym_target_outer_hull)
        
        # the model_mesh is supposed to be the target (ie the known environement observed 
        # by the camera)
        
        # Converting the target mesh to a target pcl
        self.target_pcl = mesh2pcl(pn_target_outer_hull, .5)
        self.previous_transformation = np.array([])
        self.previous_transformations = []
        self.source_to_target_transformation = np.array([])
        self.source_pcl = pn.PointCloud()
    
    def update_source(self, camera_point_cloud):
        """
        Function to add new camera frame data to the source point cloud
        
        Parameters: self.source_pcl
            camera_point_cloud (open3d.PointCloud) : point cloud from the camera frame 
        """
        camera_pcl = copy.deepcopy(camera_point_cloud)
        
        if self.source_pcl.is_empty():
            self.source_pcl = self.source_pcl + camera_pcl
            return
        else:
            icp_result = icp_registration(self.source_pcl,
                                          camera_pcl,
                                          3)
            if icp_result.fitness > 0.95:
                icp_result.transformation = icp_result.transformation
                self.previous_transformations.append(icp_result.transformation)
                self.source_pcl.transform(icp_result.transformation)
                self.source_pcl = self.source_pcl + camera_pcl
                self.source_pc = pn.voxel_down_sample(self.source_pc, 1)
                self.source_to_target_transformation = \
                    self.source_to_target_transformation.dot(icp_result.transformation)
        if len(self.previous_transformations) > 300:
            self.previous_transformations.pop(0)
        return
    
    
    def update_source_to_target_transformation(self):
        """
        Function to update the source to target transformation matrix
        
         
        """
        source = copy.deepcopy(self.source_pcl)
        target = pn.voxel_down_sample(self.target_pcl,
                                      target_voxel_size)
        # PREPROCESSING POINt CLOUD
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
        
        # GLOBAL REGISTRATION
        
        
        return
    
    
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