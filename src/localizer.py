"""
This script implement the class that does the localisation 
"""

import open3d as pn
import numpy as np
from src.util import mesh2pcl, pn_mesh2pym_mesh , pym_mesh2pn_mesh, fragment_pcl
import pymesh as pym
import copy 
from numpy.linalg import inv


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
            icp_result.transformation = icp_result.transformation
            print icp_result.fitness
            self.previous_transformations.append(icp_result.transformation)
            self.source_pcl.transform(icp_result.transformation)
            self.source_pcl = self.source_pcl + camera_pcl
            self.source_pcl = pn.voxel_down_sample(self.source_pcl, 2)
            if self.source_to_target_transformation.any():
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
        
        if self.source_to_target_transformation.size > 0:
            # if the global registration have already been done
            source.transform(self.source_to_target_transformation)
            icp_result = pn.registration_icp(source, target,
                                             distance_threshold_icp,
                                             np.eye(4),
                                             pn.TransformationEstimationPointToPlane())
            if icp_result.fitness > 0.8:
                self.source_to_target_transformation =\
                    icp_result.transformation.dot(self.source_to_target_transformation)
                return
        
        # we fail to align source & target with ICP
        # GLOBAL REGISTRATION
        # PREPROCESSING POINT CLOUD
        source = copy.deepcopy(self.source_pcl)
        
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
        
        # GLOBAL REGISTRATION 
        ransac_result = pn.registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh,
                distance_threshold_ransac,
                pn.TransformationEstimationPointToPoint(False), 4,
                [
                        pn.CorrespondenceCheckerBasedOnDistance(distance_threshold_ransac)
                ],
                pn.RANSACConvergenceCriteria(10000000, 10000))
        
        source.transform(ransac_result.transformation)

        icp_result = pn.registration_icp(source, target,
                                         distance_threshold_icp,
                                         np.eye(4),
                                         pn.TransformationEstimationPointToPlane())
        
        self.source_to_target_transformation = \
            icp_result.transformation.dot(ransac_result.transformation)
        
        return
    
    def camera_coordinates(self):
        """
        Function to retrive the camera coordinates (O, i, j, k) system in the model 
        coordinates system
        
        Return 
            O (numpy.ndarray) : the origin
            i (numpy.ndarray) :
            j (numpy.ndarray) :
            k (numpy.ndarray) :
        """
        if self.source_to_target_transformation.size == 0:
            print "You need to update source to targt transformation first"
            return
        
        coordinates_system =   pn.PointCloud()
        coordinates_system.points = pn.Vector3dVector(np.array([[0, 0, 0],
                                                                [1, 0, 0],
                                                                [0, 1, 0],
                                                                [0, 0, 1]]))
        
        coordinates_system.transform(self.source_to_target_transformation)
        
        return [coordinates_system.points[i] for i in range(4)]
    
    
    def camera_coordinates_history(self):
        """
        Function to retrive the previous camera coordinates (O, i, j, k) system in the model 
        coordinates system
        
        Return 
            camera_coordinates_history (list): 
        """
        if self.source_to_target_transformation.size == 0:
            print "You need to update source to target transformation first"
            return
        
        coordinates_system =   pn.PointCloud()
        coordinates_system.points = pn.Vector3dVector(np.array([[0, 0, 0],
                                                                [1, 0, 0],
                                                                [0, 1, 0],
                                                                [0, 0, 1]]))
        
        coordinates_system.transform(self.source_to_target_transformation)
        coordinates_history = []
        coordinates_history.append(np.asarray(coordinates_system.points).copy())
        for previous_transformation in self.previous_transformations[::-1]:
            coordinates_system.transform(inv(previous_transformation))
            coordinates_history.append(np.asarray(coordinates_system.points).copy())
        return coordinates_history[::-1]