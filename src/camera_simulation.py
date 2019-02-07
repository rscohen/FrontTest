"""
This script implement a class module that simulate a 3d camera
"""
import numpy as np 
import random
import pandas as pd 
import tqdm
import open3d as pn
from geo_function import norm, scalar_product ,norm_pandas, scalar_product_pandas, scalar_product_vect_pandas
import trimesh as tr

def rotate_vectors(vectors, theta_x, phi_y, omega_z):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta_x), -1 * np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]])
    R_y = np.array([[np.cos(phi_y), 0, np.sin(phi_y)],
                    [0, 1, 0],
                    [-1 * np.sin(phi_y), 0, np.cos(phi_y)]])
    R_Z = np.array([[np.cos(omega_z), -1 * np.sin(omega_z), 0],
                    [np.sin(omega_z), np.cos(omega_z), 0],
                    [0, 0, 1]])
    return R_Z.dot(R_y).dot(R_x).dot(vectors.T).T

def translate_vectors(vectors, translation):
    return vectors + np.array([translation]*vectors.shape[0])


class camera(object):
    def __init__(self, step=0.1,
                 resolution = [224,171],
                 FOV = [62, 45],
                 camera_position = [0, 0, 0],
                 orientation = [0, 0, 0]):
        self.resolution = resolution
        self.FOV = np.array(FOV) * np.pi / 180 #champ de vition de la camera
        self.step = step
        self.camera_position = np.array(camera_position)
        self.orientation = orientation

    
    def ray_directions(self):
        cam_ray_directions = []
        for p in np.linspace(-1. * self.FOV[1] / 2,
                             1. * self.FOV[1] / 2,
                             self.resolution[1]):
            for t in np.linspace(-1. * self.FOV[0] / 2,
                                 1. * self.FOV[0] / 2, 
                                 self.resolution[0]):
                ray = [np.cos(t)*np.cos(p),  np.sin(t)*np.cos(p) , np.sin(p)]
                cam_ray_directions.append(ray)
              
        cam_ray_directions = np.array(cam_ray_directions)
        ray_directions = rotate_vectors(cam_ray_directions,
                                        self.orientation[0],
                                        self.orientation[1],
                                        self.orientation[2])
        return ray_directions, cam_ray_directions
    
    def simulate_camera_pcl(self, pcl, get_inf_value = True):
        result = []
        rays, cam_ray_directions = self.ray_directions()
        for vect in rays:
            projeted_point = self.projet_point(vect, pcl) 
            if get_inf_value or not np.isnan(projeted_point.sum()):
                result.append(projeted_point)
        result = np.array(result)
         
        result = translate_vectors(result, -self.camera_position)
        result = rotate_vectors(result,
                                +self.orientation[0],
                                +self.orientation[1],
                                -self.orientation[2])  
        return result     
    
    def simulate_camera_mesh(self, mesh, real=True, err=0.01):
        trimesh_mesh = tr.Trimesh(vertices=np.asarray(mesh.vertices),
                                  faces=np.asarray(mesh.triangles))
        ray_directions, cam_ray_directions = self.ray_directions()
        results = []
        bar = tqdm.tqdm(total = len(ray_directions))
        depth_map = []
        for i, ray in enumerate(ray_directions):
            locations, index_ray, index_tri = trimesh_mesh.ray.intersects_location(
                    ray_origins=np.array([self.camera_position]),
                    ray_directions=np.array([ray]))
            if len(locations)>0:
                t = min((locations -np.array([self.camera_position]*len(locations))).dot(ray))
                t = t + t * err * random.random()
                depth_map.append(t)
                if real:
                     results.append(t * cam_ray_directions[i])
                else:
                    results.append(self.camera_position + t * ray )
            else :
                depth_map.append(0)
            bar.update(1)
        bar.close()
        depth_map = np.array(depth_map)
        depth_map = depth_map.reshape(self.resolution[::-1])
        results = np.array([list(x) for x in results])
        return results, depth_map
    

    
#def simulate_camera_mesh(self, mesh):
#    
#    vertices = np.asarray(mesh.vertices)
#    vertices = pd.DataFrame(vertices).apply(lambda r: tuple(r), axis=1).apply(np.array)
#    vertices = pd.DataFrame(vertices,columns=['vertex'])
#    triangles = pd.DataFrame(np.asarray(mesh.triangles),columns = ['v1','v2','v3'])
#    triangles['v1'] = triangles.merge(vertices,how='left',left_on='v1', right_index=True)['vertex']
#    triangles['v2'] = triangles.merge(vertices,how='left',left_on='v2', right_index=True)['vertex']
#    triangles['v3'] = triangles.merge(vertices,how='left',left_on='v3', right_index=True)['vertex']
#    
#    #generating orth coordinate system
#    triangles['i'] = triangles['v2'] - triangles['v1'] 
#    triangles['i'] = triangles['i'] / norm_pandas(triangles['i'])
#    triangles['j'] = triangles['v3'] - triangles['v1'] 
#    triangles['j'] = triangles['j'] - scalar_product_pandas(triangles['j'],triangles['i']) * triangles['i']
#    triangles['j'] = triangles['j'] / norm_pandas(triangles['j'])
#    print '---------------'
#    triangles['k'] = triangles.apply(lambda triangle: np.cross(triangle['i'], triangle['j']),
#                     axis=1)
#    print '---------------'
#    triangles['CA.k'] = scalar_product_pandas(triangles['v1'] - self.camera_position,
#             triangles['k'])
#    triangles['A/rk'] = [np.array([0,0])]*len(triangles)
#    
#    AB = triangles['v2'] - triangles['v1'] 
#    triangles['B/rk_i'] = scalar_product_pandas(AB, triangles['i'])
#    triangles['B/rk_j'] = scalar_product_pandas(AB, triangles['j'])
#    
#    AC = triangles['v3'] - triangles['v1'] 
#    triangles['C/rk_i'] = scalar_product_pandas(AC, triangles['i'])
#    triangles['C/rk_j'] = scalar_product_pandas(AC, triangles['j'])
#   
#    triangles['AC'] =  self.camera_position - triangles['v1']
#    results = []
#    vects = []
#    
#    print '---------------'
#    for p in np.linspace(-1. * self.FOV[1] / 2,
#                         1. * self.FOV[1] / 2,
#                         self.resolution[1]):
#        for t in np.linspace(-1. * self.FOV[0] / 2,
#                             1. * self.FOV[0] / 2, 
#                             self.resolution[0]):
#            vect = [np.cos(t)*np.cos(p),  np.sin(t)*np.cos(p) , np.sin(p)]
#            vects.append(vect)
#            
#    vects = np.array(vects)
#    vects = rotate_vectors(vects,
#                           self.orientation[0],
#                           self.orientation[1],
#                           self.orientation[2])   
#    print 'starting simulation'
#    bar = tqdm.tqdm(total = len(vects))
#    for d in vects:
#        tmp = triangles.copy()
#        tmp['d'] = [d]*len(triangles)
#        tmp['t'] = tmp['CA.k'] / scalar_product_vect_pandas(d, tmp['k'])
#        tmp['t'] = tmp['t'].replace([np.inf, -np.inf], np.nan)
#        tmp = tmp[tmp['t']>0]
#        tmp = tmp.dropna(subset=['t'])
#        tmp = tmp.reset_index()
#        tmp['AP'] = tmp['t'] * tmp['d'] + tmp['AC']
#        tmp['X'] = scalar_product_pandas(tmp['AP'], tmp['i'])
#        tmp['Y'] = scalar_product_pandas(tmp['AP'], tmp['j'])
#        
#        tmp['w1'] = - (tmp['Y']*tmp['C/rk_i'] - tmp['X']*tmp['C/rk_j']) / (tmp['C/rk_j']*tmp['B/rk_i'])
#        tmp['w2'] = tmp['Y'] / tmp['C/rk_j']
#        tmp['in_triangle'] = (tmp['w1'] >= 0) & (tmp['w2'] >= 0) & ((tmp['w1'] + tmp['w2']) <= 1)
#        tmp = tmp[tmp['in_triangle']]
#        t = tmp['t'].min()
#        if t > 0 :
#            results.append(self.camera_position + t * d )
#        bar.update(1)
#    bar.close()
#    results = np.array([list(x) for x in results])
#    results = translate_vectors(results, -self.camera_position)
#    results = rotate_vectors(results,
#                            +self.orientation[0],
#                            +self.orientation[1],
#                            -self.orientation[2])
#    return results