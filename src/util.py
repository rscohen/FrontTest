"""
This sript implement function to transform 3d object format 
"""

import open3d as pn
import numpy as np
from geo_function import norm, scalar_product ,norm_pandas, scalar_product_pandas, scalar_product_vect_pandas
import pandas as pd
import tqdm 
import trimesh as tr
import pymesh as pym

def mesh2pcl(mesh, step):
    vertices = np.asarray(mesh.vertices)
    vertices = pd.DataFrame(vertices).apply(lambda r: tuple(r), axis=1).apply(np.array)
    vertices = pd.DataFrame(vertices,columns=['vertex'])
    triangles = pd.DataFrame(np.asarray(mesh.triangles),columns = ['v1','v2','v3'])
    triangles['v1'] = triangles.merge(vertices,how='left',left_on='v1', right_index=True)['vertex']
    triangles['v2'] = triangles.merge(vertices,how='left',left_on='v2', right_index=True)['vertex']
    triangles['v3'] = triangles.merge(vertices,how='left',left_on='v3', right_index=True)['vertex']
    
    #generating orth coordinate system
    triangles['AB'] = triangles['v2'] - triangles['v1'] 
    triangles['|AB|'] = norm_pandas(triangles['AB'])
    triangles['too_sparse'] = triangles['|AB|'] > step
    triangles['AC'] = triangles['v3'] - triangles['v1'] 
    
    triangles['i'] = triangles['AB']
    triangles['i'] = triangles['i'] / triangles['|AB|']
    triangles['j'] = triangles['AC']
    triangles['j'] = triangles['j'] - scalar_product_pandas(triangles['j'],triangles['i']) * triangles['i']
    triangles['j'] = triangles['j'] / norm_pandas(triangles['j'])
    triangles['AC.j'] = scalar_product_pandas(triangles['j'],triangles['AC'])
    triangles['too_sparse'] = triangles['too_sparse'] |( triangles['AC.j'] > step)
    
    points = []
    triangles = triangles[triangles['too_sparse']]
    bar = tqdm.tqdm(total = len(triangles))
    
    for i,triangle in triangles.iterrows():
        y = np.linspace(0, 1, max(0, int(triangle['AC.j']//step)))
        x = np.linspace(0, 1, max(0,int(triangle['|AB|']//step)))
        X,Y = np.meshgrid(x,y)
        XY = np.array([X.flatten(),Y.flatten()]).T
        XY = XY[(XY[:,1] + XY[:,0]<1)&(XY[:,1] + XY[:,0]>0)]
        tmp = np.ones((XY.shape[0],1)).dot(np.array([triangle['v1']])) +\
        np.array([XY[:,0]]).T.dot(np.array([triangle['AB']])) +\
        np.array([XY[:,1]]).T.dot(np.array([triangle['AC']]))
        points.append(tmp)
        
        bar.update(1)
    bar.close()
    points = np.concatenate(points)
    points = np.concatenate((points,np.asarray(mesh.vertices)), axis=0)
    pcl = pn.PointCloud()
    pcl.points = pn.Vector3dVector(points)
    return pcl


def pn_mesh2tr_mesh(pn_mesh):
    tr_mesh = tr.Trimesh(vertices=np.asarray(pn_mesh.vertices),
                         faces=np.asarray(pn_mesh.triangles))
    
    return tr_mesh

def tr_mesh2pn_mesh(tr_mesh):
    pn_mesh = pn.TriangleMesh()
    pn_mesh.vertices = pn.Vector3dVector(np.array(tr_mesh.vertices))
    pn_mesh.triangles = pn.Vector3iVector(np.array(tr_mesh.faces))
    return pn_mesh


def merge_tr_mesh(mesh_1, mesh_2):
    if len(mesh_1.vertices) > len(mesh_2.vertices) :
        mesh_1, mesh_2 = mesh_2.copy(), mesh_1.copy()
        
    mesh_1_face_to_remove = []
    mesh_2_face_to_remove = []

    for face_id, normal in enumerate(mesh_1.face_normals):
        x = normal.dot(mesh_2.face_normals.T)
 
def pn_mesh2pym_mesh(pn_mesh):
    pym_mesh = pym.form_mesh(vertices=np.asarray(pn_mesh.vertices),
                             faces=np.asarray(pn_mesh.triangles))
    return pym_mesh

def pym_mesh2pn_mesh(pym_mesh):
    pn_mesh = pn.TriangleMesh()
    pn_mesh.vertices = pn.Vector3dVector(pym_mesh.vertices)
    pn_mesh.triangles = pn.Vector3iVector(pym_mesh.faces)
    return pn_mesh

def merge_mesh_list(tr_mesh_list):
    if len(tr_mesh_list) == 1:
        return tr_mesh_list[0]
    
    mesh_1 = tr_mesh_list[0]
    mesh_2 =  merge_mesh_list(tr_mesh_list[1:])
    
    return merge_tr_mesh(mesh_1, mesh_2)

def fragment_pcl(pn_pcl, min_dist):
    max_bound = pn_pcl.get_max_bound()
    min_bound = pn_pcl.get_min_bound()

    dims = max_bound - min_bound
    minor_ax = np.argmin(dims)
    
    step_nb = dims // min_dist
    step_nb[minor_ax] = 2
    
    step = [dims[i] / step_nb[i] for i in range(3)]
        
    fragments = []
    for x in np.linspace(min_bound[0], max_bound[0], int(step_nb[0])+1)[:-1]:
        for y in np.linspace(min_bound[1], max_bound[1], int(step_nb[1])+1)[:-1]:
            for z in np.linspace(min_bound[2], max_bound[2], int(step_nb[2])+1)[:-1]:
                bound = np.array([x, y, z])
                fragment = pn.crop_point_cloud(pn_pcl, bound, bound +step)
                fragments.append(fragment)
    return fragments



def transformation_matrix(source_points, target_points):
    """
    Compute the transformation matrix between to set of matching points. 
  
    Parameters: 
        source_points (numpy.arrays): the source points array 
        target_points (numpy.arrays): the target points array matching the 
            source
        
    Returns: 
        transformation_matrix (numpy.arrays) : The 4D transformation matrix from source to target
        
    """
    assert(len(source_points)>=3 and len(target_points)==len(source_points))
    source = pn.PointCloud()
    source.points = pn.Vector3dVector(source_points)
    target = pn.PointCloud()
    target.points = pn.Vector3dVector(target_points)
    corr = np.array(2*[range(len(source_points))]).T
    p2p = pn.TransformationEstimationPointToPoint()
    trans = p2p.compute_transformation(source, target,
             pn.Vector2iVector(corr))
    return trans