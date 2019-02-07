"""
This script implement function to import 3D model
"""

import numpy as np
from stl import mesh 
import open3d as pn
import pandas as pd

def stl2ply(stl_mesh):
    points = pd.DataFrame(stl_mesh.v0).append(pd.DataFrame(stl_mesh.v1)).append(pd.DataFrame(stl_mesh.v2))
    points = points.drop_duplicates()
    points = points.reset_index(drop=True)
    points = pd.DataFrame(points.values, columns=['x','y','z'])
    points['index'] = points.index
    edge = pd.DataFrame(stl_mesh.points,columns=['x0','y0','z0','x1','y1','z1','x2','y2','z2'])
    edge['id0'] = edge.merge(points,how='left',left_on=['x0','y0','z0'],right_on=['x','y','z'])['index']
    edge['id1'] = edge.merge(points,how='left',left_on=['x1','y1','z1'],right_on=['x','y','z'])['index']
    edge['id2'] = edge.merge(points,how='left',left_on=['x2','y2','z2'],right_on=['x','y','z'])['index']
    
    edge = edge[['id0','id1','id2']].values
    points = points[['x','y','z']].values
    
    ply_mesh = pn.TriangleMesh()
    ply_mesh.vertices =  pn.Vector3dVector(points)
    ply_mesh.triangles = pn.Vector3iVector(edge)
    ply_mesh.triangle_normals = pn.Vector3dVector(stl_mesh.normals)
    return ply_mesh
    
def read_mesh_file(file_adress):
    extention = file_adress.split('.')[-1]
    if extention == 'stl':
        stl_mesh = mesh.Mesh.from_file(file_adress)
        return stl2ply(stl_mesh)
    if extention == 'ply':
        return 
    else:
        print 'file not supported'
    
    

