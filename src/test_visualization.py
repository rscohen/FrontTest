from src.visualization import visualizer_3D
import open3d as pn
import numpy as np
import time
from src.fileIO import read_mesh_file


model = read_mesh_file('3d_model/Motor.stl')
model.compute_vertex_normals()
model = pn.PointCloud()
vis = visualizer_3D(model)

for phi in np.linspace(0,np.pi,1000):
    time.sleep(.01)
    vis.update_camera_coordinates(np.array([[800*np.cos(phi), 800*np.sin(phi), 0],
                                       [1, 0, 0],
                                       [0, 1, 0],
                                       [0, 0, 1]]))
    
vis.exit_visualization()