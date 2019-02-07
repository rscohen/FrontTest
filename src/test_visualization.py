from visualization import visualizer
import numpy as np
import time
from fileIO import read_mesh_file


model = read_mesh_file('3d_model/Motor.stl')
model.compute_vertex_normals()

vis = visualizer(model)

for phi in np.linspace(0,np.pi,100):
    time.sleep(.1)
    vis.update_tool_position(100*np.array([np.cos(phi),np.sin(phi),0]),
                             100*np.array([np.cos(phi),np.sin(phi),10]))
    
vis.exit_visualization()