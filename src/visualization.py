"""
This script implement visualisation tool to plot the result the localisation algorithm
"""

import open3d as pn
import matplotlib.pyplot as plt

class visualizer_3D(object):
    def __init__(self, pn_model):
        self.vis = pn.Visualizer()
        self.vis.create_window()
        self.camera_representation = pn.LineSet()
        self.camera_trace = pn.LineSet()
        self.vis.add_geometry(pn_model)
        self.vis.add_geometry(self.camera_representation)
        self.vis.run()
        return
    
    def update_camera_coordinates(self, camera_coordinates):
        self.camera_representation.points = pn.Vector3dVector(camera_coordinates)
        self.camera_representation.lines = pn.Vector2iVector([[0,1],
                                                              [0,2],
                                                              [0,3]])
        self.vis.add_geometry(self.camera_representation)
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()
        return
    
    def update_trace(self, camera_coordinates_history):
        origines = [x[0] for x in camera_coordinates_history]
        self.camera_trace.points = pn.Vector3dVector(origines)
        self.camera_trace.lines = \
            pn.Vector2iVector([[i,i+1] for i in range(len(origines)-1)])
        
        self.vis.add_geometry(self.camera_trace)
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()
        return
     
    def exit_visualization(self):
        self.vis.destroy_window()
        return
        
class visualizer_2D(object):
    def __init__(self, visualizer_id):
        self.visualizer_id = visualizer_id
        plt.figure(visualizer_id); plt.clf()
        return
    
    def update(self,image):
        plt.figure(self.visualizer_id); plt.clf()
        plt.imshow(image)
        plt.pause(.001)
        