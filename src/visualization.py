"""
This script implement visualisation tool to plot the result the localisation algorithm
"""

import open3d as pn
import matplotlib.pyplot as plt

class visualizer_3D(object):
    def __init__(self, pn_model):
        # look to the open3d (pn) library exemple here : http://www.open3d.org/docs/tutorial/Advanced/customized_visualization.html
        #TODO CREATE WINDOWS
        #TODO ADD MODEL TO VISUALIZATION 
        #TODO CREATE TOOL REPRENTATION
        #TODO ADD TOOL TO WINDOWS
        return
    
    def update_tool_position(self, tool_head_position, tool_camera_position):
        """
        uptade the tool reprensentation
        """
        #TODO COMPUTE THE TRANSMORMATION MATRIX
        #TODO TRANSFORM THE TOOL OBJECT
        #UPDATE VIS
        return
     
    def exit_visualization(self):
        """
        Kill the window and visualization
        """
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
        