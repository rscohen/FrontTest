import pyrealsense2 as rs
import numpy as np
import open3d as pn


class frame(object):
    """ 
    This is a class define a frame. 
      
    Attributes: 
        pipeline (pyrealsense2.pipeline) : 
        pc (int) (pyrealsense2.pointcloud): camera frame rate.
        camera_settings (dict) : dictionnary of custom parameters .
    """
    
    def __init__(self, color_image, depth_image, point_cloud): 
        """ 
        The constructor for ComplexNumber class. 
  
        Parameters: 
            color_image (numpy.ndarray) : .
            depth_image (numpy.ndarray) : .
            point_cloud (opend3d.PointCloud) : .
        """
        self.color_image = depth_image
        self.depth_image = depth_image
        self.point_cloud = point_cloud
        

class camera(object):
    """ 
    This is a class for handling camera IO. 
      
    Attributes: 
        pipeline (pyrealsense2.pipeline) : 
        pc (int) (pyrealsense2.pointcloud): camera frame rate.
        camera_settings (dict) : dictionnary of custom parameters .
    """
    
    def __init__(self, resolution, frame_rate, camera_settings=None): 
        """ 
        The constructor for ComplexNumber class. 
  
        Parameters: 
            resolution ([int, int]) : desired resolution of the camera.
            frame_rate (int) : camera frame rate.
            camera_settings (dict) : dictionnary of custom parameters .
        """
        assert(resolution in [[1280, 720],
                              [848, 480],
                              [640, 480],
                              [640, 360],
                              [480, 270],
                              [424, 240]])
        self.pipeline = rs.pipeline()
        self.pc = rs.pointcloud()
        config = rs.config()
        config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
        self.pipeline.start(config)
        
    def new_frame(self):
        """
        The function to retrieve a new frame from the camera
        
        Return
            frame (frame) : The last available frame
        """
        while True:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            points = self.pc.calculate(depth_frame)
            vtx = np.asarray(points.get_vertices(), dtype=np.ndarray)
            vtx = np.array(list(vtx))
            point_cloud = pn.PointCloud() 
            point_cloud.points = pn.Vector3dVector(vtx)
            
            return frame(color_image, depth_image, point_cloud)