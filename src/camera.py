import pyrealsense2 as rs
import numpy as np
import open3d as pn
import json

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6",
                   "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]

def find_device_that_supports_advanced_mode() :
    ctx = rs.context()
    devices = ctx.query_devices();
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and \
        str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            return dev
    raise Exception("No device that supports advanced mode was found")

def filter_point_cloud(point_cloud):
    """
    The function to filter point_cloud based on the closest figure
    
    Parameters: 
        point_cloud (numpy.ndarray) : the point cloud to filter.
        
    Return
        point_cloud (numpy.ndarray) : the filtered point cloud.
    """
    dist_from_origin = np.sqrt(point_cloud[:,0]**2 + point_cloud[:,1]**2 + \
                               point_cloud[:,2]**2)
    min_dist = dist_from_origin[dist_from_origin > 0].min()
    max_dist = min_dist * 1.5
    point_cloud = point_cloud[(dist_from_origin < max_dist) & \
                              (dist_from_origin > min_dist)]
    return point_cloud
    

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
        self.color_image = color_image
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
        config.enable_stream(rs.stream.depth, resolution[0], resolution[1],
                             rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, resolution[0], resolution[1],
                             rs.format.bgr8, frame_rate)
        self.pipeline.start(config)
        dev = find_device_that_supports_advanced_mode()
        self.advnc_mode = rs.rs400_advanced_mode(dev)
        
    def new_frame(self):
        """
        The function to retrieve a new frame from the camera
        
        Return
            frame (frame) : The last available frame
        """
        while True:
            try:
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
#                vtx = filter_point_cloud(vtx)
                point_cloud = pn.PointCloud() 
                point_cloud.points = pn.Vector3dVector(vtx)
                
                return frame(color_image, depth_image, point_cloud)
            except:
                continue
        
    def stop_recording(self):
        """
        The function to stop recording
        """
        try:
            self.pipeline.stop()
            return
        except:
            return
    
    def start_recording(self):
        """
        The function to start recording
        """
        try:
            self.pipeline.start()   
            return
        except:
            return
    
    def settings(self, camera_settings=dict()):
        """
        The function to get or modify the setting
        
        Parameters: 
            camera_settings (dict) : dictionnary of parameters to modify.
            
        Return
            camera_settings (dict) : dictionnary of all parameters.
        """
        if len(camera_settings)>0:
            serialized_string = self.advnc_mode.serialize_json()
            as_json_object = json.loads(serialized_string)
            if type(next(iter(as_json_object))) != str:
                as_json_object = {k.encode('utf-8'): v.encode("utf-8") 
                for k, v in as_json_object.items()}
            json_string = str(as_json_object).replace("'", '\"').replace(".",",")
            self.advnc_mode.load_json(json_string)
        #TODO : CHECK THAT SETTINGS VALUES ARE IN THE GOOD RANGE
        serialized_string = self.advnc_mode.serialize_json()
        as_json_object = json.loads(serialized_string)
        return as_json_object
        