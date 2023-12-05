import pyrealsense2 as rs  
import numpy as np
import open3d as o3d
  
class RealsenseVideo:  
    def __init__(self, mode='stream'):  
        self.pipeline = rs.pipeline()  
        self.config = rs.config()
        if mode == 'stream':
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)        
        elif mode == 'file':
            self.config.enable_device_from_file('../test_example/d435i_walk_around.bag')#load from file
        else:
            raise ValueError("Invalid mode. Please use 'stream' or 'file'.")
        self.pipeline.start(self.config)  
        self.align = rs.align(rs.stream.color)

    def get_frame(self):  
        frames = self.pipeline.wait_for_frames()  
        align_to = rs.stream.color
        align = rs.align(align_to)
        aligned_frames = self.align.process(frames)
        profile = aligned_frames.get_profile()
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        depth_frame = aligned_frames.get_depth_frame()  
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:  
            return None
        self.depth_frame = depth_frame  
        self.color_frame = color_frame  
        depth_image = np.asanyarray(depth_frame.get_data())  
        color_image = np.asanyarray(color_frame.get_data())  

        return depth_image, color_image, pinhole_camera_intrinsic
  
    def close(self):  
        self.pipeline.stop()