import open3d as o3d
from PIL import Image
import cv2
import numpy as np
import random
import time



def generate_pcd(img_rgb, img_depth, pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)):
    #color_raw = o3d.io.read_image(img_rgb)   #from image
    #depth_raw = o3d.io.read_image(img_depth)
    color_raw = o3d.geometry.Image(img_rgb)  #from np
    depth_raw = o3d.geometry.Image(img_depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    return pcd




def removePlane(pcd):
    '''
    plane1,inliers1 = pcd.segment_plane(distance_threshold=0.6,
                                            ransac_n=3,
                                            num_iterations=100)
    
    pcd_RemovePlane = pcd.select_by_index(inliers1)
    '''

    plane,inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)
    
    pcd_RemovePlane = pcd.select_by_index(inliers, invert=True)
    return pcd_RemovePlane
    

def dbcluster(pcd, threshold = 0.6):
    #pcd = pcd.voxel_down_sample(voxel_size=0.01) #downsampling

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=20, print_progress=True))

    mask = labels < 0 #去噪
    max_label = labels.max()

    
    labels[mask] = max_label+10
    most_label = np.argmax(np.bincount(labels))
    index = np.argwhere(labels == most_label)
    ratio_cluster = index.size/labels.size
    if (most_label < max_label and ratio_cluster > threshold):
        pcd_cluster = pcd.select_by_index(index)
    else:
        pcd_cluster = pcd
    #o3d.visualization.draw_geometries([pcd_cluster])
    return pcd_cluster
            