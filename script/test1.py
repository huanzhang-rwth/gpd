from ultralytics import YOLO
import open3d as o3d
from PIL import Image
import cv2
import pyrealsense2 as rs
import numpy as np
import random
import time
import pdb
import sys,os
import ctypes

import PcdProcess
from gpd import CGPD
from gpd import StructGrasp
import threading
from RealsenseVideo import RealsenseVideo
from UdpCommunication import UdpServer


import socket
import select
  






grasp_msg_list = []
model = YOLO('yolov8n-seg.pt')  # Load a seg model
number = 0

if __name__ == "__main__":
    #server_socket = UdpServer()
    so_path = '../build/'
    sys.path.append(so_path)
    os.chdir(so_path)
    gpd = CGPD()
    config_filename = b'../cfg/eigen_params.cfg'
    normals_filename = b''
    realsense_video = RealsenseVideo(stream)


    #pdb.set_trace()
    while True:
        try:
            depth_image, color_image, intrinsic = realsense_video.get_frame()
            number +=1
            result = model.predict(color_image)
            boxes = result[0].boxes
            if boxes:
                box = boxes[0]  # returns one box
                C_box = box.xyxy.tolist()
                x1, y1, x2, y2 = map(int, C_box[0][:4])
                mask = np.zeros_like(color_image, dtype=bool)  
                mask[y1:y2,x1:x2] = True
                mask2 = np.zeros_like(depth_image, dtype=bool)  
                mask2[y1:y2,x1:x2] = True  
                color_image[~mask] = 0
                depth_image[~mask2] = 0
            else:
                pass
            #yolo
     

            
            if  number%30 == 0 & boxes:# 收到请求后进行pcd处理
                pcd = PcdProcess.generate_pcd(color_image, depth_image, intrinsic)#生成pcd
                #pcd_noplane = PcdProcess.removeplane(pcd)#去除桌面
                pcd_cluster = PcdProcess.dbcluster(pcd)#输出高于阈值的点云
                points = np.asarray(pcd_cluster.points)
                grasp_msg = gpd.detectGraspsInCloud(config_filename, points)
                print(grasp_msg)
                continue
                
            else:
                continue



            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            if key == -1:  #
                cv2.destroyAllWindows()  # 关闭所有打开的窗口
                break


        except Exception as e:
            print("Error occurred: ", e)  
            #报告error
            server_socket.close()
            time.sleep(10)
            break

    realsense_video.close()

    #print(grasp_msg_list)
