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


import socket  
  






grasp_msg_list = []
model = YOLO('yolov8n-seg.pt')  # Load a seg model
number = 0


if __name__ == "__main__":
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_addr = ('127.0.0.1', 6006)  
    server_socket.bind(local_addr)
    # 绑定本地地址和端口  
    so_path = '../build/'
    sys.path.append(so_path)
    os.chdir(so_path)
    gpd = CGPD()
    config_filename = b'../cfg/eigen_params.cfg'
    normals_filename = b''
    while True:
        try:
            data, addr = server_socket.recvfrom(1024)
            mode = data.decode()
            if mode == 'file':
                realsense_video = RealsenseVideo('file')
                print('file mode')
                response_msg = 'file mode'
                server_socket.sendto(response_msg.encode(), addr)
                break
            elif mode == 'stream':
                realsense_video = RealsenseVideo('stream')
                response_msg = 'stream mode'
                server_socket.sendto(response_msg.encode(), addr)
                print('stream mode')
                break 
            else:
                print("Invalid mode. Please enter 'file' or 'stream'.")
                response_msg = 'retry'
                server_socket.sendto(response_msg.encode(), addr)
                time.sleep(1)
        except Exception as e:
            response_msg = 'Device loaded failed'
            server_socket.sendto(response_msg.encode(), addr)
            continue


    #pdb.set_trace()
    while True:
        try:
            data, addr = server_socket.recvfrom(1024)
              # 接收数据，最大长度为1024字节
            if not data:
                continue
            else:
                message = data.decode()  # 将接收到的数据解码成字符串  
                print('收到来自 {} 的消息: {}'.format(addr, message))

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
            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            #images = np.hstack((color_image, depth_colormap))
            # Show images
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow('RealSense', images)
            # 可视化
            #key = cv2.waitKey(100)
            
            if message is None or not message:
                response_msg = ''
                server_socket.sendto(response_msg.encode(), addr)
                continue
            elif  message == 'visual':# 可视化
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            #Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))
            # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
            # 可视化
                key = cv2.waitKey(10000)
                if key == -1:  # 如果返回值是 -1，说明已经过了设定的时间  
                    cv2.destroyAllWindows()  # 关闭所有打开的窗口
                response_msg = 'showed'
                server_socket.sendto(response_msg.encode(), addr)
                continue
            elif  message == 'request':# 收到请求后进行pcd处理
                pcd = PcdProcess.generate_pcd(color_image, depth_image, intrinsic)#生成pcd
                #pcd_noplane = PcdProcess.removeplane(pcd)#去除桌面
                pcd_cluster = PcdProcess.dbcluster(pcd)#输出高于阈值的点云
                points = np.asarray(pcd_cluster.points)
                grasp_msg = gpd.detectGraspsInCloud(config_filename, points)
                grasp_msg_list.append(grasp_msg)
                #pdb.set_trace()
                response_msg = str(grasp_msg)
                server_socket.sendto(response_msg.encode(), addr)
                # 发送数据
                continue
            elif message == 'stop' or number > 10000:
                cv2.destroyAllWindows()
                response_msg = 'stopped'
                server_socket.sendto(response_msg.encode(), addr)
                break

            else:
                response_msg = 'invalid command'
                server_socket.sendto(response_msg.encode(), addr)
                print('invalid msg')
                continue



        except Exception as e:
            print("Error occurred: ", e)  
            #报告error
            server_socket.close()
            time.sleep(10)
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            server_socket.bind(local_addr)  
            break

    realsense_video.close()
    server_socket.close()

    #print(grasp_msg_list)
