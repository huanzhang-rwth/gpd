import socket
import time
import sys
import subprocess  
import os
import threading  
'''
subprocess.Popen(['python', 'demo.py']) 
time.sleep(5)
print('loaded')
'''
# 创建socket对象并连接到服务器端程序  
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
local_ip = socket.gethostbyname(socket.gethostname()) 

while True:
    user_input = input("输入: ")
    client_socket.sendto(user_input.encode(), (local_ip, 8000))
    response = client_socket.recv(1024) 
    print(response)

        
        
client_socket.close()