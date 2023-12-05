import socket
import time
import sys
import select

number = 0
# 创建UDP套接字  
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
local_ip = socket.gethostbyname(socket.gethostname()) 

# 绑定本地地址和端口  
local_addr = (local_ip, 8000)  
sock.bind(local_addr)  

read_list = [sock, sys.stdin]
# 接收数据报文  
while True:  
    ready_streams = select.select(read_list, [], [])  
    if sock in ready_streams:  
        data, addr = sock.recvfrom(1024)  
        if data:  
            # 如果有消息，执行对应操作  
            message = data.decode()
            print('收到来自 {} 的消息: {}'.format(addr, message))  
            # 在这里添加对应于接收到的消息的操作代码  
        else:  
            # 如果没有消息，执行默认操作  
            number += 1
            print(number)  
            # 在这里添加默认操作代码，例如发送默认响应等  
    else:  
        if sys.stdin in ready_streams:  
            input_data = sys.stdin.readline()
            break



sock.close()
