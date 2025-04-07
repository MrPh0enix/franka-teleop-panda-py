import socket
import panda_py
import time


FOLLOWER_IP = '172.22.3.6'
FOLLOWER_PORT = 5050
LEADER_IP = '172.22.3.6'
LEADER_PORT = 5051

# Create a UDP socket server to receive info
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((FOLLOWER_IP, FOLLOWER_PORT))

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

freq = 10
time_sleep = 1/freq


while True:


    data, _ = recv_sock.recvfrom(1024)
    print(data.decode())

    message = 'hello'
    bytes = recv_sock.sendto(message.encode(), (LEADER_IP, LEADER_PORT))


    time.sleep(time_sleep)