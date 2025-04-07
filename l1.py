import socket
import panda_py
import time
from pubsub import pub


FOLLOWER_IP = '172.22.3.6'
FOLLOWER_PORT = 5050
LEADER_IP = '172.22.3.6'
LEADER_PORT = 5051

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Create UDP sockect server for receiving data
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((LEADER_IP, LEADER_PORT))

# subscriber to listen to follower data
follower_state = None
leader_state = None
def listener(data):
    global follower_state
    follower_state = pickle.loads(data)
pub.subscribe(listener, 'follower_data')

freq = 10
time_sleep = 1/freq


while True:

    message = 'hello'
    send_sock.sendto(message.encode(), (FOLLOWER_IP, FOLLOWER_PORT))

   
    data, _ = recv_sock.recvfrom(1024)
    print(data.decode())
    


    time.sleep(time_sleep)