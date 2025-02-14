import socket
import sys
import time
import panda_py
import pickle
import json

if len(sys.argv) != 4:
    raise ValueError("Provide robot ip, follower ip and follower port")

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

leader_robot = panda_py.Panda(config["leader_robot_ip"])
leader_robot.move_to_start()
leader_robot.teaching_mode(active = True)

ROBOT_IP = sys.argv[1]
FOLLOWER_IP = sys.argv[2]
FOLLOWER_PORT = int(sys.argv[3])

frequency = config["message_frequency"] #messages per second

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print('socket running')

while True:

    leader_state = leader_robot.get_state()
    state_data = leader_state.q + leader_state.dq
    message = pickle.dumps(state_data)
    sock.sendto(message, (FOLLOWER_IP, FOLLOWER_PORT))

    time.sleep(1/frequency)

# sock.close()