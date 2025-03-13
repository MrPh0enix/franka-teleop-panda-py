import socket
import sys
import panda_py
import pickle
import numpy as np
import json

import panda_py.controllers

if len(sys.argv) != 3:
    raise ValueError("Provide python3 follower.py <computer ip> <computer port>")

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

follower_robot = panda_py.Panda(config['follower_robot_ip'])
follower_robot.move_to_start()

ROBOT_IP = sys.argv[1]
ROBOT_PORT = int(sys.argv[2])

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ROBOT_IP, ROBOT_PORT))

print('socket running')
# print('press q to close')

#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
follower_robot.start_controller(trqController)

def calc_torque(leader_data, follower_data, K_p = [20, 15, 30, 20, 10, 4, 4], K_d = [0.7, 0.02, 0.7, 0.7, 0.3, 0.3, 0.3]):
    torques = [0, 0, 0, 0, 0, 0, 0]

    for i in range(7):

        torques[i] = K_p[i] * (leader_data[i] - follower_data[i]) + K_d[i] * (follower_data[i+7]) # T = Kp * (leader_pos - follower_pos) + Kd * (follower_velocity)

    torques = np.array(torques)
    return torques

print('Teleop follower running')

while True:

    print('in')

    #get leader data
    data, leader_addr = sock.recvfrom(1024)
    leader_data = pickle.loads(data)
    # print(leader_data)

    #get follower data
    follower_state = follower_robot.get_state()
    follower_data = follower_state.q + follower_state.dq

    torques = calc_torque(leader_data, follower_data)

    trqController.set_control(torques)

    # print(torques)

# sock.close()