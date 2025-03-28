import socket
import sys
import panda_py
import pickle
import numpy as np
import json
import adaptive_positioning
import panda_py.controllers
import keyboard
import time

if len(sys.argv) != 3:
    raise ValueError("Provide python3 follower.py <computer ip> <computer port>")

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

follower_robot = panda_py.Panda(config['follower_robot_ip'])
init_pos = adaptive_positioning.get_init_pos()
follower_robot.move_to_joint_position(init_pos)

ROBOT_IP = sys.argv[1]
ROBOT_PORT = int(sys.argv[2])

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ROBOT_IP, ROBOT_PORT))


#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
follower_robot.start_controller(trqController)

def calc_torque(leader_data, follower_data):

    torques = [0, 0, 0, 0, 0, 0, 0]

    # PD gains
    pgain = 0.07 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
    dgain = 0.07 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)


    for i in range(7):

        torques[i] = pgain[i] * (leader_data[i] - follower_data[i]) - dgain[i] * (follower_data[i+7]) # T = Kp * (leader_pos - follower_pos) + Kd * (follower_velocity)

    torques = np.array(torques)

    return torques

def print_instructions():
    print("(0) No force feedback")   
    print("(1) Use virtual fixture force feedback")
    print("(q) Exit")


with follower_robot.create_context(frequency=30) as ctx2:

    print('Teleop follower running')
    print_instructions()
    
    while ctx2.ok():

        if keyboard.is_pressed('q'):
            break

        #get leader data
        data, leader_addr = sock.recvfrom(1024)
        leader_data = pickle.loads(data)
        # print(leader_data)

        #get follower data
        follower_state = follower_robot.get_state()
        follower_data = follower_state.q + follower_state.dq

        torques = calc_torque(leader_data, follower_data)

        trqController.set_control(torques)


sock.close()