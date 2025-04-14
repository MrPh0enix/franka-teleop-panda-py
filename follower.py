import socket
import panda_py
import pickle
import numpy as np
import json
import adaptive_positioning
import panda_py.controllers
import keyboard
import time



with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

follower_robot = panda_py.Panda(config['follower_robot_ip'])
init_pos = adaptive_positioning.get_init_pos()
follower_robot.move_to_joint_position(init_pos)

FOLLOWER_IP = config['follower_computer_ip']
FOLLOWER_PORT = int(config['follower_computer_port'])
LEADER_IP = config['leader_computer_ip']
LEADER_PORT = int(config['leader_computer_port'])
frequency = int(config["message_frequency"])

# Create a UDP socket server to receive info
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv_sock.bind((FOLLOWER_IP, FOLLOWER_PORT))

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
follower_robot.start_controller(trqController)


def calc_torque(leader_data, follower_data):

    torques = [0, 0, 0, 0, 0, 0, 0]

    # PD gains
    pgain = 0.07 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
    dgain = 0.07 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    for i in range(7):
        torques[i] = pgain[i] * (leader_data[i] - follower_data[i]) - dgain[i] * (follower_data[i+7])

    torques = np.array(torques)

    return torques


def print_instructions():
    print("(q) Exit")


with follower_robot.create_context(frequency=frequency) as ctx2:

    print('Teleop follower running')
    print_instructions()
    
    while ctx2.ok():

        # if keyboard.is_pressed('q'):
        #     follower_robot.stop_controller()
        #     recv_sock.close()
        #     send_sock.close()
        #     break
        
        #get follower data and send it to leader
        follower_state = follower_robot.get_state()
        follower_data = follower_state.q + follower_state.dq
        message = pickle.dumps(follower_data)
        send_sock.sendto(message, (LEADER_IP, LEADER_PORT))

        try:
            #get leader data
            data, leader_addr = recv_sock.recvfrom(1024)
            leader_data = pickle.loads(data)
            if leader_data == 'STOP':
                time.sleep(0.5)
                break
        except:
            leader_data = follower_data

        torques = calc_torque(leader_data, follower_data)

        trqController.set_control(torques)


try:
    follower_robot.stop_controller()
    recv_sock.close()
    send_sock.close()
except:
    pass