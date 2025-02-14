import socket
import sys
import keyboard
import panda_py
import pickle
import numpy as np

import panda_py.controllers

if len(sys.argv) != 3:
    raise ValueError("Provide robot ip and robot port")

follower_robot = panda_py.Panda('172.22.2.4')
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

prev_error = [0] * 7
time_step = 1 / 10 #4 is the frequency from the leader code

def calc_torque(leader_data, follower_data, prev_error, time_step, K_p = [20, 20, 20, 20, 3, 3, 3], K_d = [1, 1, 1, 1, 0.5, 0.5, 0.5]):
    torques = [0, 0, 0, 0, 0, 0, 0]

    for i in range(7):

        # error_p = leader_data[i] - follower_data[i]
        # error_p_dot = (error_p - prev_error[i]) / time_step

        # torques[i] = K_p[i] * error_p - K_d[i] * error_p_dot

        # prev_error[i] = error_p

        torques[i] = K_p[i] * (leader_data[i] - follower_data[i]) + K_d[i] * (leader_data[i+7])

    torques = np.array(torques)
    return torques


while True:

    # if keyboard.is_pressed('q'):
    #     print('Exiting...')
    #     break

    #get leader data
    data, leader_addr = sock.recvfrom(1024)
    leader_data = pickle.loads(data)
    # leader_data = leader_state.q + leader_state.dq

    #get follower data
    follower_state = follower_robot.get_state()
    follower_data = follower_state.q + follower_state.dq

    torques = calc_torque(leader_data, follower_data, prev_error, time_step)

    trqController.set_control(torques)

    print(torques)

sock.close()