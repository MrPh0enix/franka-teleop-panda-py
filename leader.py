import socket
import sys
import time
import panda_py
import panda_py.controllers
import pickle
import json
import numpy as np
import adaptive_positioning
import keyboard

if len(sys.argv) != 3:
    raise ValueError("Provide python3 leader.py <follower_computer ip> <follower_computer port>")

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

leader_robot = panda_py.Panda(config["leader_robot_ip"])
init_pos = adaptive_positioning.get_init_pos()
leader_robot.move_to_joint_position(init_pos)

FOLLOWER_IP = sys.argv[1]
FOLLOWER_PORT = int(sys.argv[2])

frequency = config["message_frequency"] #messages per second

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)


def calc_adaptive_trq(leader_robot_state):

    # PD gains
    pgain = 0.3 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
    dgain = 0.3 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    current_pose = np.array([leader_robot_state.q]) # could change to follower pose
    desired_pose = adaptive_positioning.euclidean_dist_pos(current_pose) # we only need the desired pose
    desired_pose = desired_pose.reshape(1, 7)
    pose_diff = desired_pose - leader_robot_state.q
    tau_adapt = pgain *  pose_diff - dgain * leader_robot_state.dq

    # Set desired torque
    tau_desired = tau_adapt
    
    tau_desired = tau_desired.reshape(-1)

    # clipped to prevent undesirably high torques
    min_values = np.array([-1.4, -1.1, -1.1, -1.4, -0.9, -0.8, -0.6])
    max_values = np.array([1.4, 1.1, 1.1, 1.4, 0.9, 0.8, 0.6])
    tau_desired = np.clip(tau_desired, min_values, max_values)

    return tau_desired

def print_instructions():
    print("(0) No force feedback")   
    print("(1) Use virtual fixture force feedback")
    print("(q) Exit")


with leader_robot.create_context(frequency=30) as ctx1:

    print('Teleop leader running')
    print_instructions()

    while ctx1.ok():

        if keyboard.is_pressed('q'):
            break
    
        leader_state = leader_robot.get_state()
        state_data = leader_state.q + leader_state.dq
        message = pickle.dumps(state_data)
        sock.sendto(message, (FOLLOWER_IP, FOLLOWER_PORT))

        torques = calc_adaptive_trq(leader_state)

        trqController.set_control(torques)


leader_robot.stop_controller()

sock.close()