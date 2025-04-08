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


# if len(sys.argv) != 5:
#     raise ValueError("Provide python3 leader.py <follower_computer ip> <follower_computer port> <leader ip> <leader computer port>")

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

leader_robot = panda_py.Panda(config["leader_robot_ip"])
init_pos = adaptive_positioning.get_init_pos()
leader_robot.move_to_joint_position(init_pos)
#Fix for cartesian reflex error. only try this on the leader side as it increases the collision threshold on the robot.
leader_robot_settings = leader_robot.get_robot()
leader_robot_settings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_torque_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_force_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]], 
                                             upper_force_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             lower_force_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             upper_force_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])


FOLLOWER_IP = '172.22.3.6'
FOLLOWER_PORT = 5050
LEADER_IP = '172.22.3.6'
LEADER_PORT = 5051

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Create UDP sockect server for receiving data
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv_sock.bind((LEADER_IP, LEADER_PORT))


#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)


def calc_adaptive_trq(leader_robot_state, follower_data):

    # PD gains
    pgain = 0.6 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
    dgain = 0.6 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

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


def no_feedback(leader_robot_state, follower_data):

    tau_desired = np.zeros((7,))
    return tau_desired


def bilateral_teleop(leader_robot_state, follower_data):
    pass



modes = {
    'no_feedback' : no_feedback,
    'adaptive_guidance' : calc_adaptive_trq,
    'bilateral_teleop' : bilateral_teleop,
}

# fb_method = getattr(self, self.fb_methods[mode], self._nofeedback)

def print_instructions():
    print("(0) No force feedback mode")   
    print("(1) Adaptive guidance mode")
    print("(2) Bilateral teleoperation mode")
    print("(q) Exit")


with leader_robot.create_context(frequency=60) as ctx1:

    print('Teleop leader running')
    print_instructions()
    trq_calc = modes['no_feedback']

    while ctx1.ok():

        if keyboard.is_pressed('q'):
            leader_robot.stop_controller()
            recv_sock.close()
            send_sock.close()
            break
        if keyboard.is_pressed('0'):
            trq_calc = modes['no_feedback']
            print('No force feedback activated')
            while keyboard.is_pressed('0'): # prevent multiple presses
                pass
        if keyboard.is_pressed('1'):
            trq_calc = modes['adaptive_guidance']
            print('Adaptive force feedback activated')
        if keyboard.is_pressed('2'):
            trq_calc = modes['bilateral_teleop']
            pass  
    
        leader_state = leader_robot.get_state()
        leader_data = leader_state.q + leader_state.dq
        message = pickle.dumps(leader_data)
        send_sock.sendto(message, (FOLLOWER_IP, FOLLOWER_PORT))

        try:
            follower_data, _ = recv_sock.recvfrom(1024)
            follower_data = pickle.loads(follower_data)
        except:
            follower_data = leader_data

        torques = trq_calc(leader_state, follower_data)

        trqController.set_control(torques)


leader_robot.stop_controller()
recv_sock.close()
send_sock.close()