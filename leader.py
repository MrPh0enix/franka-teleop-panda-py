import socket
import time
import panda_py
import panda_py.controllers
import pickle
import json
import numpy as np
import adaptive_positioning
import keyboard



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


FOLLOWER_IP = config['follower_computer_ip']
FOLLOWER_PORT = int(config['follower_computer_port'])
LEADER_IP = config['leader_computer_ip']
LEADER_PORT = int(config['leader_computer_port'])
frequency = int(config["message_frequency"])

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Create UDP sockect server for receiving data
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv_sock.bind((LEADER_IP, LEADER_PORT))


#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)


def calc_static_vfx_trq(leader_robot_state, follower_data):
    ''' adaptive guidance: adaptive guidance forces on the leader '''
    # PD gains
    pgain = 0.06 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
    dgain = 0.06 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    current_pose = np.array([leader_robot_state.q]) # could change to follower pose
    desired_pose, _ = adaptive_positioning.euclidean_dist_pos(current_pose) # we only need the desired pose
    desired_pose = desired_pose.reshape(1, 7)
    pose_diff = desired_pose - leader_robot_state.q
    tau_adapt = pgain *  pose_diff - dgain * leader_robot_state.dq

    # Set desired torque
    tau_desired = tau_adapt
    
    tau_desired = tau_desired.reshape(-1)

    # # clippig for a strict force
    # min_values = np.array([-1.7, -1.3, -1.3, -1.7, -1, -1, -0.8])
    # max_values = np.array([1.7, 1.3, 1.3, 1.7, 1, 1, 0.8])
    # tau_desired = np.clip(tau_desired, min_values, max_values)

    return tau_desired


def calc_adaptive_vfx_trq(leader_robot_state, follower_data):
    ''' adaptive guidance: adaptive guidance forces on the leader '''
    # PD gains
    pgain = 0.06 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
    dgain = 0.06 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    current_pose = np.array([leader_robot_state.q]) # could change to follower pose
    desired_pose, stdDev = adaptive_positioning.euclidean_dist_pos(current_pose) # we only need the desired pose
    desired_pose = desired_pose.reshape(1, 7)
    pose_diff = desired_pose - leader_robot_state.q

    tau_adapt = ((pgain *  pose_diff) / (1 + stdDev)) - dgain * leader_robot_state.dq

    # Set desired torque
    tau_desired = tau_adapt
    
    tau_desired = tau_desired.reshape(-1)

    # # clipping for a strict force
    # min_values = np.array([-1.7, -1.3, -1.3, -1.7, -1, -1, -0.8])
    # max_values = np.array([1.7, 1.3, 1.3, 1.7, 1, 1, 0.8])
    # tau_desired = np.clip(tau_desired, min_values, max_values)

    return tau_desired


def no_feedback(leader_robot_state, follower_data):
    ''' unilateral teleop: no force on the leader '''
    tau_desired = np.zeros((7,))
    return tau_desired


def bilateral_teleop(leader_robot_state, follower_data):
    ''' bilateral teleop: bothe the leader and the follower try to achieve equilibrium i.e, reach the same state.
    PD controller forces on the leader. Lower than what is felt on the follower'''
    torques = [0, 0, 0, 0, 0, 0, 0]

    # PD gains
    pgain = 0.04 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
    dgain = 0.04 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    for i in range(7):
        torques[i] = pgain[i] * (follower_data[i] - leader_robot_state.q[i]) - dgain[i] * (leader_robot_state.dq[i])

    # pose_diff = follower_data[:7] - leader_robot_state.q
    # torques = pgain * pose_diff - dgain * leader_robot_state.dq

    torques = np.array(torques)

    return torques


def bilateral_teleop_adaptive_guidance(leader_robot_state, follower_data):
    # need to implement
    pass


modes = {
    'no_feedback' : no_feedback,
    'adaptive_guidance' : calc_adaptive_vfx_trq,
    'static_guidance' : calc_static_vfx_trq,
    'bilateral_teleop' : bilateral_teleop,
    'adaptive + bilateral teleop combined' : bilateral_teleop_adaptive_guidance,
}


def print_instructions():
    print("(0) No force feedback mode")   
    print("(1) Static guidance mode")
    print("(2) Adaptive guidance mode")
    print("(3) Bilateral teleoperation mode")
    print("(4) Bilateral teleop + Adaptive guidance mode")
    print("(q) Exit")


with leader_robot.create_context(frequency=frequency) as ctx1:

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
            print('\nNo force feedback activated')
            while keyboard.is_pressed('0'): # prevent multiple presses
                time.sleep(0.05)
        if keyboard.is_pressed('1'):
            trq_calc = modes['static_guidance']
            print('\nStatic force feedback activated')
            while keyboard.is_pressed('1'): # prevent multiple presses
                time.sleep(0.05)
        if keyboard.is_pressed('2'):
            trq_calc = modes['adaptive_guidance']
            print('\nAdaptive force feedback activated')
            while keyboard.is_pressed('2'): # prevent multiple presses
                time.sleep(0.05)
        if keyboard.is_pressed('3'):
            trq_calc = modes['bilateral_teleop']
            print('\nBilateral teleop activated')
            while keyboard.is_pressed('3'): # prevent multiple presses
                time.sleep(0.05) 
        if keyboard.is_pressed('4'):
            trq_calc = modes['adaptive + bilateral teleop combined']
            print('\nBilateral teleop + Adaptive guidance activated')
            while keyboard.is_pressed('4'): # prevent multiple presses
                time.sleep(0.05) 
    
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


try:
    leader_robot.stop_controller()
    recv_sock.close()
    send_sock.close()
except:
    pass