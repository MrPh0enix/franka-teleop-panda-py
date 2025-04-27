import panda_py
import panda_py.controllers
import time
import json
import keyboard
import csv
import os
import numpy as np


with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

current_demo = len(os.listdir('NEW_DEMOS'))+1

leader_robot = panda_py.Panda(config["leader_robot_ip"])
follower_robot = panda_py.Panda(config['follower_robot_ip'])
follower_state = follower_robot.get_state()
leader_robot.move_to_joint_position(follower_state.q)
leader_robot.teaching_mode(active = True)
folTrqController = panda_py.controllers.AppliedTorque()
follower_robot.start_controller(folTrqController)

start_time = time.time()

frequency = 10 #frequency of recording
time_delay = 1/frequency

recordings = [['Time', 'pos1', 'pos2', 'pos3', 'pos4', 'pos5', 'pos6', 'pos7']]



def calc_torque(leader_state, follower_state, 
                K_p = 0.07 * np.array([600.0, 800.0, 600.0, 650.0, 250.0, 150.0, 50.0], dtype=np.float64), 
                K_d = 0.07 * np.array([50.0, 70.0, 50.0, 70.0, 30.0, 25.0, 15.0], dtype=np.float64)):

    torques = [0, 0, 0, 0, 0, 0, 0]
    for i in range(7):
        torques[i] = K_p[i] * (leader_state.q[i] - follower_state.q[i]) - K_d[i] * (follower_state.dq[i])
    torques = np.array(torques)
    return torques


while True:

    if keyboard.is_pressed('q'):
        break

    leader_state = leader_robot.get_state()
    follower_state = follower_robot.get_state()

    torques = calc_torque(leader_state, follower_state)
    folTrqController.set_control(torques)

    curr_step = [time.time()-start_time, leader_state.q[0], leader_state.q[1], leader_state.q[2],
                leader_state.q[3], leader_state.q[4], leader_state.q[5], leader_state.q[6]]
    recordings.append(curr_step)
    time.sleep(time_delay)


follower_robot.stop_controller()

with open(f'NEW_DEMOS/output{current_demo}.csv', "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerows(recordings)  

os.chmod(f'NEW_DEMOS/output{current_demo}.csv', 0o644)
