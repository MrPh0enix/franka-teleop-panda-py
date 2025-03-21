import panda_py
import panda_py.controllers
import time
import json
import keyboard
import csv


with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)


leader_robot = panda_py.Panda(config["leader_robot_ip"])
leader_robot.teaching_mode(active = True)
# follower_robot = panda_py.Panda(config['follower_robot_ip'])


frequency = 100
time_delay = 1/frequency

recordings = []


while True:

    if keyboard.is_pressed('q'):
        break

    leader_robot_state = leader_robot.get_state()
    # follower_robot_state = follower_robot.get_state()

    curr_step = {'Time': leader_robot_state.time, 'pos1': leader_robot_state.q[0], 'pos2': leader_robot_state.q[1], 'pos3': leader_robot_state.q[2],
                'pos4': leader_robot_state.q[3], 'pos5': leader_robot_state.q[4], 'pos6': leader_robot_state.q[5], 'pos7': leader_robot_state.q[6]}
    recordings.append(curr_step)
    time.sleep(time_delay)


with open("output.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerows(recordings)  
