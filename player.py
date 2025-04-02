import panda_py
import panda_py.controllers
import time
import json
import keyboard
import csv
import numpy as np

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

frequency = 10
time_delay = 1/frequency

leader_robot = panda_py.Panda(config["leader_robot_ip"])
# follower_robot = panda_py.Panda(config['follower_robot_ip'])


with open("output1.csv", "r") as file:
    reader = csv.reader(file)
    rows = [row for row in reader]
    first_pos = rows[1][1:]


leader_robot.move_to_joint_position(first_pos)


trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)


K_p = [20, 15, 30, 20, 10, 4, 4]
K_d = [0.7, 0.02, 0.7, 0.7, 0.3, 0.3, 0.3]


for row in rows[1:]:

    recorded_pos = row[1:]

    leader_robot_state = leader_robot.get_state()
    leader_robot_pos = leader_robot_state.q


    if keyboard.is_pressed('q'):
        break

    torques = [0, 0, 0, 0, 0, 0, 0]
    for i in range(7):

        torques[i] = K_p[i] * (float(recorded_pos[i]) - leader_robot_pos[i]) - K_d[i] * (leader_robot_state.dq[i])
    
    torques = np.array(torques)
    
    trqController.set_control(torques)

    time.sleep(time_delay)

print('Done')
leader_robot.stop_controller()






