import panda_py
import panda_py.controllers
import time
import json
import keyboard
import csv

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)


leader_robot = panda_py.Panda(config["leader_robot_ip"])
# follower_robot = panda_py.Panda(config['follower_robot_ip'])

posController = panda_py.controllers.JointPosition()
leader_robot.start_controller(posController)

with open("output.csv", "r") as file:
    reader = csv.reader(file)

first_pos = reader[0][1:]
print(first_pos)

#Send to start pos
# posController.set_control()
