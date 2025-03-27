
import panda_py
import panda_py.controllers
import numpy as np
import json
import time

with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

frequency = config["message_frequency"] #messages per second

leader_robot = panda_py.Panda(config["leader_robot_ip"])
leader_robot.move_to_start()

#Start the torque controller for the robot
trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)

zerotau = np.array([0,0,0,0,0,0,0])
#Max tested values
#1.5
#1.2
#1.2
#1.7
#1
#0.9
#0.7


while True:

    trqController.set_control(zerotau)

    time.sleep(1/frequency)
