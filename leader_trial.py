import panda_py
import json
import numpy as np
import panda_py.controllers
import time


with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)

leader_robot = panda_py.Panda(config["leader_robot_ip"])
leader_robot.move_to_start()

trqController = panda_py.controllers.AppliedTorque()
leader_robot.start_controller(trqController)

zerotau = np.zeros((7,))

frequency = config["message_frequency"] #messages per second

while True:

    # cmd = str(input("Enter Command...")).casefold()
    # if cmd == 'q':
    #     break

    trqController.set_control(zerotau)

    time.sleep(1 / frequency)

    #print(zerotau)