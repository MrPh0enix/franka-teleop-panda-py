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

# tau = np.array([-1.05406232e-01,  3.25757529, 1.43338947e-01, -1.49278097e-01, -5.46629683e-01, -3.04951778e-04,  5.05826310e-09])
tau = np.zeros((7,))

frequency = config["message_frequency"] #messages per second

while True:

    # cmd = str(input("Enter Command...")).casefold()
    # if cmd == 'q':
    #     break

    trqController.set_control(tau)

    time.sleep(1 / frequency)

    #print(zerotau)