''' To reset the position of the robot by sending zero trq commands.
 To be used with recorder.py'''


import panda_py
import time
import json
import keyboard


with open('teleop_params.config', 'r') as teleop_params:
    config = json.load(teleop_params)


frequency = 10
time_delay = 1/frequency


leader_robot = panda_py.Panda(config["leader_robot_ip"])
follower_robot = panda_py.Panda(config['follower_robot_ip'])
leader_robot.teaching_mode(active = True)
follower_robot.teaching_mode(active = True)


while True:

    if keyboard.is_pressed('q'):
        break

    time.sleep(time_delay)