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
#cartesian reflex error fix.
leader_robot_settings = leader_robot.get_robot()
leader_robot_settings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_torque_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_force_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]], 
                                             upper_force_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             lower_force_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             upper_force_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])
follower_robot_settings = follower_robot.get_robot()
follower_robot_settings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_torque_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             upper_torque_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                             lower_force_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]], 
                                             upper_force_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             lower_force_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                             upper_force_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])


while True:

    if keyboard.is_pressed('q'):
        break

    time.sleep(time_delay)