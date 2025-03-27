import json
import panda_py
import panda_py.controllers
import threading
from collections import deque
import time
import numpy as np
import DTW2


class TeleopControllerScheduler(threading.Thread):
    
    def __init__(self,):
        threading.Thread.__init__(self)
        
        self.controller  = None
        self.controllerLock = threading.Lock()
        
        self.is_leader_control = None
        self.is_follower_control = None
        
        self.log_name = ""
        self.doControl = False
        self.star_time = None
        
    def setController(self, controller):
        
        self.controllerLock.acquire()
        self.controller = controller
        self.controller.initController()
        
        #check instances of controllers
        self.is_leader_control = isinstance(controller, LeaderController)
        self.is_follower_control = isinstance(controller, FollowerController)
        
        self.controllerLock.release()
        
        
    def run(self):
        
        global leader_robot, follower_robot
        global desired_pose, covariance_value
        
        desired_pose = np.zeros((7,))
        covariance_value = np.zeros((7,))
        
        control_frequency = 50 # Control loop frequency (Hz)
        time_step = 1.0 / control_frequency
        
        if self.is_leader_control:
            
            while self.doControl:
                
                self.controllerLock.acquire()
                
                leader_robot_state = leader_robot.get_state()
                follower_robot_state = follower_robot.get_state()
                    
                ctrl_action = self.controller.getControl()
                
                self.controller.trq_leader_controller.set_control(ctrl_action)
                
                self.controllerLock.release()
                
                time.sleep(time_step)
                
                
        elif self.is_follower_control:
            
            while self.doControl:
                self.controllerLock.acquire()
                
                ctrl_action = self.controller.getControl()
                
                self.controller.trq_follower_controller.set_control(ctrl_action)
                
                self.controllerLock.release()
                
                time.sleep(time_step)

    def compute_error(self, actual_positions, desired_positions):
            if len(actual_positions) != len(desired_positions):
                raise ValueError("Input lists must have the same length")
            
            #errors = [actual - desired for actual, desired in zip(actual_positions, desired_positions)]
            errors = np.array(actual_positions) - np.array(desired_positions)
            error_norm = np.linalg.norm(errors)
            return error_norm  
            
    def startControl(self):
        self.doControl = True
        self.star_time = time.time()
        self.start()
    
    def stopControl(self):
        self.doControl = False
        self.join()
        
        


class LeaderController():
    
    def __init__(self):
        
        global leader_robot
        
        self.trq_leader_controller = panda_py.controllers.AppliedTorque()
        leader_robot.start_controller(self.trq_leader_controller)
        
        self.leader_history = None
        self.history_length = 150
        self.leader_init_ts = None
        self.follower_init_ts = None
        
        self.fb_methods = ("_nofeedback", "_adaptivefeedback")
        self.fb_method = self._nofeedback

        self.zerotau = np.zeros((7,))
        # PD gains
        self.pgain = 0.03 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
        self.dgain = 0.03 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

        self.__gainsquish = np.vectorize(self.__gainsquish_scalar)

        self.adaptiveG_init_pos = DTW2.get_init_pos()
        self.init_pos_reached = True

    
    def initController(self):
        
        leader_robot_state = leader_robot.get_state()
        follower_robot_state = follower_robot.get_state()
        
        self.leader_init_ts = leader_robot_state.time
        self.follower_init_ts = follower_robot_state.time

        self.leader_history = deque([leader_robot_state.q], maxlen=self.history_length)
        
    def getControl(self):
        
        leader_robot_state = leader_robot.get_state()
        self.leader_history.append(leader_robot_state.q)
        tau = self.fb_method()
        return tau
    
    def _nofeedback(self):
    
        return self.zerotau
    
    def _adaptivefeedback(self):

        # The adaptive guidance ought to furnish a virtual fixture force (torque) 
        # for every joint of the master robot while generating the trajectory.
        # tau_adapt = self.pgain * (q_desired - master_robot.current_j_pos) + self.dgain * master_robot.current_j_vel

        """
        Calculate adaptive torque based on desired pose and guidance gain.

        Returns:
            numpy.ndarray: Desired adaptive torque.
        """

        leader_robot_state = leader_robot.get_state()

        

        #send it to the initial pos first
        if not self.init_pos_reached:
            leader_robot.stop_controller()
            leader_robot.move_to_joint_position(self.adaptiveG_init_pos)
            leader_robot.set_default_behavior
            if np.all(abs(pose_diff) < 0.05):
                print('reached')
                self.init_pos_reached = True
        else:
            # Calculate desired pose using DTW
            current_pose = np.array([leader_robot_state.q]) # could change to follower pose
            desired_pose = DTW2.DtW(current_pose) # we only need the desired pose
            desired_pose = desired_pose.reshape(1, 7)
            pose_diff = desired_pose - leader_robot_state.q

        tau_adapt = self.pgain *  pose_diff - self.dgain * leader_robot_state.dq

        # Set desired torque
        tau_desired = tau_adapt

        # if np.any(tau_desired > 1):
        #     # tau_desired = np.zeros_like(tau_desired)
        #     tau_desired[np.abs(tau_desired) > 0.1] = tau_desired[np.abs(tau_desired) > 0.1] / 100.0
        
        tau_desired = tau_desired.reshape(-1)

        # clipped to prevent undesirably high torques
        tau_desired = np.clip(tau_desired, -1, 1)

        # print("TAU des: ", tau_desired)

        return tau_desired
    
    # modified sigmoid type function squishes values between -1 and 1 to 0 while maintaining the values
    # above and below. Used to filter out small values of forces/torques
    def __gainsquish_scalar(self, x, a=4, b=3, c=0):
        if abs(x) > 1:
            return x
        return x**3
    
    def set_feedback_mode(self, mode):
        
        self.fb_method = getattr(self, self.fb_methods[mode], self._nofeedback)
        print("Set Control Method to ", self.fb_method)
        


class FollowerController():
    
    def __init__(self):
        
        global follower_robot
        
        # PD gains
        self.pgain = 0.03 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64) #originally 0.0003
        self.dgain = 0.03 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)
        
        self.trq_follower_controller = panda_py.controllers.AppliedTorque()
        follower_robot.start_controller(self.trq_follower_controller)

    def initController(self):
        return
    
    def getControl(self):
        # self.paramsLock.acquire()
        tau = self.__pdcontrol()
        # self.paramsLock.release()
        return tau

    def __pdcontrol(self):
        leader_robot_state = leader_robot.get_state()
        follower_robot_state  = follower_robot.get_state()
        return self.pgain * (np.array(leader_robot_state.q) - np.array(follower_robot_state.q)) - self.dgain * np.array(follower_robot_state.dq)
        


def print_instructions():
    print("(0) No force feedback")   
    print("(1) Use virtual fixture force feedback")
    print("(q) Exit")   



if __name__ == "__main__":
    
    global leader_robot, follower_robot
    
    with open('teleop_params.config', 'r') as teleop_params:
        config = json.load(teleop_params)
        
    follower_robot = panda_py.Panda(config['follower_robot_ip'])
    leader_robot = panda_py.Panda(config["leader_robot_ip"])
    
    #move leader to follower position
    fol_state = follower_robot.get_state()
    leader_robot.move_to_joint_position(fol_state.q)
    
    print("====================\nROBOTS CONNECTED\n====================")
    
    #init controllers for both robots
    tc_follower = TeleopControllerScheduler()
    tc_leader = TeleopControllerScheduler()
    tc_follower.setController(FollowerController())
    tc_leader.setController(LeaderController())
    logname = str(input("Enter Log Name: ")).casefold()
    tc_leader.log_name = logname
    
    tc_follower.startControl()
    tc_leader.startControl()
    print("Teleoperation running")
    print_instructions()
    
    while True:
        cmd = str(input("Enter Command...")).casefold()
        if cmd == 'q':
            break
        
        try:
            cmd = int(cmd)
            if cmd in range(2):
                tc_leader.controller.set_feedback_mode(cmd)
            else:
                print("Wrong Command")
                print_instructions()
                
        except ValueError:
            print("Wrong Command")
            print_instructions()
    
    tc_leader.stopControl()
    tc_follower.stopControl()