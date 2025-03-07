
import panda_py
import json
import threading
import time
import numpy as np
from scipy.linalg import inv
from collections import deque
from collections import defaultdict

import ProMP_AdaptiveGuidance as ProMP_AdaptiveGuidance
from ProMP_AdaptiveGuidance import ProMp
from ProMP_AdaptiveGuidance import Learner
import DTW


class KalmanFilter:
    def __init__(self, X, dt=0.001, x_cov=1.0e-7, dx_cov=0, obs_noise_cov=1.2e-03):
        # Time interval
        self.sz = X.shape[0]

        # State vector
        self.X = np.append(X, np.zeros((self.sz,)))

        # Motion Model
        self.F = np.diag(np.ones(2 * self.sz, ))
        self.F[:self.sz, self.sz:] = np.diag(np.full((self.sz,), dt))

        # Motion Noise Covariance
        self.Q = np.diag(np.concatenate([np.full((self.sz,), x_cov), np.full((self.sz,), dx_cov)]))

        # Correlation Matrix
        self.P = self.Q

        # Observation Model
        self.H = np.zeros((7, 14))
        np.fill_diagonal(self.H, 1)

        # Observation Noise Covariance (load - grav)
        self.R = np.diag(np.full((self.sz,), obs_noise_cov))

        self.S = np.zeros((self.sz, self.sz))
        self.K = self.X
        self.prev_t = time.time()

    def get_filtered(self, Z):

        self.X = self.F.dot(self.X)
        self.P = self.F.dot(self.P).dot(self.F.transpose()) + self.Q

        self.S = self.H.dot(self.P).dot(self.H.transpose()) + self.R
        self.K = self.P.dot(self.H.transpose()).dot(inv(self.S))
        self.X = self.X + self.K.dot(Z - self.H.dot(self.X))
        self.P = self.P - self.K.dot(self.S).dot(self.K.transpose())
        return self.X[:self.sz]



class TeleopControllerScheduler(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

        self.controller = None
        self.controllerLock = threading.Lock()
        self.log_counter = 0
        self.log_name = ""
        self.star_time = time.time()
        self.fb_mode = 0
        self.logging = False

        self.doControl = True

    def setController(self, controller, use_inv_dyn=False):
        self.controllerLock.acquire()
        self.controller = controller
        self.controller.initController(leader_robot, follower_robot)

        #check instances of controllers
        self.is_leader_control = isinstance(controller, LeaderController)
        self.is_follower_control = isinstance(controller, FollowerController)

        if self.is_leader_control:
            leader_robot.use_inv_dyn = use_inv_dyn #####find symilar property from panda_py

        self.controllerLock.release()

    def run(self):
        global leader_robot
        global follower_robot
        global desired_pose, Forces, covariance_value

        desired_pose = np.zeros((7,)) ###HARD CODED 7
        Forces  = np.zeros((7,)) ###HARD CODED 7
        covariance_value = np.zeros((7,)) ###HARD CODED 7
        iteration = 0
        num_steps = 5000

        # Control loop
        total_duration = 15 # Total duration of trajectory execution (in seconds)
        control_frequency = 100 # Control loop frequency (Hz)
        time_step = 1.0 / control_frequency
        error_threshold = 0.08  # Set your desired error threshold here

        self.controllerLock.acquire()
        if self.controller is not None:
            self.controller.initController(leader_robot, follower_robot)
        self.controllerLock.release()
        called_already = False # Initialize called_already here

        if self.is_leader_control:
            while self.doControl:
                self.controllerLock.acquire()

                if self.controller:
                    ctrl_action = self.controller.getControl(leader_robot, follower_robot)
                else:
                    ctrl_action = np.zeros((7,)) ###HARD CODED 7

                
                # Ensure master and slave joint positions are synchronized
                leader_robot_state = leader_robot.get_state()
                abs_joint_pos_diff = np.array(abs(leader_robot_state.q))
                j_pos_thres_deg = 10
                j_pos_thres = np.ones(7,) * (j_pos_thres_deg * np.pi / 180)
                j_pos_check = np.sum(np.multiply(np.greater(abs_joint_pos_diff, j_pos_thres), 1))

                if j_pos_check:
                    ctrl_action = np.zeros((7,)) ###HARD CODED 7
                    print('Master & Slave out of sync - Please slow down')

                # Subtract the leader external torque sensor value for smoother zero torque leader operation
                fb_gain = 0.3 * np.array([600, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
                current_pose = np.array([leader_robot_state.q])
                Forces, desired_pose, covariance_value = DTW.DtW(current_pose, fb_gain)
                desired_pose = desired_pose.reshape(1, 7)
                Error = self.compute_error(current_pose, desired_pose)

                if Error > 0.1:
                    called_already = False # Set called_already to False
                if self.controller.fb_method == self.controller._adaptivefeedback:
                    # Simulate current robot position and error computation
                    error = np.linalg.norm(current_pose - desired_pose)
                    if not called_already:
                        num_steps = 1
                        iteration = 0
                        while iteration < num_steps:
                            # Simulate trajectory generator and control action
                            trajectory_generator = self.controller.getControl(leader_robot, follower_robot) ##Added follower robot which wasnt in original
                            ctrl_action = trajectory_generator
                            error_threshold = 0.2

                            if error > error_threshold:
                                assistance = ctrl_action
                            else:
                                abs_joint_pos_diff = np.array(abs(leader_robot_state.q))
                                j_pos_thres_deg = 10
                                j_pos_thres = np.ones(7,) * (j_pos_thres_deg * np.pi / 180)
                                j_pos_check = np.sum(np.multiply(np.greater(abs_joint_pos_diff, j_pos_thres), 1))

                                if j_pos_check:
                                    ctrl_action = np.zeros((7,))
                                    print('Master & Slave out of sync - Please slow down') ## Added myself
                                assistance = ctrl_action
                            
                            iteration += 1
                        print("Adaptive guidance feedback enabled for controller")
                        ext_tau_fb_gain = 0.9
                        assistance = ctrl_action
                        print('Assistance for controller', assistance)
                        Command = assistance
                        leader_robot.command = Command ### find comand in panda_py
                        leader_robot.nextStep()
                    else:
                        print('No feedback mode available')
                        Command = np.zeros((7,))
                        leader_robot.command = Command ### find comand in panda_py
                        leader_robot.nextStep()
                        self.controller.fb_method == self.controller._nofeedback
                
                elif self.controller.fb_method == self.controller._nofeedback:
                    ext_tau_fb_gain = 0.9
                    ctrl_action = np.zeros((7,))
                    leader_robot.command = ctrl_action ### find comand in panda_py
                    leader_robot.nextStep()
                
                self.controllerLock.release()
        
        elif self.is_follower_control: # might conflict with leader control check setController, but might not since its set to _pd control
            while self.doControl:
                self.controllerLock.acquire()
                # master_robot.nextStep()
                if (self.controller):
                    ctrl_action = self.controller.getControl(leader_robot, follower_robot)
                else:
                    ctrl_action = np.zeros((7,))
                follower_robot.command = ctrl_action ### find comand in panda_py
                if follower_robot.counter % 50 == 0:
                    follower_robot.set_gripper_width = leader_robot.gripper_width ### can probably deleto or find alternate function

                follower_robot.nextStep()
                self.controllerLock.release()


    def startControl(self):
        self.doControl = True
        self.start()
        self.star_time = time.time()



class LeaderController():

    def __init__(self):
        self.zerotau = np.zeros((7,))
        self.vfixts = []

        # The tuple holds the function names for force feedback
        self.fb_methods = ("_nofeedback", "_adaptivefeedback",  "_torquefeedback", "_positionfeedback")
        self.operating_mode = 0
        self.fb_method = self._nofeedback
        self.history_length = 150
        self.time_delay = 0
        self.__gainsquish = np.vectorize(self.__gainsquish_scalar)

        # dictionary to store key-value pairs of variable names and lists
        self.log_data = defaultdict(list)

        # torque feedback gains
        self.guide_gain = -0.4 * np.array([1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0], dtype=np.float64)

        # self.fb_gain = -0.45 * np.array([2.0, 2.0, 1.0, 2.0, 1.0, 2.0, 2.0], dtype=np.float64)
        self.fb_gain = -0.9 * np.array([2.0, 2.0, 1.0, 2.0, 1.0, 2.0, 2.0], dtype=np.float64)

        # PD gains
        self.pgain = 0.2 * np.array([600.0, 600.0, 600.0, 600.0, 100.0, 100.0, 20.0], dtype=np.float64)
        self.dgain = 0.15 * np.array([50.0, 50.0, 50.0, 50.0, 15.0, 15.0, 5.0], dtype=np.float64)


    def initController(self, leader_robot, follower_robot):

        leader_robot_model = leader_robot.get_model()
        follower_robot_model = follower_robot.get_model()
        leader_robot_state = leader_robot.get_state()
        follower_robot_state = follower_robot.get_state()

        # initialise Kalman filter for both robots to get a filtered load value (after removing gravity and coriolis)
        self.leader_load_filter = KalmanFilter(leader_robot_model.mass() - leader_robot_model.gravity() - leader_robot_model.coriolis())
        self.follower_load_filter = KalmanFilter(follower_robot_model.mass() - follower_robot_model.gravity() - follower_robot_model.coriolis())

        self.leader_init_ts = leader_robot_state.time
        self.follower_init_ts = follower_robot_state.time

        self.leader_history = deque([leader_robot_state.q], maxlen=self.history_length)

        return
    

    def getControl(self, leader_robot, follower_robot):
        self.paramsLock.acquire() # where does this come from????

        leader_robot_state = leader_robot.get_state()
        self.leader_history.append(leader_robot_state.q)
        tau = self.fb_method(leader_robot, follower_robot) # implement different fb methods later
        self.paramsLock.release()
        return tau
    
    def _nofeedback(self, leader_robot, follower_robot):
        #self.__log_values(master_robot, slave_robot, self.zerotau)
        return self.zerotau
    
    def _torquefeedback(self, leader_robot, follower_robot):
        pass
    
    def _positionfeedback(self, leader_robot, follower_robot):
        pass

    def __assistance(self, master_robot):
        mcurr_load = self.mload_filter.get_filtered(
            master_robot.current_load - master_robot.gravity - master_robot.coriolis)
        tau = self.guide_gain * mcurr_load
        return tau, mcurr_load
    
    # modified sigmoid type function squishes values between -1 and 1 to 0 while maintaining the values
    # above and below. Used to filter out small values of forces/torques
    def __gainsquish_scalar(self, x, a=4, b=3, c=0):
        if abs(x) > 1:
            return x
        return x**3
    

    def _adaptivefeedback(self, leader_robot): #, slave_robot

        # The adaptive guidance ought to furnish a virtual fixture force (torque) 
        # for every joint of the master robot while generating the trajectory.
        # tau_adapt = self.pgain * (q_desired - master_robot.current_j_pos) + self.dgain * master_robot.current_j_vel

        """
        Calculate adaptive torque based on desired pose and guidance gain.

        Args:
            master_robot (Robot): The master robot.

        Returns:
            numpy.ndarray: Desired adaptive torque.
        """
        global desired_pose, Forces, covariance_value

        # Calculate desired pose using DTW
        guidance_gain = 0.01 * np.array([600, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
        tau_desired = np.zeros((7,))
        adaptive_force = desired_pose - master_robot.current_j_pos

        # Check if any element in covariance_value is greater than 0
        if any(value > 0 for value in covariance_value):
            # Calculate adaptive torque when at least one element in covariance_value is greater than 0
            tau_adapt = (guidance_gain ) * covariance_value * adaptive_force - self.dgain * master_robot.current_j_vel # / covariance_value[0] 
            tau_adapt = self.__gainsquish(tau_adapt)
        else:
            # Handle the case when all elements in covariance_value are not greater than 0
            # You can set tau_adapt to some default value or handle it as needed.
            tau_adapt = 0  # or any other default value

        # Set desired torque
        tau_desired = tau_adapt

        if np.any(tau_desired > 1):
            # tau_desired = np.zeros_like(tau_desired)
            tau_desired[np.abs(tau_desired) > 0.1] = tau_desired[np.abs(tau_desired) > 0.1] / 100.0

        return tau_desired
    

class FollowerController():
    pass


# ProMP 
def ProMP():
        # Parameters
        basis = 10
        dof= 7
        demonstrations = 100
        N_elements_per_demo = 100
        trajectoriesList, timeList = get_trajectory(demonstrations, N_elements_per_demo)
        n_data = len( np.array(trajectoriesList))
        Time = np.linspace(0, 1, n_data)

        # Create a ProMP object for learning
        training_model = ProMp(basis, dof, N_elements_per_demo)
        # Create a learner object and learn from data
        learner = Learner(training_model)
        learner.LearningFromData(trajectoriesList, timeList)

        # Create a ProMP object for smoothing
        ProMP_trained = ProMp(basis, dof, n_data)
        ProMP_trained.mu = training_model.mu
        ProMP_trained.cov = training_model.cov
        # Generate smoothed trajectories
        promp_trajectory = ProMP_trained.trajectory_samples(Time, 1)
        # Compute mean and covariance of the smoothed trajectories
        meanTraj, covTraj = ProMP_trained.trajectory_mean_cov(Time)
        # Get mean and standard deviation of the smoothed trajectory
        meanTraj, stdTraj = ProMP_trained.trajectoryg_mean_std(Time)

        promp_trajectory.reshape((demonstrations, dof))

        return promp_trajectory


def get_trajectory(demonstrations, N_elements_per_demo):
    trajectoriesList = []
    timeList = []
    
    # Import Data from demonstrations
    for demo in range(demonstrations):
        # Load Franka data for the demonstration
        joints_raw, poses_raw, times_raw = ProMP_AdaptiveGuidance.Franka_data('/DEMONSTRATIONS/', demo)
        
        # Ensure that there are at least N_elements_per_demo data points
        if len(joints_raw) < N_elements_per_demo:
            continue  # Skip this demonstration if it doesn't have enough data points

        # Reduce data to N_elements_per_demo samples
        joints_raw = np.asarray(joints_raw[:N_elements_per_demo])
        times_raw = np.asarray(times_raw[:N_elements_per_demo])
        
        # Append data to lists
        trajectoriesList.append(joints_raw)
        timeList.append(times_raw)
        
        # Normalize time to the interval [0, 1]
        T = times_raw
        T = np.atleast_2d(T)
        T /= np.max(T)
        T = T.flatten()
    
    return trajectoriesList, timeList


global promp_
promp_ = ProMP()
prompt_ = promp_[0:len(promp_):1, :, 0]


if  __name__ == "__main__":

    global leader_robot
    global follower_robot

    with open('teleop_params.config', 'r') as teleop_params:
        config = json.load(teleop_params)

    follower_robot = panda_py.Panda(config['follower_robot_ip'])
    leader_robot = panda_py.Panda(config["leader_robot_ip"])

    print("====================\nROBOTS CONNECTED\n====================")

    #move leader to follower position
    fol_state = follower_robot.get_state()
    leader_robot.move_to_joint_position(fol_state.q)

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
    

    
    
