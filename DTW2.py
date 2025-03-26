import numpy as np
from fastdtw import fastdtw
from ProMP_AdaptiveGuidance import ProMp
from ProMP_AdaptiveGuidance import Learner
import ProMP_AdaptiveGuidance as ProMP_AdaptiveGuidance
from scipy.spatial.distance import euclidean

# Define distance metric for joint trajectories
def euclidean_distance(x, y):
    return np.linalg.norm(x - y)

# Parameters
basis = 8
dof= 7
Nd = 100
trajectoriesList = []
timeList = []

'''     Import Data from demonstrations    '''
for demo in range(Nd):
            # Load Franka data for the demonstration
            joints_raw, poses_raw, times_raw = ProMP_AdaptiveGuidance.Franka_data('DEMONSTRATIONS/', demo)
            # Reduce data to 100 samples
            joints_raw = np.asarray(joints_raw[:350])
            times_raw = np.asarray(times_raw[:350])
            # Append data to lists
            trajectoriesList.append(joints_raw)
            timeList.append(times_raw)
            # Normalize time to the interval [0, 1]
            T = times_raw
            T = np.atleast_2d(T)
            T /= np.max(T)
            T= T.flatten()
            
# Convert trajectories list to NumPy array
joints_raw = np.array(trajectoriesList)
# Get the number of data points
n_data = len(joints_raw)
# Create a time array
Time = np.linspace(0, 1, n_data)
# Create an array to store the joint trajectory
Trajectory = np.zeros((n_data, dof))
# Extract joint trajectory for each degree of freedom
for i in range(dof):
    Trajectory[:, i] = joints_raw[1, :, i]
    
'''     ProMP    '''
# Create ProMP object

ProMP_ = ProMp(basis, dof, n_data)

# Create a ProMP object for learning
training_model = ProMp(basis, dof, n_data)
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

promp_trajectory.reshape((100, 7))

def DtW(real_time_joint_angles, teleo_guidance_gain):
    
    corresponding_iterations = []
    
    for measurement in real_time_joint_angles:
        min_distance = float('inf')  # Initialize the minimum distance to a large value
        corresponding_iteration = None
        
        for i, joint_angles in enumerate(promp_trajectory):
            distance, path = fastdtw(joint_angles, measurement, dist = euclidean_distance)  # Calculate DTW distance
            
            if distance < min_distance:
                min_distance = distance
                corresponding_iteration = i
        
        corresponding_iterations.append(corresponding_iteration)

    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = promp_trajectory[corresponding_iterations]
    desired_joint_positions = np.squeeze(desired_joint_positions)
    
    return desired_joint_positions