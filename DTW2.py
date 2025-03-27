import numpy as np
from fastdtw import fastdtw
# from ProMP_AdaptiveGuidance import ProMp
# from ProMP_AdaptiveGuidance import Learner
# import ProMP_AdaptiveGuidance as ProMP_AdaptiveGuidance
from scipy.spatial.distance import euclidean
from Full_ProMP import ProMp
from Full_ProMP import Learner
import Full_ProMP

# Define distance metric for joint trajectories
def euclidean_distance(x, y):
    return np.linalg.norm(x - y)

# Parameters
basis = 8
dof= 7
Nd = 12
trajectoriesList = []
timeList = []

'''     Import Data from demonstrations    '''
for demo in range(1, Nd+1):
            # Load Franka data for the demonstration
            joints_raw, times_raw= Full_ProMP.Franka_data2('NEW_DEMOS/', demo)
            # Reduce data to 100 samples
            indices = np.linspace(0, len(joints_raw)-1, 50, dtype = int)
            joints_raw = np.asarray([joints_raw[i] for i in indices])
            times_raw = np.asarray([times_raw[i] for i in indices])
            # Append data to lists
            trajectoriesList.append(joints_raw)
            timeList.append(times_raw)


# # Convert trajectories list to NumPy array
# joints_raw = np.array(trajectoriesList)

# Get the number of data points
n_data = len(joints_raw)
# Create a time array
Time = np.linspace(0, 1, n_data)

    
'''     ProMP    '''
# Create ProMP object
print(f'{basis}, {dof}, {n_data}')

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
meanTraj, stdTraj = ProMP_trained.trajectory_mean_std(Time)

# promp_trajectory.reshape((100, 7))



def DtW(real_time_joint_angles):
    
    corresponding_iterations = []
    
    for measurement in real_time_joint_angles:
        min_distance = float('inf')  # Initialize the minimum distance to a large value
        corresponding_iteration = None
        
        for i, joint_angles in enumerate(meanTraj):
            distance, path = fastdtw(joint_angles, measurement, dist = euclidean_distance)  # Calculate DTW distance
            
            if distance < min_distance:
                min_distance = distance
                corresponding_iteration = i
        
        corresponding_iterations.append(corresponding_iteration)

    
    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = meanTraj[corresponding_iterations]
    desired_joint_positions = np.squeeze(desired_joint_positions)
    
    return desired_joint_positions


def get_init_pos():
     
    return meanTraj[0]