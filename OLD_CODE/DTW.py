
#!/usr/bin/env python3

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
            joints_raw = np.asarray(joints_raw[:100])
            times_raw = np.asarray(times_raw[:100])
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

    
# Weightining the Virtual fixture Force by the variance
def calculate_virtual_forces(gain, measured_joint_positions, desired_joint_positions, covariance_matrix_trajectory, iteration):
    """
    Calculate virtual fixture forces based on given parameters.

    Args:
        gain (list): Proportional gains for virtual fixture.
        measured_joint_positions (numpy.ndarray): Measured joint positions.
        desired_joint_positions (numpy.ndarray): Desired joint positions.
        covariance_matrix_trajectory (numpy.ndarray): Covariance matrix for trajectory.
        iteration (int): Current iteration.

    Returns:
        numpy.ndarray: Computed virtual forces.
        numpy.ndarray: Weighted stiffness values.
    """

    # Parameters
    proportional_gains = gain
    desired_positions = desired_joint_positions.squeeze()
    measured_positions = np.squeeze(measured_joint_positions)

    # Initialize arrays
    num_dofs = len(proportional_gains)
    virtual_forces = np.zeros(num_dofs)
    weighted_stiffness = np.zeros(num_dofs)

    # Normalize variance values between 0 and 1
    normalized_variances = (covariance_matrix_trajectory - np.min(covariance_matrix_trajectory)) / (np.max(covariance_matrix_trajectory) - np.min(covariance_matrix_trajectory))


    # Assign weights based on normalized variance (inverse relation)
    weights = 1.0 - normalized_variances
    # Calculate virtual forces and stiffness

    # error_ = desired_positions - measured_positions
    covariance_value_list = []
    weights_value_list = []

    covariance_value_list.clear()
    for i in range(num_dofs):
        error = desired_positions[i] - measured_positions[i]
        covariance_value = covariance_matrix_trajectory[i, i, iteration]
        covariance_value_list.append(covariance_value)

        weights_value = weights[i, i, iteration]
        weights_value_list.append(weights_value)

        covariance_list = [value[0] for value in covariance_value_list]
        weights_list = [value[0] for value in weights_value_list]

        if covariance_value != 0:
            # Calculate virtual force based on proportional gain and covariance
            # virtual_forces[i] = proportional_gains[i] * (1 / covariance_value) * error

            virtual_forces[i] =  (weights[i, i, iteration]) *  error 

            # Calculate weighted stiffness
            weighted_stiffness[i] = proportional_gains[i] / covariance_value
    
    virtual_forces_ = proportional_gains * virtual_forces
    return virtual_forces_, weighted_stiffness, weights_list




def DtW(real_time_joint_angles, teleo_guidance_gain):

    '''     Dynamic Time Wrapper  2  '''
    # real_time_joint_angles = np.random.rand(10,7)  # Example: random joint angle measurements
    # real_time_joint_angles =np.array([[-0.194897060038202,	0.733697550996658	,0.634285769126056	,-1.75209141547144	,-0.499717813567585	,2.36190677530823	,1.39570638260717]]) # Example real-time joint trajectory

    corresponding_iterations = []  # Store the corresponding iterations for each measurement

    for measurement in real_time_joint_angles:
        min_distance = float('inf')  # Initialize the minimum distance to a large value
        corresponding_iteration = None
        
        for i, joint_angles in enumerate(promp_trajectory):
            distance, path = fastdtw(joint_angles, measurement, dist = euclidean_distance)  # Calculate DTW distance
            
            if distance < min_distance:
                min_distance = distance
                corresponding_iteration = i
                corresponding_path = path

        # Print the aligned joint positions for the current iteration
        # print("Iteration", i+1)
        # for i, j in path:
            #print('joint angles',joint_angles[i])
            #print("mesurement ",measurement)

        corresponding_iterations.append(corresponding_iteration)

    # print('corresponding_iterations',corresponding_iterations)

    # Calculate Virtual Fixture Forces

    # Uncomment and define if not using teleo adaptive guidance
    # teleo_guidance_gain = [0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

    # Uncomment and define if needed
    # reference_trajectory = np.array(trajectoriesList)

    # Use real-time joint angles as the measured positions
    measured_joint_angles = real_time_joint_angles

    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = promp_trajectory[corresponding_iterations]
    desired_joint_positions = np.squeeze(desired_joint_positions)

    virtual_forces = np.zeros((7,)) ################## MY ADDITION ####################
    covariance_value = np.zeros((7,))

    # Calculate virtual fixture forces and stiffness
    # virtual_forces, stiffness_values, covariance_value  = calculate_virtual_forces(teleo_guidance_gain, measured_joint_angles, desired_joint_positions, covTraj, corresponding_iterations)

    # Return the calculated forces and the desired joint positions
    return virtual_forces, desired_joint_positions, covariance_value 



    '''     Dynamic Time Wrapper  1  '''

    '''for i in range(len(promp_trajectory)):

        # Perform DTW alignment for each time step
        distance, path = fastdtw(promp_trajectory[:i+1],  [real_time_joint_angles], dist=euclidean_distance)

        # Get the indices of the corresponding joint angles
        promp_index = path[-1][0]
        real_time_index = 0  # Only a single real-time joint angle is used
        
        # Get the corresponding joint angles
        promp_joint_angles = promp_trajectory[promp_index]
        real_time_joint_angles = real_time_joint_angles
        
        print(f"ProMP Joint Angles: {promp_joint_angles}, Real-Time Joint Angles: {real_time_joint_angles},Indice Joint Angles: {promp_index}")'''

    # ----------------------------------------------------------------
    '''while True:
            # Obtain the real-time joint angles in each iteration (example: random joint angles)
            real_time_joint_angles = np.random.rand(3, 4)  # Example: random joint angles

            # Perform DTW alignment
            distance, path = fastdtw(promp_trajectory, real_time_joint_angles, dist=euclidean_distance)

            # Print the aligned corresponding joint angles
            for point in path:
                promp_joint_angles = promp_trajectory[point[0]]
                real_time_joint_angles = real_time_joint_angles[point[1]]
                print(f"ProMP Joint Angles: {promp_joint_angles}, Real-Time Joint Angles: {real_time_joint_angles}")'''
    

    '''     Dynamic Time Wrapper 3  '''

    '''# Function to calculate the distance between two joint positions
    def distance(position1, position2):
        return euclidean(position1, position2)

    # Function to align the generated trajectory with real-time measurements
    def align_trajectory(generated_trajectory, real_time_measurements):
        # Resample the generated trajectory and real-time measurements to have the same length
        resampled_generated_trajectory = np.linspace(0, 1, len(generated_trajectory))
        resampled_real_time_measurements = np.linspace(0, 1, len(real_time_measurements))

        # Calculate the distance matrix
        distance_matrix = np.zeros((len(resampled_generated_trajectory), len(resampled_real_time_measurements)))
        for i in range(len(resampled_generated_trajectory)):
            for j in range(len(resampled_real_time_measurements)):
                distance_matrix[i, j] = distance(generated_trajectory[i], real_time_measurements[j])

        # Apply dynamic time warping
        alignment_path, _ = fastdtw(distance_matrix)

        # Get the aligned joint positions and corresponding force values
        aligned_positions = [generated_trajectory[i] for (i, _) in alignment_path]

        return aligned_positions

    # Example usage
    generated_trajectory = np.array([[1, 2, 3], [2, 3, 4], [3, 4, 5], [4, 5, 6], [5, 6, 7], [6, 7, 8], [7, 8, 9]])
    generated_forces = [10, 20, 30, 40]

    real_time_measurements = np.array([[0.5, 1, 2], [1.5, 2.5, 3.5], [3, 4, 5]])

    aligned_positions = align_trajectory(generated_trajectory, real_time_measurements)

    print("Aligned Positions:", aligned_positions)'''

