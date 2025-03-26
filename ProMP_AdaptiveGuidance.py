#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import numpy.matlib as mat
#import tensorflow as tf
import csv
import numpy 
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from std_msgs.msg import Header
from matplotlib.pyplot import cm
from itertools import cycle
import itertools

"""      A revised version now incorporates the adaptive virtual fixture force by the variance component throughout the trajectory    """

class ProMp:

    def __init__(self, num_basis, num_dof, num_steps):

        # Store the number of basis functions, degrees of freedom, and time steps
        self.num_basis = num_basis
        self.num_dof = num_dof
        self.num_steps = num_steps

        # Compute the total number of weights
        self.num_weights = self.num_basis * self.num_dof

        # Define the covariance matrix for each basis function
        self.cov = np.eye(self.num_weights) # * 0.01
        self.mu = np.zeros(self.num_weights)

        # Generate the mean trajectory by setting all weights to zero
        # self.mean_trajectory = np.linspace(0 ,1 , num_basis)
        self.mean_trajectory = np.zeros(self.num_weights)
        
        # Define the time steps for evaluation & the center and width of the radial basis function
        #self.time_steps = np.linspace(0, 1, 1000)
        self.time_steps = np.linspace(0, 1, num_steps)
        self.centers = np.linspace(0, 1, self.num_basis)
        self.h = 0.7     
        self.f = 1
        self.dt = 1 / (num_steps - 1)

        # Compute the basis function sequence for all time steps and degrees of freedom
        self.all_phi = np.kron(np.eye(self.num_dof, dtype=int), self.basis_function_phi_sequence(self.time_steps)) # self.basis_func_gauss_glb())  # (T * n_dof, n * n_dof)

        # Transpose the basis function sequence for easy computation of weights later on
        self.all_phi_t = np.transpose(self.all_phi)

    def WeightsFromTrajecory(self, traj): # mean_trajectory, cov ):

        # Get the number of steps in the trajectory
        n_steps = len(traj)

        # Generate the weight matrix by sampling from a multivariate Gaussian distribution
        # Weights = np.random.multivariate_normal(self.mean_trajectory, self.cov, size=7)

        # Compute the basis function sequence for the given trajectory
        phi = self.basis_function_phi_sequence(traj)

        # Compute the weights by solving a linear system using the Moore-Penrose pseudoinverse of the basis function sequence
        # This gives us the weights for each degree of freedom and each basis function
        weights = np.transpose(np.dot(np.linalg.pinv(phi), traj))  # (n_dof, n_basis)

        # Return the weights as a vector
        weights = weights.reshape(-1, )

        return weights

    # Radial basis function / Gaussian basis function
    def basis_function_phi_(self, Time):
        
        duration = 1 / Time
        phi = np.exp(-(Time- self.centers[:]) ** 2 / (2.0 * self.h))

        # Normalise Phi
        phi /= phi.sum(axis=0)  # normalize activations for each step

        return phi

    # Radial basis function / Gaussian basis function sequence
    def basis_function_phi_sequence(self, traj):#, Time):
    
        n_steps = len(traj)
        dt = 1/(n_steps)  # (n_steps-1)
        # normalize time to interval [0, 1]
        traj = np.atleast_2d(traj)
        traj /= np.max(traj)

        phi_s = np.zeros((n_steps, self.num_basis))
        for z in range(0, n_steps):

            # Replace time with phase representation
            t = z*dt
            rbf = np.zeros((1, self.num_basis))

            for i in range(1, self.num_basis + 1):
                centers = (i - 1) / (self.num_basis - 1)
                rbf[0, i - 1] = np.exp(-(self.f * t - centers) ** 2/(2 * self.h))
            phi_s[z, :self.num_basis] = rbf[0, :self.num_basis]

        # all phi_s
        # Normalize basis functions
        phi_s = phi_s / np.transpose(mat.repmat(np.sum(phi_s, axis=1), self.num_basis, 1))
        return phi_s

    def trajectory_from_weights(self, weights):

        #trajectory = np.matmul(weights, self.all_phi_t)
        trajectory = np.dot(weights, self.all_phi_t)
        n_t= len(trajectory)
        # Reshape into Matrix 
        trajectory = np.transpose(np.reshape(trajectory, (self.num_dof, self.num_steps)))

        return trajectory 

    def evaluate(self):

        # Evaluate the ProMP trajectory for each time step
        Promp_Trajectory = np.zeros((len(self.Time_steps), 7))
        weights = self.WeightsFromTrajecory(self.mean_trajectory, self.cov)
        for i in range(1000):
            for j in range(self.num_basis):
               Promp_Trajectory[i,:] += weights[j,:] * np.exp(-0.5 * (self.Time_steps[i] - self.mean_trajectory[j])**2 / self.cov[0,0])

        return  Promp_Trajectory 
    
    # Sample trajectory      
    def trajectory_samples(self, Time, n_samples = 1):

        weights =  np.random.multivariate_normal(self.mu, self.cov, n_samples)
        weights = weights .transpose()

        # trajecyoty_weights = Basis_MultiDof.dot(weights)  # Prevouis Method
        trajecyoty_weights = self.all_phi.dot(weights)
        trajecyoty_weights = trajecyoty_weights.reshape((self.num_dof,int(trajecyoty_weights.shape[0]/self.num_dof), n_samples))
        trajecyoty_weights = np.transpose(trajecyoty_weights, (1, 0, 2))

        return trajecyoty_weights

    def trajectory_samples_conditioning(self, Time, n_samples = 1):

        weights =  np.random.multivariate_normal(self.mu, self.cov, n_samples)
        weights = weights.transpose()
        basis_Matrix = np.kron(np.eye(self.num_dof, dtype=int), self.basis_function_phi_sequence(Time))  #self.all_phi
        trajecyoty_weights = basis_Matrix.dot(weights)
        trajecyoty_weights = trajecyoty_weights.reshape((self.num_dof,int(trajecyoty_weights.shape[0]/self.num_dof), n_samples))
        trajecyoty_weights = np.transpose(trajecyoty_weights, (1, 0, 2))

        return trajecyoty_weights

    # Trajectories Mean and Covariance 
    def MeanAndCovariance(self):

        # Equation (5)
        weight_mean = (self.weight_mean+ common_term.dot(y_mean - Psi_t.T.dot(self.weight_mean)))
        # Equation (6)
        weight_cov = (self.weight_cov - common_term.dot(Psi_t.T).dot(self.weight_cov))

    def mean_trajectory(self, T):
       
        trajectoryMean = trajectoryFlat.reshape((self.numDoF, trajectoryFlat.shape[0] / self.numDoF))

        return trajectoryMean

    def trajectory_mean_cov(self, Time):

        weights =  np.random.multivariate_normal(self.mu, self.cov)
        weights = weights .transpose()
        a =  self.trajectory_from_weights(weights)
        b = self.all_phi.dot(self.mu.transpose())

        one_dimensional_trajectory = self.all_phi.dot(self.mu.transpose())
        trajectory_mean = one_dimensional_trajectory.reshape(self.num_dof, one_dimensional_trajectory.shape[0]//self.num_dof)
        trajectory_mean = np.transpose(trajectory_mean, (1, 0))

        trajectory_cov  = np.zeros((self.num_dof, self.num_dof, len(Time)))

        for i in range (len(Time)):
            basis =self.all_phi[slice(i, (self.num_dof - 1) * len(Time) + i + 1, len(Time)), :]
            Cov_steps = basis.dot(self.cov).dot(basis.transpose())
            trajectory_cov[:,:,i] = Cov_steps

        return trajectory_mean, trajectory_cov

    def trajectoryg_mean_std(self, time):

        one_dimensional_trajectory = self.all_phi.dot(self.mu.transpose())
        trajectory_mean = one_dimensional_trajectory.reshape(self.num_dof, one_dimensional_trajectory.shape[0]//self.num_dof)
        trajectory_mean = np.transpose(trajectory_mean, (1, 0))
        stdTrajectory  = np.zeros((len(time), self.num_dof))

        for i in range (len(time)):
            basis =self.all_phi[slice(i, (self.num_dof - 1) * len(time) + i + 1, len(time)), :]
            Cov_steps = basis.dot(self.cov).dot(basis.transpose())
            stdTrajectory[i, :] = np.sqrt(np.diag(Cov_steps))

        return trajectory_mean, stdTrajectory

    # Joint Conditioning

    def jointSpaceConditioning(self, time, desired_position, desired_var):

        step = (np.linspace(0, 1, 1)) 
        time = np.array([time])
        newProMP = ProMp(self.num_basis, self.num_dof, self.num_steps)
        basisMatrix = np.kron(np.eye(self.num_dof, dtype=int), self.basis_function_phi_(time))  #self.all_phi
        temp = self.cov.dot(basisMatrix.transpose())

        L = np.linalg.solve(desired_var + basisMatrix.dot(temp), temp.transpose())
        L = L.transpose()
        newProMP.mu = self.mu + L.dot(desired_position - basisMatrix.dot(self.mu))
        newProMP.cov = self.cov - L.dot(basisMatrix).dot(self.cov)

        return newProMP

# For all Trajectories used in demonstration 
class Learner():

    def __init__(self, proMP, regularizationCoeff=10**-9, priorCovariance=10**-4, priorWeight=1):

        self.proMP = proMP
        self.priorCovariance = priorCovariance
        self.priorWeight = priorWeight
        self.regularizationCoeff = regularizationCoeff

    def LearningFromData(self, trajectoryList, timeList):

        num_traj = len(trajectoryList)
        weight_Matrix = np.zeros((num_traj, self.proMP.num_weights))
        for i in range(num_traj):

            trajectory = trajectoryList[i]
            time = timeList[i]
            trajectory = trajectory.transpose().reshape(trajectory.shape[0] * trajectory.shape[1])
            phi_Matrix = self.proMP.all_phi
            temp = phi_Matrix.transpose().dot(phi_Matrix) + np.eye(self.proMP.num_weights) * self.regularizationCoeff
            weight_Vector = np.linalg.solve(temp, phi_Matrix.transpose().dot(trajectory))
            weight_Matrix[i, :] = weight_Vector

        self.proMP.mu = np.mean(weight_Matrix, axis=0)
        Cov = np.cov(weight_Matrix.transpose())
        self.proMP.cov = (num_traj * Cov + self.priorCovariance * np.eye(self.proMP.num_weights)) / (num_traj + self.priorCovariance)
    
    # Weightining the Virtual fixture Force by the variance
    def Virtual_FFV(self, gain, joint_pos_m, joint_pos_d, covTraj, iter):

        # Parameters
        k = gain
        q_d = joint_pos_d[iter,:].reshape(7)
        # q_m = joint_pos_m[iter,:]
        q_m = np.squeeze(joint_pos_m)
 
        # F  = np.empty(7, dtype=object) 
        F =  np.zeros(7)
        weighted_stifness = np.zeros(7)
        
        # Compute the Force to apply for virtual fixture Virtual_FFV
        # F = k*1/cov * (mean - (q_m - q_promp))
        for i in range(self.proMP.num_dof):

            error = q_d[i] - q_m[i]
            mu = covTraj[i, i, iter]
            if mu != 0:
                # Compute the Force to apply for virtual fixture
                # Uncomment this if you want to change the force
                F[i] = k[i] * 1 /mu * error #  F.append()    # ( k[i] * 1 /self.proMP.cov ) * (self.proMP.mu[i] - q_m[i]-q_d[i])

                # Generated weighted_stifness
                weighted_stifness[i] = k[i] / mu

         # torque to apply tau =  ( k[i] * 1 /mu )  * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i]
        # F = F.reshape(1,7)

        return F, weighted_stifness   
    
# Franka data set from demonstrations 
def Franka_data( path, n):
            
    joints = []
    poses = []
    times = []
    joints_path = ('%sJOINTS%s.csv' % (path, n))
    poses_path = ('%sPOSES%s.csv' % (path, n))
    times_path = ('%sTIMES%s.csv' % (path, n))
    with open(joints_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            joints.append(row)
    with open(poses_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            poses.append(np.reshape(row, (4, 4)).transpose())
    with open(times_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            times.append(float(row[0]))
    return joints, poses, times


def Franka_data2( path, n):
            
    joints = []
    times = []
    output_path = ('%soutput%s.csv' % (path, n))
    
    with open(output_path) as csvfile:
        reader = csv.reader(csvfile)
        data = list(reader)[1:]
        for row in data:
            joint_data = [float(item) for item in row[1:]]
            joints.append(joint_data)
            time_data = float(row[0])
            times.append(time_data)
    return joints, times


    
def random_polynomial(t, n_zeros, scale, y_offset):

        zeros = np.random.uniform(np.min(t), np.max(t), n_zeros)
        y = np.ones_like(t)
        for t_0 in zeros:
            y *= t - t_0
        y_min = np.min(y)
        y_max = np.max(y)

        return y_offset + (y - y_min) / (y_max - y_min) * scale





