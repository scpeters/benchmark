import sys
import os 
import pandas as pd
import numpy as np
from gz.math7 import Quaterniond
import matplotlib.pyplot as plt

DIRECTORY_NAMES = []

metrics = ["position","ang_position","velocity","angular_momentum","energy"]
class PostProcessing:
       def __init__(self):
           self.m = 10
           self.g = 9.8
           box_x = 0.1
           box_y = 0.4
           box_z = 0.9
           Ixx = self.m/12.0 * (box_y**2 + box_z**2)
           Iyy = self.m/12.0 * (box_z**2 + box_x**2)
           Izz = self.m/12.0 * (box_x**2 + box_y**2)
           self.I = np.diag([Ixx, Iyy, Izz])
           self.sim_duration = 10
           self.pos0 = np.array([0,0,0])
           self.rot0 = np.array([1,0,0,0])

       def read_file(self,file_path: str):
           benchmark_config = pd.read_csv(file_path, nrows=1).to_numpy()
           states = pd.read_csv(file_path,skiprows=2).to_numpy()
           return benchmark_config, states
    
       def get_file_names(self, result_folder: str):
           '''Method to obtain the file names and file paths of benchmark result'''
           current_dir = os.getcwd()
           root_dir = os.path.dirname(current_dir)
           result_dir = os.path(root_dir, "test_results", result_folder, "CSV")
           file_names = os.listdir(result_dir) 
           return file_names
       
       def get_analytical_sol(self,complex: bool, sim_time: np.ndarray):
           '''method to get the analytical solution for box benchmark'''

           if not complex:
            v0 = np.array([-0.9, 0.4, 0.1])
            w0 = np.array([0.5, 0.0, 0.0])
            gravity = np.array([0, 0, 0])
           else:
            v0 = np.array([-2.0, 2.0, 8.0])
            w0 = np.array([0.1, 5.0, 0.1])
            gravity = np.array([0, 0, -self.g])

            self.N = len(sim_time)
            self.pos_a = np.zeros((self.N,3))
            self.v_a = np.zeros((self.N,3))

            # calculation of initial energy and angular momentum
            self.L0 = self.I.dot(w0)
            self.L0_mag = np.linalg.norm(self.L0)
            T0 = 0.5*self.m*v0.dot(v0) + 0.5*w0.dot(self.I.dot(w0))
            V0 = self.m*gravity.dot(self.pos0)
            self.E0 = T0 + V0

            # calculation of velocity and position profile with time
            for i, t in enumerate(sim_time):
                self.v_a[i] = v0 + gravity*t
                self.pos_a[i] = self.pos0 + self.v_a*t

       def cal_error(self,states: np.ndarray, complex: bool):
            '''Method for calculating the various error/metrics'''
            sim_time = states[:,0]
            wall_time = states[:,1]
            v = states[:,2:5]
            omega = states[:, 5:8]
            pos = states[:, 8:11]
            rot = states[:, 11:]
            
            # calculation of velocity and postion error and their magnitude
            v_error = (v - self.v_a)
            self.v_error_mag = np.array([np.linalg.norm(v) for v in v_error])
            pos_error = (pos - self.pos_a)
            self.pos_error_mag = np.array([np.linalg.norm(p) for p in pos_error])

            # calculation of energy and angular momentum error
            E = np.zeros(self.N)
            L = np.zeros((self.N,3))
            for i in range(self.N):
                tran_E = 0.5*self.m*v[i].dot(v[i])
                rot_E = 0.5*omega[i].dot(self.I.dot(omega[i]))
                E[i] = tran_E + rot_E
                L[i] = self.I.dot(omega[i])
 
            self.energy_error = (E - self.E0)
            angmomentum_error = (L - self.L0)/self.L0_mag
            self.angmomentum_error_mag = np.array([np.linalg.norm(l) for l in angmomentum_error])
            
            # calculating angle error
            angle_error = np.zeros((self.N,3))
            if not complex:
                for i in range(self.N):
                    quat = rot[i].tolist()
                    euler_angle = Quaterniond(quat[0], quat[1], quat[2], quat[3]).euler()
                    true_angle = Quaterniond(self.w0[0]*sim_time[i], self.w0[1]*sim_time[i], self.w0[2]*sim_time[i])
                    angle_error[i] = (euler_angle - true_angle)

            # calculating computional time for simulation
            self.computional_time = np.divide(wall_time/sim_time)

       def get_maxabs_error(self):
           '''Method for calculation of maximum absolute error'''
           v_maxabs_error = np.max(self.v_error_mag)
           p_maxabs_error = np.max(self.pos_error_mag)
           E_maxabs_error = np.max(self.energy_error)
           L_maxabs_error = np.max(self.angmomentum_error_mag)

       def get_avgabs_error(self):
           '''Method for calculation of avg absolute error'''
           v_avgabs_error = np.sum(self.v_error_mag)/self.N
           p_avgabs_error = np.sum(self.pos_error_mag)/self.N
           E_avgabs_error = np.sum(self.energy_error)/self.N
           L_avgabs_error = np.sum(self.angmomentum_error_mag)/self.N

       
if __name__ == "__main__":
    pass




