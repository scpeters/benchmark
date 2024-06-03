import sys
import os 
import pandas as pd
import numpy as np
from gz.math7 import Quaterniond
import matplotlib.pyplot as plt


DIRECTORY_NAMES = ["BENCHMARK_boxes_model_count", "BENCHMARK_boxes_dt"]

class PostProcessing:
       def __init__(self):
           self.m = 10
           self.g = 9.8
           self.box_x = 0.1
           self.box_y = 0.4
           self.box_z = 0.9
           Ixx = self.m/12.0 * (self.box_y**2 + self.box_z**2)
           Iyy = self.m/12.0 * (self.box_z**2 + self.box_x**2)
           Izz = self.m/12.0 * (self.box_x**2 + self.box_y**2)
           self.I = np.diag([Ixx, Iyy, Izz])
           self.sim_duration = 10
           self.rot0 = np.array([1,0,0,0])

       def read_file(self,file_path: str):
           benchmark_config = pd.read_csv(file_path, nrows=1).to_numpy()
           states = pd.read_csv(file_path,skiprows=2).to_numpy()
           return benchmark_config, states
    
       def get_file_names(self, result_folder: str):
           '''Method to obtain the file names and file paths of benchmark result'''
           current_dir = os.getcwd()
           result_dir = os.path.join(current_dir, "test_results", result_folder, "CSV")
           file_names = os.listdir(result_dir) 
           return result_dir, file_names
       
       def get_analytical_sol(self,complex: bool, sim_time: np.ndarray, model_no: int):
           '''method to get the analytical solution for box benchmark'''

           if not complex:
            v0 = np.array([-0.9, 0.4, 0.1])
            w0 = np.array([0.5, 0.0, 0.0])
            self.gravity = np.array([0, 0, 0])
           else:
            v0 = np.array([-2.0, 2.0, 8.0])
            w0 = np.array([0.1, 5.0, 0.1])
            self.gravity = np.array([0, 0, -self.g])

           self.pos0 = np.array([0,2*self.box_z*model_no,0])
           print(f"  Initial position is {self.pos0} \n")

           self.N = len(sim_time)
           self.pos_a = np.zeros((self.N,3))
           self.v_a = np.zeros((self.N,3))   
           # calculation of initial energy and angular momentum
           self.L0 = self.I.dot(w0)
           self.L0_mag = np.linalg.norm(self.L0)
           T0 = 0.5*self.m*v0.dot(v0) + 0.5*w0.dot(self.I.dot(w0))
           V0 = - self.m*self.gravity.dot(self.pos0)
           self.E0 = T0  + V0 
           self.E0_mag = np.linalg.norm(self.E0)
           # calculation of velocity and position profile with time
           for i, t in enumerate(sim_time):
               self.v_a[i] = v0 + self.gravity*t
               self.pos_a[i] = self.pos0 + v0*t + 0.5*self.gravity*t**2

       def cal_error(self,states: np.ndarray, complex: bool):
            '''Method for calculating the various error/metrics'''
            sim_time = states[:,0]
            wall_time = states[:,1]
            v = states[:,3:6]
            omega = states[:, 6:9]
            pos = states[:, 9:12]
            rot = states[:, 12:]
            
            

            # calculation of energy and angular momentum error
            E = np.zeros(self.N)
            L = np.zeros((self.N,3))
            for i in range(self.N):
                tran_E = 0.5*self.m*v[i].dot(v[i])
                rot_E = 0.5*omega[i].dot(self.I.dot(omega[i]))
                V = - self.m*self.gravity.dot(pos[i])
                E[i] = tran_E + rot_E + V
                L[i] = self.I.dot(omega[i])
            
            # calculation of velocity and postion error and their magnitude
            v_error = (v - self.v_a)
            self.v_error_mag = np.array([np.linalg.norm(x) for x in v_error])
            pos_error = (pos - self.pos_a)
            self.pos_error_mag = np.array([np.linalg.norm(p) for p in pos_error])
 
            self.energy_error = (E - self.E0)/self.E0_mag
            self.energy_error_mag = np.array([np.linalg.norm(e) for e in self.energy_error])
            angmomentum_error = (L - self.L0)/self.L0_mag
            self.angmomentum_error_mag = np.array([np.linalg.norm(l) for l in angmomentum_error])
            
            # calculating angle error
            angle_error = np.zeros((self.N,3))
            if not complex:
                w0 = np.array([0.5, 0.0, 0.0])
                for i in range(self.N):
                    quat = rot[i].tolist()
                    euler_angle = Quaterniond(quat[0], quat[1], quat[2], quat[3]).euler()
                    true_angle = Quaterniond(w0[0]*sim_time[i], w0[1]*sim_time[i], w0[2]*sim_time[i]).euler()
                    error = (euler_angle - true_angle)
                    angle_error[i] = np.array([error.x(), error.y(), error.z()])

            # calculating computional time for simulation
            self.time_ratio = np.divide(wall_time,sim_time)
            print(f"  Time ratio: {self.time_ratio[-1]} \n")

       def get_maxabs_error(self):
           '''Method for calculation of maximum absolute error'''

           v_maxabs_error = np.max(self.v_error_mag)
           p_maxabs_error = np.max(self.pos_error_mag)
           E_maxabs_error = np.max(self.energy_error_mag)
           L_maxabs_error = np.max(self.angmomentum_error_mag)

           print("  -> Max absolute error")
           print("  ----------------------------------")
           print(f"  Linear velocity:  {v_maxabs_error} \n  Position:         {p_maxabs_error} \
                 \n  Energy:           {E_maxabs_error} \n  Angular momentum: {L_maxabs_error}")
           print("  ---------------------------------- \n")
           
       def get_avgabs_error(self):
           '''Method for calculation of avg absolute error'''

           v_avgabs_error = np.sum(self.v_error_mag)/self.N
           p_avgabs_error = np.sum(self.pos_error_mag)/self.N
           E_avgabs_error = np.sum(self.energy_error_mag)/self.N
           L_avgabs_error = np.sum(self.angmomentum_error_mag)/self.N

           print("  -> Average absolute error")
           print("  ----------------------------------")
           print(f"  Linear velocity:  {v_avgabs_error} \n  Position:         {p_avgabs_error} \
                 \n  Energy:           {E_avgabs_error} \n  Angular momentum: {L_avgabs_error} ")
           print("  ---------------------------------- \n")

       
if __name__ == "__main__":
    for dir in DIRECTORY_NAMES:
        print(f"BENCHMARK: {dir}")
        post_processing = PostProcessing()
        result_dir , file_names = post_processing.get_file_names(dir)
        for file in file_names:
            print(f"TEST: {file}")
            file_path = os.path.join(result_dir,file)
            config, states = post_processing.read_file(file_path)
            physic_engine = config[0,0]
            dt = config[0,1]
            complex = bool(config[0,2])
            no_of_models = config[0,4]
            states_per_model = int(len(states[:,0])/no_of_models)
            states = states.reshape(no_of_models, states_per_model,-1)
            print(f" Physics engines: {physic_engine} \n Timestep: {dt} \n Complex: {complex} \n Number of models: {no_of_models}")
            for i in range(no_of_models):
                print(f" => Model number: {i+1}")
                model_states = states[i]
                sim_time = model_states[:,0]
                post_processing.get_analytical_sol(complex, sim_time, i)
                post_processing.cal_error(model_states, complex)
                post_processing.get_maxabs_error()
                post_processing.get_avgabs_error()
            





