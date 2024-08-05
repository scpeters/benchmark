import sys
import os 
import pandas as pd
import time
import numpy as np
from gz.math7 import Quaterniond, Vector3d
import matplotlib.pyplot as plt
import csv
import time

SOURCE_FOLDER = os.path.dirname(os.path.abspath(__file__))

BENCHMARK_NAME = sys.argv[1]

class PostProcessing:
       
       def __init__(self, test_name: str):
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
           
           timestr = time.strftime("%Y%m%d-%H%M%S")
           metrics_filename = test_name + "_" + timestr + ".csv"
           self.metrics_path = os.path.join(SOURCE_FOLDER, "test_results", metrics_filename)
           print(f"metrics path is {self.metrics_path}")

           self.csv_file = open(self.metrics_path, mode='w', newline='')
           self.csv_writer = csv.writer(self.csv_file)

           metrics = ["angMomentum0", "angMomentumErr_maxAbs","angPositionErr_x_maxAbs", 
                      "angPositionErr_y_maxAbs", "angPositionErr_z_maxAbs", "collision", 
                      "dt", "energyError_maxAbs", 	"engine", "isComplex", "linPositionErr_maxAbs",
                      "linVelocityErr_maxAbs", "modelCount", "simTime", "time", "timeRatio", "classname"]
           self.csv_writer.writerow(metrics)



       def read_file(self,file_path: str):
           benchmark_config = pd.read_csv(file_path, nrows=1).to_numpy()
           states = pd.read_csv(file_path,skiprows=2).to_numpy()
           return benchmark_config, states
    
       def get_file_names(self, result_folder: str):
           '''Method to obtain the file names and file paths of benchmark result'''
           result_dir = os.path.join(SOURCE_FOLDER, "test_results", BENCHMARK_NAME, "CSV")
           file_names = os.listdir(result_dir) 
           return result_dir, file_names
       
       def set_test_parameters(self, physic_engine, dt, complex, 
                               collision, no_of_models, computation_time,
                               class_name):
           self.physics_engine = physic_engine
           self.dt = dt
           self.complex = complex
           self.collision = collision
           self.no_of_models = no_of_models
           self.computation_time = computation_time
           self.class_name = class_name
       
       def get_analytical_sol(self, sim_time: np.ndarray, model_no: int):
           '''method to get the analytical solution for box benchmark'''

           if not self.complex:
            v0 = np.array([-0.9, 0.4, 0.1])
            self.w0 = np.array([0.5, 0.0, 0.0])
            self.gravity = np.array([0, 0, 0])
           else:
            v0 = np.array([-2.0, 2.0, 8.0])
            self.w0 = np.array([0.1, 5.0, 0.1])
            self.gravity = np.array([0, 0, -self.g])

           self.pos0 = np.array([0,2*self.box_z*model_no,0])
           print(f"  Initial position is {self.pos0} \n")

           self.N = len(sim_time)
           self.pos_a = np.zeros((self.N,3))
           self.v_a = np.zeros((self.N,3))   

           # calculation of initial energy and angular momentum
           self.L0 = self.I.dot(self.w0)
           self.L0_mag = np.linalg.norm(self.L0)

           T0 = 0.5*self.m*v0.dot(v0) + 0.5*self.w0.dot(self.I.dot(self.w0))
           V0 = - self.m*self.gravity.dot(self.pos0)
           self.E0 = T0  + V0 
           self.E0_mag = np.linalg.norm(self.E0)
           
           # calculation of velocity and position profile with time
           for i, t in enumerate(sim_time):
               self.v_a[i] = v0 + self.gravity*t
               self.pos_a[i] = self.pos0 + v0*t + 0.5*self.gravity*t**2

       def cal_metrics(self,states: np.ndarray):
            '''Method for calculating the various error/metrics'''
            sim_time = states[:,0]
            v = states[:,2:5]
            omega = states[:, 5:8]
            pos = states[:, 8:11]
            rot = states[:, 11:]

            # calculation of energy and angular momentum error
            E = np.zeros(self.N)
            L = np.zeros((self.N,3))
            for i in range(self.N):
                # angular velocity in body frame
                omega_w = omega[i].tolist()
                quat = rot[i].tolist()
                quat = Quaterniond(quat[0], quat[1], quat[2], quat[3])
                omega_b = quat.rotate_vector_reverse(Vector3d(omega_w[0], omega_w[1], omega_w[2]))
                omega_b = np.array([omega_b[0], omega_b[1], omega_b[2]])

                # translation energy + rotational energy + potential energy
                tran_E = 0.5*self.m*v[i].dot(v[i])
                rot_E = 0.5*omega_b.dot(self.I.dot(omega_b))
                V = - self.m*self.gravity.dot(pos[i])
                E[i] = tran_E + rot_E + V

                # angular momentum in body frame 
                l_b = self.I.dot(omega_b).tolist()

                # angular momentum in world frame
                l_vector =  Vector3d(l_b[0], l_b[1],l_b[2])
                l_w = quat.rotate_vector(l_vector)
                L[i] = np.array([l_w[0], l_w[1], l_w[2]])

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
            self.angle_error = np.zeros((self.N,3))
            if not self.complex:
                w0 = np.array([0.5, 0.0, 0.0])
                for i in range(self.N):
                    quat = rot[i].tolist()
                    euler_angle = Quaterniond(quat[0], quat[1], quat[2], quat[3]).euler()
                    true_angle = Quaterniond(w0[0]*sim_time[i], w0[1]*sim_time[i], w0[2]*sim_time[i]).euler()
                    error = (euler_angle - true_angle)
                    self.angle_error[i] = np.array([error.x(), error.y(), error.z()])
            self.angle_error_mag = np.absolute(self.angle_error)

            # calculating computional time for simulation
            self.total_sim_time = sim_time[-1]
            self.time_ratio = self.computation_time/self.total_sim_time
            print(f"  Time ratio: {self.time_ratio} \n")
          
       def get_maxabs_error(self):
           '''Method for calculation of maximum absolute error'''

           self.v_maxabs_error = np.max(self.v_error_mag)
           self.p_maxabs_error = np.max(self.pos_error_mag)
           self.E_maxabs_error = np.max(self.energy_error_mag)
           self.L_maxabs_error = np.max(self.angmomentum_error_mag)
           self.a_maxabs_error_x = np.max(self.angle_error_mag[:, 0])
           self.a_maxabs_error_y = np.max(self.angle_error_mag[:, 1])
           self.a_maxabs_error_z = np.max(self.angle_error_mag[:, 2])

           print("  -> Max absolute error")
           print("  ----------------------------------")
           print(f"  Linear velocity:  {self.v_maxabs_error} \n  Position:         {self.p_maxabs_error} \
                 \n  Energy:           {self.E_maxabs_error} \n  Angular momentum: {self.L_maxabs_error} \
                 \n  Angle_maxabs_x:   {self.a_maxabs_error_x} \n  Angle_maxabs_y:   {self.a_maxabs_error_y} \
                 \n  Angle_maxabs_x    {self.a_maxabs_error_z}")
           print("  ---------------------------------- \n")
           
       def get_avgabs_error(self):
           '''Method for calculation of avg absolute error'''

           v_avgabs_error = np.sum(self.v_error_mag)/self.N
           p_avgabs_error = np.sum(self.pos_error_mag)/self.N
           E_avgabs_error = np.sum(self.energy_error_mag)/self.N
           L_avgabs_error = np.sum(self.angmomentum_error_mag)/self.N
           a_avgabs_error_x = np.sum(self.angle_error_mag[:, 0])/self.N
           a_avgabs_error_y = np.sum(self.angle_error_mag[:, 1])/self.N
           a_avgabs_error_z = np.sum(self.angle_error_mag[:, 2])/self.N


           print("  -> Average absolute error")
           print("  ----------------------------------")
           print(f"  Linear velocity:  {v_avgabs_error} \n  Position:         {p_avgabs_error} \
                 \n  Energy:           {E_avgabs_error} \n  Angular momentum: {L_avgabs_error} ")
           print("  ---------------------------------- \n")

       def save_metrics(self):
           
           '''Save the current test metrics to csv file'''
           self.get_maxabs_error()
               
           self.csv_writer.writerow([self.L0_mag, self.L_maxabs_error, self.a_maxabs_error_x, 
                                     self.a_maxabs_error_y, self.a_maxabs_error_z, self.collision,
                                     self.dt, self.E_maxabs_error, self.physics_engine, self.complex,
                                     self.p_maxabs_error, self.v_maxabs_error, self.no_of_models,
                                     self.total_sim_time, self.computation_time, self.time_ratio,
                                     self.class_name])
    

       
if __name__ == "__main__":
    dir = BENCHMARK_NAME
    print(f"BENCHMARK: {dir}")

    post_processing = PostProcessing(dir)
    result_dir , file_names = post_processing.get_file_names(dir)
    file_names = sorted(file_names, reverse=True)

    for file in file_names:
        print(f"TEST: {file}")
        file_path = os.path.join(result_dir,file)
        config, states = post_processing.read_file(file_path)
        physic_engine = config[0,0]
        dt = config[0,1]
        complex = bool(config[0,2])
        collision = bool(config[0,3])
        modelCount = config[0,4]
        computation_time = config[0,5]
        log_multiple = bool(config[0,6])
        class_name = config[0,7]
            
        print(f" Physics engines: {physic_engine} \n Timestep: {dt} \n Complex: {complex} \n Number of models: {modelCount}")
        post_processing.set_test_parameters(physic_engine, dt, complex, collision, modelCount, computation_time, class_name)

        if log_multiple:
            no_of_models = modelCount
        else:
            no_of_models = 1 
        states_per_model = int(len(states[:,0])/no_of_models)
        states = states.reshape(no_of_models, states_per_model,-1)

        for i in range(no_of_models):
            print(f" => Model number: {i+1}")
            model_states = states[i]
            sim_time = model_states[:,0]

            post_processing.get_analytical_sol(sim_time, i)
            post_processing.cal_metrics(model_states)
            post_processing.save_metrics()
    
    post_processing.csv_file.close()
    print(f"final metrics path {post_processing.metrics_path}")
    data = pd.read_csv(post_processing.metrics_path)
    storted_data = data.sort_values(by='dt')
    storted_data.to_csv(post_processing.metrics_path, index=False)
