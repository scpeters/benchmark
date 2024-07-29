
import os
import sys
benchmark_dir = os.path.dirname(os.getcwd())
sys.path.append(os.path.join(benchmark_dir,"mcap/python/mcap"))
sys.path.append(os.path.join(benchmark_dir, "mcap/python/mcap-protobuf-support"))
from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader
import csv

DIRECTORY_NAME = sys.argv[1]

STATES_NAMES = ["sim_time", "model_no", "linear_accel_x", "linear_accel_y",
                "linear_accel_z", "angular_accel_x", "angular_accel_y",
                "angular_accel_z", "linear_vel_x", "linear_vel_y",
                "linear_vel_z", "angular_vel_x", "angular_vel_y",
                "angular_vel_z", "pos_x", "pos_y", "pos_z", "quat_w",
                "quat_x", "quat_y", "quat_z", "contact1_pos_x", "contact1_pos_y",
                "contact1_pos_z", "contact1_normal_x", "contact1_normal_y",
                "contact1_normal_z", "contact1_force_x", "contact1_force_y",
                "contact1_force_z", "contact1_torque_x", "contact1_torque_y",
                "contact1_torque_z", "contact2_pos_x", "contact2_pos_y", "contact2_pos_z",
                "contact2_normal_x", "contact2_normal_y", "contact2_normal_z",
                "contact2_force_x", "contact2_force_y", "contact2_force_z", 
                "contact2_torque_x", "contact2_torque_y", "contact2_torque_z", "contact3_pos_x",
                "contact3_pos_y", "contact3_pos_z", "contact3_normal_x", "contact3_normal_y",
                "contact3_normal_z", "contact3_force_x", "contact3_force_y", "contact3_force_z",
                "contact3_torque_x", "contact3_torque_y", "contact3_torque_z"]

CONFIGURATION  = ["physics_engine", "time_step", "complex", 
                  "slope", "friction_coefficeint","friction_model", 
                  "wall_time"]



def get_file_names(result_folder):
    result_dir = os.path.join(benchmark_dir,"test_results", result_folder)
    result_dir = os.path.expanduser(result_dir)
    mcap_dir = os.path.join(result_dir, "MCAP")
    file_names = os.listdir(mcap_dir)
    csv_dir = os.path.join(result_dir,"CSV")
    if not os.path.isdir(csv_dir):
        os.mkdir(csv_dir)
    return result_dir, file_names

def add_twist(data, t):
    return [data.twists[t].linear.x, data.twists[t].linear.y,
            data.twists[t].linear.z, data.twists[t].angular.x, 
            data.twists[t].angular.y, data.twists[t].angular.z]

def add_pose(data, t):
    return [data.poses[t].position.x, data.poses[t].position.y, 
            data.poses[t].position.z, data.poses[t].orientation.w, 
            data.poses[t].orientation.x, data.poses[t].orientation.y,
            data.poses[t].orientation.z]

def add_acceleration(data, t):
    return [data.acceleration[t].linear.x, data.acceleration[t].linear.y,
            data.acceleration[t].linear.z, data.acceleration[t].angular.x, 
            data.acceleration[t].angular.y, data.acceleration[t].angular.z]

def add_contact_position(data, t, idx):
    return [data.contact_info[idx].contact_position[t].x, 
            data.contact_info[idx].contact_position[t].y,
            data.contact_info[idx].contact_position[t].z]

def add_contact_normal(data, t ,idx):
    return [data.contact_info[idx].contact_normal[t].x, 
            data.contact_info[idx].contact_normal[t].y,
            data.contact_info[idx].contact_normal[t].z]

def add_contact_wrench(data, t, idx):
    return [data.contact_info[idx].contact_wrench[t].forces.x, 
            data.contact_info[idx].contact_wrench[t].forces.y,
            data.contact_info[idx].contact_wrench[t].forces.z,
            data.contact_info[idx].contact_wrench[t].torques.x, 
            data.contact_info[idx].contact_wrench[t].torques.y,
            data.contact_info[idx].contact_wrench[t].torques.z]

def MCAP_to_CSV(result_dir, file_name):

    csv_filename = file_name.split('.mcap')[0] + '.csv'
    
    csv_filepath = os.path.join(result_dir,"CSV",csv_filename)
    mcap_filepath = os.path.join(result_dir,"MCAP",file_name)

    csv_file = open(csv_filepath, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CONFIGURATION)
    

    with open(mcap_filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
            
        for schema, channel, message, proto_msg in reader.iter_decoded_messages():
            time_steps = proto_msg.sim_time
            physics_engine = proto_msg.physics_engine
            slope  = proto_msg.slope
            complex = proto_msg.complex
            friction_model = proto_msg.friction_model
            friction_coefficeint = proto_msg.friction_coefficient
            wall_time = proto_msg.computation_time
            dt = 1e-3


            if "dartsim-plugin" in physics_engine:
               engine = "dart"
            elif "bullet-featherstone-plugin" in physics_engine:
               engine = "bullet-featherstone"
            elif "bullet-plugin" in physics_engine:
               engine = "bullet"
            else:
                engine = physics_engine

            csv_writer.writerow([engine, dt, complex, slope, friction_coefficeint,
                                 friction_model, wall_time])

            csv_writer.writerow(STATES_NAMES)
            for data in proto_msg.data:
                for t in range(len(time_steps)):
                    row = [time_steps[t], data.model_no] + add_acceleration(data, t) +\
                          add_twist(data, t) + add_pose(data, t) 
                    
                    for idx in range(3):
                        row = row + add_contact_position(data, t , idx) +\
                              add_contact_normal(data, t, idx) + add_contact_wrench(data, t, idx)
                    
                    csv_writer.writerow(row)
                
        csv_file.close()
    
            

print("Started converting files from MCAP to CSV")

result_dir,file_names = get_file_names(DIRECTORY_NAME)
for file_name in file_names:
    MCAP_to_CSV(result_dir, file_name)

print("Successfully !! converted all files from MCAP to CSV")
