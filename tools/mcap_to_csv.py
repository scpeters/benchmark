import os
import sys
from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader
import csv

DIRECTORY_NAME = sys.argv[1]

STATES_NAMES = [
    "sim_time",
    "wall_time",
    "model_no",
    "linear_velocity_x",
    "linear_velocity_y",
    "linear_velocity_z",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
    "position_x",
    "position_y",
    "position_z",
    "quaternion_w",
    "quaternion_x",
    "quaternion_y",
    "quaternion_z",
]
CONFIGURATION  = ["physics_engine", "time_step", "complex", "collisiion", "model_count"]



def get_file_names(result_folder):

    current_dir = os.getcwd()
    parent_dir = os.path.dirname(current_dir)
    result_dir = os.path.join(parent_dir, "test_results", result_folder)
    mcap_dir = os.path.join(result_dir, "MCAP")
    file_names = os.listdir(mcap_dir)
    csv_dir = os.path.join(result_dir,"CSV")
    if not os.path.isdir(csv_dir):
        os.mkdir(csv_dir)
    return result_dir, file_names


def MCAP_to_CSV(result_dir, file_name):

    csv_filename = file_name.split('.')[0] + '.csv'
    
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
            dt = proto_msg.dt
            complex = proto_msg.complex
            collision = proto_msg.collision
            model_count = proto_msg.model_count
            wall_time = proto_msg.computation_time
            csv_writer.writerow([physics_engine, dt, complex, collision, model_count])

            csv_writer.writerow(STATES_NAMES)
            for data in proto_msg.data:
                for t in range(len(time_steps)):
                    row = [ time_steps[t], wall_time[t], data.model_no, 
                            data.twists[t].linear.x, data.twists[t].linear.y,
                            data.twists[t].linear.z, data.twists[t].angular.x, 
                            data.twists[t].angular.y, data.twists[t].angular.z, 
                            data.poses[t].position.x, data.poses[t].position.y, 
                            data.poses[t].position.z, data.poses[t].orientation.w, 
                            data.poses[t].orientation.x, data.poses[t].orientation.y,
                            data.poses[t].orientation.z]
                    
                    csv_writer.writerow(row)
                
        csv_file.close()
    
            

print("Started converting files from MCAP to CSV")

result_dir,file_names = get_file_names(DIRECTORY_NAME)
for file_name in file_names:
    MCAP_to_CSV(result_dir, file_name)

print("Successfully !! converted all files from MCAP to CSV")

