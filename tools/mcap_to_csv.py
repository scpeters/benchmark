import os
import sys
from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader
import csv

DIRECTORY_NAME = sys.argv[1]

STATES_NAMES = [
    "sim_time",
    "wall_time",
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
CONFIGURATION  = ["physics_engine", "time_step", "complex", "collisiion"]



def get_file_names(result_folder):

    current_dir = os.getcwd()
    parent_dir = os.path.dirname(current_dir)
    result_dir = os.path.join(parent_dir, "test_results", result_folder)
    mcap_dir = os.path.join(result_dir, "MCAP")
    file_names = os.listdir(mcap_dir)
    csv_dir = os.path.join(result_dir,"CSV")
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
            msg = proto_msg.data[0]
            n = len(msg.sim_time)
            csv_writer.writerow([proto_msg.physics_engine, proto_msg.dt, proto_msg.complex, proto_msg.collision])

            csv_writer.writerow(STATES_NAMES)

            for i in range(n):
                data = [msg.sim_time[i],msg.computation_time[i],
                          msg.twists[i].linear.x,msg.twists[i].linear.y,
                          msg.twists[i].linear.z,msg.twists[i].angular.x,
                          msg.twists[i].angular.y,msg.twists[i].angular.z,
                          msg.poses[i].position.x,msg.poses[i].position.y,
                          msg.poses[i].position.z,msg.poses[i].orientation.w,
                          msg.poses[i].orientation.x,msg.poses[i].orientation.y,
                          msg.poses[i].orientation.z]
                
                csv_writer.writerow(data)

        csv_file.close()
    
            

print("Started converting files from MCAP to CSV")

result_dir,file_names = get_file_names(DIRECTORY_NAME)
for file_name in file_names:
    MCAP_to_CSV(result_dir, file_name)

print("Successfully !! converted all files from MCAP to CSV")

