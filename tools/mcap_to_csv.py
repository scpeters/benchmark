import os
from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader
import csv

DIRECTORY_NAMES = ["BENCHMARK_boxes_dt", "BENCHMARK_boxes_model_count"]

FEILD_NAMES = [
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
    "roll",
    "pitch",
    "yaw",
]



def get_file_names(result_folder):

    current_dir = os.getcwd()
    result_dir = os.path.join(current_dir, "test_results", result_folder)
    mcap_dir = os.path.join(result_dir, "MCAP")
    file_names = os.listdir(mcap_dir)

    return result_dir, file_names


def MCAP_to_CSV(result_dir, file_name):

    csv_filename = file_name.split('.')[0] + '.csv'
    csv_filepath = os.path.join(result_dir,"CSV",csv_filename)
    mcap_filepath = os.path.join(result_dir,"MCAP",file_name)

    csv_file = open(csv_filepath, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(FEILD_NAMES)

    with open(mcap_filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])

        for schema, channel, message, proto_msg in reader.iter_decoded_messages():
            msg = proto_msg.data[0]
            n = len(msg.sim_time)

            for i in range(n):
                data = [msg.sim_time[i],msg.computation_time[i],
                          msg.twists[i].linear.x,msg.twists[i].linear.y,
                          msg.twists[i].linear.z,msg.twists[i].angular.x,
                          msg.twists[i].angular.y,msg.twists[i].angular.z,
                          msg.poses[i].position.x,msg.poses[i].position.y,
                          msg.poses[i].position.z,msg.poses[i].orientation.x,
                          msg.poses[i].orientation.y,msg.poses[i].orientation.z]
                
                csv_writer.writerow(data)

        csv_file.close()
    
            

print("Stared converting files from MCAP to CSV")

for folder in DIRECTORY_NAMES:
    result_dir,file_names = get_file_names(folder)

    for file_name in file_names:
        MCAP_to_CSV(result_dir, file_name)
        
print("Successfully !! converted all files from MCAP to CSV")

