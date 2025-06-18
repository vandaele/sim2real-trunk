"""Script for open-loop control of trunk (for data collection)

  Examples:
  python open_loop_control.py # for hardcoded set of actions
  python open_loop_control.py --data_path dataset_fixed_actions/49TOJ.npy # for loading a set of actions
"""

# Importing Libraries 
import serial 
import time
import numpy as np
import argparse

from natnet_client import DataFrame, NatNetClient

import pandas as pd
import os
import datetime

PORT = '/dev/tty.usbmodem11301'
DELAY = 100*0.001
INIT_POS = [[0.1045, -4.1515,37.52175], [2.06775,-19.974,73.605], [5.67975, -40.5375,120.3565], [10.593, -62.528,166.6675], [0.0,-82.819,99.998]]

AKLWI_1 = [5, 12, 12, 12, 7, 7, 3, 3, 7, 7, 13, 13, 11, 11, 11, 11, 1, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14]
AKLWI_2 = [5, 12, 12, 12, 7, 7, 3, 3, 7, 7, 13, 11, 11, 1, 5, 5, 15, 15, 15, 15, 4, 4, 4, 4, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14]
# policy = [5, 5, 5, 5, 5, 12, 12, 12, 7, 7, 7, 3, 3, 3, 3, 13, 13, 13, 5, 5, 5, 5, 4, 4, 4, 0, 0, 0, 2, 13, 13, 13, 13, 6, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13]
policy = AKLWI_2

class Arduino:
    def __init__(self, port):
        self.dev = serial.Serial(port, baudrate=115200, timeout=.1)
        time.sleep(1)

    def query(self, message):
        self.dev.write((message + '\n').encode('ascii'))

def load_actions_from_file(path):
	"""Load offline actions from file path"""
	actions = np.load(path)
	return actions

def receive_new_frame(data_frame: DataFrame):
    global num_frames
    global last_frame
    last_frame = data_frame
    num_frames += 1

def record_frame(timestamp, natnet_frame):
    global data_records
    frame_data = {}
    frame_data["timestamp"] = timestamp
    frame_data["frame"] = natnet_frame.prefix.frame_number
    for b in natnet_frame.rigid_bodies:
        frame_data[f"x{ b.id_num }"] = b.pos[0] * 1000
        frame_data[f"y{ b.id_num }"] = b.pos[1] * 1000
        frame_data[f"z{ b.id_num }"] = b.pos[2] * 1000
        if b.id_num == 5: # cube
            frame_data[f"rot_x{ b.id_num }"] = b.rot[0]
            frame_data[f"rot_y{ b.id_num }"] = b.rot[1]
            frame_data[f"rot_z{ b.id_num }"] = b.rot[2]
            frame_data[f"rot_w{ b.id_num }"] = b.rot[3]
    data_records.append(frame_data)

def main():
    global num_frames, policy, last_frame, data_records
    
    streaming_client = NatNetClient(server_ip_address="193.51.236.195", local_ip_address="193.51.236.48")
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
    
    ard = Arduino(PORT)

    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', default=None, type=str, help='Path to custom offline dataset')
    args = parser.parse_args()

    if args.data_path is not None:
        policy = load_actions_from_file(args.data_path)
        print("Loaded actions from path ", args.data_path)
    else:
        print("Loaded default actions")
    
    print(f"Policy: {len(policy)} actions {policy}\n")

    with streaming_client:
        streaming_client.request_modeldef()

        while True:
            inp = input("Press:\n"
                        "\t- 'i' to set init position\n"
                        "\t- 'l' to set low position\n"
                        "\t- 's' to start the policy\n"
                        "\t- 'q' to quit\n")
            if inp == 'i':
                print("Set init position\n")
                ard.query(inp)
                time.sleep(255 * 11.2 / 1000)
                streaming_client.update_sync()
                for b in last_frame.rigid_bodies:
                    diff = [abs(pos_i-init_i) for pos_i, init_i in zip([i * 1000 for i in b.pos], INIT_POS[b.id_num -1])]
                    diff = ["-" if d<5 else "X" for d in diff]
                    print(f"Id {b.id_num}: {diff}")
            elif inp == 'l':
                print("Set low position\n")
                ard.query(inp)
            elif inp == 's':
                init_time = time.time()
                step = 0
                last_update = 0
                data_records = []  # Store received frames
                while step < len(policy):
                    now = time.time();
                    if (now - last_update) >= DELAY:
                        streaming_client.update_sync()
                        record_frame(now, last_frame)
                        print(step, now - init_time, " action: ", policy[step])
                        ard.query(str(policy[step]))
                        last_update = now;
                        step+=1;
                time.sleep(DELAY)
                streaming_client.update_sync()
                record_frame(time.time(), last_frame)
                print(f"Policy executed.\nTotal time: {time.time() - init_time}.\n{len(data_records)} records")

                # Convert collected data into DataFrame
                df = pd.DataFrame.from_dict(data_records)
                actions = policy.copy()
                actions.insert(0, -1) # initial position do not have any action
                df['action'] = actions
                
                # Save data to CSV
                date = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
                os.makedirs('optitrack_data', exist_ok=True)
                csv_filename = f'optitrack_data/collected_data_{date}.csv'
                df.to_csv(csv_filename, index=False)
                print(f"Data saved to {csv_filename}")
            elif inp == 'q':
                break


if __name__ == '__main__':
    num_frames = 0
    main()