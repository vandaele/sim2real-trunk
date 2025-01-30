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


PORT = '/dev/tty.usbmodem401101'
DELAY = 100*0.001

policy = [5, 5, 5, 5, 5, 12, 12, 12, 7, 7, 7, 3, 3, 3, 3, 13, 13, 13, 5, 5, 5, 5, 4, 4, 4, 0, 0, 0, 2, 13, 13, 13, 13, 6, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13]

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
    num_frames += 1
    last_frame = data_frame

def print_frame(frame):
    for b in frame.rigid_bodies:
        print('\t Id {}: ({: 5.5f}, {: 5.5f}, {: 5.5f})'.format(
                        b.id_num, b.pos[0]*1000, b.pos[1]*1000, b.pos[2]*1000
                    ))

def main():
    global num_frames, policy
    
    streaming_client = NatNetClient(server_ip_address="193.49.212.238", local_ip_address="193.49.212.156", use_multicast=False)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
    
    # ard = Arduino(PORT)

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
                        "\t- 'q' to quit\n"
                        "\t- anything else to start the policy\n")
            if inp == 'i':
                print("Set init position\n")
                # ard.query(inp)
            elif inp == 'l':
                print("Set low position\n")
                # ard.query(inp)
            elif inp == 'q':
                break
            else:
                init_time = time.time()
                step = 0
                last_update = 0
                while step < len(policy):
                    now = time.time();
                    if (now - last_update) >= DELAY:
                        streaming_client.update_sync()
                        print(f"Received {num_frames} frames")
                        print_frame(last_frame)
                        print(step, now - init_time, " action: ", policy[step])
                        # ard.query(str(policy[step]))
                        last_update = now;
                        step+=1;
                time.sleep(DELAY)
                streaming_client.update_sync()
                print(f"Received {num_frames} frames")
                print_frame(last_frame)
                print("Policy executed.\nTotal time: ", time.time() - init_time)

if __name__ == '__main__':
    num_frames = 0
    main()