"""Script for open-loop control of trunk (for data collection)

  Examples:
  python close_loop_control.py
"""

# Importing Libraries 
import serial 
import time
import numpy as np
import argparse
import filtering
import os
from datetime import datetime
from policy.policy import Policy

from natnet_client import DataFrame, NatNetClient
import pdb

#PORT = '/dev/ttyACM1'
PORT = '/dev/tty.usbmodem401101'
DELAY = 100*0.001

INIT_POS = [[0.1045, -4.1515,37.52175], [2.06775,-19.974,73.605], [5.67975, -40.5375,120.3565], [10.593, -62.528,166.6675], [0.0,-82.819,99.998]]
THRESHOLD = 5 # Threshold for difference between curent position and INIT_POS
# Fixed goal position
goal_position = [100.0, -100.0, 50.0]

filtering_data = True
# Define the desired frame rate and number of actions
optitrack_fps = 120  # Frames per second for Optitrack
control_policy_hz = 10  # Frequency of actions in Hz
frames_per_action = 1  # Since we now have data directly at control frequency
DT = 1.0 / control_policy_hz  # Time step matching control policy

class Arduino:
    def __init__(self, port):
        self.dev = serial.Serial(port, baudrate=115200, timeout=.1)
        time.sleep(1)

    def query(self, message):
        self.dev.write((message + '\n').encode('ascii'))


def apply_ukf_vectorized(ukfs, obs):
    """Apply UKF filtering"""
    for i in range(4): # trunk sections
        ukfs[i].predict()
        ukfs[i].update(obs[i*3:i*3+3])
        obs[i*3:i*3+3] = ukfs[i].x[:3]

    ukfs[4].predict()
    ukfs[4].update(obs[15:18])  # Cube position
    obs[15:18] = ukfs[4].x[:3]

    ukfs[5].predict()
    ukfs[5].update(obs[18:22])  # Cube rotation
    obs[18:22] = ukfs[5].x[:4] # Extract and normalize the quaternion
    obs[18:22] /= np.linalg.norm(obs[18:22])  # Normalize to maintain unit norm
    return obs

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

def print_initial_position_diff():
    max_diff = 0
    for b in last_frame.rigid_bodies:
        diff = [abs(pos_i-init_i) for pos_i, init_i in zip([i * 1000 for i in b.pos], INIT_POS[b.id_num -1])]
        flags = ["-" if d<THRESHOLD else "X" for d in diff]
        print(f"Id {b.id_num}: {flags} \t", [f"{x:.2f}" for x in diff])
        if b.id_num < 5:
            diff.append(max_diff)
            max_diff = max(diff)
    print(f"max diff is {max_diff:.2f}")

def frame_to_obs(natnet_frame):
    if natnet_frame is None:
        print("Error: No data received from NatNetClient yet.")
    obs = np.array([])
    for b in natnet_frame.rigid_bodies:
        if b.id_num < 5: # trunk
            obs = np.append(obs, [b.pos[0] * 1000, b.pos[1] * 1000, b.pos[2] * 1000])
        else:
            obs = np.append(obs, goal_position) # goal
            obs = np.append(obs, [b.pos[0] * 1000, b.pos[1] * 1000, b.pos[2] * 1000]) # cube pos
            obs = np.append(obs, [b.rot[0], b.rot[1], b.rot[2], b.rot[3]]) # cube rot
    return obs

def evaluate(obs):
    goal_pos = obs[12:15]
    cube_pos = obs[15:18]
    distance = np.linalg.norm(goal_pos - cube_pos)
    print("Initial distance: ", distance)

def get_current_date():
	return datetime.today().strftime('%Y_%m_%d_%H_%M_%S')

def create_dirs(path):
	try:
		os.makedirs(os.path.join(path))
	except OSError as error:
		pass
     
def load_policy(load_path):
    if os.path.isdir(load_path):
        run_path = load_path+"/test/"+get_current_date()+"/"
        create_dirs(run_path)
        load_path  = os.path.join(load_path, "best_model.zip")
        assert os.path.exists(load_path), "best_model.zip hasn't been saved because too few evaluations have been performed. Check --eval_freq and -t in train.py"
        #test_env = gym.make("trunkcube-v0")
        size_layer=[]
        for _ in range(2):
            size_layer.append(128)
        policy = Policy(algo="ppo",
                env=None, # check if it's ok
                device="cpu",
                seed=0,
                lr=1e-3, 
                batch_size=64,
                size_layer=size_layer,
                load_from_pathname=load_path)
    else:
        raise ValueError(f"{load_path}: data path is not correct")
    return policy

def main():
    global num_frames, policy, last_frame, data_records
    num_frames = 0
    
    streaming_client = NatNetClient(server_ip_address="193.49.212.238", local_ip_address="193.49.212.156", use_multicast=True)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
    
    ard = Arduino(PORT)

    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', default=None, type=str, help='Path to custom offline dataset')
    parser.add_argument('--max_steps', default=100, type=int, help='Number of steps in a single policy execution')
    args = parser.parse_args()

    if args.data_path is not None:
            policy = load_policy(args.data_path)
            print("Loaded policy from path ", args.data_path)
    else:
        print("ERROR: please provide a policy.")
        exit()

    with streaming_client:
        streaming_client.request_modeldef()

        while True:
            inp = input("Press:\n"
                        "\t- 'i' to set init position\n"
                        "\t- 'l' to set low position\n"
                        "\t- 's' start the policy\n"
                        "\t- 'q' to quit\n")
            if inp == 'i':
                print("Set init position\n")
                ard.query(inp)
                time.sleep(255 * 11.2 / 1000)
                streaming_client.update_sync()
                print_initial_position_diff()
            elif inp == 'l':
                print("Set low position\n")
                ard.query(inp)
            elif inp == 's':
                print("Start the policy\n")
                init_time = time.time()
                times = []
                dataset = {
                        'observations': [],
                        'actions': [],
                        'next_observations': [],
                        'terminals': []
                    }
                step = 0
                last_update = 0
                streaming_client.update_sync()
                obs = frame_to_obs(last_frame)
                evaluate(obs)
                dataset['observations'].append(obs)
                ukfs = [filtering.init_ukf(obs[i*3:i*3+3], dt=DT) for i in range(4)]
                ukfs.append(filtering.init_ukf(obs[15:18], dt=DT))  # UKF for cube position
                ukfs.append(filtering.initialize_quaternion_ukf(dt=DT,  # UKF for cube rotation
                                      initial_quaternion=obs[18:22], 
                                      initial_angular_velocity=[0, 0, 0]))  
                while step < args.max_steps:
                    now = time.time()
                    if (now - last_update) >= DELAY:
                        action, _states = policy.predict(obs, deterministic = True) # action from policy
                        ard.query(str(action))
                        dataset['actions'].append(action)
                        time.sleep(DELAY) # do I need ?
                        streaming_client.update_sync()
                        obs = frame_to_obs(last_frame)
                        done = [False]
                        if filtering_data:
                            obs = apply_ukf_vectorized(ukfs, obs)
                        evaluate(obs)
                        dataset['next_observations'].append(obs)
                        dataset['terminals'].append(done[0])
                        if not done[0]:
                            dataset['observations'].append(obs)
                        step_time = time.time() - last_update
                        print("Step ", step ,"- Took action: ", action, " - ", step_time, "s")
                        times.append(step_time)
                        last_update = now
                        step+=1
                print("Policy executed\n")
                print("--- Tot per episode: %s seconds ---" % (sum(times)))
                print("--- Avg per steps: %s seconds ---" % (sum(times)/len(times)))
            elif inp == 'q':
                break
            

if __name__ == '__main__':
    main()