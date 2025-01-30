"""Script for open-loop control of trunk (for data collection)

  Examples:
  python close_loop_control.py
"""

# Importing Libraries 
import serial 
import time
import numpy as np
import filtering

PORT = '/dev/ttyACM1'
import argparse

DELAY = 100*0.001
filtering_data = True

def load_actions_from_file(path):
	"""Load offline actions from file path"""
	actions = np.load(path)
	return actions

def apply_ukf_vectorized(ukfs, obs):
    """Apply UKF filtering"""
    for i in range(4): # trunk sections
        ukfs[i].predict()
        ukfs[i].update(obs[0][i*3:i*3+3])
        obs[0][i*3:i*3+3] = ukfs[i].x[:3]

    ukfs[4].predict() # cube
    ukfs[4].update(obs[0][15:21])
    obs[0][15:21] = ukfs[4].x[:7]
    return obs

class Arduino:
    def __init__(self, port):
        self.dev = serial.Serial(port, baudrate=115200, timeout=.1)
        time.sleep(1)

    def query(self, message):
        self.dev.write((message + '\n').encode('ascii'))

ard = Arduino(PORT)

parser = argparse.ArgumentParser()
parser.add_argument('--data_path', default=None, type=str, help='Path to custom offline dataset')
parser.add_argument('--max_steps', default=100, type=int, help='Number of steps in a single policy execution')
args = parser.parse_args()

while True:
    if args.data_path is not None:
         policy = load_actions_from_file(args.data_path)
         print("Loaded policy from path ", args.data_path)
    else:
         print("ERROR: please provide a policy.")
         exit()
    inp = input("Press:\n\t- 'i' to set init position\n\t- 'l' to set low position\n\t- anything else to start the policy\n")
    if inp == 'i':
        print("Set init position\n")
        ard.query(inp)
    elif inp == 'l':
        print("Set low position\n")
        ard.query(inp)
    else:
        print("Start the policy\n")
        times = []
        dataset = {
				'observations': [],
				'actions': [],
				'next_observations': [],
				'terminals': []
			}
        obs = ... # data from optitrack [format numpy array of shape (1,N)]
        goal_pos = obs[0][12:15]
        cube_pos = obs[0][15:18]
        distance = np.linalg.norm(goal_pos - cube_pos)
        print("Initial distance: ", distance)
        dataset['observations'].append(obs[0])
        ukfs = [filtering.init_ukf(obs[0][i*3:i*3+3]) for i in range(4)]
        ukfs.append(filtering.init_ukf(obs[0][15:21]))  # Cube pos & rotation
        for _ in range(args.max_steps):
            start_time = time.time()
            action, _states = policy.predict(obs, deterministic = True) # action from policy
            ard.query(str(action))
            dataset['actions'].append(action[0])
            time.sleep(DELAY)
            obs = ... # data from optitrack
            goal_pos = obs[0][12:15]
            cube_pos = obs[0][15:18]
            distance = np.linalg.norm(goal_pos - cube_pos)
            done = [False]
            if filtering_data:
                obs = apply_ukf_vectorized(ukfs, obs)
            dataset['next_observations'].append(obs[0])
            dataset['terminals'].append(done[0])
            if not done[0]:
                dataset['observations'].append(obs[0])
            print("Step ", id ,"- Took action: ", action[0], " - distance", distance)
            step_time = time.time() - start_time
            times.append(step_time)
        print("Final distance: ", distance)
        print("Policy executed\n")
        print("--- Tot per episode: %s seconds ---" % (sum(times)))
        print("--- Avg per steps: %s seconds ---" % (sum(times)/len(times)))
