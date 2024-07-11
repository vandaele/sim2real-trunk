"""Script for open-loop control of trunk (for data collection)

  Examples:
  python close_loop_control.py
"""

# Importing Libraries 
import serial 
import time
import numpy as np

PORT = '/dev/ttyACM1'
import argparse

DELAY = 100*0.001

# T3YW2
policy = [5, 12, 12, 12, 12, 12, 7, 7, 7, 3, 3, 3, 3, 13, 13, 13, 5, 5, 5, 5, 4, 4, 4, 0, 0, 0, 2, 13, 13, 13, 13, 6, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13]

def load_actions_from_file(path):
	"""Load offline actions from file path"""
	actions = np.load(path)
	return actions

class Arduino:
    def __init__(self, port):
        self.dev = serial.Serial(port, baudrate=115200, timeout=.1)
        time.sleep(1)

    def query(self, message):
        self.dev.write((message + '\n').encode('ascii'))
        time.sleep(DELAY)

ard = Arduino(PORT)

parser = argparse.ArgumentParser()
parser.add_argument('--data_path', default=None, type=str, help='Path to custom offline dataset')
args = parser.parse_args()

while True:
    if args.data_path is not None:
         policy = load_actions_from_file(args.data_path)
         print("Loaded policy from path ", args.data_path)
    else:
         print("Loaded default policy")
    inp = input("Press:\n\t- 'i' to set init position\n\t- 'l' to set low position\n\t- anything else to start the policy\n")
    if inp == 'i':
        print("Set init position\n")
        ard.query(inp)
    elif inp == 'l':
        print("Set low position\n")
        ard.query(inp)
    else:
        steps = 100
        for _ in steps:
            obs = ... # data from optitrack
            action = ... # action from policy
            ard.query(str(action))
        print("Policy executed\n")