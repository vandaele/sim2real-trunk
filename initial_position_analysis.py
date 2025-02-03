"""Script for initial position analysis (helping for calibration)
"""

# Importing Libraries 
import serial 
import time

from natnet_client import DataFrame, NatNetClient

PORT = '/dev/tty.usbmodem401101'
DELAY = 100*0.001
INIT_POS = [[0.1045, -4.1515,37.52175], [2.06775,-19.974,73.605], [5.67975, -40.5375,120.3565], [10.593, -62.528,166.6675], [0.0,-82.819,99.998]]
THRESHOLD = 5

class Arduino:
    def __init__(self, port):
        self.dev = serial.Serial(port, baudrate=115200, timeout=.1)
        time.sleep(1)

    def query(self, message):
        self.dev.write((message + '\n').encode('ascii'))

def receive_new_frame(data_frame: DataFrame):
    global num_frames
    global last_frame
    last_frame = data_frame
    num_frames += 1

def print_initial_position_diff():
    max_diff = 0
    for b in last_frame.rigid_bodies:
        # if b.id_num < 5:    
        diff = [abs(pos_i-init_i) for pos_i, init_i in zip([i * 1000 for i in b.pos], INIT_POS[b.id_num -1])]
        flags = ["-" if d<THRESHOLD else "X" for d in diff]
        print(f"Id {b.id_num}: {flags} \t", [f"{x:.2f}" for x in diff])
        diff.append(max_diff)
        max_diff = max(diff)
    print(f"max diff is {max_diff:.2f}")

def main():
    global num_frames, last_frame
    
    streaming_client = NatNetClient(server_ip_address="193.49.212.238", local_ip_address="193.49.212.156", use_multicast=False)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()
        while True:
            streaming_client.update_sync()
            print_initial_position_diff()
            time.sleep(1)


if __name__ == '__main__':
    num_frames = 0
    main()