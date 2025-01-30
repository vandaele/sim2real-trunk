import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import pdb
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
import plotly.graph_objs as go
import plotly.express as px
from matplotlib.animation import FuncAnimation

from filterpy.kalman import KalmanFilter

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

from filterpy.monte_carlo import systematic_resample
from numpy.random import uniform, randn
from scipy.spatial.transform import Rotation as R

# DT = 0.008333
DT = 0.05

# Define the state transition function for UKF
def fx(x, dt):
    """
    State transition function for a constant velocity model.
    """
    # state transition model: x' = x + velocity * dt
    # x = [position_x, velocity_x, position_y, velocity_y, position_z, velocity_z]
    F = np.array([
        [1, 0, 0, dt,  0,  0],
        [0, 1, 0,  0, dt,  0],
        [0, 0, 1,  0,  0, dt],
        [0, 0, 0,  1,  0,  0],
        [0, 0, 0,  0,  1,  0],
        [0, 0, 0,  0,  0,  1]
    ])
    return np.dot(F, x)

# Define the measurement function for UKF
def hx(x):
    """
    Measurement function: extracts position from state.
    """
    return x[:3] # Assuming you only measure position

# Initialize the UKF
def initialize_ukf(dt, initial_state, initial_covariance, process_noise, measurement_noise):
    # Initialize the sigma points
    points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2., kappa=0)
    
    ukf = UKF(dim_x=6, dim_z=3, fx=fx, hx=hx, dt=dt, points=points)
    ukf.x = initial_state
    ukf.P = initial_covariance
    ukf.Q = process_noise
    ukf.R = measurement_noise
    return ukf

def init_ukf(initial_position, dt=DT):
    """
    Apply UKF to a single marker's data.
    """

    # Initialize state: position from first measurement, velocity assumed zero
    initial_velocity = [0, 0, 0]
    initial_state = np.hstack((initial_position, initial_velocity))

    # Initialize covariance matrices
    # initial_covariance = np.eye(6) * 100.0  # Initial uncertainty
    # process_noise = np.eye(6) * 1e-1       # Process noise
    # measurement_noise = np.eye(3) * 1e-2   # Measurement noise

    # initial_covariance = np.eye(6) * 1.0  # Initial uncertainty
    # process_noise = np.eye(6) * 1e-3       # Process noise
    # measurement_noise = np.eye(3) * 1e-1   # Measurement noise
    
    initial_covariance = np.eye(6) * 1e-1  # Initial uncertainty
    process_noise = np.eye(6) * 1e-2       # Process noise (level of disturbances or model inaccuracies)
    measurement_noise = np.eye(3) * 1e-2  # Measurement noise (accuracy and precision of sensors)

    ukf = initialize_ukf(dt, initial_state, initial_covariance, process_noise, measurement_noise)

    return ukf

def fx_quat(q, dt):
    """
    State transition function for quaternion filtering.
    Uses quaternion multiplication to propagate orientation.
    """
    # Convert quaternion state to scipy Rotation object
    rot = R.from_quat(q[:4])  # First 4 elements are quaternion

    # Apply a small rotation to simulate continuous motion (assume small angular velocity)
    delta_rot = R.from_rotvec(q[4:] * dt)  # Last 3 elements are angular velocity

    # Update quaternion using rotation multiplication
    new_q = (rot * delta_rot).as_quat()

    # Normalize quaternion to prevent drift
    new_q /= np.linalg.norm(new_q)

    # Return updated quaternion + unchanged angular velocity
    return np.hstack((new_q, q[4:]))  

def hx_quat(q):
    """
    Measurement function: extracts the quaternion (first 4 elements).
    """
    return q[:4]  # Extract quaternion

def initialize_quaternion_ukf(dt, initial_quaternion, initial_angular_velocity):
    """
    Initializes a UKF for quaternion filtering.
    """
    # Define sigma points for a 7D state (4D quaternion + 3D angular velocity)
    points = MerweScaledSigmaPoints(n=7, alpha=0.1, beta=2., kappa=0)

    # Initialize UKF with 7D state (Quaternion + Angular Velocity)
    ukf = UKF(dim_x=7, dim_z=4, fx=fx_quat, hx=hx_quat, dt=dt, points=points)

    # Initial state: quaternion + angular velocity
    ukf.x = np.hstack((initial_quaternion, initial_angular_velocity))

    # Covariances (adjust based on noise levels)
    ukf.P = np.eye(7) * 0.1  # Initial uncertainty
    ukf.Q = np.eye(7) * 0.01  # Process noise
    ukf.R = np.eye(4) * 0.01  # Measurement noise

    return ukf