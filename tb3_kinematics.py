from sensor_msgs.msg import JointState

import numpy as np
from typing import List

# Indexing values
LEFT = 0
RIGHT = 1

X = 0
Y = 1
THETA = 2

#PASSED
def calculate_wheel_change(new_joint_states: JointState, prev_joint_states: JointState) -> List[float]:
    # Inputs:
    #   new_joint_states    new joint states
    #   prev_joint_states   previous joint states
    # Outputs:
    #   delta_wheel_l       change in left wheel angle [rad]
    #   delta_wheel_r       change in right wheel angle [rad]
    #   delta_time          change in time [s]

    delta_wheel_l = new_joint_states.position[LEFT] -prev_joint_states.position[LEFT]  # FILL THIS IN
    delta_wheel_r = new_joint_states.position[RIGHT] -prev_joint_states.position[RIGHT] # FILL THIS IN
    delta_time    = new_joint_states.header.stamp.to_sec() - prev_joint_states.header.stamp.to_sec() # FILL THIS IN
    
    # Data validation
    if np.isnan(delta_wheel_l):
        delta_wheel_l = 0.0
    if np.isnan(delta_wheel_r):
        delta_wheel_r = 0.0

    return (delta_wheel_l, delta_wheel_r, delta_time)

#PASSED
def calculate_displacement(delta_wheel_l: float, delta_wheel_r: float, wheel_radius: float, wheel_separation: float) -> List[float]:
    # Inputs:
    #   delta_wheel_l       change in left wheel angle [rad]
    #   delta_wheel_r       change in right wheel angle [rad]
    #   wheel_radius        wheel radius [m]
    #   wheel_separation    wheel separation [m]
    # Outputs:
    #   delta_s         linear displacement [m]
    #   delta_theta     angular displacement [rad]

    delta_s     = (wheel_radius * delta_wheel_r + wheel_radius * delta_wheel_l) / 2 # FILL THIS IN, linear displacement [m]
    delta_theta = (wheel_radius * delta_wheel_r - wheel_radius * delta_wheel_l) / wheel_separation # FILL THIS IN, angular displacement [rad]

    return (delta_s, delta_theta)

#PASSED
def calculate_pose(prev_pose: List[float], delta_s: float, delta_theta: float) -> List[float]:
    # Inputs:
    #   prev_pose       input pose in format (x, y, theta) [m, m, rad]
    #   delta_s         linear displacement [m]
    #   delta_theta     angular displacement [rad]
    # Outputs:
    #   pose            output pose in format (x, y, theta) [m, m, rad]

    x_prev, y_prev, theta_prev = prev_pose 
    x = x_prev + delta_s * np.cos(theta_prev + (delta_theta/2))
    y = y_prev + delta_s * np.sin(theta_prev+ (delta_theta/2))
    theta = theta_prev + delta_theta   
    pose = [x, y, theta]

    return pose
