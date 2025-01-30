import numpy as np
import rospy

def no_switching_function(behaviors, params):
    selected_behavior = behaviors[0]
    return 0, selected_behavior

def time_switching_function(behaviors, params):
    current_time = rospy.Time.now().to_sec()
    selected_index = 0
    switching_frequency = 0.1  # Hz
    period = 1 / switching_frequency
    if current_time % period < period / 2:
        selected_index = 0
    else:
        selected_index = 1
    return selected_index, behaviors[selected_index]

def pos_switching_function(behaviors, params):
    """switch if x is within some set"""
    sb = params['switch_border']
    x = params['x']
    if x[0] >= sb[0] and x[1] >= sb[1]:
        selected_index = 1
    else:
        selected_index = 0
    print(selected_index)
    return selected_index, behaviors[selected_index]