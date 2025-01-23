import numpy as np
import rospy

def no_switching_function(behaviors, params):
    selected_behavior = behaviors[0]
    return 0, selected_behavior

def time_switching_function(behaviors, params):
    current_time = rospy.Time.now().to_sec()
    selected_index = 0
    switching_frequency = 0.5  # Hz
    period = 1 / switching_frequency
    if current_time % period < period / 2:
        selected_index = 0
    else:
        selected_index = 1
    return selected_index, behaviors[selected_index]