import numpy as np
import rospy

def no_switching_function(behaviors, params):
    selected_behavior = behaviors[0]
    return 0, selected_behavior

def time_switching_function(behaviors, params):
    current_time = rospy.Time.now().to_sec()
    selected_index = 0
    switching_frequency = 3. # 0.1 for base movement  # Hz
    period = 1 / switching_frequency
    if current_time % period < period / 2:
        selected_index = 0
    else:
        selected_index = 1
    print(selected_index)
    return selected_index, behaviors[selected_index]

def pos_switching_function(behaviors, params):
    """switch if x is within some set"""
    sb = params['switch_border']
    x = params['x']
    x_obst1 = np.array([0.5, -0.5])
    x_obst2 = np.array([3.5, 0.5])
    x_obst3 = np.array([3., -1.5])
    if (np.linalg.norm(x[:2] - x_obst1) <= 1.06) or (np.linalg.norm(x[:2] - x_obst2) <= 1.06) or (x[3] <= 0.25):  # before was <= 1.
        selected_index = 1
    else:
        selected_index = 0
    print(selected_index)
    return selected_index, behaviors[selected_index]