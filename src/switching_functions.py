import numpy as np

def time_switching_function(self, behaviors, params):
    n_behaviors = len(behaviors)  # number of behaviors that we can switch in between
    switch_memory = params['switch_memory']
    selected_behavior = switch_memory if switch_memory < n_behaviors - 1 else 0
    return selected_behavior