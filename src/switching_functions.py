import numpy as np

def no_switching_function(behaviors, params):
    selected_behavior = behaviors[0]
    return selected_behavior

def time_switching_function(behaviors, params):
    switch_counter = params['switch_counter']
    selected_behavior = behaviors[(switch_counter // 5) % 2]
    return selected_behavior