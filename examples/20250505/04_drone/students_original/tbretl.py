import numpy as np

class Controller:
    def __init__(self):
        self.variables_to_log = []

    def get_color(self):
        return [0., 1., 0.]

    def reset(self, p_x, p_y, p_z, yaw):
        pass

    def run(self, pos_markers, pos_ring, dir_ring, is_last_ring, pos_others):
        
        tau_x = 0.
        tau_y = 0.
        tau_z = 0.
        f_z = 10.

        return tau_x, tau_y, tau_z, f_z