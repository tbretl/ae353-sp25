import numpy as np

import time

# import ae353_drone

K = np.array([1., 2., 3.])


class Controller:
    def __init__(self):
        # print('print some debug information during init')
        self.variables_to_log = []

    def get_color(self):
        return [0., 1., 0.]

    def reset(self, p_x, p_y, p_z, yaw):
        time.sleep(1.)
        pass

    def run(self, pos_markers, pos_ring, dir_ring, is_last_ring, pos_others):
        
        K2 = K.copy()

        tau_x = 0.
        tau_y = 0.
        tau_z = 0.
        f_z = 10.

        # return tau_x, tau_y, tau_z #, f_z
        return tau_x, tau_y, tau_z, f_z