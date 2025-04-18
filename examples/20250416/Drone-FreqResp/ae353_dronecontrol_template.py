import numpy as np

class Controller:
    def __init__(self):
        # Define all variables (FIXME)
        #
        # self.A = ...
        # self.B = ...
        # ...
        # ...
        
        self.variables_to_log = ['xhat', 'xdes', 'ring']

    def get_color(self):
        return [1., 0., 0.]

    def reset(
            self,
            p_x, p_y, p_z, # <-- approximate initial position of drone (meters)
            yaw,           # <-- approximate initial yaw angle of drone (radians)
        ):
        
        # Choose initial state estimate (FIXME)
        # self.xhat = ...
        
        self.t_controller = 0.
        self.count = 0

    def run(
            self,
            pos_markers,
            pos_ring,
            dir_ring,
            is_last_ring,
            pos_others,
        ):
        
        # Store ring position for later analysis
        self.ring = np.array([p_x_ring, p_y_ring, p_z_ring])
        
        # Get xdes
        self.xdes = self.get_xdes(self.count * self.dt, self.xhat, pos_ring, dir_ring, is_last_ring, pos_others)
        
        # Apply controller and observer... (FIXME)
        #
        # u = ...
        # y = ...
        # self.xhat += ...
        #
        # tau_x = ...
        # tau_y = ...
        # tau_z = ...
        # f_z = ...
        
        return tau_x, tau_y, tau_z, f_z
    
    def get_xdes(self, t, xhat, pos_ring, dir_ring, is_last_ring, pos_others):
        xdes = np.zeros(12)
        xdes[0:3] = np.array([-10., 0., 0.5])
        return xdes