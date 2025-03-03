import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time
import json
import importlib
import meshcat
from pathlib import Path
import umsgpack
from playsound import playsound

colors = {
    'industrial-blue': [0.11372549019607843, 0.34509803921568627, 0.6549019607843137, 1.0],
    'arches-blue': [0.0, 0.6235294117647059, 0.8313725490196079, 1.0],
    'heritage-orange': [0.96078431, 0.50980392, 0.11764706, 1.0],
}

class Simulator:
    def __init__(
                self,
                display=True,
                seed=None,
                dt=0.01,
                maximum_initial_airspeed_error=0.5,
                maximum_initial_psi=(np.pi / 6),
                maximum_initial_theta=(np.pi / 6),
                maximum_initial_phi=(np.pi / 6),
            ):
        
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Time step
        self.dt = dt

        # Elevon angles
        self.delta_r = 0.
        self.delta_l = 0.

        # Parameters
        self.initial_airspeed = 5.
        self.maximum_initial_airspeed_error = maximum_initial_airspeed_error
        self.maximum_initial_psi = maximum_initial_psi
        self.maximum_initial_theta = maximum_initial_theta
        self.maximum_initial_phi = maximum_initial_phi
        self.landing_length = 25.
        self.landing_width = 5.
        self.landing_height = 1.
        self.landing_position = [150. + (2 * self.landing_length / 5), 0., 15. + (self.landing_height / 2)]
        self.maximum_elevon_deflection = 0.5        # <-- slightly less than 30 degrees
        self.params = {
            'g': 9.81,               # Gravity (m/s²)
            'm': 1.56,               # Mass of the UAV (kg)
            'J_x': 0.1147,           # Moment of inertia about x-axis (kg·m²) [UPDATED 02/28/2025]
            'J_y': 0.0576,           # Moment of inertia about y-axis (kg·m²) [UPDATED 02/28/2025]
            'J_z': 0.1712,           # Moment of inertia about z-axis (kg·m²) [UPDATED 02/28/2025]
            'J_xz': 0.0015,          # Product of inertia (kg·m²)             [UPDATED 02/28/2025]
            
            'S': 0.4696,             # Wing area (m²)
            'b': 1.4224,             # Wingspan (m)
            'c': 0.3302,             # Mean aerodynamic chord (m)

            'rho': 1.2682,           # Air density (kg/m³)

            # Lift Coefficients
            'C_L_0': 0.2,            # Lift coefficient at zero AoA
            'C_L_alpha': 4.8,        # Lift curve slope (1/rad)
            'C_L_q': 2.2,            # Pitch rate effect on lift (1/rad)

            # Drag Coefficients
            'C_D_0': 0.02,           # Zero-lift drag coefficient
            'C_D_alpha': 0.30,       # Drag change per AoA (1/rad)
            'C_D_q': 0.0,            # Pitch rate effect on drag (1/rad)
            'C_D_p': 0.03,           # Parasitic drag coefficient

            # Pitching Moment Coefficients
            'C_m_0': -0.02,          # Pitching moment at zero AoA
            'C_m_alpha': -0.6,       # Pitching moment change per AoA (1/rad)
            'C_m_q': -1.8,           # Pitch rate effect on moment (1/rad)
            'C_m_delta_e': -0.35,    # Effect of elevator deflection on pitching moment (1/rad)

            # Side Force Coefficients
            'C_Y_0': 0.0,            # Side force at zero sideslip
            'C_Y_beta': -0.08,       # Side force per sideslip angle (1/rad)
            'C_Y_p': 0.0,            # Side force due to roll rate
            'C_Y_r': 0.0,            # Side force due to yaw rate
            'C_Y_delta_a': 0.0,      # Side force due to aileron deflection

            # Roll Moment Coefficients
            'C_l_0': 0.0,            # Roll moment at zero sideslip
            'C_l_beta': -0.10,       # Roll moment due to sideslip (1/rad)
            'C_l_p': -0.45,          # Roll damping derivative (1/rad)
            'C_l_r': 0.03,           # Roll moment due to yaw rate (1/rad)
            'C_l_delta_a': 0.18,     # Aileron effect on roll (1/rad)

            # Yaw Moment Coefficients
            'C_n_0': 0.0,            # Yaw moment at zero sideslip
            'C_n_beta': 0.008,       # Yaw moment due to sideslip (1/rad)
            'C_n_p': -0.022,         # Yaw moment due to roll rate (1/rad)
            'C_n_r': -0.009,         # Yaw damping derivative (1/rad)
            'C_n_delta_a': -0.004,   # Aileron effect on yaw (1/rad)

            # Control Derivatives
            'C_L_delta_e': 0.30,     # Effect of elevator deflection on lift (1/rad)
            'C_D_delta_e': 0.32,     # Effect of elevator deflection on drag (1/rad)

            # Efficiency Factors
            'e': 0.85,               # Oswald efficiency factor
            'alpha_0': 0.45,         # Zero-lift angle of attack (rad)

            # Additional Drag & Lift Coefficients
            'M': 50.0,               # Sigmoid blending function parameter
            'k_e': 0.01,             # Drag due to elevator deflection (empirical coefficient)
            'k': 0.048               # Induced drag factor
        }
        
        # Connect to and configure pybullet
        self.display_meshcat = display
        self.bullet_client = bullet_client.BulletClient(
            connection_mode=pybullet.DIRECT,
        )
        self.bullet_client.setGravity(0, 0, 9.81)   # <-- +z is down
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )
        
        # Load platform
        self.platform_id = self.bullet_client.loadURDF(
            str(Path('./urdf/platform.urdf')),
            basePosition=np.array(self.landing_position),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE),
            useFixedBase=1,
        )

        # Load platform
        self.ring_id = self.bullet_client.loadURDF(
            str(Path('./urdf/ring.urdf')),
            basePosition=np.array([-0.5, 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE),
            useFixedBase=1,
        )

        # Load robot
        self.robot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/zagi.urdf')),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ))
        
        # Set contact and damping parameters
        for object_id in [self.robot_id]:
            for joint_id in range(-1, self.bullet_client.getNumJoints(object_id)):
                self.bullet_client.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=1.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1,
                    linearDamping=0.,
                    angularDamping=0.,
                )

        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()

        # Set default camera view
        self.camera_launchview()

    def get_sensor_measurements(self):
        # Position and orientation
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        rpy = self.bullet_client.getEulerFromQuaternion(ori)

        # Linear and angular velocity
        vel = self.bullet_client.getBaseVelocity(self.robot_id)
        v_world = np.array(vel[0])
        w_world = np.array(vel[1])
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        v_body = R_body_in_world.T @ v_world
        w_body = R_body_in_world.T @ w_world

        # Get components of everything
        p_x, p_y, p_z = pos
        phi, theta, psi = rpy
        v_x, v_y, v_z = v_body
        w_x, w_y, w_z = w_body

        return p_x, p_y, p_z, psi, theta, phi, v_x, v_y, v_z, w_x, w_y, w_z

    def has_landed(self):
        p_x, p_y, p_z, psi, theta, phi, v_x, v_y, v_z, w_x, w_y, w_z = self.get_sensor_measurements()

        # Airspeed must be near zero
        V_a = np.sqrt(v_x**2 + v_y**2 + v_z**2)

        if V_a > 1e-2:
            return False
        
        if p_z < 14.75:
            return False
        
        if p_z > 15.00:
            return False
        
        if p_y > self.landing_width / 2:
            return False

        if p_y < -self.landing_width / 2:
            return False
        
        if p_x > self.landing_position[0] + (self.landing_length / 2):
            return False
        
        if p_x < self.landing_position[0] - (self.landing_length / 2):
            return False
        
        return True
    
    def set_actuator_commands(
                self,
                delta_r_command,
                delta_l_command,
            ):
        
        if not np.isscalar(delta_r_command):
            raise Exception('delta_r_command must be a scalar')
        
        if not np.isscalar(delta_l_command):
            raise Exception('delta_l_command must be a scalar')
        
        self.delta_r = np.clip(delta_r_command, -self.maximum_elevon_deflection, self.maximum_elevon_deflection)
        self.delta_l = np.clip(delta_l_command, -self.maximum_elevon_deflection, self.maximum_elevon_deflection)

        return self.delta_r, self.delta_l
    
    def reset(
            self,
            initial_conditions=None,
        ):

        if initial_conditions is None:
            initial_conditions = {
                'p_x': 0.,
                'p_y': 0.,
                'p_z': 0.,
                'psi': self.rng.uniform(-self.maximum_initial_psi, self.maximum_initial_psi),
                'theta': self.rng.uniform(-self.maximum_initial_theta, self.maximum_initial_theta),
                'phi': self.rng.uniform(-self.maximum_initial_phi, self.maximum_initial_phi),
                'v_x': self.rng.uniform(self.initial_airspeed - self.maximum_initial_airspeed_error,
                                        self.initial_airspeed + self.maximum_initial_airspeed_error),
                'v_y': 0.,
                'v_z': 0.,
                'w_x': 0.,
                'w_y': 0.,
                'w_z': 0.,
            }

        # Set position and orientation
        self.bullet_client.resetBasePositionAndOrientation(
            self.robot_id,
            [
                initial_conditions['p_x'],
                initial_conditions['p_y'],
                initial_conditions['p_z'],
            ],
            self.bullet_client.getQuaternionFromEuler([
                initial_conditions['phi'],
                initial_conditions['theta'],
                initial_conditions['psi'],
            ]),
        )

        # Get position and orientation again, because it's an easy way to compute the rotation
        # matrix that describes the orientation of the body frame in the world frame
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))

        # Set linear and angular velocity
        # - Define linear velocity in body frame
        v_body = np.array([
            initial_conditions['v_x'],
            initial_conditions['v_y'],
            initial_conditions['v_z'],
        ])
        # - Compute linear velocity in world frame
        v_world = R_body_in_world @ v_body
        # - Define angular velocity in body frame
        w_body = np.array([
            initial_conditions['w_x'],
            initial_conditions['w_y'],
            initial_conditions['w_z'],
        ])
        # - Compute angular velocity in world frame
        w_world = R_body_in_world @ w_body
        # - Set linear and angular velocity in world frame
        self.bullet_client.resetBaseVelocity(
            self.robot_id,
            linearVelocity=v_world,
            angularVelocity=w_world,
        )

        # Set elevon angles
        self.delta_r = 0.
        self.delta_l = 0.
        
        # Update display
        self._update_display()
    
    def run(
            self,
            controller,
            maximum_time=5.0,
            data_filename=None,
            video_filename=None,
            print_debug=False,
        ):

        self.data = {
            't': [],
            'p_x': [],
            'p_y': [],
            'p_z': [],
            'psi': [],
            'theta': [],
            'phi': [],
            'v_x': [],
            'v_y': [],
            'v_z': [],
            'w_x': [],
            'w_y': [],
            'w_z': [],
            'delta_r_command': [],
            'delta_l_command': [],
            'delta_r': [],
            'delta_l': [],
            'f_x': [],
            'f_y': [],
            'f_z': [],
            'tau_x': [],
            'tau_y': [],
            'tau_z': [],
        }
        self.variables_to_log = getattr(controller, 'variables_to_log', [])
        for key in self.variables_to_log:
            if key in self.data.keys():
                raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
            self.data[key] = []

        # Always start from zero time
        self.t = 0.
        self.time_step = 0
        self.maximum_time_steps = 1 + int(maximum_time / self.dt)
        self.start_time = time.time()

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = 25
            if int(1 / self.dt) % fps != 0:
                raise Exception(f'To create a video, 1 / dt ({1 / self.dt}) must be an ' + \
                                 'integer that is divisible by fps ({fps})')
            if print_debug:
                print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = self.step(controller)

            self._update_display()

            if video_filename is not None:
                if self.time_step % 100 == 0:
                    if print_debug:
                        print(f' {self.time_step} / {self.maximum_time_steps}')

                # Add frame to video
                if self.time_step % int(1 / (self.dt * fps)) == 0:
                    rgba = self.snapshot()
                    w.append_data(rgba)

            if all_done:
                break

            if (self.maximum_time_steps is not None) and (self.time_step == self.maximum_time_steps):
                break

        if video_filename is not None:
            # Close video
            w.close()

        if data_filename is not None:
            with open(data_filename, 'w') as f:
                json.dump(self.data, f)

        stop_time = time.time()
        stop_time_step = self.time_step

        elapsed_time = stop_time - self.start_time
        elapsed_time_steps = stop_time_step
        if (elapsed_time > 0) and print_debug:
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ' + \
                  f'({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')

        # convert lists to numpy arrays
        data = self.data.copy()
        for key in data.keys():
            data[key] = np.array(data[key])

        return data

    def get_aerodynamic_forces_numeric(self, u, v, w, p, q, r, delta_r, delta_l, params):
        # Define parameters
        rho, S, c, b, g = params['rho'], params['S'], params['c'], params['b'], params['g']
        C_L_0, C_L_alpha, C_L_q, C_L_delta_e = params['C_L_0'], params['C_L_alpha'], params['C_L_q'], params['C_L_delta_e']
        C_D_0, C_D_alpha, C_D_q, C_D_delta_e = params['C_D_0'], params['C_D_alpha'], params['C_D_q'], params['C_D_delta_e']
        C_m_0, C_m_alpha, C_m_q, C_m_delta_e = params['C_m_0'], params['C_m_alpha'], params['C_m_q'], params['C_m_delta_e']
        C_Y_0, C_Y_beta, C_Y_p, C_Y_r, C_Y_delta_a = params['C_Y_0'], params['C_Y_beta'], params['C_Y_p'], params['C_Y_r'], params['C_Y_delta_a']
        C_l_0, C_l_beta, C_l_p, C_l_r, C_l_delta_a = params['C_l_0'], params['C_l_beta'], params['C_l_p'], params['C_l_r'], params['C_l_delta_a']
        C_n_0, C_n_beta, C_n_p, C_n_r, C_n_delta_a = params['C_n_0'], params['C_n_beta'], params['C_n_p'], params['C_n_r'], params['C_n_delta_a']
        e, alpha_0, C_D_p, M = params['e'], params['alpha_0'], params['C_D_p'], params['M']
        k, k_e = params['k'], params['k_e']

        # Get airspeed, angle of attack, and angle of sideslip
        V_a = np.sqrt(u**2 + v**2 + w**2)
        if np.isclose(V_a, 0.):
            return 0., 0., 0., 0., 0., 0.
        if np.isclose(u, 0.):
            alpha = np.sign(u * w) * (np.pi / 2) # <-- attempting to make alpha have the correct sign
        else:
            alpha = np.atan(w / u)
        beta = np.asin(v / V_a)

        # Convert from right and left elevons to elevator and aileron
        delta_e = (delta_r + delta_l) / 2
        delta_a = (-delta_r + delta_l) / 2

        # Longitudinal aerodynamics
        sigma = (1. + np.exp(-M * (alpha - alpha_0)) + np.exp(M * (alpha + alpha_0))) / ((1. + np.exp(-M * (alpha - alpha_0))) * (1. + np.exp(M * (alpha + alpha_0))))
        C_L = (1 - sigma) * (C_L_0 + C_L_alpha * alpha) + sigma * (2. * np.sign(alpha) * (np.sin(alpha)**2) * np.cos(alpha))
        F_lift = rho * V_a**2 * S * (C_L + C_L_q * (c / (2 * V_a)) * q + C_L_delta_e * delta_e) / 2
        F_drag = rho * V_a**2 * S * ((C_D_0 + k * C_L**2) + C_D_q * (c / (2 * V_a)) * q + k_e * (C_L_delta_e * delta_e)**2) / 2
        f_x, f_z = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]]) @ np.array([-F_drag, -F_lift])
        m_y = rho * V_a**2 * S * c * (C_m_0 + C_m_alpha * alpha + C_m_q * (c / (2 * V_a)) * q + C_m_delta_e * delta_e) / 2

        # Lateral aerodynamics (no rudder)
        f_y = rho * V_a**2 * S * (C_Y_0 + C_Y_beta * beta + C_Y_p * (b / (2 * V_a)) * p + C_Y_r * (b / (2 * V_a)) * r + C_Y_delta_a * delta_a) / 2
        m_x = rho * V_a**2 * S * b * (C_l_0 + C_l_beta * beta + C_l_p * (b / (2 * V_a)) * p + C_l_r * (b / (2 * V_a)) * r + C_l_delta_a * delta_a) / 2
        m_z = rho * V_a**2 * S * b * (C_n_0 + C_n_beta * beta + C_n_p * (b / (2 * V_a)) * p + C_n_r * (b / (2 * V_a)) * r + C_n_delta_a * delta_a) / 2

        return f_x, f_y, f_z, m_x, m_y, m_z

    def step(self, controller):
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        p_x, p_y, p_z, psi, theta, phi, v_x, v_y, v_z, w_x, w_y, w_z = self.get_sensor_measurements()
        
        # Get the actuator commands
        delta_r_command, delta_l_command = controller.run(
            self.t,
            p_x, p_y, p_z, psi, theta, phi, v_x, v_y, v_z, w_x, w_y, w_z,
        )

        # Apply the actuator commands
        delta_r, delta_l = self.set_actuator_commands(delta_r_command, delta_l_command)
        
        # Get aerodynamic forces and torques
        f_x, f_y, f_z, tau_x, tau_y, tau_z = self.get_aerodynamic_forces_numeric(
            v_x, v_y, v_z, w_x, w_y, w_z, delta_r, delta_l, self.params,
        )

        # Apply aerodynamic forces
        self.bullet_client.applyExternalForce(
            self.robot_id,
            -1,
            np.array([f_x, f_y, f_z]),
            np.array([0., 0., 0.]),
            self.bullet_client.LINK_FRAME,
        )

        # Apply aerodynamic torques
        self.bullet_client.applyExternalTorque(
            self.robot_id,
            -1,
            np.array([tau_x, tau_y, tau_z]),
            self.bullet_client.LINK_FRAME,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['p_x'].append(p_x)
        self.data['p_y'].append(p_y)
        self.data['p_z'].append(p_z)
        self.data['psi'].append(psi)
        self.data['theta'].append(theta)
        self.data['phi'].append(phi)
        self.data['v_x'].append(v_x)
        self.data['v_y'].append(v_y)
        self.data['v_z'].append(v_z)
        self.data['w_x'].append(w_x)
        self.data['w_y'].append(w_y)
        self.data['w_z'].append(w_z)
        self.data['delta_r_command'].append(delta_r_command)
        self.data['delta_l_command'].append(delta_l_command)
        self.data['delta_r'].append(delta_r)
        self.data['delta_l'].append(delta_l)
        self.data['f_x'].append(f_x)
        self.data['f_y'].append(f_y)
        self.data['f_z'].append(f_z)
        self.data['tau_x'].append(tau_x)
        self.data['tau_y'].append(tau_y)
        self.data['tau_z'].append(tau_z)
        for key in self.variables_to_log:
            val = getattr(controller, key, np.nan)
            if not np.isscalar(val):
                val = val.flatten().tolist()
            self.data[key].append(val)

        # Try to stay real-time
        if self.display_meshcat:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.75 * time_to_wait)
                time_to_wait = t - time.time()

        # Take a simulation step
        self.bullet_client.stepSimulation()

        # Increment time step
        self.time_step += 1

        return all_done
    
    def meshcat_snapshot(self):
        # Get image from visualizer
        rgba = np.asarray(self.vis.get_image())

        # Shrink width and height to be multiples of 16
        height, width, channels = rgba.shape
        m = 16
        return np.ascontiguousarray(rgba[
            :(m * np.floor(height / m).astype(int)),
            :(m * np.floor(width / m).astype(int)),
            :,
        ])
    
    def snapshot(self):
        if self.display_meshcat:
            return self.meshcat_snapshot()
        else:
            raise Exception('you must set display=True in order to take a snapshot')

    def _update_display(self):
        if self.display_meshcat:
            if self.is_catview:
                pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
                x, y, z = pos
                y *= -1
                z *= -1
                self.vis.set_cam_pos([x - 1.5, y, z + 0.5])
                self.vis.set_cam_target([x, y, z])

            self.meshcat_update()

    def camera_catview(self):
        if not self.display_meshcat:
            return
        
        self.is_catview = True
        self._update_display()
    
    def camera_launchview(self):
        if not self.display_meshcat:
            return
        
        self.is_catview = False

        self.vis.set_cam_pos([-3.8, 0., .2])
        self.vis.set_cam_target([0., 0., 0.])
        
        self._update_display()
    
    def camera_landview(self):
        if not self.display_meshcat:
            return

        self.is_catview = False

        cam_pos = [
            self.landing_position[0] + (3 * self.landing_length / 5) + 2.,
            -(self.landing_position[1]),
            -(self.landing_position[2] - 2.),
        ]
        cam_target = [
            self.landing_position[0],
            -(self.landing_position[1]),
            -(self.landing_position[2] - 2.),
        ]
        
        self.vis.set_cam_pos(cam_pos)
        self.vis.set_cam_target(cam_target)

        self._update_display()
    
    def _wxyz_from_xyzw(self, xyzw):
        return np.roll(xyzw, 1)

    def _convert_color(self, rgba):
        color = int(rgba[0] * 255) * 256**2 + int(rgba[1] * 255) * 256 + int(rgba[2] * 255)
        opacity = rgba[3]
        transparent = opacity != 1.0
        return {
            'color': color,
            'opacity': opacity,
            'transparent': transparent,
        }

    def meshcat_lights(self):
        # As of 1/21/2025, meshcat-python has a bug that does not allow
        # setting a property with characters that aren't all lower-case.
        # The reason is that, prior to sending commands, the name of the
        # property is converted to lower-case. So, we will DIY it here.

        lights = ['SpotLight', 'PointLightNegativeX', 'PointLightPositiveX']
        intensity = 0.5
        for light in lights:
            self.vis[f'/Lights/{light}'].set_property('visible', True)
            self.vis[f'/Lights/{light}/<object>'].set_property('intensity', intensity)
            cmd_data = {
                u'type': u'set_property',
                u'path': self.vis[f'/Lights/{light}/<object>'].path.lower(),
                u'property': u'castShadow',
                u'value': True,
            }
            self.vis.window.zmq_socket.send_multipart([
                cmd_data['type'].encode('utf-8'),
                cmd_data['path'].encode('utf-8'),
                umsgpack.packb(cmd_data),
            ])
            res = self.vis.window.zmq_socket.recv()
            if res != b'ok':
                raise Exception(f'bad result on meshcat_lights() for light "{light}": {res}')
        
        self.vis['/Lights/PointLightPositiveX/<object>'].set_property('intensity', 1)
        self.vis['/Lights/PointLightPositiveX/<object>'].set_property('distance', 10)
        self.vis['/Lights/PointLightNegativeX/<object>'].set_property('position', [175, 0, -10])
        self.vis['/Lights/PointLightNegativeX/<object>'].set_property('distance', 20)
        self.vis['/Lights/PointLightNegativeX/<object>'].set_property('intensity', 0.75)

    
    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Add platform
        shape_data = self.bullet_client.getVisualShapeData(self.platform_id)
        if len(shape_data) != 1:
            raise Exception(f'platform has bad number of links {len(shape_data)}')
        s = shape_data[0]
        if s[1] != -1:
            raise Exception(f'base link has bad id {s[1]}')
        link_scale = s[3]
        stl_filename = s[4].decode('UTF-8')
        color = self._convert_color(s[7])
        self.vis['platform'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Set pose of platform
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.platform_id)
        S = np.diag(np.concatenate((link_scale, [1.0])))
        Rx = meshcat.transformations.rotation_matrix(np.pi, [1., 0., 0.])
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['platform'].set_transform(Rx @ T @ S)

        # Add ring
        shape_data = self.bullet_client.getVisualShapeData(self.ring_id)
        if len(shape_data) != 1:
            raise Exception(f'launcher has bad number of links {len(shape_data)}')
        s = shape_data[0]
        if s[1] != -1:
            raise Exception(f'base link has bad id {s[1]}')
        link_scale = s[3]
        stl_filename = s[4].decode('UTF-8')
        color = self._convert_color(s[7])
        self.vis['launcher'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Set pose of ring
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.ring_id)
        S = np.diag(np.concatenate((link_scale, [1.0])))
        Rx = meshcat.transformations.rotation_matrix(np.pi, [1., 0., 0.])
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['launcher'].set_transform(Rx @ T @ S)
        
        # Add robot
        shape_data = self.bullet_client.getVisualShapeData(self.robot_id)
        if len(shape_data) != 1:
            raise Exception(f'robot has bad number of links {len(shape_data)}')
        s = shape_data[0]
        if s[1] != -1:
            raise Exception(f'base link has bad id {s[1]}')
        if not np.allclose(s[3], 1.):
            raise Exception(f'base link has bad scale {s[3]}')
        stl_filename = s[4].decode('UTF-8')
        color = self._convert_color(s[7])
        self.vis['robot'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Add right elevon
        color = self._convert_color(colors['heritage-orange'])
        self.vis['robot']['elevon-right'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(str(Path('./urdf/elevon-right.stl'))),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Add left elevon
        color = self._convert_color(colors['heritage-orange'])
        self.vis['robot']['elevon-left'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(str(Path('./urdf/elevon-left.stl'))),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Add cat pilot
        color = self._convert_color(colors['heritage-orange'])
        self.vis['robot']['cat-pilot'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(str(Path('./urdf/cat-pilot.stl'))),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Turn off grid
        self.vis['/Grid'].set_property('visible', False)

        # Add lights
        self.meshcat_lights()

        # Set background color
        self.vis['/Background'].set_property('top_color', [0/255, 159/255, 212/255])
        self.vis['/Background'].set_property('bottom_color', [1, 1, 1])

        # Set clipping range of camera
        self.vis['/Cameras/default/rotated/<object>'].set_property('near', 0.1)
        self.vis['/Cameras/default/rotated/<object>'].set_property('far', 500.)

        # Turn off axes
        self.vis[f'/Axes/<object>'].set_property('visible', False)

        
    def meshcat_update(self):
        # Set pose of robot
        Rx = meshcat.transformations.rotation_matrix(np.pi, [1., 0., 0.])
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['robot'].set_transform(Rx @ T)

        # Set position of spotlight
        # self.vis['/Lights/SpotLight/<object>'].set_property('position', [pos[0], -pos[1], -(pos[2] - 1.)])
        self.vis['/Lights/PointLightPositiveX/<object>'].set_property('position', [pos[0], -pos[1], -(pos[2] - 1.)])

        # Set pose of right elevon
        T = meshcat.transformations.translation_matrix([-0.23404, 0.15886, 0.008353])
        R = meshcat.transformations.rotation_matrix(self.delta_r, [-0.37352883778028634, 0.9275991332385393, -0.006004611695959905])
        self.vis['robot']['elevon-right'].set_transform(T @ R)

        # Set pose of left elevon
        T = meshcat.transformations.translation_matrix([-0.23404, -0.15886, 0.008353])
        R = meshcat.transformations.rotation_matrix(self.delta_l, [0.37352883778028634, 0.9275991332385393, 0.006004611695959905])
        self.vis['robot']['elevon-left'].set_transform(T @ R)

        # Set pose of cat pilot
        cat_pilot_scale = 0.1
        S = np.diag(np.concatenate((cat_pilot_scale * np.ones(3), [1.0])))
        T = meshcat.transformations.translation_matrix([0.2, 0., 0.7])
        R = meshcat.transformations.rotation_matrix(np.pi / 2, [0., 0., 1.])
        self.vis['robot']['cat-pilot'].set_transform(Rx @ S @ T @ R)