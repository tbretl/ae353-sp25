import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time
import json
import importlib
import meshcat
from pathlib import Path
import umsgpack


class Simulator:
    def __init__(
            self,
            dt=0.01,
            display=True,
            display_pybullet=False,
            seed=None,
            scope_noise=0.1,
            width=640,
            height=480,
        ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = dt

        # Other parameters
        # - Maximum applied torque
        self.tau_max = 2.
        # - Maximum wheel speed (50 rad/s is about 500 rpm)
        self.v_max = 50.
        # - Docking parameters
        self.docking_speed = 0.1
        self.docking_radius = 1.7
        self.docking_buffer = 0.1
        # - Star parameters
        self.scope_radius = 0.8 / 2.1
        self.scope_angle = np.arctan(self.scope_radius)
        self.scope_noise = scope_noise
        self.star_depth = 25.
        self.max_num_stars = 10

        # Connect to and configure pybullet
        self.display_pybullet = display_pybullet
        self.display_meshcat = display
        if self.display_pybullet:
            options = '--background_color_red=0   ' \
                    + '--background_color_blue=0  ' \
                    + '--background_color_green=0 ' \
                    + f'--width={width} --height={height}'
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI,
                options=options,
            )
            self.bullet_client.configureDebugVisualizer(
                self.bullet_client.COV_ENABLE_GUI, 0,
            )
        else:
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.DIRECT,
            )
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load robot
        self.robot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/spacecraft.urdf')),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            useFixedBase=0,
            flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                   self.bullet_client.URDF_USE_INERTIA_FROM_FILE  )
        )

        # Load shooting star
        self.shootingstar = True
        self.shot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/shootingstar.urdf')),
            basePosition=np.array([0., 0., 2. * self.star_depth]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            useFixedBase=0,
            flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                    self.bullet_client.URDF_USE_INERTIA_FROM_FILE  )
        )
        self.bullet_client.changeDynamics(self.shot_id, -1, linearDamping=0., angularDamping=0.)

        # Load cat pilot
        self.cat_id = self.bullet_client.loadURDF(
            str(Path('./urdf/space-cat.urdf')),
            basePosition=np.array([0., 0., 3.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE),
        )
        
        # Create a dictionary that maps joint names to joint indices and
        # link names to link indices
        self.joint_map = {}
        self.link_map = {}
        for joint_id in range(self.bullet_client.getNumJoints(self.robot_id)):
            joint_name = self.bullet_client.getJointInfo(self.robot_id, joint_id)[1].decode('UTF-8')
            link_name = self.bullet_client.getJointInfo(self.robot_id, joint_id)[12].decode('UTF-8')
            self.joint_map[joint_name] = joint_id
            self.link_map[link_name] = joint_id

        # Create an array with the index of each joint we care about
        self.joint_names = [
            'bus_to_wheel_1',
            'bus_to_wheel_2',
            'bus_to_wheel_3',
            'bus_to_wheel_4',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to zero
        for id in self.joint_ids:
            self.bullet_client.changeDynamics(self.robot_id, id, jointDamping=0.)
        
        # Set other contact and damping parameters
        for object_id in [self.robot_id, self.shot_id, self.cat_id]:
            for joint_id in range(-1, self.bullet_client.getNumJoints(object_id)):
                self.bullet_client.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1,
                    linearDamping=0.,
                    angularDamping=0.,
                )
        
        # Disable velocity control on each robot joint so we can use torque control
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints),
        )

        # Place stars
        with open(Path('urdf/stars.json'), 'r') as f:
            self.stars = json.load(f)
        if len(self.stars) > self.max_num_stars:
            raise Exception(f'There must be no more than {self.max_num_stars} stars (you defined {len(self.stars)}).')
        for star in self.stars:
            star['pos'] = np.array([[np.cos(star['alpha']) * np.cos(star['delta'])],
                                    [np.sin(star['alpha']) * np.cos(star['delta'])],
                                    [np.sin(star['delta'])]]) * self.star_depth
            star['id'] = self.bullet_client.loadURDF(
                str(Path('./urdf/sphere.urdf')),
                basePosition=star['pos'].flatten(),
                useFixedBase=1,
            )
        
        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()

        # Set default camera view
        self.camera_sideview()
    
    def get_sensor_measurements(self):
        """
        returns a 1d numpy array of length 2 * num_stars with image coordinates of star i
        (in 1, 2, ...), or nan if out of scope, at indices (2 * i - 2) and (2 * i - 1)
        """
        
        # position of each star in the image frame
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        pos_in_image = []
        for star in self.stars:
            pos_in_body = (R_body_in_world.T @ (-o_body_in_world + star['pos'])).flatten()
            star['y'] = (pos_in_body[1] / pos_in_body[0]) / self.scope_radius
            star['z'] = (pos_in_body[2] / pos_in_body[0]) / self.scope_radius
            if (star['y']**2 + star['z']**2) <= 1.:
                pos_in_image.append([star['y'], star['z']])
            else:
                pos_in_image.append([np.nan, np.nan])

        pos_in_image = np.array(pos_in_image)
        pos_in_image += self.scope_noise * self.rng.standard_normal(pos_in_image.shape)

        return pos_in_image.flatten()

    def get_state(self):
        # orientation and angular velocity of spacecraft
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        rpy = self.bullet_client.getEulerFromQuaternion(ori)
        vel = self.bullet_client.getBaseVelocity(self.robot_id)

        # angular velocity of each reaction wheel
        joint_states = self.bullet_client.getJointStates(self.robot_id, self.joint_ids)
        v = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            v[i] = joint_states[i][1]

        return rpy, vel[1], v

    def set_actuator_commands(
            self,
            torque_1_command,
            torque_2_command,
            torque_3_command,
            torque_4_command,
        ):
        if not np.isscalar(torque_1_command):
            raise Exception('torque_1_command must be a scalar')
        if not np.isscalar(torque_2_command):
            raise Exception('torque_2_command must be a scalar')
        if not np.isscalar(torque_3_command):
            raise Exception('torque_3_command must be a scalar')
        if not np.isscalar(torque_4_command):
            raise Exception('torque_4_command must be a scalar')
        
        torque_1 = np.clip(torque_1_command, -self.tau_max, self.tau_max)
        torque_2 = np.clip(torque_2_command, -self.tau_max, self.tau_max)
        torque_3 = np.clip(torque_3_command, -self.tau_max, self.tau_max)
        torque_4 = np.clip(torque_4_command, -self.tau_max, self.tau_max)
        self.set_joint_torque(
            np.array([
                torque_1,
                torque_2,
                torque_3,
                torque_4,
            ])
        )
        return torque_1, torque_2, torque_3, torque_4
    
    def set_joint_torque(self, tau):
        if tau.shape[0] != self.num_joints:
            raise Exception('tau must be same length as number of joints')
        zero_gains = tau.shape[0] * (0.,)
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains,
        )

    def place_shootingstar(self):
        if not self.shootingstar:
            self.bullet_client.resetBasePositionAndOrientation(
                self.shot_id,
                [0., 0., 2 * self.star_depth],
                self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            )
            self.bullet_client.resetBaseVelocity(
                self.shot_id,
                linearVelocity=[0., 0., 0.],
                angularVelocity=[0., 0., 0.],
            )
            return

        # Choose the position and velocity of shooting star so that
        # it will hit one of the reaction wheels
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        i = self.rng.choice([0, 1, 2, 3])
        p = np.array(link_states[i][0])
        p /= np.linalg.norm(p)
        while True:
            q = self.rng.standard_normal(size=(3))
            q /= np.linalg.norm(q)
            if np.linalg.norm(p - q) > 0.5:
                break
        v = np.cross(p, q)
        pos = (2.2 * p) - (10. * v)
        self.bullet_client.resetBasePositionAndOrientation(self.shot_id,
                                          pos,
                                          self.bullet_client.getQuaternionFromEuler([0., 0., 0.]))
        self.bullet_client.resetBaseVelocity(self.shot_id,
                            linearVelocity=4. * v,
                            angularVelocity=[0., 0., 0.])

    def has_docked(self):
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.cat_id)
        return np.linalg.norm(pos) <= self.docking_radius

    def reset(
            self,
            initial_conditions=None,
            scope_noise=None,
            space_debris=True,
            docking_time=30.,
        ):
        # Scope noise (if specified)
        if scope_noise is not None:
            self.scope_noise = scope_noise

        # Reaction wheels
        q = np.zeros(self.num_joints)
        v = np.zeros(self.num_joints)
        for i, joint_id in enumerate(self.joint_ids):
            self.bullet_client.resetJointState(self.robot_id, joint_id, q[i], v[i])

        # Base position, orientation, and velocity
        pos = np.array([0., 0., 0.])
        if initial_conditions is None:
            while True:
                rpy = 0.05 * self.rng.standard_normal(3)
                ori = self.bullet_client.getQuaternionFromEuler(rpy)
                self.bullet_client.resetBasePositionAndOrientation(self.robot_id, pos, ori)
                star_meas = self.get_sensor_measurements()
                if not np.isnan(star_meas).any():
                    break
            angvel = 0.05 * self.rng.standard_normal(3)
        else:
            rpy = np.array([
                initial_conditions['phi'],
                initial_conditions['theta'],
                initial_conditions['psi'],
            ])
            angvel = np.array([
                initial_conditions['w_x'],
                initial_conditions['w_y'],
                initial_conditions['w_z'],
            ])
            ori = self.bullet_client.getQuaternionFromEuler(rpy)
            self.bullet_client.resetBasePositionAndOrientation(self.robot_id, pos, ori)
            star_meas = self.get_sensor_measurements()
            if np.isnan(star_meas).any():
                raise Exception('some stars are out of view from initial orientation')
        self.bullet_client.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[0., 0., 0.],
            angularVelocity=angvel,
        )

        # Shooting star position, orientation, and velocity
        self.shootingstar = space_debris
        self.place_shootingstar()

        # Cat position, orientation, and velocity
        pos = np.array([0., 0., (self.docking_radius - self.docking_buffer) + (self.docking_speed * docking_time)])
        rpy = np.array([0., 0., 0.])
        ori = self.bullet_client.getQuaternionFromEuler(rpy)
        self.bullet_client.resetBasePositionAndOrientation(self.cat_id, pos, ori)
        self.bullet_client.resetBaseVelocity(self.cat_id,
                            linearVelocity=[0., 0., -self.docking_speed],
                            angularVelocity=[0., 0., 0.])
        
        # Update camera and display
        self._update_camera()
        self._update_display()

    def run(
            self,
            controller,
            max_time=5.0,
            data_filename=None,
            video_filename=None,
            print_debug=False
        ):

        self.data = {
            't': [],
            'psi': [],
            'theta': [],
            'phi': [],
            'w_x': [],
            'w_y': [],
            'w_z': [],
            'torque_1': [],
            'torque_2': [],
            'torque_3': [],
            'torque_4': [],
            'torque_1_command': [],
            'torque_2_command': [],
            'torque_3_command': [],
            'torque_4_command': [],
            'wheel_1_velocity': [],
            'wheel_2_velocity': [],
            'wheel_3_velocity': [],
            'wheel_4_velocity': [],
            'star_meas': [],
        }
        self.variables_to_log = getattr(controller, 'variables_to_log', [])
        for key in self.variables_to_log:
            if key in self.data.keys():
                raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
            self.data[key] = []

        # Always start from zero time
        self.t = 0.
        self.time_step = 0
        self.max_time_steps = 1 + int(max_time / self.dt)
        self.start_time = time.time()

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
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

            if self.display_meshcat or self.display_pybullet:
                self._update_camera()
            
            if self.display_meshcat:
                self.meshcat_update()

            if video_filename is not None:
                if self.time_step % 25 == 0:
                    if print_debug:
                        print(f' {self.time_step} / {self.max_time_steps}')

                # Add frame to video
                rgba = self.snapshot()
                w.append_data(rgba)

            if all_done:
                break

            if (self.max_time_steps is not None) and (self.time_step == self.max_time_steps):
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
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')

        # convert lists to numpy arrays
        data = self.data.copy()
        for key in data.keys():
            data[key] = np.array(data[key])

        return data

    def step(self, controller):
        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        star_meas = self.get_sensor_measurements()

        # Stop if any star is out of view
        if np.isnan(star_meas).any():
            return True

        # Get the state
        rpy, angvel, v = self.get_state()

        # Stop if any wheel exceeds maximum velocity
        if (np.abs(v) > self.v_max).any():
            return True

        # Get torque commands (run the controller)
        (
            torque_1_command,
            torque_2_command,
            torque_3_command,
            torque_4_command,
         ) = controller.run(self.t, star_meas)

        # Apply the torque commands
        (
            torque_1,
            torque_2,
            torque_3,
            torque_4,
        ) = self.set_actuator_commands(
            torque_1_command,
            torque_2_command,
            torque_3_command,
            torque_4_command,
        )

        # Apply external force to keep spacecraft position fixed
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        vel = self.bullet_client.getBaseVelocity(self.robot_id)
        f = - 150. * np.array(pos) - 50. * np.array(vel[0])
        self.bullet_client.applyExternalForce(
            self.robot_id,
            -1,
            f,
            pos,
            self.bullet_client.WORLD_FRAME,
        )

        # Reset shooting star
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.shot_id)
        if np.linalg.norm(np.array(pos)) > 15:
            self.place_shootingstar()

        # Log data
        self.data['t'].append(self.t)
        self.data['psi'].append(rpy[2])
        self.data['theta'].append(rpy[1])
        self.data['phi'].append(rpy[0])
        self.data['w_x'].append(angvel[0])
        self.data['w_y'].append(angvel[1])
        self.data['w_z'].append(angvel[2])
        self.data['torque_1'].append(torque_1)
        self.data['torque_2'].append(torque_2)
        self.data['torque_3'].append(torque_3)
        self.data['torque_4'].append(torque_4)
        self.data['torque_1_command'].append(torque_1_command)
        self.data['torque_2_command'].append(torque_2_command)
        self.data['torque_3_command'].append(torque_3_command)
        self.data['torque_4_command'].append(torque_4_command)
        self.data['wheel_1_velocity'].append(v[0])
        self.data['wheel_2_velocity'].append(v[1])
        self.data['wheel_3_velocity'].append(v[2])
        self.data['wheel_4_velocity'].append(v[3])
        self.data['star_meas'].append(star_meas)
        for key in self.variables_to_log:
            val = getattr(controller, key, np.nan)
            if not np.isscalar(val):
                val = val.flatten().tolist()
            self.data[key].append(val)

        # Try to stay real-time
        if self.display_pybullet or self.display_meshcat:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # Take a simulation step
        self.bullet_client.stepSimulation()

        # Increment time step
        self.time_step += 1

        return False

    def pybullet_snapshot(self):
        # scope view
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        p_eye = o_body_in_world
        p_target = (o_body_in_world + R_body_in_world @ np.array([[10.0], [0.], [0.]])).flatten()
        v_up = (R_body_in_world[:, 2]).flatten()
        view_matrix = self.bullet_client.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10.0)
        im = self.bullet_client.getCameraImage(128, 128, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL, shadow=0)
        rgba_scope = im[2]

        # hack to get black background color
        depth_scope = im[3]
        for i in range(3):
            rgba_scope[:, :, i] = np.where(depth_scope >= 0.99, 0., rgba_scope[:, :, i])

        # spacecraft view
        p_eye = 1.1 * np.array([-3., -4., 4.])
        p_target = np.array([0., 0., 0.])
        v_up = np.array([0., 0., 1.])
        view_matrix = self.bullet_client.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=1.0, farVal=20.0)
        im = self.bullet_client.getCameraImage(480, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba_world = im[2]

        # add "I" to scope view
        rgba_scope[40:42, 47:80, 0] = 255
        rgba_scope[40:42, 47:80, 1] = 255
        rgba_scope[40:42, 47:80, 2] = 255
        rgba_scope[87:89, 47:80, 0] = 255
        rgba_scope[87:89, 47:80, 1] = 255
        rgba_scope[87:89, 47:80, 2] = 255
        rgba_scope[41:87, 63:65, 0] = 255
        rgba_scope[41:87, 63:65, 1] = 255
        rgba_scope[41:87, 63:65, 2] = 255

        # hack to get black background color
        depth_world = im[3]
        for i in range(3):
            rgba_world[:, :, i] = np.where(depth_world >= 0.99, 0., rgba_world[:, :, i])

        # put scope view inside spacecraft view (picture-in-picture)
        rgba_world[10:138, 10:138, :] = rgba_scope

        return rgba_world
    
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
            return self.pybullet_snapshot()
    
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

    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Add robot
        self.robot_links = []
        for s in self.bullet_client.getVisualShapeData(self.robot_id):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])

            # It is only possible to ask pybullet for the name of non-base links
            is_base = (link_id == -1)
            if is_base:
                link_name = 'base'
            else:
                joint_info = self.bullet_client.getJointInfo(self.robot_id, link_id)
                link_name = joint_info[12].decode('UTF-8')

            self.robot_links.append({
                'name': link_name,
                'id': link_id,
                'scale': scale,
                'is_base': is_base,
            })

            self.vis['robot'][link_name].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )
        
        # Add cat
        s = self.bullet_client.getVisualShapeData(self.cat_id)[0]
        if s[1] != -1:
            raise Exception('bad cat link id')
        stl_filename = s[4].decode('UTF-8')
        self.cat_scale = s[3]
        color = self._convert_color(s[7])
        self.vis['cat'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )
        
        # Add stars
        for star in self.stars:
            s = self.bullet_client.getVisualShapeData(star['id'])[0]
            if s[1] != -1:
                raise Exception('bad star link id')
            radius = s[3][0]
            color = self._convert_color(s[7])
            star['name'] = f'{star["id"]}'
            star['radius'] = radius
            self.vis['stars'][star['name']].set_object(
                meshcat.geometry.Sphere(radius),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )

        # Add shooting star
        s = self.bullet_client.getVisualShapeData(self.shot_id)[0]
        if s[1] != -1:
            raise Exception('bad shooting star link id')
        radius = s[3][0]
        color = self._convert_color(s[7])
        self.vis['shooting_star'].set_object(
            meshcat.geometry.Sphere(radius),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )

        # Add sky
        self.vis['sky'].set_object(
            meshcat.geometry.Sphere(self.star_depth),
            meshcat.geometry.MeshPhongMaterial(
                transparent=0.,
                opacity=1.0,
                reflectivity=0.2,
                map=meshcat.geometry.ImageTexture(
                    image=meshcat.geometry.PngImage.from_file(str(Path('./urdf/sky.png'))),
                    wrap=[1001, 1001],
                    repeat=[1, 1],
                ),
            )
        )
        
        # Turn off grid
        self.vis['/Grid'].set_property('visible', False)

        # Set background color
        self.vis['/Background'].set_property('top_color', [0, 0, 0])
        self.vis['/Background'].set_property('bottom_color', [0, 0, 0])

        # Add lights
        self.meshcat_lights()
        
    def meshcat_update(self):

        #
        # TODO   Use getLinkStates instead of getLinkState to make
        #        this function more efficient.
        #

        if self.view == 'scopeview':
            pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
            T_ori = meshcat.transformations.quaternion_matrix(np.roll(ori, 1))
            T_flip = meshcat.transformations.rotation_matrix(np.pi, [0., 0., 1.])
            T_cam_in_world = T_ori @ T_flip
            T_world_in_cam = meshcat.transformations.inverse_matrix(T_cam_in_world)
        else:
            T_world_in_cam = meshcat.transformations.identity_matrix()
        
        # Set pose of robot links
        for link in self.robot_links:
            if link['is_base']:
                pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
            else:
                link_state = self.bullet_client.getLinkState(self.robot_id, link['id'])
                pos = link_state[4]
                ori = link_state[5]
            
            S = np.diag(np.concatenate((link['scale'], [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['robot'][link['name']].set_transform(T_world_in_cam @ T @ S)

        # Set pose of stars
        for star in self.stars:
            self.vis['stars'][star['name']].set_transform(
                T_world_in_cam @ meshcat.transformations.translation_matrix(star['pos'].flatten()),
            )
        
        # Set pose of shooting star
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.shot_id)
        self.vis['shooting_star'].set_transform(
            T_world_in_cam @ meshcat.transformations.translation_matrix(pos),
        )

        # Set pose of cat
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.cat_id)
        S = np.diag(np.concatenate((self.cat_scale, [1.0])))
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['cat'].set_transform(T_world_in_cam @ T @ S)

        # Set pose of sky
        self.vis['sky'].set_transform(T_world_in_cam)

    def meshcat_camera(self):
        self.vis['/Cameras/default'].set_transform(
            meshcat.transformations.compose_matrix(
                angles=[
                    0.,
                    np.deg2rad(-self.camera_pitch),
                    np.deg2rad(self.camera_yaw - 180),
                ],
                translate=[0., 0., 0.],
            )
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'position', [self.camera_distance, 0., 0.],
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'fov', 90,
        )
    
    def _update_display(self):
        if self.display_pybullet:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = self.bullet_client.getKeyboardEvents()
        
        if self.display_meshcat:
            self.meshcat_update()

    def _update_camera(self):
        if self.display_pybullet or self.display_meshcat:
            if self.view == 'scopeview':
                pass
            elif self.view == 'sideview':
                pass
            elif self.view == 'catview':
                if self.display_pybullet:
                    print('WARNING: pybullet display does not support catview')
                else:
                    pos, ori = self.bullet_client.getBasePositionAndOrientation(self.cat_id)
                    x, y, z = pos
                    T = meshcat.transformations.identity_matrix()
                    self.vis['/Cameras/default'].set_transform(T)
                    self.vis['/Cameras/default/rotated/<object>'].set_property(
                        'fov', 90,
                    )
                    self.vis.set_cam_pos([x, y, z + 0.5])
                    self.vis.set_cam_target([x, y, z])
            else:
                raise Exception('invalid camera view')
    
    def camera_sideview(self):
        self.view = 'sideview'
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        if self.display_pybullet:
            self.bullet_client.resetDebugVisualizerCamera(6, -30, -40, (0., 0., 0.))
        
        if self.display_meshcat:
            self.camera_distance = 5.
            self.camera_yaw = 60.
            self.camera_pitch = 30.
            self.camera_target = [0., 0., 0.]
            self.meshcat_camera()
        
        self._update_display()
    
    def camera_scopeview(self):
        self.view = 'scopeview'
        if not (self.display_meshcat or self.display_pybullet):
            return

        if self.display_meshcat:
            T = meshcat.transformations.identity_matrix()
            self.vis['/Cameras/default'].set_transform(T)
            self.vis['/Cameras/default/rotated/<object>'].set_property(
                'position', [1e-8, 0., 0.],
            )
            self.vis['/Cameras/default/rotated/<object>'].set_property(
                'fov', 45,
            )
        
        if self.display_pybullet:
            print('WARNING: pybullet display does not support scopeview')
        
        self._update_display()
    
    def camera_catview(self):
        self.view = 'catview'
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        self._update_camera()
        self._update_display()
