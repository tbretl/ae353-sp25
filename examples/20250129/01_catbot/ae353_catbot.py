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


class Simulator:
    def __init__(
                self,
                display=True,
                sound=False,
                seed=None,
                maximum_wheel_torque=5.,
                number_of_cats=10,
                maximum_cat_target=2.5,
                dt=0.01,
                log_hidden_variables=False,
            ):
        
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Time step
        self.dt = dt
        
        # Other parameters
        # - Passed
        self.sound = sound
        self.maximum_wheel_torque = maximum_wheel_torque
        self.number_of_cats = number_of_cats
        self.maximum_cat_target = maximum_cat_target
        self.log_hidden_variables = log_hidden_variables
        # - Hard-coded
        self.meow = str(Path('./urdf/meow.wav'))
        self.launch_duration = 5.
        self.launch_pause = 1.
        self.wheel_radius = 0.325
        self.wheel_to_body = 0.3
        self.body_height = 0.8
        self.wheel_base = 0.7
        self.chassis_to_axle = 0.3
        self.platform_length = 10.
        self.platform_width = 2.
        self.joint_damping = 0.
        
        # Connect to and configure pybullet
        self.display_meshcat = display
        self.bullet_client = bullet_client.BulletClient(
            connection_mode=pybullet.DIRECT,
        )
        self.bullet_client.setGravity(0, 0, -9.81)
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )
        
        # Load platform
        self.platform_id = self.bullet_client.loadURDF(
            str(Path('./urdf/platform.urdf')),
            basePosition=np.array([0., 0., -0.1]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE),
            useFixedBase=1,
        )

        # Load cats
        self.current_cat = -1
        self.launch_start = int(np.ceil(self.launch_pause / self.dt))
        self.launch_interval = int(np.ceil((self.launch_duration + self.launch_pause) / self.dt))
        self.cat_id = []
        for i in range(self.number_of_cats):
            self.cat_id.append(self.bullet_client.loadURDF(
                str(Path('./urdf/cat-pilot.urdf')),
                basePosition=np.array([0., 0., -96.5]),
                baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
                flags=(self.bullet_client.URDF_USE_INERTIA_FROM_FILE)))
        self.reset_launches()

        # Load robot
        self.robot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/catbot.urdf')),
            basePosition=np.array([0., 0., self.wheel_radius + self.chassis_to_axle]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                   self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ))
        
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
            'chassis_to_left_wheel',
            'chassis_to_right_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            self.bullet_client.changeDynamics(self.robot_id, id, jointDamping=self.joint_damping)

        # Set contact and damping parameters
        for object_id in [self.robot_id, self.platform_id] + self.cat_id:
            for joint_id in range(-1, self.bullet_client.getNumJoints(object_id)):
                self.bullet_client.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=1.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1, #1000., #-1,
                    contactStiffness=-1, #30000., #-1,
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

        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()

        # Set default camera view
        self.camera_sideview()

    
    def get_sensor_measurements(self):
        """
        Measurements used by student controllers are:

            wheel_position
            wheel_velocity
            pitch_angle
            pitch_rate
        
        Hidden measurements used only internally are:

            lateral_error
            heading error
            turning rate

        Everything is computed assuming both wheels roll without slipping.
        """

        # Position of each wheel (center)
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        wheel_position = pc[0]

        # Velocity of each wheel
        joint_states = self.bullet_client.getJointStates(self.robot_id, self.joint_ids)
        q = np.zeros([self.num_joints])
        v = np.zeros_like(q)
        for i in range(self.num_joints):
            q[i] = joint_states[i][0]
            v[i] = joint_states[i][1]
        vl = v[0] * self.wheel_radius
        vr = v[1] * self.wheel_radius
        wheel_velocity = (vr + vl) / 2.0

        # Lateral error (positive when located too far to left)
        lateral_error = pc[1]

        # Heading error (positive when turned too far to left)
        # - vector from left wheel to right wheel
        a_in0 = pr - pl
        # - heading error
        heading_error = np.arctan2(a_in0[0], -a_in0[1])

        # Turning rate (positive when turning to the left)
        turning_rate = (vr - vl) / np.linalg.norm(pr - pl)

        # Position, orientation, and angular velocity of chassis
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        vel = self.bullet_client.getBaseVelocity(self.robot_id)
        rpy = self.bullet_client.getEulerFromQuaternion(ori)
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        w_in_world = np.reshape(np.array(vel[1]), (3, 1))
        w_in_body = R_body_in_world.T @ w_in_world
        
        # Pitch angle and pitch rate
        pitch_angle = rpy[1]
        pitch_rate = w_in_body[1, 0]

        return (
            wheel_position,
            wheel_velocity,
            pitch_angle,
            pitch_rate,
            lateral_error,
            heading_error,
            turning_rate,
        )
    
    def set_actuator_commands(
                self,
                right_wheel_torque_command,
                left_wheel_torque_command,
            ):
        
        if not np.isscalar(right_wheel_torque_command):
            raise Exception('right_wheel_torque must be a scalar')
        
        if not np.isscalar(left_wheel_torque_command):
            raise Exception('left_wheel_torque must be a scalar')

        self.set_joint_torque(np.array([left_wheel_torque_command, right_wheel_torque_command]))
    
    def set_joint_torque(self, tau):
        if tau.shape[0] != self.num_joints:
            raise Exception('tau must be the same length as the number of joints')
        zero_gains = tau.shape[0] * (0.,)
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains,
        )

    def reset(
            self,
            initial_wheel_position=0.,
            initial_wheel_velocity=0.,
            initial_pitch_angle=0.,
            initial_pitch_rate=0.,
        ):
        
        # Compute initial conditions
        r_w = self.wheel_radius
        r_b = self.chassis_to_axle
        p_b = [
            r_b * np.sin(initial_pitch_angle) + initial_wheel_position,
            r_b * np.cos(initial_pitch_angle) + r_w,
        ]
        v_b = [
            r_b * np.cos(initial_pitch_angle) * initial_pitch_rate + initial_wheel_velocity,
            -r_b * np.sin(initial_pitch_angle) * initial_pitch_rate,
        ]
        omega_w = initial_wheel_velocity / r_w
        omega_b = initial_pitch_rate

        # Apply initial conditions to robot
        # - position and orientation of chassis
        pos = [p_b[0], 0., p_b[1]]
        ori = [0., initial_pitch_angle, 0.]
        self.bullet_client.resetBasePositionAndOrientation(
            self.robot_id,
            pos,
            self.bullet_client.getQuaternionFromEuler(ori)
        )
        # - joint angles and velocities
        for joint_id in self.joint_ids:
            self.bullet_client.resetJointState(self.robot_id, joint_id, -initial_pitch_angle, omega_w)
        # - linear and angular velocity
        self.bullet_client.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[v_b[0], 0., v_b[1]],
            angularVelocity=[0., omega_b, 0.],
        )

        # Choose targets and place cats
        self.reset_launches()

        # Update display
        self._update_display()
    
    def get_number_of_cats_saved(self):
        x_safe = 0.5 * self.platform_length
        y_safe = 0.5 * self.platform_width
        z_safe = self.wheel_radius + \
                 self.wheel_to_body + \
                 0.5 * self.body_height  # <-- requires the robot to be upright
        number_of_cats_saved = 0
        for cat_id in self.cat_id:
            pos, ori = self.bullet_client.getBasePositionAndOrientation(cat_id)
            is_x_safe = np.abs(pos[0]) < x_safe
            is_y_safe = np.abs(pos[1]) < y_safe
            is_z_safe = pos[2] > z_safe
            if is_x_safe and is_y_safe and is_z_safe:
                number_of_cats_saved += 1
        return number_of_cats_saved
    
    def get_launch(self, i, x, max_iters=50, min_sep=0.3):
        for j in range(max_iters):
            x1 = self.rng.uniform(-self.maximum_cat_target, self.maximum_cat_target)
            xdot0 = 5. if x1 < 0. else -5.
            z1 = 1.2
            zdot1 = -5.
            x0, z0, zdot0 = self.get_flying_cat(self.launch_duration, x1, z1, xdot0, zdot1)
            is_collision = False
            for x_i in x:
                if np.abs(x_i - x0) < min_sep:
                    is_collision = True
            if not is_collision:
                return {
                    'time_step': self.launch_start + (i * self.launch_interval),
                    'x1': x1,
                    'z1': z1,
                    'x0': x0,
                    'z0': z0,
                    'xdot0': xdot0,
                    'zdot0': zdot0,
                }
        return None

    def get_launches(self):
        x = []
        launches = []
        for i in range(self.number_of_cats):
            launch = self.get_launch(i, x)
            if launch is None:
                return None
            launches.append(launch)
            x.append(launch['x0'])
        return launches

    def reset_launches(self, max_iters=20):
        x = []
        for j in range(max_iters):
            launches = self.get_launches()
            if launches is not None:
                break
        if launches is None:
            raise Exception('Could not place cats with enough separation. Try again.')
        self.launches = launches
        for i, launch in enumerate(self.launches):
            pos = [launch['x0'], 0., launch['z0']]
            ori = [0., 0., 0.]
            self.bullet_client.resetBasePositionAndOrientation(
                self.cat_id[i],
                pos,
                self.bullet_client.getQuaternionFromEuler(ori)
            )
        self.current_cat = -1
        self.cat_target = 0.

    def launch_cat(self, which_cat):
        launch = self.launches[which_cat]
        cat_id = self.cat_id[which_cat]
        pos = [launch['x0'], 0., launch['z0']]
        ori = [0., 0., 0.]
        self.bullet_client.resetBasePositionAndOrientation(
            cat_id,
            pos,
            self.bullet_client.getQuaternionFromEuler(ori)
        )
        self.bullet_client.resetBaseVelocity(
            cat_id,
            linearVelocity=[launch['xdot0'], 0., launch['zdot0']],
            angularVelocity=self.rng.uniform(low=-0.1, high=0.1, size=3).tolist(),
        )
        self.cat_target = launch['x1']
        if self.sound:
            playsound(self.meow, block=False)

    def get_flying_cat(self, t1, x1, z1, xdot0, zdot1, g=9.81):
        zdot0 = zdot1 + g * t1
        z0 = z1 - (zdot0 * t1 - 0.5 * g * t1**2)
        x0 = x1 - xdot0 * t1
        return x0, z0, zdot0

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
            'wheel_position': [],
            'wheel_velocity': [],
            'pitch_angle': [],
            'pitch_rate': [],
            'cat_target': [],
            'wheel_torque': [],
            'wheel_torque_command': [],
        }
        if self.log_hidden_variables:
            self.data['lateral_error'] = []
            self.data['heading_error'] = []
            self.data['turning_rate'] = []
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
            if int(1 / self.dt) % 25 != 0:
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

            if self.display_meshcat:
                self.meshcat_update()

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

    def step(self, controller):
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Launch cat (if necessary)
        if self.current_cat + 1 < self.number_of_cats:
            launch = self.launches[self.current_cat + 1]
            if self.time_step == launch['time_step']:
                self.current_cat += 1
                self.launch_cat(self.current_cat)

        # Get the sensor measurements
        (
            wheel_position,
            wheel_velocity,
            pitch_angle,
            pitch_rate,
            lateral_error,
            heading_error,
            turning_rate,
        ) = self.get_sensor_measurements()
        
        # Get wheel torque command (run the controller)
        wheel_torque_command = controller.run(
            self.t,
            wheel_position,
            wheel_velocity,
            pitch_angle,
            pitch_rate,
            self.cat_target,
        )

        # Verify wheel torque command
        if not np.isscalar(wheel_torque_command):
            raise Exception('wheel_torque must be a scalar')
        
        # Clip wheel torque command
        wheel_torque = np.clip(wheel_torque_command, -self.maximum_wheel_torque, self.maximum_wheel_torque)
        
        # Get right and left wheel torque commands
        right_wheel_torque_command = (0.5 * wheel_torque) - 5. * lateral_error - 5. * heading_error - 3. * turning_rate
        left_wheel_torque_command = (0.5 * wheel_torque) + 5. * lateral_error + 5. * heading_error + 3. * turning_rate

        # Apply the torque commands
        self.set_actuator_commands(
            right_wheel_torque_command,
            left_wheel_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['wheel_position'].append(wheel_position)
        self.data['wheel_velocity'].append(wheel_velocity)
        self.data['pitch_angle'].append(pitch_angle)
        self.data['pitch_rate'].append(pitch_rate)
        self.data['cat_target'].append(self.cat_target)
        self.data['wheel_torque'].append(wheel_torque)
        self.data['wheel_torque_command'].append(wheel_torque_command)
        if self.log_hidden_variables:
            self.data['lateral_error'].append(lateral_error)
            self.data['heading_error'].append(heading_error)
            self.data['turning_rate'].append(turning_rate)
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
            self.meshcat_update()

    def camera_catview(self):
        if not self.display_meshcat:
            return
        
        if self.number_of_cats <= 0:
            return

        which_cat = 0 if self.current_cat < 0 else self.current_cat
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.cat_id[which_cat])
        self.vis.set_cam_pos([pos[0], pos[1] - 1., pos[2]])
        self.vis.set_cam_target(pos)

        self._update_display()
    
    def camera_topview(self):
        if not self.display_meshcat:
            return
        
        self.vis.set_cam_pos([0., 0., 5.])
        self.vis.set_cam_target([0., 0., 0.])

        self._update_display()
    
    def camera_sideview(self):
        if not self.display_meshcat:
            return

        self.vis.set_cam_pos([0., -5., 1.])
        self.vis.set_cam_target([0., 0., 0.])
        
        self._update_display()
    
    def camera_wideview(self):
        if not self.display_meshcat:
            return
        
        self.vis.set_cam_pos([0., -80., -45.])
        self.vis.set_cam_target([0., 0., -45.])
        
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
    
    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Add platform
        for s in self.bullet_client.getVisualShapeData(self.platform_id):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            link_scale = s[3]
            color = self._convert_color(s[7])

            # It is only possible to ask pybullet for the name of non-base links
            is_base = (link_id == -1)
            if is_base:
                link_name = 'base'
                pos, ori = self.bullet_client.getBasePositionAndOrientation(self.platform_id)
            else:
                joint_info = self.bullet_client.getJointInfo(self.platform_id, link_id)
                link_name = joint_info[12].decode('UTF-8')
                link_state = self.bullet_client.getLinkState(self.platform_id, link_id)
                pos = link_state[4]
                ori = link_state[5]
            
            self.vis['platform'][link_name].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )

            S = np.diag(np.concatenate((link_scale, [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['platform'][link_name].set_transform(T @ S)
        
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
        for cat_id in self.cat_id:
            s = self.bullet_client.getVisualShapeData(cat_id)[0]
            if s[1] != -1:
                raise Exception('bad cat link id')
            stl_filename = s[4].decode('UTF-8')
            self.cat_scale = s[3]
            color = self._convert_color(s[7])
            self.vis['cat'][f'{cat_id}'].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )

        # Add cat target
        color = self._convert_color([125 / 255, 62 / 255, 19 / 255, 1.])
        self.vis['cat_target'].set_object(
            meshcat.geometry.Sphere(0.1),
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

        # Enable things to be seen from farther away
        self.vis['/Cameras/default/rotated/<object>'].set_property('far', 1000)

        
    def meshcat_update(self):

        #
        # TODO   Use getLinkStates instead of getLinkState to make
        #        this function more efficient.
        #

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
            # self.vis['robot'][link['name']].set_transform(T_world_in_cam @ T @ S)
            self.vis['robot'][link['name']].set_transform(T @ S)
        
        # Set pose of cat
        for cat_id in self.cat_id:
            pos, ori = self.bullet_client.getBasePositionAndOrientation(cat_id)
            S = np.diag(np.concatenate((self.cat_scale, [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['cat'][f'{cat_id}'].set_transform(T @ S)
        
        # Set pose of cat target
        self.vis['cat_target'].set_transform(
            meshcat.transformations.translation_matrix([self.cat_target, 0., 0.]),
        )
