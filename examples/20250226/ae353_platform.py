import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time
import json
import importlib
import meshcat
from pathlib import Path

class Simulator:
    def __init__(
                self,
                display=True,
                display_pybullet=False,
                seed=None,
                width=640,
                height=480,
                roll=0.,
                damping=0.,
                tau_max=5.,
            ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = 0.01

        # Other parameters
        self.roll = roll
        self.damping = damping
        self.tau_max = tau_max

        # Connect to and configure pybullet
        self.display_pybullet = display_pybullet
        self.display_meshcat = display
        if self.display_pybullet:
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI,
                options=f'--width={width} --height={height}',
            )
            self.bullet_client.configureDebugVisualizer(
                self.bullet_client.COV_ENABLE_GUI, 0,
                lightPosition=[10., 10., 10.],
            )
        else:
            self.bullet_client = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
        self.bullet_client.setGravity(0, 0, -9.81)
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load plane
        self.plane_id = self.bullet_client.loadURDF(
            str(Path('./urdf/plane.urdf')),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., self.roll, 0.]),
            useFixedBase=1,
        )
        
        # Load platform
        self.robot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/platform.urdf')),
            flags=(
                    self.bullet_client.URDF_USE_IMPLICIT_CYLINDER |
                    self.bullet_client.URDF_USE_INERTIA_FROM_FILE
            ),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., self.roll, 0.]),
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
            'base_to_platform',
            'connector_to_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            self.bullet_client.changeDynamics(self.robot_id, id, jointDamping=damping)

        # Disable velocity control on each joint so we can use torque control
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints)
        )

        # Eliminate linear and angular damping (a poor model of drag)
        self.bullet_client.changeDynamics(self.robot_id, -1, linearDamping=0., angularDamping=0.)
        for joint_id in range(self.bullet_client.getNumJoints(self.robot_id)):
            self.bullet_client.changeDynamics(self.robot_id, joint_id, linearDamping=0., angularDamping=0.)
        
        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()

        # Camera view
        self.camera_sideview()
    
    def set_roll(self, roll):
        self.roll = roll
        self.bullet_client.resetBasePositionAndOrientation(
            self.plane_id,
            np.array([0., 0., 0.]),
            self.bullet_client.getQuaternionFromEuler([0., self.roll, 0.]),
        )
        self.bullet_client.resetBasePositionAndOrientation(
            self.robot_id,
            np.array([0., 0., 0.]),
            self.bullet_client.getQuaternionFromEuler([0., self.roll, 0.]),
        )
        self.reset()

    def get_sensor_measurements(self):
        joint_states = self.bullet_client.getJointStates(self.robot_id, self.joint_ids)
        platform_angle = joint_states[0][0]
        platform_velocity = joint_states[0][1]
        wheel_angle = joint_states[1][0]
        wheel_velocity = joint_states[1][1]
        return platform_angle, platform_velocity, wheel_angle, wheel_velocity

    def set_actuator_commands(self, wheel_torque_command):
        assert(np.isscalar(wheel_torque_command))
        wheel_torque = np.clip(wheel_torque_command, -self.tau_max, self.tau_max)
        self.set_joint_torque(np.array([0., wheel_torque]))
        return wheel_torque
    
    def set_joint_torque(self, tau):
        assert(tau.shape[0] == self.num_joints)
        zero_gains = tau.shape[0] * (0.,)
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains
        )
    
    def reset(
            self,
            platform_angle=0.,
            platform_velocity=0.,
            wheel_angle=0.,
            wheel_velocity=0.,
        ):
        
        self.bullet_client.resetJointState(
            self.robot_id,
            self.joint_map['base_to_platform'],
            platform_angle,
            platform_velocity,
        )

        self.bullet_client.resetJointState(
            self.robot_id,
            self.joint_map['connector_to_wheel'],
            wheel_angle,
            wheel_velocity,
        )

        self.update_display()

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
            'platform_angle': [],
            'platform_velocity': [],
            'wheel_angle': [],
            'wheel_velocity': [],
            'wheel_torque': [],
            'wheel_torque_command': [],
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
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        platform_angle, platform_velocity, wheel_angle, wheel_velocity = self.get_sensor_measurements()

        # Get the torque command (run the controller)
        wheel_torque_command = controller.run(
            self.t,
            platform_angle,
            platform_velocity,
            wheel_angle,
            wheel_velocity,
        )

        # Apply the torque command
        wheel_torque = self.set_actuator_commands(
            wheel_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['platform_angle'].append(platform_angle)
        self.data['platform_velocity'].append(platform_velocity)
        self.data['wheel_angle'].append(wheel_angle)
        self.data['wheel_velocity'].append(wheel_velocity)
        self.data['wheel_torque'].append(wheel_torque)
        self.data['wheel_torque_command'].append(wheel_torque_command)
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

        return all_done

    def pybullet_snapshot(self):
        pos = self.camera_target
        yaw = -90 + self.camera_yaw
        aspect = self.width / self.height
        view_matrix = self.bullet_client.computeViewMatrixFromYawPitchRoll(pos, self.camera_distance, yaw, -self.camera_pitch, 0., 2)
        projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=90, aspect=aspect, nearVal=0.01, farVal=100.0)
        im = self.bullet_client.getCameraImage(
            self.width, self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL,
            shadow=1,
            lightDirection=[10., 10., 10.],
        )
        rgba = im[2]
        return rgba
    
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

    def update_display(self):
        if self.display_pybullet:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = self.bullet_client.getKeyboardEvents()
        
        if self.display_meshcat:
            self.meshcat_update()

    def camera(self):
        if self.display_pybullet:
            self.bullet_client.resetDebugVisualizerCamera(
                self.camera_distance,
                -90 + self.camera_yaw,
                -self.camera_pitch,
                self.camera_target,
            )

        if self.display_meshcat:
            self.vis['/Cameras/default'].set_transform(
                meshcat.transformations.compose_matrix(
                    angles=[
                        0.,
                        np.deg2rad(-self.camera_pitch),
                        np.deg2rad(self.camera_yaw - 180)
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

    def camera_topview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 4.
        self.camera_pitch = np.clip(90. - np.rad2deg(self.roll), -89.9, 89.9)
        self.camera_yaw = 180.
        self.camera()
        self.update_display()

    def camera_sideview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 3.
        self.camera_pitch = 30.
        self.camera_yaw = 150.
        self.camera()
        self.update_display()
    
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

    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Add plane
        self.vis['plane'].set_object(
            meshcat.geometry.ObjMeshGeometry.from_file(str(Path('./urdf/plane.obj'))),
            meshcat.geometry.MeshPhongMaterial(
                map=meshcat.geometry.ImageTexture(
                    image=meshcat.geometry.PngImage.from_file(str(Path('./urdf/checker_blue.png'))),
                    wrap=[1, 1],
                    repeat=[1, 1],
                ),
            ),
        )

        # Set pose of plane base
        self.plane_scale = self.bullet_client.getVisualShapeData(self.plane_id)[0][3]
        pos_and_ori = self.bullet_client.getBasePositionAndOrientation(self.plane_id)
        pos = pos_and_ori[0]
        ori = pos_and_ori[1]
        S = np.diag(np.concatenate((self.plane_scale, [1.0])))
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['plane'].set_transform(T @ S)
        
        # Add platform
        self.robot_links = []
        for s in self.bullet_client.getVisualShapeData(self.robot_id):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])

            joint_info = self.bullet_client.getJointInfo(self.robot_id, link_id)
            link_name = joint_info[12].decode('UTF-8')

            self.robot_links.append({
                'name': link_name,
                'id': link_id,
                'scale': scale,
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
        
        # Set pose of everything
        self.meshcat_update()
        
    def meshcat_update(self):

        #
        # TODO   Use getLinkStates instead of getLinkState to make
        #        this function more efficient.
        #

        # Set pose of plane base
        pos_and_ori = self.bullet_client.getBasePositionAndOrientation(self.plane_id)
        pos = pos_and_ori[0]
        ori = pos_and_ori[1]
        S = np.diag(np.concatenate((self.plane_scale, [1.0])))
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['plane'].set_transform(T @ S)

        # Set pose of robot links
        for link in self.robot_links:
            link_state = self.bullet_client.getLinkState(self.robot_id, link['id'])
            pos = link_state[4]
            ori = link_state[5]
            S = np.diag(np.concatenate((link['scale'], [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['robot'][link['name']].set_transform(T @ S)
