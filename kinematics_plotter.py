import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

def get_rotation_z(theta):
    """Returns a 4x4 rotation matrix around the Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

def get_translation_z(d):
    """Returns a 4x4 translation matrix along the Z-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

def get_translation_x(a):
    """Returns a 4x4 translation matrix along the X-axis."""
    return np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def get_rotation_x(alpha):
    """Returns a 4x4 rotation matrix around the X-axis."""
    return np.array([
        [1, 0,             0,              0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha),  np.cos(alpha), 0],
        [0, 0,             0,              1]
    ])

def create_transformation_matrix(translation, rotation_matrix):
    """
    Creates a 4x4 transformation matrix from a translation vector and a rotation matrix.
    """
    T = np.identity(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T

class RobotArm:
    def __init__(self, config_file):
        """
        Initializes the RobotArm from a YAML configuration file.
        """
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.joints = config['joints']
        self.name = config.get('robot_name', 'Robot Arm')

    def forward_kinematics(self, joint_angles):
        """
        Computes the forward kinematics for the robot arm.
        
        :param joint_angles: A list or array of angles for each revolute joint.
        :return: A list of transformation matrices, one for each joint frame relative to the base.
        """
        if len(joint_angles) != len(self.joints):
            raise ValueError("Number of joint angles must match the number of joints.")

        # Start with the base frame (identity matrix)
        T_cumulative = np.identity(4)
        frame_transforms = [T_cumulative]

        for i, joint in enumerate(self.joints):
            # 1. Get the static transform from parent to joint
            origin = joint['origin']
            xyz = origin['xyz']
            rpy = origin['rpy']
            
            # Static rotation from RPY
            R_static = Rotation.from_euler('xyz', rpy).as_matrix()
            T_static = create_transformation_matrix(xyz, R_static)

            # 2. Get the dynamic transform from the joint rotation
            axis = joint['axis']['xyz']
            angle = joint_angles[i]
            
            # Dynamic rotation from axis-angle
            R_dynamic = Rotation.from_rotvec(np.array(axis) * angle).as_matrix()
            T_dynamic = create_transformation_matrix([0, 0, 0], R_dynamic)
            
            # 3. Chain the transforms: T_world_child = T_world_parent * T_parent_child
            # The transform to this joint's frame is T_static followed by T_dynamic
            T_cumulative = T_cumulative @ T_static @ T_dynamic
            frame_transforms.append(T_cumulative)
            
        return frame_transforms

    def plot(self, joint_angles):
        """
        Calculates forward kinematics and plots the robot arm in 3D.
        """
        frame_transforms = self.forward_kinematics(joint_angles)
        
        # Extract the origin of each frame (the translation part of the matrix)
        joint_positions = [T[:3, 3] for T in frame_transforms]
        
        # Unzip the coordinates for plotting
        x_coords = [p[0] for p in joint_positions]
        y_coords = [p[1] for p in joint_positions]
        z_coords = [p[2] for p in joint_positions]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot links
        ax.plot(x_coords, y_coords, z_coords, 'b-', marker='o', label='Robot Links')
        
        # Plot joints
        ax.scatter(x_coords, y_coords, z_coords, c='r', s=100, label='Joints')
        
        # Set plot labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title(f'Kinematics for {self.name}')
        
        # Set aspect ratio to be equal
        max_range = np.array([x_coords, y_coords, z_coords]).max()
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
        ax.set_aspect('equal', adjustable='box')

        plt.legend()
        plt.show()

    def animate_movement(self, initial_angles, final_angles, frames=200, interval=50):
        """
        Animates the robot's movement from an initial to a final joint configuration.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Interpolate joint angles
        joint_angles_over_time = np.linspace(initial_angles, final_angles, frames)

        # Calculate initial positions to set plot limits
        initial_transforms = self.forward_kinematics(initial_angles)
        initial_positions = [T[:3, 3] for T in initial_transforms]
        x_coords = [p[0] for p in initial_positions]
        y_coords = [p[1] for p in initial_positions]
        z_coords = [p[2] for p in initial_positions]

        # Set plot limits based on a rough estimate of the workspace
        max_range = np.sum([np.linalg.norm(joint['origin']['xyz']) for joint in self.joints]) * 1.1
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title(f'Kinematics for {self.name}')
        ax.set_aspect('equal', adjustable='box')

        # Initialize plot elements
        links, = ax.plot(x_coords, y_coords, z_coords, 'b-', marker='o', label='Robot Links')
        joints = ax.scatter(x_coords, y_coords, z_coords, c='r', s=100, label='Joints')
        end_effector_trace, = ax.plot([], [], [], 'g--', label='End-Effector Path')
        
        end_effector_positions = []

        def update(frame):
            # Get joint angles for the current frame
            current_angles = joint_angles_over_time[frame]
            
            # Compute forward kinematics
            frame_transforms = self.forward_kinematics(current_angles)
            joint_positions = [T[:3, 3] for T in frame_transforms]
            
            # Update link and joint positions
            x_coords = [p[0] for p in joint_positions]
            y_coords = [p[1] for p in joint_positions]
            z_coords = [p[2] for p in joint_positions]
            
            links.set_data_3d(x_coords, y_coords, z_coords)
            joints._offsets3d = (x_coords, y_coords, z_coords)

            # Update end-effector trace
            end_effector_positions.append(joint_positions[-1])
            trace_x = [p[0] for p in end_effector_positions]
            trace_y = [p[1] for p in end_effector_positions]
            trace_z = [p[2] for p in end_effector_positions]
            end_effector_trace.set_data_3d(trace_x, trace_y, trace_z)

            return links, joints, end_effector_trace

        ani = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False)
        plt.legend()
        plt.show()