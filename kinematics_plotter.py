import yaml
import numpy as np
import matplotlib.pyplot as plt
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

def dh_transformation_matrix(a, alpha, d, theta):
    """
    Computes the Denavit-Hartenberg transformation matrix for a single joint.
    The transformation is from frame i-1 to frame i.
    T = Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)
    """
    # Note: The order of multiplication is important.
    # We are using post-multiplication, so the order is as written.
    rot_z = get_rotation_z(theta)
    trans_z = get_translation_z(d)
    trans_x = get_translation_x(a)
    rot_x = get_rotation_x(alpha)
    
    T = rot_z @ trans_z @ trans_x @ rot_x
    return T

class RobotArm:
    def __init__(self, config_file):
        """
        Initializes the RobotArm from a YAML configuration file.
        """
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.dh_params = config['dh_parameters']
        self.name = config.get('robot_name', 'Robot Arm')

    def forward_kinematics(self, joint_angles):
        """
        Computes the forward kinematics for the robot arm.
        
        :param joint_angles: A list or array of angles for each revolute joint.
        :return: A list of transformation matrices, one for each joint frame relative to the base.
        """
        if len(joint_angles) != len(self.dh_params):
            raise ValueError("Number of joint angles must match the number of DH parameters.")

        # Start with the base frame (identity matrix)
        T_current = np.identity(4)
        frame_transforms = [T_current]

        for i, params in enumerate(self.dh_params):
            a = params['a']
            alpha = params['alpha']
            d = params['d']
            # Use the provided joint angle for theta
            theta = joint_angles[i]

            # Get the transformation from the previous frame to the current one
            T_link = dh_transformation_matrix(a, alpha, d, theta)
            
            # Chain the transformation: T_0_i = T_0_{i-1} * T_{i-1}_i
            T_current = T_current @ T_link
            frame_transforms.append(T_current)
            
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