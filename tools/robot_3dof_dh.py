import numpy as np
import yaml


class Robot3DOF_DH:
    """
    3-DOF Cylindrical Robot Arm using Denavit-Hartenberg parameters.
    
    Robot structure:
    - Base joint (q1): Revolute, rotates around vertical Z-axis
    - Second joint (q2): Revolute, rotates around vertical axis at height L1
    - Prismatic joint (q3): Extends vertically upward
    
    DH parameters:
    - Joint 1: theta=q1*, d=L1, a=0, alpha=90°
    - Joint 2: theta=q2*, d=0, a=0, alpha=-90°
    - Joint 3: theta=0°, d=q3*, a=0, alpha=0°
    
    End-effector position:
    - x = q3 * sin(q2) * cos(q1)
    - y = q3 * sin(q2) * sin(q1)
    - z = L1 + q3 * cos(q2)
    """
    
    def __init__(self, config_file=None, L1=1.0):
        """
        Initialize the 3DOF robot.
        
        :param config_file: Optional YAML configuration file with DH parameters
        :param L1: Length parameter L1 (default: 1.0)
        """
        if config_file:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.name = config.get('robot_name', '3DOF DH Robot')
            self.L1 = config.get('L1', L1)
            self.dh_params = config.get('dh_parameters', [])
        else:
            self.name = '3DOF DH Robot'
            self.L1 = L1
            self.dh_params = [
                {'joint_type': 'revolute', 'theta_offset': 0, 'd': L1, 'a': 0, 'alpha': np.pi/2},
                {'joint_type': 'revolute', 'theta_offset': 0, 'd': 0, 'a': 0, 'alpha': -np.pi/2},
                {'joint_type': 'prismatic', 'theta': 0, 'd_offset': 0, 'a': 0, 'alpha': 0}
            ]
    
    def dh_transform(self, theta, d, a, alpha):
        """
        Compute the DH transformation matrix.
        
        :param theta: Joint angle (rotation about z-axis)
        :param d: Link offset (translation along z-axis)
        :param a: Link length (translation along x-axis)
        :param alpha: Link twist (rotation about x-axis)
        :return: 4x4 transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_values):
        """
        Compute forward kinematics using DH parameters.
        
        The robot structure:
        - Base at origin
        - Joint 1 at height L1, rotates around Z
        - Joint 2 at same position as Joint 1, rotates around Z
        - Joint 3 extends vertically from Joint 2
        
        :param joint_values: [q1, q2, q3] where q1, q2 are angles (radians) and q3 is distance
        :return: List of transformation matrices from base to each joint frame
        """
        if len(joint_values) != 3:
            raise ValueError("Expected 3 joint values [q1, q2, q3]")
        
        q1, q2, q3 = joint_values
        
        # Base frame at origin
        T_base = np.eye(4)
        
        # Joint 1: revolute with theta=q1, d=L1, a=0, alpha=90°
        # This lifts the frame to height L1 and rotates around Z by q1
        T_0_1 = self.dh_transform(
            theta=q1,
            d=self.L1,
            a=0,
            alpha=np.pi/2
        )
        
        # Joint 2: revolute with theta=q2, d=0, a=0, alpha=-90°
        # This rotates around the new Z axis by q2
        T_1_2 = self.dh_transform(
            theta=q2,
            d=0,
            a=0,
            alpha=-np.pi/2
        )
        
        # Joint 3: prismatic with theta=0, d=q3, a=0, alpha=0°
        # This extends along the Z axis by q3
        T_2_3 = self.dh_transform(
            theta=0,
            d=q3,
            a=0,
            alpha=0
        )
        
        # Compute cumulative transforms from base
        T_0_1_cumulative = T_base @ T_0_1
        T_0_2_cumulative = T_0_1_cumulative @ T_1_2
        T_0_3_cumulative = T_0_2_cumulative @ T_2_3
        
        # Return transforms: [base, joint1, joint2, joint3/end-effector]
        return [T_base, T_0_1_cumulative, T_0_2_cumulative, T_0_3_cumulative]
    
    def inverse_kinematics(self, target_position):
        """
        Compute inverse kinematics using analytical solution.
        
        From the DH forward kinematics:
        x = -q3 * sin(q2) * cos(q1)
        y = -q3 * sin(q2) * sin(q1)
        z = L1 + q3 * cos(q2)
        
        Analytical Solution:
        q1 = atan2(y, x)
        q2 = atan2(-sqrt(x^2 + y^2), z - L1)
        q3 = sqrt(x^2 + y^2 + (z - L1)^2)
        
        :param target_position: [x, y, z] target position
        :return: [q1, q2, q3] joint values
        """
        x, y, z = target_position
        
        # Analytical solution
        q1 = np.arctan2(y, x)
        q2 = np.arctan2(-np.sqrt(x**2 + y**2), z - self.L1)
        q3 = np.sqrt(x**2 + y**2 + (z - self.L1)**2)
        
        return np.array([q1, q2, q3])
    
    def verify_ik_solution(self, target_position, joint_values, tolerance=1e-6):
        """
        Verify that the IK solution reaches the target position.
        
        :param target_position: Desired [x, y, z] position
        :param joint_values: [q1, q2, q3] joint values from IK
        :param tolerance: Maximum acceptable error
        :return: (success, error, achieved_position)
        """
        transforms = self.forward_kinematics(joint_values)
        achieved_position = transforms[-1][:3, 3]
        error = np.linalg.norm(achieved_position - target_position)
        success = error < tolerance
        
        return success, error, achieved_position
    
    def compute_workspace(self, q1_range=None, q2_range=None, q3_range=None, resolution=20):
        """
        Compute the robot's workspace by sampling joint values.
        
        :param q1_range: [min, max] for q1 in radians (default: [-π, π])
        :param q2_range: [min, max] for q2 in radians (default: [-π/2, π/2])
        :param q3_range: [min, max] for q3 in meters (default: [0, 2*L1])
        :param resolution: Number of samples per joint
        :return: Array of reachable 3D points
        """
        if q1_range is None:
            q1_range = [-np.pi, np.pi]
        if q2_range is None:
            q2_range = [-np.pi/2, np.pi/2]
        if q3_range is None:
            q3_range = [0, 2*self.L1]
        
        q1_values = np.linspace(q1_range[0], q1_range[1], resolution)
        q2_values = np.linspace(q2_range[0], q2_range[1], resolution)
        q3_values = np.linspace(q3_range[0], q3_range[1], resolution)
        
        workspace_points = []
        total = resolution ** 3
        count = 0
        
        print(f"Computing workspace with {resolution} samples per joint...")
        print(f"Total points to compute: {total:,}")
        
        for q1 in q1_values:
            for q2 in q2_values:
                for q3 in q3_values:
                    count += 1
                    if count % 10000 == 0 or count == total:
                        print(f"  ... calculated {count:,} of {total:,} points ({(count/total)*100:.1f}%)")
                    
                    transforms = self.forward_kinematics([q1, q2, q3])
                    end_effector_position = transforms[-1][:3, 3]
                    workspace_points.append(end_effector_position)
        
        print(f"Workspace computation finished. Found {len(workspace_points)} reachable points.")
        return np.array(workspace_points)
