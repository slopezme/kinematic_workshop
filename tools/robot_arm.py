import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
from .transforms import create_transformation_matrix

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
            T_cumulative = np.matmul(np.matmul(T_cumulative, T_static), T_dynamic)
            frame_transforms.append(T_cumulative)
            
        return frame_transforms

    def compute_workspace(self, resolution_deg=30):
        """
        Computes the robot's workspace by sampling joint angles.

        :param resolution_deg: The step size in degrees for sampling each joint's range of motion.
        :return: A list of reachable 3D points for the end-effector.
        """
        import itertools
        print(f"Computing workspace with a resolution of {resolution_deg} degrees...")
        
        angle_ranges = []
        for joint in self.joints:
            # Use limits from config, or default to full 360-degree range
            limit = joint.get('limit', {'lower': 0, 'upper': 360})
            lower_rad = np.deg2rad(limit['lower'])
            upper_rad = np.deg2rad(limit['upper'])
            resolution_rad = np.deg2rad(resolution_deg)
            # Add resolution_rad to upper_rad to make sure the upper limit is included in the range
            angle_ranges.append(np.arange(lower_rad, upper_rad + resolution_rad, resolution_rad))

        # Calculate and print the total number of points to be computed.
        num_steps_per_joint = [len(r) for r in angle_ranges]
        total_combinations = np.prod(num_steps_per_joint) if num_steps_per_joint else 0
        print(f"Total points to compute: {total_combinations:,}")

        # Use itertools.product for a memory-efficient way to get all angle combinations
        joint_angle_iterator = itertools.product(*angle_ranges)

        workspace_points = []
        for i, angles in enumerate(joint_angle_iterator):
            # Print progress update
            if (i + 1) % 100000 == 0 or (i + 1) == total_combinations:
                print(f"  ... calculated {i+1:,} of {total_combinations:,} points ({((i+1)/total_combinations)*100:.1f}%)")
            frame_transforms = self.forward_kinematics(angles)
            end_effector_position = frame_transforms[-1][:3, 3]
            workspace_points.append(end_effector_position)
        
        print(f"Workspace computation finished. Found {len(workspace_points)} reachable points.")
        return np.array(workspace_points)

    def inverse_kinematics(self, target_position, initial_guess_angles, tolerance=1e-3, max_iterations=100):
        """
        Computes inverse kinematics using numerical optimization.

        :param target_position: The desired 3D position for the end-effector.
        :param initial_guess_angles: A seed/starting point for the joint angles optimization.
        :param tolerance: The acceptable position error tolerance.
        :param max_iterations: The maximum number of iterations for the optimizer.
        :return: A tuple containing (final_joint_angles, final_position, final_error).
        """
        
        # Objective function: Minimize the distance between end-effector and target
        def error_function(angles):
            transforms = self.forward_kinematics(angles)
            current_position = transforms[-1][:3, 3]
            error = np.linalg.norm(current_position - target_position)
            return error

        # Get joint limits from config to use as bounds for the optimizer
        bounds = []
        for joint in self.joints:
            limit = joint.get('limit', {'lower': -360, 'upper': 360}) # Default to full rotation if no limits
            bounds.append((np.deg2rad(limit['lower']), np.deg2rad(limit['upper'])))

        # Run the optimization
        result = minimize(
            error_function,
            initial_guess_angles,
            method='L-BFGS-B',  # A method that respects bounds
            bounds=bounds,
            tol=tolerance,
            options={'maxiter': max_iterations}
        )

        # Get the final results from the optimization
        final_angles = result.x
        final_transforms = self.forward_kinematics(final_angles)
        final_position = final_transforms[-1][:3, 3]
        final_error = np.linalg.norm(final_position - target_position)

        if not result.success:
            print(f"Warning: IK optimization may not have converged. Message: {result.message}")

        return final_angles, final_position, final_error