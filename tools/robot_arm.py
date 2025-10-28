import yaml
import numpy as np
from scipy.spatial.transform import Rotation
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
            T_cumulative = T_cumulative @ T_static @ T_dynamic
            frame_transforms.append(T_cumulative)
            
        return frame_transforms