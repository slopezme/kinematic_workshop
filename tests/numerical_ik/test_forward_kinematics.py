import sys
import numpy as np
import os
import yaml
import argparse

# Add parent directory to path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_arm import RobotArm
from kinematics_plotter import animate_movement

def test_kinematic_movement(robot, initial_pos, final_pos, frames=200):
    """
    Tests the forward kinematics by animating the robot's movement
    from an initial to a final joint configuration.

    :param robot: An instance of the RobotArm class.
    :param initial_pos: A list of initial joint angles (in radians).
    :param final_pos: A list of final joint angles (in radians).
    :param frames: The number of frames to use for the animation timeline.
    """
    print("Starting kinematic movement test...")
    print(f"Initial Joint Angles: {np.rad2deg(initial_pos)} degrees")
    print(f"Final Joint Angles: {np.rad2deg(final_pos)} degrees")
    
    # Store the animation object to prevent it from being garbage-collected
    anim = animate_movement(robot, initial_pos, final_pos, frames=frames)
    print("Animation finished.")

def main():
    """
    Main function to test the robot kinematics and plotting.
    """
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Test forward kinematics with robot animation.")
    parser.add_argument('--config', type=str, default='robot_config.yaml',
                        help="Robot configuration file (default: robot_config.yaml)")
    args = parser.parse_args()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, '../../configs', args.config)
    
    # Load the full configuration to get test parameters
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Create a RobotArm instance
    robot = RobotArm(config_file)

    # Get initial and final joint angles from the config file
    test_params = config['test_config']['forward_kinematics_animation']
    initial_angles_deg = test_params['initial_angles_deg']
    final_angles_deg = test_params['final_angles_deg']

    initial_joint_angles = np.deg2rad(initial_angles_deg)
    final_joint_angles = np.deg2rad(final_angles_deg)
    test_kinematic_movement(robot, initial_joint_angles, final_joint_angles)

if __name__ == "__main__":
    main()