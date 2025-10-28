import numpy as np
from kinematics_plotter import RobotArm

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
    
    robot.animate_movement(initial_pos, final_pos, frames=frames)
    print("Animation finished.")

def main():
    """
    Main function to test the robot kinematics and plotting.
    """
    # Path to the robot configuration file
    config_file = 'robot_config.yaml'

    # Create a RobotArm instance
    robot = RobotArm(config_file)

    # Define initial and final joint positions for the movement test
    initial_joint_angles = [0, 0, 0]
    final_joint_angles_degrees = [180, 90, -180]

    final_joint_angles = np.deg2rad(final_joint_angles_degrees)
    # Run the kinematic movement test
    test_kinematic_movement(robot, initial_joint_angles, final_joint_angles)

if __name__ == "__main__":
    main()