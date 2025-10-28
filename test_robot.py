import numpy as np
from kinematics_plotter import RobotArm

def main():
    """
    Main function to test the robot kinematics and plotting.
    """
    # Path to the robot configuration file
    config_file = 'robot_config.yaml'

    # Create a RobotArm instance
    robot = RobotArm(config_file)

    # Define the joint angles for the robot (in radians)
    # For our 3-DOF robot, we need 3 angles.
    joint_angles = [np.pi/4, np.pi/4, np.pi/4]  # 45 degrees for each joint

    # Plot the robot with the specified joint angles
    robot.plot(joint_angles)

if __name__ == "__main__":
    main()