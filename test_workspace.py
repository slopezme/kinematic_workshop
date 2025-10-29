import os
import argparse
from tools.robot_arm import RobotArm
from kinematics_plotter import plot_workspace

def main():
    """
    Main function to compute and plot the robot's workspace.
    """
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Kinematic Workshop: Compute and plot the robot's workspace.")
    parser.add_argument('--res', type=int, default=5, 
                        help="Resolution in degrees for sampling joint angles. Smaller is more detailed but slower.")
    args = parser.parse_args()

    # Path to the robot configuration file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, 'robot_config.yaml')

    # Create a RobotArm instance
    robot = RobotArm(config_file)

    # --- Workspace Computation ---
    # Compute the workspace points. This might take a while depending on the resolution.
    # Note: A high resolution (low --res value) can be very computationally expensive!
    workspace_points = robot.compute_workspace(resolution_deg=args.res)

    # --- Plotting ---
    # Plot the collected workspace points all at once.
    if workspace_points.size > 0:
        plot_workspace(robot, workspace_points)

if __name__ == "__main__":
    main()