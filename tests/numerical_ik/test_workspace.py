import sys
import os
import argparse

# Add parent directory to path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

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
    config_file = os.path.join(script_dir, '../../configs/robot_config.yaml')

    # Create a RobotArm instance
    robot = RobotArm(config_file)

    # --- Resolution Adjustment ---
    # Adjust resolution based on the number of joints to keep computation time reasonable.
    num_dof = len(robot.joints)
    
    # Define minimum resolution (larger number means coarser) based on DOF
    resolution_caps = {
        4: 15,  # For 4-DOF, resolution should be at least 15 degrees
        5: 30,  # For 5-DOF, at least 30 degrees
        6: 45   # For 6-DOF, at least 45 degrees
    }
    # For 3-DOF or less, any user-provided resolution is generally fine.
    min_resolution_for_dof = resolution_caps.get(num_dof, 1) # Default to 1 (no limit) for low DOF

    final_resolution = args.res
    if final_resolution < min_resolution_for_dof:
        print(f"Warning: Requested resolution of {args.res}° is very fine for a {num_dof}-DOF robot.")
        print(f"         Adjusting to {min_resolution_for_dof}° to ensure reasonable computation time.")
        final_resolution = min_resolution_for_dof

    # --- Workspace Computation ---
    # Compute the workspace points. This might take a while depending on the resolution.
    # Note: A high resolution (low --res value) can be very computationally expensive!
    workspace_points = robot.compute_workspace(resolution_deg=final_resolution)

    # --- Plotting ---
    # Plot the collected workspace points all at once.
    if workspace_points.size > 0:
        plot_workspace(robot, workspace_points)

if __name__ == "__main__":
    main()