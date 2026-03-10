import sys
import numpy as np
import os

# Add parent directory to path to import modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_arm import RobotArm
from kinematics_plotter import animate_movement

def main():
    """
    Main function to test the inverse kinematics by making the robot
    follow a straight line path with its end-effector.
    """
    # --- Setup ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, '../../configs/robot_config.yaml')
    robot = RobotArm(config_file)

    # --- Path Definition ---
    start_pos = np.array([0.5, 0.5, 0.5])
    end_pos = np.array([-1.0, 1.0, 1.5])
    num_steps = 100
    path_points = np.linspace(start_pos, end_pos, num_steps)

    # --- IK Calculation ---
    print("Calculating Inverse Kinematics for the path...")
    
    # Use a starting seed for the first point (e.g., home position)
    # Subsequent points will use the previous result as a seed.
    # Initialize seed angles to zeros, with a size matching the number of joints.
    num_joints = len(robot.joints)
    seed_angles = np.zeros(num_joints)
    
    all_joint_angles = []
    actual_path = []

    for i, target_pos in enumerate(path_points):
        print(f"Step {i+1}/{num_steps}: Target = [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")
        
        # Calculate IK for the current target point
        final_angles, final_pos, final_error = robot.inverse_kinematics(
            target_position=target_pos,
            initial_guess_angles=seed_angles
        )
        
        print(f"  -> Angles: {np.rad2deg(final_angles).round(1)} deg, Error: {final_error:.4f}")

        # Store results and update the seed for the next iteration
        all_joint_angles.append(final_angles)
        actual_path.append(final_pos)
        seed_angles = final_angles

    # --- Animation ---
    # Animate the robot moving through the calculated joint angles
    print("\nAnimating the calculated robot movement...")
    # Pass the actual path start and end points to the animation for plotting.
    animate_movement(robot, all_joint_angles[0], all_joint_angles[-1], 
                     path_start_pos=start_pos, path_end_pos=end_pos, frames=num_steps)

if __name__ == "__main__":
    main()