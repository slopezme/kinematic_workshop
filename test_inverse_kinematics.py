import numpy as np
import os
from tools.robot_arm import RobotArm
from kinematics_plotter import animate_movement

def main():
    """
    Main function to test the inverse kinematics by making the robot
    follow a straight line path with its end-effector.
    """
    # --- Setup ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, 'robot_config.yaml')
    robot = RobotArm(config_file)

    # --- Path Definition ---
    # Define a straight line path for the end-effector
    start_pos = np.array([1.5, 0.5, 0.5])
    end_pos = np.array([-1.0, 1.0, 1.5])
    num_steps = 100
    path_points = np.linspace(start_pos, end_pos, num_steps)

    # --- IK Calculation ---
    print("Calculating Inverse Kinematics for the path...")
    
    # Use a starting seed for the first point (e.g., home position)
    # Subsequent points will use the previous result as a seed.
    seed_angles = np.deg2rad([90, 45, 45])
    
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
    # We need to create a smooth animation, so we'll animate between each step.
    animate_movement(robot, all_joint_angles[0], all_joint_angles[-1], frames=len(all_joint_angles))

if __name__ == "__main__":
    main()