import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def plot_robot_trajectory(robot, start_pos, end_pos, num_points=50):
    """
    Plot the robot following a trajectory from start to end position.
    
    :param robot: Robot3DOF_DH instance
    :param start_pos: Starting [x, y, z] position
    :param end_pos: Ending [x, y, z] position
    :param num_points: Number of points along the trajectory
    """
    # Generate trajectory points (linear interpolation)
    trajectory = np.linspace(start_pos, end_pos, num_points)
    
    print(f"\n{'='*70}")
    print("3DOF Robot - Trajectory Following with Analytical IK")
    print(f"{'='*70}\n")
    
    print(f"Start Position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
    print(f"End Position:   [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")
    print(f"Number of points: {num_points}\n")
    
    # Compute IK for all trajectory points
    joint_trajectories = []
    achieved_positions = []
    errors = []
    
    print("Computing inverse kinematics for trajectory...")
    for i, target in enumerate(trajectory):
        # Compute IK
        joint_values = robot.inverse_kinematics(target)
        joint_trajectories.append(joint_values)
        
        # Verify with FK
        success, error, achieved = robot.verify_ik_solution(target, joint_values)
        achieved_positions.append(achieved)
        errors.append(error)
        
        if (i + 1) % 10 == 0 or i == 0 or i == num_points - 1:
            print(f"  Point {i+1}/{num_points}: Target = [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
            print(f"    -> q1={np.rad2deg(joint_values[0]):6.1f}°, q2={np.rad2deg(joint_values[1]):6.1f}°, q3={joint_values[2]:.4f}m")
            print(f"    -> Error: {error:.2e} m")
    
    max_error = max(errors)
    avg_error = np.mean(errors)
    print(f"\nTrajectory Statistics:")
    print(f"  Maximum error: {max_error:.2e} m")
    print(f"  Average error: {avg_error:.2e} m")
    print(f"  All errors < 1e-10: {'✓ YES' if max_error < 1e-10 else '✗ NO'}")
    
    # Convert to numpy arrays
    joint_trajectories = np.array(joint_trajectories)
    achieved_positions = np.array(achieved_positions)
    
    # Create visualization
    fig = plt.figure(figsize=(18, 6))
    
    # Plot 1: 3D trajectory
    ax1 = fig.add_subplot(131, projection='3d')
    
    # Plot desired trajectory
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
             'g--', linewidth=2, label='Desired Path', alpha=0.7)
    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2],
                color='green', s=200, marker='o', label='Start', edgecolors='black', linewidths=2)
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2],
                color='red', s=200, marker='s', label='End', edgecolors='black', linewidths=2)
    
    # Plot achieved trajectory
    ax1.plot(achieved_positions[:, 0], achieved_positions[:, 1], achieved_positions[:, 2],
             'b-', linewidth=1.5, label='Achieved Path', alpha=0.8)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title('End-Effector Trajectory', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Joint angles over time
    ax2 = fig.add_subplot(132)
    time_steps = np.arange(num_points)
    
    ax2.plot(time_steps, np.rad2deg(joint_trajectories[:, 0]), 'r-', linewidth=2, label='q1 (deg)')
    ax2.plot(time_steps, np.rad2deg(joint_trajectories[:, 1]), 'g-', linewidth=2, label='q2 (deg)')
    ax2.set_xlabel('Step', fontsize=10)
    ax2.set_ylabel('Angle (degrees)', fontsize=10)
    ax2.set_title('Joint Angles (Revolute)', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Prismatic joint and errors
    ax3 = fig.add_subplot(133)
    ax3_twin = ax3.twinx()
    
    ax3.plot(time_steps, joint_trajectories[:, 2], 'b-', linewidth=2, label='q3 (m)')
    ax3.set_xlabel('Step', fontsize=10)
    ax3.set_ylabel('Extension (m)', fontsize=10, color='b')
    ax3.tick_params(axis='y', labelcolor='b')
    ax3.grid(True, alpha=0.3)
    
    ax3_twin.plot(time_steps, errors, 'r--', linewidth=1.5, label='Position Error', alpha=0.7)
    ax3_twin.set_ylabel('Error (m)', fontsize=10, color='r')
    ax3_twin.tick_params(axis='y', labelcolor='r')
    ax3_twin.set_yscale('log')
    
    ax3.set_title('Prismatic Joint & Error', fontsize=12, fontweight='bold')
    
    # Combine legends
    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=9)
    
    plt.tight_layout()
    
    # Save figure
    output_file = 'docs/3dof_trajectory_test.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n{'='*70}")
    print(f"Visualization saved as '{output_file}'")
    print(f"{'='*70}\n")
    
    plt.show()


def test_multiple_trajectories():
    """
    Test multiple trajectories with different characteristics.
    """
    robot = Robot3DOF_DH(L1=1.0)
    
    # Define test trajectories
    trajectories = [
        {
            'name': 'Straight Line (Horizontal)',
            'start': [0.5, 0.5, 1.5],
            'end': [1.0, 1.0, 1.5],
            'points': 30
        },
        {
            'name': 'Diagonal Path',
            'start': [0.3, 0.3, 1.0],
            'end': [0.8, 0.8, 2.0],
            'points': 40
        },
        {
            'name': 'Vertical Motion',
            'start': [0.5, 0.5, 1.0],
            'end': [0.5, 0.5, 2.0],
            'points': 25
        }
    ]
    
    for i, traj in enumerate(trajectories):
        print(f"\n{'#'*70}")
        print(f"# Trajectory {i+1}: {traj['name']}")
        print(f"{'#'*70}")
        plot_robot_trajectory(robot, 
                            np.array(traj['start']), 
                            np.array(traj['end']), 
                            traj['points'])


def main():
    """
    Main test function.
    """
    robot = Robot3DOF_DH(L1=1.0)
    
    # Test a single trajectory
    start_position = np.array([0.5, 0.5, 1.5])
    end_position = np.array([1.0, 0.0, 2.0])
    
    plot_robot_trajectory(robot, start_position, end_position, num_points=50)


if __name__ == "__main__":
    main()
