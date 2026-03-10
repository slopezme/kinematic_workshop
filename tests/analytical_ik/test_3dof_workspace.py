import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import itertools

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def compute_workspace(robot, resolution_deg=10):
    """
    Computes the robot's workspace by sampling joint angles.
    
    :param robot: Robot3DOF_DH instance
    :param resolution_deg: The step size in degrees for sampling each joint's range of motion
    :return: Array of reachable 3D points for the end-effector
    """
    print(f"\nComputing workspace with a resolution of {resolution_deg} degrees...")
    
    # Define joint ranges based on physical limits
    # q1: Full rotation -180° to 180°
    # q2: Tilt range -90° to 90°
    # q3: Extension 0 to 1.5m (sample in meters, not degrees)
    
    q1_range = np.deg2rad(np.arange(-180, 180 + resolution_deg, resolution_deg))
    q2_range = np.deg2rad(np.arange(-90, 90 + resolution_deg, resolution_deg))
    q3_range = np.arange(0.0, 1.5 + 0.1, 0.1)  # Sample every 0.1m
    
    # Calculate total number of points
    num_q1 = len(q1_range)
    num_q2 = len(q2_range)
    num_q3 = len(q3_range)
    total_combinations = num_q1 * num_q2 * num_q3
    
    print(f"Joint samples: q1={num_q1}, q2={num_q2}, q3={num_q3}")
    print(f"Total points to compute: {total_combinations:,}")
    
    # Use itertools.product for efficient iteration
    joint_angle_iterator = itertools.product(q1_range, q2_range, q3_range)
    
    workspace_points = []
    for i, (q1, q2, q3) in enumerate(joint_angle_iterator):
        # Print progress
        if (i + 1) % 10000 == 0 or (i + 1) == total_combinations:
            print(f"  Progress: {i+1:,}/{total_combinations:,} ({100*(i+1)/total_combinations:.1f}%)")
        
        # Compute forward kinematics
        angles = np.array([q1, q2, q3])
        transforms = robot.forward_kinematics(angles)
        end_effector_position = transforms[-1][:3, 3]
        workspace_points.append(end_effector_position)
    
    workspace_points = np.array(workspace_points)
    print(f"\n✓ Workspace computation finished: {len(workspace_points):,} reachable points")
    
    return workspace_points


def plot_workspace(robot, workspace_points):
    """
    Plot the workspace as a 3D point cloud.
    
    :param robot: Robot3DOF_DH instance
    :param workspace_points: Array of workspace positions
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot workspace points
    ax.scatter(workspace_points[:, 0], 
               workspace_points[:, 1], 
               workspace_points[:, 2],
               c='green', alpha=0.3, s=1, label='Workspace')
    
    # Add base position
    ax.scatter([0], [0], [0], c='red', s=100, marker='o', label='Base')
    
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title('3DOF Robot Workspace\nReachable End-Effector Positions', 
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Set equal aspect ratio
    max_range = np.array([workspace_points[:, 0].max() - workspace_points[:, 0].min(),
                          workspace_points[:, 1].max() - workspace_points[:, 1].min(),
                          workspace_points[:, 2].max() - workspace_points[:, 2].min()]).max() / 2.0
    
    mid_x = (workspace_points[:, 0].max() + workspace_points[:, 0].min()) * 0.5
    mid_y = (workspace_points[:, 1].max() + workspace_points[:, 1].min()) * 0.5
    mid_z = (workspace_points[:, 2].max() + workspace_points[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.savefig('docs/3dof_workspace.png', dpi=150, bbox_inches='tight')
    print("\n✓ Workspace plot saved as 'docs/3dof_workspace.png'")
    plt.show()


def main():
    """
    Main function to compute and visualize the 3DOF robot workspace.
    """
    print("="*70)
    print("3DOF Robot Workspace Computation")
    print("="*70)
    
    # Initialize robot
    robot = Robot3DOF_DH(L1=1.0)
    
    print("\nRobot Configuration:")
    print(f"  L1 (base height): {robot.L1}m")
    print(f"  Joint 1 (q1): Revolute, full rotation (-180° to 180°)")
    print(f"  Joint 2 (q2): Revolute, tilt (-90° to 90°)")
    print(f"  Joint 3 (q3): Prismatic, extension (0 to 1.5m)")
    
    # Compute workspace
    resolution = 15  # degrees (adjust for speed vs accuracy)
    workspace_points = compute_workspace(robot, resolution_deg=resolution)
    
    # Plot workspace
    if workspace_points.size > 0:
        plot_workspace(robot, workspace_points)
    
    print("\n✓ Analysis complete!")


if __name__ == "__main__":
    main()
