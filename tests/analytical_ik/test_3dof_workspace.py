import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import itertools

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def compute_configuration_space(robot, q1_limits=(-180, 180), q2_limits=(-130, 90), 
                                q3_limits=(0.0, 1.5), resolution_deg=10):
    """
    Computes the robot's configuration space by sampling all joint angles within physical limits.
    
    :param robot: Robot3DOF_DH instance
    :param q1_limits: Tuple (min, max) in degrees for joint 1
    :param q2_limits: Tuple (min, max) in degrees for joint 2
    :param q3_limits: Tuple (min, max) in meters for joint 3 (prismatic)
    :param resolution_deg: The step size in degrees for sampling each joint's range of motion
    :return: Array of reachable 3D points for the end-effector
    """
    print(f"\nComputing CONFIGURATION SPACE (physical limits) with resolution of {resolution_deg}°...")
    
    # Define joint ranges based on provided limits
    q1_range = np.deg2rad(np.arange(q1_limits[0], q1_limits[1] + resolution_deg, resolution_deg))
    q2_range = np.deg2rad(np.arange(q2_limits[0], q2_limits[1] + resolution_deg, resolution_deg))
    q3_range = np.arange(q3_limits[0], q3_limits[1] + 0.1, 0.1)  # Sample every 0.1m
    
    # Calculate total number of points
    num_q1 = len(q1_range)
    num_q2 = len(q2_range)
    num_q3 = len(q3_range)
    total_combinations = num_q1 * num_q2 * num_q3
    
    print(f"  Physical limits: q1=[{q1_limits[0]}°, {q1_limits[1]}°], q2=[{q2_limits[0]}°, {q2_limits[1]}°], q3=[{q3_limits[0]}m, {q3_limits[1]}m]")
    print(f"  Joint samples: q1={num_q1}, q2={num_q2}, q3={num_q3}")
    print(f"  Total points to compute: {total_combinations:,}")
    
    # Use itertools.product for efficient iteration
    joint_angle_iterator = itertools.product(q1_range, q2_range, q3_range)
    
    config_space_points = []
    for i, (q1, q2, q3) in enumerate(joint_angle_iterator):
        # Print progress
        if (i + 1) % 10000 == 0 or (i + 1) == total_combinations:
            print(f"  Progress: {i+1:,}/{total_combinations:,} ({100*(i+1)/total_combinations:.1f}%)")
        
        # Compute forward kinematics
        angles = np.array([q1, q2, q3])
        transforms = robot.forward_kinematics(angles)
        end_effector_position = transforms[-1][:3, 3]
        config_space_points.append(end_effector_position)
    
    config_space_points = np.array(config_space_points)
    print(f"✓ Configuration space computed: {len(config_space_points):,} reachable points")
    
    return config_space_points


def compute_workspace(robot, q1_limits=(-90, 90), q2_limits=(-45, 60), 
                      q3_limits=(0.2, 1.2), resolution_deg=10):
    """
    Computes the robot's workspace with OPERATIONAL CONSTRAINTS (stricter limits).
    These constraints represent external factors like obstacles, safety zones, cable limits, etc.
    
    :param robot: Robot3DOF_DH instance
    :param q1_limits: Tuple (min, max) in degrees for joint 1 operational constraints
    :param q2_limits: Tuple (min, max) in degrees for joint 2 operational constraints
    :param q3_limits: Tuple (min, max) in meters for joint 3 operational constraints
    :param resolution_deg: The step size in degrees for sampling each joint's range of motion
    :return: Array of reachable 3D points for the end-effector with constraints
    """
    print(f"\nComputing WORKSPACE (with operational constraints) with resolution of {resolution_deg}°...")
    
    # Define joint ranges with provided OPERATIONAL CONSTRAINTS
    # These are stricter than physical limits due to:
    # - Obstacle avoidance
    # - Cable management
    # - Safety zones
    # - Preferred operating region
    
    q1_range = np.deg2rad(np.arange(q1_limits[0], q1_limits[1] + resolution_deg, resolution_deg))
    q2_range = np.deg2rad(np.arange(q2_limits[0], q2_limits[1] + resolution_deg, resolution_deg))
    q3_range = np.arange(q3_limits[0], q3_limits[1] + 0.1, 0.1)
    
    # Calculate total number of points
    num_q1 = len(q1_range)
    num_q2 = len(q2_range)
    num_q3 = len(q3_range)
    total_combinations = num_q1 * num_q2 * num_q3
    
    print(f"  Operational constraints: q1=[{q1_limits[0]}°, {q1_limits[1]}°], q2=[{q2_limits[0]}°, {q2_limits[1]}°], q3=[{q3_limits[0]}m, {q3_limits[1]}m]")
    print(f"  Joint samples: q1={num_q1}, q2={num_q2}, q3={num_q3}")
    print(f"  Total points to compute: {total_combinations:,}")
    
    # Use itertools.product for efficient iteration
    joint_angle_iterator = itertools.product(q1_range, q2_range, q3_range)
    
    workspace_points = []
    for i, (q1, q2, q3) in enumerate(joint_angle_iterator):
        # Print progress
        if (i + 1) % 5000 == 0 or (i + 1) == total_combinations:
            print(f"  Progress: {i+1:,}/{total_combinations:,} ({100*(i+1)/total_combinations:.1f}%)")
        
        # Compute forward kinematics
        angles = np.array([q1, q2, q3])
        transforms = robot.forward_kinematics(angles)
        end_effector_position = transforms[-1][:3, 3]
        workspace_points.append(end_effector_position)
    
    workspace_points = np.array(workspace_points)
    print(f"✓ Workspace computed: {len(workspace_points):,} reachable points")
    
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
    ax.set_title('3DOF Robot Workspace\nReachable End-Effector Positions (Point Cloud)', 
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
    plt.savefig('docs/3dof_workspace_points.png', dpi=150, bbox_inches='tight')
    print("\n✓ Workspace point cloud saved as 'docs/3dof_workspace_points.png'")
    plt.show()


def plot_workspace_volume(robot, workspace_points):
    """
    Plot the workspace as a 3D volumetric surface using convex hull.
    
    :param robot: Robot3DOF_DH instance
    :param workspace_points: Array of workspace positions
    """
    from scipy.spatial import ConvexHull
    
    print("\nComputing convex hull for volumetric visualization...")
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Compute convex hull
    try:
        hull = ConvexHull(workspace_points)
        
        # Plot the convex hull surface
        for simplex in hull.simplices:
            triangle = workspace_points[simplex]
            # Create a triangular surface
            tri = plt.matplotlib.tri.Triangulation(triangle[:, 0], triangle[:, 1])
            ax.plot_trisurf(triangle[:, 0], triangle[:, 1], triangle[:, 2],
                           triangles=[[0, 1, 2]], 
                           color='green', alpha=0.3, edgecolor='darkgreen', linewidth=0.2)
        
        print(f"✓ Convex hull computed: {len(hull.simplices)} triangular faces")
        
    except Exception as e:
        print(f"Warning: Could not compute convex hull: {e}")
        print("Falling back to point cloud visualization...")
        ax.scatter(workspace_points[:, 0], 
                   workspace_points[:, 1], 
                   workspace_points[:, 2],
                   c='green', alpha=0.3, s=1)
    
    # Add base position
    ax.scatter([0], [0], [0], c='red', s=100, marker='o', label='Base', zorder=10)
    
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title('3DOF Robot Workspace\nReachable Volume (Convex Hull)', 
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
    plt.savefig('docs/3dof_workspace_volume.png', dpi=150, bbox_inches='tight')
    print("✓ Workspace volume saved as 'docs/3dof_workspace_volume.png'")
    plt.show()


def plot_workspace_combined(robot, config_space_points, workspace_points):
    """
    Plot both configuration space and workspace in a single figure with two subplots.
    Shows point cloud and volumetric surface views side by side.
    
    :param robot: Robot3DOF_DH instance
    :param config_space_points: Array of configuration space positions (physical limits)
    :param workspace_points: Array of workspace positions (with operational constraints)
    """
    from scipy.spatial import ConvexHull
    
    fig = plt.figure(figsize=(20, 9))
    
    # Calculate common axis limits for both plots (use config space for full range)
    max_range = np.array([config_space_points[:, 0].max() - config_space_points[:, 0].min(),
                          config_space_points[:, 1].max() - config_space_points[:, 1].min(),
                          config_space_points[:, 2].max() - config_space_points[:, 2].min()]).max() / 2.0
    
    mid_x = (config_space_points[:, 0].max() + config_space_points[:, 0].min()) * 0.5
    mid_y = (config_space_points[:, 1].max() + config_space_points[:, 1].min()) * 0.5
    mid_z = (config_space_points[:, 2].max() + config_space_points[:, 2].min()) * 0.5
    
    # Subplot 1: Point Cloud View
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Plot configuration space (green)
    ax1.scatter(config_space_points[:, 0], 
                config_space_points[:, 1], 
                config_space_points[:, 2],
                c='green', alpha=0.2, s=1, label='Configuration Space (Physical Limits)')
    
    # Plot workspace (grey/dark grey)
    ax1.scatter(workspace_points[:, 0], 
                workspace_points[:, 1], 
                workspace_points[:, 2],
                c='dimgrey', alpha=0.5, s=2, label='Workspace (Operational Constraints)')
    
    ax1.scatter([0], [0], [0], c='red', s=100, marker='o', label='Base', zorder=10)
    
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_zlabel('Z (m)', fontsize=11)
    ax1.set_title('Point Cloud View\nConfiguration Space vs Workspace', 
                  fontsize=12, fontweight='bold')
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Subplot 2: Volumetric Surface View
    ax2 = fig.add_subplot(122, projection='3d')
    
    print("  Computing convex hulls for volumetric visualization...")
    
    # Plot configuration space hull (green)
    try:
        hull_config = ConvexHull(config_space_points)
        
        for simplex in hull_config.simplices:
            triangle = config_space_points[simplex]
            ax2.plot_trisurf(triangle[:, 0], triangle[:, 1], triangle[:, 2],
                            triangles=[[0, 1, 2]], 
                            color='green', alpha=0.2, edgecolor='darkgreen', linewidth=0.1)
        
        print(f"  ✓ Configuration space hull: {len(hull_config.simplices)} triangular faces")
        
    except Exception as e:
        print(f"  Warning: Could not compute config space hull: {e}")
    
    # Plot workspace hull (grey)
    try:
        hull_workspace = ConvexHull(workspace_points)
        
        for simplex in hull_workspace.simplices:
            triangle = workspace_points[simplex]
            ax2.plot_trisurf(triangle[:, 0], triangle[:, 1], triangle[:, 2],
                            triangles=[[0, 1, 2]], 
                            color='dimgrey', alpha=0.5, edgecolor='black', linewidth=0.2)
        
        print(f"  ✓ Workspace hull: {len(hull_workspace.simplices)} triangular faces")
        
    except Exception as e:
        print(f"  Warning: Could not compute workspace hull: {e}")
    
    ax2.scatter([0], [0], [0], c='red', s=100, marker='o', label='Base', zorder=10)
    
    ax2.set_xlabel('X (m)', fontsize=11)
    ax2.set_ylabel('Y (m)', fontsize=11)
    ax2.set_zlabel('Z (m)', fontsize=11)
    ax2.set_title('Volumetric View\nConfiguration Space vs Workspace', 
                  fontsize=12, fontweight='bold')
    
    # Create custom legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', alpha=0.3, label='Configuration Space'),
        Patch(facecolor='dimgrey', alpha=0.5, label='Workspace'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=8, label='Base')
    ]
    ax2.legend(handles=legend_elements, loc='upper left', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    ax2.set_xlim(mid_x - max_range, mid_x + max_range)
    ax2.set_ylim(mid_y - max_range, mid_y + max_range)
    ax2.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Add main title
    fig.suptitle('3DOF Robot: Configuration Space vs Workspace Analysis', fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('docs/3dof_workspace_combined.png', dpi=150, bbox_inches='tight')
    print("\n✓ Combined visualization saved as 'docs/3dof_workspace_combined.png'")
    plt.show()


def main():
    """
    Main function to compute and visualize the 3DOF robot configuration space and workspace.
    """
    print("="*70)
    print("3DOF Robot: Configuration Space vs Workspace Analysis")
    print("="*70)
    
    # Initialize robot
    robot = Robot3DOF_DH(L1=1.0)
    
    print("\nRobot Configuration:")
    print(f"  L1 (base height): {robot.L1}m")
    print(f"  Joint 1 (q1): Revolute")
    print(f"  Joint 2 (q2): Revolute")
    print(f"  Joint 3 (q3): Prismatic")
    
    print("\n" + "="*70)
    print("Computing Spaces")
    print("="*70)
    
    resolution = 15  # degrees (adjust for speed vs accuracy)
    
    # Define PHYSICAL LIMITS for Configuration Space
    config_q1_limits = (-180, 180)  # Full rotation
    config_q2_limits = (-130, 90)   # Physical tilt range
    config_q3_limits = (0.0, 1.5)   # Full extension
    
    # Define OPERATIONAL CONSTRAINTS for Workspace
    # These are stricter due to obstacles, safety zones, cable limits, etc.
    workspace_q1_limits = (-90, 90)   # Limited rotation
    workspace_q2_limits = (0, 70)   # Limited tilt
    workspace_q3_limits = (0.2, 1.2)  # Limited extension with safety margins
    
    # 1. Compute Configuration Space (physical limits)
    config_space_points = compute_configuration_space(
        robot, 
        q1_limits=config_q1_limits,
        q2_limits=config_q2_limits,
        q3_limits=config_q3_limits,
        resolution_deg=resolution
    )
    
    # 2. Compute Workspace (operational constraints)
    workspace_points = compute_workspace(
        robot,
        q1_limits=workspace_q1_limits,
        q2_limits=workspace_q2_limits,
        q3_limits=workspace_q3_limits,
        resolution_deg=resolution
    )
    
    # Generate visualizations
    if config_space_points.size > 0 and workspace_points.size > 0:
        print("\n" + "="*70)
        print("Generating Visualizations")
        print("="*70)
        
        print("\nCreating combined visualization...")
        print(f"  Configuration space: {len(config_space_points):,} points (green)")
        print(f"  Workspace: {len(workspace_points):,} points (grey)")
        
        plot_workspace_combined(robot, config_space_points, workspace_points)
    
    print("\n" + "="*70)
    print("✓ Analysis complete!")
    print("="*70)
    print("\nSummary:")
    print(f"  Configuration Space: All positions reachable within physical limits")
    print(f"  Workspace: Positions reachable with operational constraints")
    print(f"  Constraints represent: obstacles, safety zones, cable limits, etc.")


if __name__ == "__main__":
    main()
