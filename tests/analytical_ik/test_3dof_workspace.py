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


def compute_task_space(workspace_points, x_limits, y_limits, z_limits):
    """
    Computes the task space as a rectangular 3D region within the workspace.
    Task space represents the specific region where the robot performs its task.
    
    :param workspace_points: Array of workspace positions
    :param x_limits: Tuple (min, max) for X coordinate in meters
    :param y_limits: Tuple (min, max) for Y coordinate in meters
    :param z_limits: Tuple (min, max) for Z coordinate in meters
    :return: Array of task space points (subset of workspace)
    """
    print(f"\nComputing TASK SPACE (rectangular region within workspace)...")
    print(f"  Task region: X=[{x_limits[0]}m, {x_limits[1]}m], Y=[{y_limits[0]}m, {y_limits[1]}m], Z=[{z_limits[0]}m, {z_limits[1]}m]")
    
    # Filter workspace points that fall within the task space bounds
    mask = (
        (workspace_points[:, 0] >= x_limits[0]) & (workspace_points[:, 0] <= x_limits[1]) &
        (workspace_points[:, 1] >= y_limits[0]) & (workspace_points[:, 1] <= y_limits[1]) &
        (workspace_points[:, 2] >= z_limits[0]) & (workspace_points[:, 2] <= z_limits[1])
    )
    
    task_space_points = workspace_points[mask]
    print(f"✓ Task space computed: {len(task_space_points):,} points")
    
    return task_space_points


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


def plot_workspace_combined(robot, config_space_points, workspace_points, task_space_points=None, task_limits=None):
    """
    Plot configuration space, workspace, and task space in a single figure with two subplots.
    Shows point cloud and volumetric surface views side by side.
    
    :param robot: Robot3DOF_DH instance
    :param config_space_points: Array of configuration space positions (physical limits)
    :param workspace_points: Array of workspace positions (with operational constraints)
    :param task_space_points: Array of task space positions (specific task region), optional
    :param task_limits: Tuple of (x_limits, y_limits, z_limits) for task space box, optional
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
                c='green', alpha=0.15, s=1, label='Configuration Space (Physical Limits)')
    
    # Plot workspace (grey/dark grey)
    ax1.scatter(workspace_points[:, 0], 
                workspace_points[:, 1], 
                workspace_points[:, 2],
                c='dimgrey', alpha=0.4, s=2, label='Workspace (Operational Constraints)')
    
    # Plot task space (red/orange) if provided
    if task_space_points is not None and len(task_space_points) > 0:
        ax1.scatter(task_space_points[:, 0], 
                    task_space_points[:, 1], 
                    task_space_points[:, 2],
                    c='orangered', alpha=0.8, s=3, label='Task Space (Specific Task Region)')
    
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
                            color='green', alpha=0.15, edgecolor='darkgreen', linewidth=0.1)
        
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
                            color='dimgrey', alpha=0.4, edgecolor='black', linewidth=0.2)
        
        print(f"  ✓ Workspace hull: {len(hull_workspace.simplices)} triangular faces")
        
    except Exception as e:
        print(f"  Warning: Could not compute workspace hull: {e}")
    
    # Plot task space as rectangular box if provided
    if task_space_points is not None and len(task_space_points) > 0 and task_limits is not None:
        # Get the actual task space bounds from the limits used
        # We'll draw a wireframe box to represent the task space region
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Use the exact task limits provided
        tx_min, tx_max = task_limits[0]
        ty_min, ty_max = task_limits[1]
        tz_min, tz_max = task_limits[2]
        
        # Define the 8 vertices of the rectangular box
        vertices = [
            [tx_min, ty_min, tz_min],
            [tx_max, ty_min, tz_min],
            [tx_max, ty_max, tz_min],
            [tx_min, ty_max, tz_min],
            [tx_min, ty_min, tz_max],
            [tx_max, ty_min, tz_max],
            [tx_max, ty_max, tz_max],
            [tx_min, ty_max, tz_max]
        ]
        
        # Define the 6 faces of the box
        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]]   # Top
        ]
        
        # Create the 3D polygon collection for the box
        box = Poly3DCollection(faces, alpha=0.5, facecolor='orangered', edgecolor='darkred', linewidth=2)
        ax2.add_collection3d(box)
        
        print(f"  ✓ Task space box: rectangular region")
    
    ax2.scatter([0], [0], [0], c='red', s=100, marker='o', label='Base', zorder=10)
    
    ax2.set_xlabel('X (m)', fontsize=11)
    ax2.set_ylabel('Y (m)', fontsize=11)
    ax2.set_zlabel('Z (m)', fontsize=11)
    ax2.set_title('Volumetric View\nConfiguration Space vs Workspace vs Task Space', 
                  fontsize=12, fontweight='bold')
    
    # Create custom legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', alpha=0.3, label='Configuration Space'),
        Patch(facecolor='dimgrey', alpha=0.5, label='Workspace'),
    ]
    if task_space_points is not None and len(task_space_points) > 0:
        legend_elements.append(Patch(facecolor='orangered', alpha=0.7, label='Task Space'))
    legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=8, label='Base'))
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
    
    # Analyze workspace bounds to help define task space
    print(f"\nWorkspace bounds:")
    print(f"  X: [{workspace_points[:, 0].min():.2f}m, {workspace_points[:, 0].max():.2f}m]")
    print(f"  Y: [{workspace_points[:, 1].min():.2f}m, {workspace_points[:, 1].max():.2f}m]")
    print(f"  Z: [{workspace_points[:, 2].min():.2f}m, {workspace_points[:, 2].max():.2f}m]")
    
    # 3. Define TASK SPACE bounds (rectangular region within workspace)
    # This represents the specific 3D region where the robot performs its task
    # These bounds must be within the workspace region
    # Set task space to be a larger region covering more of the workspace
    x_min, x_max = workspace_points[:, 0].min(), workspace_points[:, 0].max()
    y_min, y_max = workspace_points[:, 1].min(), workspace_points[:, 1].max()
    z_min, z_max = workspace_points[:, 2].min(), workspace_points[:, 2].max()
    
    # Create a task space covering about 25% of workspace in each dimension
    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    
    task_x_limits = (x_min + 0.375 * x_range, x_max - 0.375 * x_range)   # Central 25% in X
    task_y_limits = (y_min + 0.375 * y_range, y_max - 0.375 * y_range)   # Central 25% in Y
    task_z_limits = (z_min + 0.5 * z_range, z_max - 0.25 * z_range)      # Upper 25% in Z
    
    # Compute Task Space
    task_space_points = compute_task_space(
        workspace_points,
        x_limits=task_x_limits,
        y_limits=task_y_limits,
        z_limits=task_z_limits
    )
    
    # Generate visualizations
    if config_space_points.size > 0 and workspace_points.size > 0:
        print("\n" + "="*70)
        print("Generating Visualizations")
        print("="*70)
        
        print("\nCreating combined visualization...")
        print(f"  Configuration space: {len(config_space_points):,} points (green)")
        print(f"  Workspace: {len(workspace_points):,} points (grey)")
        if task_space_points.size > 0:
            print(f"  Task space: {len(task_space_points):,} points (orange-red)")
        
        # Pass task limits to ensure box uses exact bounds
        task_limits_tuple = (task_x_limits, task_y_limits, task_z_limits)
        plot_workspace_combined(robot, config_space_points, workspace_points, task_space_points, task_limits_tuple)
    
    print("\n" + "="*70)
    print("✓ Analysis complete!")
    print("="*70)
    print("\nSummary:")
    print(f"  Configuration Space: All positions reachable within physical limits")
    print(f"  Workspace: Positions reachable with operational constraints")
    print(f"  Task Space: Specific rectangular region for the task")
    print(f"  Relationship: Configuration Space ⊃ Workspace ⊃ Task Space")


if __name__ == "__main__":
    main()
