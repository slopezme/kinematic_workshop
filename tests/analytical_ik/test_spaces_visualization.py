import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def compute_configuration_space(robot, q1_range, q2_range, q3_range, resolution=20):
    """
    Compute the configuration space (C) - all possible joint configurations.
    
    :param robot: Robot3DOF_DH instance
    :param q1_range: [min, max] for q1 in radians
    :param q2_range: [min, max] for q2 in radians
    :param q3_range: [min, max] for q3 in meters
    :param resolution: Number of samples per joint
    :return: Array of joint configurations [q1, q2, q3]
    """
    print("\n" + "="*70)
    print("Computing Configuration Space (C)")
    print("="*70)
    
    q1_values = np.linspace(q1_range[0], q1_range[1], resolution)
    q2_values = np.linspace(q2_range[0], q2_range[1], resolution)
    q3_values = np.linspace(q3_range[0], q3_range[1], resolution)
    
    configurations = []
    total = resolution ** 3
    count = 0
    
    print(f"Sampling {resolution} values per joint...")
    print(f"Total configurations: {total:,}")
    
    for q1 in q1_values:
        for q2 in q2_values:
            for q3 in q3_values:
                configurations.append([q1, q2, q3])
                count += 1
                if count % 10000 == 0 or count == total:
                    print(f"  Progress: {count:,}/{total:,} ({100*count/total:.1f}%)")
    
    print(f"✓ Configuration space computed: {len(configurations):,} configurations")
    return np.array(configurations)


def compute_workspace(robot, configurations, constraints=None):
    """
    Compute the workspace (W) - all reachable end-effector positions.
    
    :param robot: Robot3DOF_DH instance
    :param configurations: Array of joint configurations
    :param constraints: Optional dict with joint constraints
    :return: Array of reachable positions [x, y, z]
    """
    print("\n" + "="*70)
    print("Computing Workspace (W)")
    print("="*70)
    
    workspace_points = []
    valid_configs = []
    
    total = len(configurations)
    
    if constraints:
        print("Applying constraints:")
        for joint, (min_val, max_val) in constraints.items():
            if joint in ['q1', 'q2']:
                print(f"  {joint}: [{np.rad2deg(min_val):.1f}°, {np.rad2deg(max_val):.1f}°]")
            else:
                print(f"  {joint}: [{min_val:.2f}m, {max_val:.2f}m]")
    else:
        print("No constraints applied (full workspace)")
    
    for i, config in enumerate(configurations):
        q1, q2, q3 = config
        
        # Apply constraints if provided
        if constraints:
            if 'q1' in constraints:
                if q1 < constraints['q1'][0] or q1 > constraints['q1'][1]:
                    continue
            if 'q2' in constraints:
                if q2 < constraints['q2'][0] or q2 > constraints['q2'][1]:
                    continue
            if 'q3' in constraints:
                if q3 < constraints['q3'][0] or q3 > constraints['q3'][1]:
                    continue
        
        # Compute forward kinematics
        transforms = robot.forward_kinematics(config)
        end_effector_pos = transforms[-1][:3, 3]
        workspace_points.append(end_effector_pos)
        valid_configs.append(config)
        
        if (i + 1) % 10000 == 0 or i + 1 == total:
            print(f"  Progress: {i+1:,}/{total:,} ({100*(i+1)/total:.1f}%)")
    
    workspace_points = np.array(workspace_points)
    print(f"✓ Workspace computed: {len(workspace_points):,} reachable points")
    
    return workspace_points, np.array(valid_configs)


def define_task_space(workspace_points, task_type='plane', task_params=None):
    """
    Define the task space (T) - specific subset of workspace for a given task.
    
    :param workspace_points: Array of workspace positions
    :param task_type: Type of task ('plane', 'box', 'cylinder')
    :param task_params: Parameters defining the task space
    :return: Array of task space positions
    """
    print("\n" + "="*70)
    print("Computing Task Space (T)")
    print("="*70)
    
    if task_type == 'plane':
        # Define a plane: z = constant, within x and y bounds
        z_plane = task_params.get('z', 1.5)
        x_range = task_params.get('x_range', [0.3, 0.8])
        y_range = task_params.get('y_range', [-0.3, 0.3])
        tolerance = task_params.get('tolerance', 0.05)
        
        print(f"Task: Planar region at z={z_plane}m")
        print(f"  X range: [{x_range[0]}, {x_range[1]}]")
        print(f"  Y range: [{y_range[0]}, {y_range[1]}]")
        print(f"  Z tolerance: ±{tolerance}m")
        
        # Filter points near the plane
        mask = (np.abs(workspace_points[:, 2] - z_plane) < tolerance) & \
               (workspace_points[:, 0] >= x_range[0]) & \
               (workspace_points[:, 0] <= x_range[1]) & \
               (workspace_points[:, 1] >= y_range[0]) & \
               (workspace_points[:, 1] <= y_range[1])
        
        task_points = workspace_points[mask]
        
    elif task_type == 'box':
        # Define a 3D box region
        x_range = task_params.get('x_range', [0.3, 0.7])
        y_range = task_params.get('y_range', [-0.2, 0.2])
        z_range = task_params.get('z_range', [1.3, 1.7])
        
        print(f"Task: Box region")
        print(f"  X: [{x_range[0]}, {x_range[1]}]")
        print(f"  Y: [{y_range[0]}, {y_range[1]}]")
        print(f"  Z: [{z_range[0]}, {z_range[1]}]")
        
        mask = (workspace_points[:, 0] >= x_range[0]) & \
               (workspace_points[:, 0] <= x_range[1]) & \
               (workspace_points[:, 1] >= y_range[0]) & \
               (workspace_points[:, 1] <= y_range[1]) & \
               (workspace_points[:, 2] >= z_range[0]) & \
               (workspace_points[:, 2] <= z_range[1])
        
        task_points = workspace_points[mask]
    
    else:
        task_points = workspace_points
    
    print(f"✓ Task space computed: {len(task_points):,} points")
    
    return task_points


def plot_separate_spaces(configurations, workspace_points, task_points, robot):
    """
    Create separate plots for Configuration Space, Workspace, and Task Space.
    """
    fig = plt.figure(figsize=(18, 6))
    
    # Plot 1: Configuration Space as filled voxel volume (physical joint limits)
    ax1 = fig.add_subplot(131, projection='3d')
    
    # Configuration space represents PHYSICAL joint limits
    # Create a 3D voxel grid to show the volume
    
    # Define the bounds
    q1_min, q1_max = -180, 180  # degrees
    q2_min, q2_max = -90, 90    # degrees
    q3_min, q3_max = 0, 1.5     # meters
    
    # Create voxel grid (lower resolution for visualization)
    voxel_res = 20
    q1_voxels = np.linspace(q1_min, q1_max, voxel_res)
    q2_voxels = np.linspace(q2_min, q2_max, voxel_res)
    q3_voxels = np.linspace(q3_min, q3_max, voxel_res)
    
    # Create filled voxel array (all True = filled volume)
    filled = np.ones((voxel_res-1, voxel_res-1, voxel_res-1), dtype=bool)
    
    # Create color array (blue with transparency)
    colors = np.zeros(filled.shape + (4,))
    colors[filled] = [0.2, 0.4, 0.8, 0.3]  # RGBA: blue with alpha=0.3
    
    # Plot voxels
    ax1.voxels(q1_voxels, q2_voxels, q3_voxels, filled, 
               facecolors=colors, edgecolor='k', linewidth=0.1)
    
    ax1.set_xlabel('q1 (degrees)', fontsize=10)
    ax1.set_ylabel('q2 (degrees)', fontsize=10)
    ax1.set_zlabel('q3 (meters)', fontsize=10)
    ax1.set_title('1. Configuration Space (C)\nJoint Values (Volume)', fontsize=12, fontweight='bold')
    ax1.set_xlim([q1_min, q1_max])
    ax1.set_ylim([q2_min, q2_max])
    ax1.set_zlim([q3_min, q3_max])
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Workspace (3D scatter of end-effector positions)
    ax2 = fig.add_subplot(132, projection='3d')
    
    # Sample workspace for visualization
    sample_size = min(5000, len(workspace_points))
    sample_indices = np.random.choice(len(workspace_points), sample_size, replace=False)
    sampled_workspace = workspace_points[sample_indices]
    
    ax2.scatter(sampled_workspace[:, 0], 
                sampled_workspace[:, 1], 
                sampled_workspace[:, 2],
                c='green', alpha=0.3, s=1)
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_zlabel('Z (m)', fontsize=10)
    ax2.set_title('2. Workspace (W)\nReachable Positions', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Task Space (3D scatter of task-specific positions)
    ax3 = fig.add_subplot(133, projection='3d')
    
    if len(task_points) > 0:
        ax3.scatter(task_points[:, 0], 
                    task_points[:, 1], 
                    task_points[:, 2],
                    c='red', alpha=0.5, s=2)
    ax3.set_xlabel('X (m)', fontsize=10)
    ax3.set_ylabel('Y (m)', fontsize=10)
    ax3.set_zlabel('Z (m)', fontsize=10)
    ax3.set_title('3. Task Space (T)\nTask-Specific Region', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('docs/spaces_separate.png', dpi=150, bbox_inches='tight')
    print("\n✓ Separate plots saved as 'docs/spaces_separate.png'")
    plt.show()


def plot_combined_spaces(configurations, workspace_points, task_points, robot):
    """
    Create a combined plot showing all three spaces in the same 3D view.
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Sample for visualization
    sample_size_config = min(2000, len(configurations))
    sample_indices_config = np.random.choice(len(configurations), sample_size_config, replace=False)
    sampled_configs = configurations[sample_indices_config]
    
    sample_size_workspace = min(3000, len(workspace_points))
    sample_indices_workspace = np.random.choice(len(workspace_points), sample_size_workspace, replace=False)
    sampled_workspace = workspace_points[sample_indices_workspace]
    
    # Plot workspace (green, transparent)
    ax.scatter(sampled_workspace[:, 0], 
               sampled_workspace[:, 1], 
               sampled_workspace[:, 2],
               c='green', alpha=0.2, s=1, label='Workspace (W)')
    
    # Plot task space (red, more opaque)
    if len(task_points) > 0:
        ax.scatter(task_points[:, 0], 
                   task_points[:, 1], 
                   task_points[:, 2],
                   c='red', alpha=0.6, s=3, label='Task Space (T)')
    
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title('Combined View: Workspace (W) and Task Space (T)\n' + 
                 'Configuration Space (C) maps to Workspace through Forward Kinematics',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('docs/spaces_combined.png', dpi=150, bbox_inches='tight')
    print("✓ Combined plot saved as 'docs/spaces_combined.png'")
    plt.show()


def main():
    """
    Main function to compute and visualize all three spaces.
    """
    print("\n" + "="*70)
    print("3DOF Robot: Configuration Space, Workspace, and Task Space Analysis")
    print("="*70)
    
    # Initialize robot
    robot = Robot3DOF_DH(L1=1.0)
    
    # Define PHYSICAL joint limits for Configuration Space (C)
    # These represent the mechanical/physical limits of the robot
    print("\nPhysical Joint Limits (Configuration Space):")
    q1_range = [-np.pi, np.pi]  # Full rotation: -180° to 180°
    q2_range = [-np.pi/2, np.pi/2]  # Tilt range: -90° to 90°
    q3_range = [0.0, 1.5]  # Extension: 0 to 1.5 meters
    print(f"  q1: [{np.rad2deg(q1_range[0]):.0f}°, {np.rad2deg(q1_range[1]):.0f}°]")
    print(f"  q2: [{np.rad2deg(q2_range[0]):.0f}°, {np.rad2deg(q2_range[1]):.0f}°]")
    print(f"  q3: [{q3_range[0]:.2f}m, {q3_range[1]:.2f}m]")
    
    # 1. Compute Configuration Space (C) - based on physical joint limits
    resolution = 15  # Adjust for speed vs. accuracy
    configurations = compute_configuration_space(robot, q1_range, q2_range, q3_range, resolution)
    
    # 2. Compute Workspace (W) - ALL reachable positions from configuration space
    # This is the FULL workspace based on physical limits
    print("\nComputing FULL Workspace from Configuration Space...")
    workspace_full, _ = compute_workspace(robot, configurations, constraints=None)
    
    # 3. Compute CONSTRAINED Workspace - with additional operational constraints
    # These constraints might be due to: obstacles, safety zones, operational requirements
    print("\nApplying Additional Operational Constraints...")
    constraints = {
        'q1': [-np.pi/2, np.pi/2],  # Operational limit: -90° to 90° (e.g., avoid cables)
        'q2': [-np.pi/3, np.pi/3],  # Operational limit: -60° to 60° (e.g., stability)
        'q3': [0.2, 1.2]  # Operational limit: 0.2m to 1.2m (e.g., avoid obstacles)
    }
    workspace_constrained, valid_configs = compute_workspace(robot, configurations, constraints)
    
    # 4. Define Task Space (T) - a planar region for a specific task
    task_params = {
        'z': 1.5,  # Plane at z = 1.5m
        'x_range': [0.3, 0.8],
        'y_range': [-0.3, 0.3],
        'tolerance': 0.08
    }
    task_points = define_task_space(workspace_constrained, task_type='plane', task_params=task_params)
    
    # Print summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"Configuration Space (C):     {len(configurations):,} configurations")
    print(f"  └─ Based on: Physical joint limits")
    print(f"\nWorkspace (W) - Full:        {len(workspace_full):,} reachable points")
    print(f"  └─ Computed from: All configurations in C")
    print(f"\nWorkspace (W) - Constrained: {len(workspace_constrained):,} reachable points")
    print(f"  └─ Additional constraints: Operational limits, obstacles, safety")
    print(f"\nTask Space (T):              {len(task_points):,} task-specific points")
    print(f"  └─ Subset of: Constrained workspace for specific task")
    print(f"\nRelationship: C → W (full) → W (constrained) → T")
    print(f"Dimensions: dim(C) = 3, dim(W) = 3, dim(T) = 2 (planar task)")
    print(f"Reachability condition: dim(C) ≥ dim(T) ✓")
    print("="*70)
    
    # Create visualizations
    plot_separate_spaces(configurations, workspace_constrained, task_points, robot)
    plot_combined_spaces(configurations, workspace_constrained, task_points, robot)
    
    print("\n✓ Analysis complete!")


if __name__ == "__main__":
    main()
