import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def animate_3dof_trajectory(robot, start_pos, end_pos, num_steps=100, frames=200, interval=50, L2=0.3):
    """
    Animate the 3DOF robot following a trajectory from start to end position.
    Similar to the numerical IK test but using analytical IK.
    
    :param robot: Robot3DOF_DH instance
    :param start_pos: Starting [x, y, z] position
    :param end_pos: Ending [x, y, z] position
    :param num_steps: Number of points along the trajectory
    :param frames: Number of animation frames
    :param interval: Milliseconds between frames
    :param L2: Fixed base length of prismatic joint (shown in blue)
    """
    # Generate trajectory points (linear interpolation)
    path_points = np.linspace(start_pos, end_pos, num_steps)
    
    print("Calculating Inverse Kinematics for the path...")
    
    # Compute IK for all trajectory points using analytical solution
    all_joint_angles = []
    actual_path = []
    
    for i, target_pos in enumerate(path_points):
        print(f"Step {i+1}/{num_steps}: Target = [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")
        
        # Calculate analytical IK
        joint_values = robot.inverse_kinematics(target_pos)
        
        # Verify with FK
        transforms = robot.forward_kinematics(joint_values)
        achieved_pos = transforms[-1][:3, 3]
        error = np.linalg.norm(achieved_pos - target_pos)
        
        print(f"  -> Angles: {np.rad2deg(joint_values).round(1)} deg, Error: {error:.2e}")
        
        all_joint_angles.append(joint_values)
        actual_path.append(achieved_pos)
    
    all_joint_angles = np.array(all_joint_angles)
    actual_path = np.array(actual_path)
    
    print("\nAnimating the calculated robot movement...")
    
    # Create animation
    def update(frame):
        # Clear the end-effector path at the beginning
        if frame == 0:
            end_effector_positions.clear()
        
        # Get joint angles for the current frame
        current_angles = joint_angles_over_time[frame]
        
        # Compute forward kinematics
        frame_transforms = robot.forward_kinematics(current_angles)
        joint_positions = [T[:3, 3] for T in frame_transforms]
        
        # Update link and joint positions
        x_coords = [p[0] for p in joint_positions]
        y_coords = [p[1] for p in joint_positions]
        z_coords = [p[2] for p in joint_positions]
        
        # Calculate intermediate point for fixed base length L2 of prismatic joint
        # The prismatic joint extends from joint2 position
        joint2_pos = joint_positions[2]
        joint3_pos = joint_positions[3]
        
        # Direction vector from joint2 to joint3
        direction = joint3_pos - joint2_pos
        total_length = np.linalg.norm(direction)
        
        if total_length > 0:
            direction_normalized = direction / total_length
            # Fixed base point at L2 distance from joint2
            fixed_base_end = joint2_pos + direction_normalized * L2
        else:
            fixed_base_end = joint2_pos
        
        # Update base links (joints 0-2) in blue
        base_links.set_data_3d(x_coords[:3], y_coords[:3], z_coords[:3])
        
        # Update fixed base of prismatic joint (joint 2 to L2 distance) in blue
        prismatic_base.set_data_3d([joint2_pos[0], fixed_base_end[0]], 
                                    [joint2_pos[1], fixed_base_end[1]], 
                                    [joint2_pos[2], fixed_base_end[2]])
        
        # Update variable extension (from L2 to end-effector) in red
        prismatic_extension.set_data_3d([fixed_base_end[0], joint3_pos[0]], 
                                        [fixed_base_end[1], joint3_pos[1]], 
                                        [fixed_base_end[2], joint3_pos[2]])
        
        joints._offsets3d = (x_coords, y_coords, z_coords)
        
        # Update end-effector trace
        end_effector_positions.append(joint_positions[-1])
        trace_x = [p[0] for p in end_effector_positions]
        trace_y = [p[1] for p in end_effector_positions]
        trace_z = [p[2] for p in end_effector_positions]
        end_effector_trace.set_data_3d(trace_x, trace_y, trace_z)
        
        # Update table with current joint data
        for i, (pos, text_cell) in enumerate(zip(joint_positions, table_cells)):
            pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
            value_str = ""
            if i > 0:  # Joint 0 (base) has no value
                if i <= 2:  # Revolute joints (q1, q2) - show angles
                    angle_deg = np.rad2deg(current_angles[i-1])
                    value_str = f"{angle_deg:.1f}"
                else:  # Prismatic joint (q3) - show distance
                    value_str = f"{current_angles[2]:.3f}"
            text_cell[0].get_text().set_text(pos_str)
            text_cell[1].get_text().set_text(value_str)
        
        return base_links, prismatic_base, prismatic_extension, joints, end_effector_trace
    
    fig = plt.figure(figsize=(18, 9))
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1])
    ax = fig.add_subplot(gs[0], projection='3d')
    ax_table = fig.add_subplot(gs[1])
    ax_table.axis('off')
    
    # Interpolate joint angles over animation frames
    joint_angles_over_time = np.linspace(all_joint_angles[0], all_joint_angles[-1], frames)
    
    # Calculate initial positions to set plot limits
    initial_transforms = robot.forward_kinematics(all_joint_angles[0])
    initial_positions = [T[:3, 3] for T in initial_transforms]
    x_coords = [p[0] for p in initial_positions]
    y_coords = [p[1] for p in initial_positions]
    z_coords = [p[2] for p in initial_positions]
    
    # Set plot limits
    max_range = 2.5
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range * 1.5])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Kinematics for 3DOF Cylindrical Robot (Analytical IK)')
    ax.set_aspect('equal', adjustable='box')
    
    # Calculate initial intermediate point for visualization
    joint2_pos = initial_positions[2]
    joint3_pos = initial_positions[3]
    direction = joint3_pos - joint2_pos
    total_length = np.linalg.norm(direction)
    if total_length > 0:
        direction_normalized = direction / total_length
        fixed_base_end = joint2_pos + direction_normalized * L2
    else:
        fixed_base_end = joint2_pos
    
    # Initialize plot elements
    # Base links (blue) - from base to joint 2
    base_links, = ax.plot(x_coords[:3], y_coords[:3], z_coords[:3], 'b-', marker='o', 
                          linewidth=3, markersize=8, label='Base Links')
    # Fixed base of prismatic joint (blue) - L2 fixed length
    prismatic_base, = ax.plot([joint2_pos[0], fixed_base_end[0]], 
                              [joint2_pos[1], fixed_base_end[1]], 
                              [joint2_pos[2], fixed_base_end[2]], 
                              'b-', linewidth=3, label=f'Prismatic Base (L2={L2}m)')
    # Variable extension (red) - from L2 to end-effector
    prismatic_extension, = ax.plot([fixed_base_end[0], joint3_pos[0]], 
                                   [fixed_base_end[1], joint3_pos[1]], 
                                   [fixed_base_end[2], joint3_pos[2]], 
                                   'r-', linewidth=3, label='Prismatic Extension (q3)')
    joints = ax.scatter(x_coords, y_coords, z_coords, c='orange', s=100, label='Joints', edgecolors='black', linewidths=1)
    end_effector_trace, = ax.plot([], [], [], 'g--', linewidth=2, label='End-Effector Path')
    
    # Plot the target path start and end points
    ax.scatter(start_pos[0], start_pos[1], start_pos[2],
               color='green', marker='*', s=300, label='Path Start Target', edgecolors='black', linewidths=1)
    ax.scatter(end_pos[0], end_pos[1], end_pos[2],
               color='red', marker='*', s=300, label='Path End Target', edgecolors='black', linewidths=1)
    
    # Plot robot's initial end-effector position
    robot_initial_ee_pos = initial_positions[-1]
    ax.scatter(robot_initial_ee_pos[0], robot_initial_ee_pos[1], robot_initial_ee_pos[2],
               color='blue', marker='o', s=200, label='Robot Initial EE Pos', edgecolors='black', linewidths=1)
    
    end_effector_positions = []
    
    ax.legend(loc='upper left', fontsize=9)
    
    # Create table
    joint_names = ['base_link', 'joint1', 'joint2', 'joint3_ee']
    table_data = [[name, "", ""] for name in joint_names]
    table = ax_table.table(cellText=table_data, colLabels=['Joint', 'Position (x,y,z)', 'Angle (°) / Dist (m)'],
                          cellLoc='left', loc='center', colWidths=[0.3, 0.5, 0.25])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    # Store table cells for updating
    table_cells = []
    for i in range(len(joint_names)):
        row_cells = [table[(i+1, 1)], table[(i+1, 2)]]
        table_cells.append(row_cells)
    
    # Create animation
    anim = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False, repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    return anim


def main():
    """
    Main test function - animate 3DOF robot following a trajectory.
    """
    robot = Robot3DOF_DH(L1=1.0)
    
    # Define trajectory
    # Start at horizontal position with minimal extension, visible from default camera
    # Using a position in the positive X-Y quadrant for better visibility
    start_position = np.array([0.5, 0.5, robot.L1])  # Horizontal at 45°, minimal extension
    end_position = np.array([1.0, 0.0, 2.0])
    
    # Animate the trajectory
    anim = animate_3dof_trajectory(robot, start_position, end_position, num_steps=100, frames=200)


if __name__ == "__main__":
    main()
