import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec


def animate_movement(robot, initial_angles, final_angles, path_start_pos=None, path_end_pos=None, frames=200, interval=50):
    """
    Animates the robot's movement from an initial to a final joint configuration.
    """
    def update(frame):
        # Clear the end-effector path at the beginning of each animation cycle
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
        
        # Print end effector position
        # end_effector_pos = joint_positions[-1]
        # print(f"Frame {frame}: End Effector Position = [{end_effector_pos[0]:.2f}, {end_effector_pos[1]:.2f}, {end_effector_pos[2]:.2f}]")

        links.set_data_3d(x_coords, y_coords, z_coords)
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
            angle_str = ""
            if i > 0: # Joint 0 (base) has no angle
                angle_deg = np.rad2deg(current_angles[i-1])
                angle_str = f"{angle_deg:.1f}"
            text_cell[0].get_text().set_text(pos_str)
            text_cell[1].get_text().set_text(angle_str)

        return links, joints, end_effector_trace
        
    fig = plt.figure(figsize=(18, 9))
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1])
    ax = fig.add_subplot(gs[0], projection='3d')
    ax_table = fig.add_subplot(gs[1])
    ax_table.axis('off') # Hide the axes for the table subplot

    # Interpolate joint angles
    joint_angles_over_time = np.linspace(initial_angles, final_angles, frames)

    # Calculate initial positions to set plot limits
    initial_transforms = robot.forward_kinematics(initial_angles)
    initial_positions = [T[:3, 3] for T in initial_transforms]
    x_coords = [p[0] for p in initial_positions]
    y_coords = [p[1] for p in initial_positions]
    z_coords = [p[2] for p in initial_positions]

    # Set plot limits based on a rough estimate of the workspace
    max_range = np.sum([np.linalg.norm(joint['origin']['xyz']) for joint in robot.joints]) * 1.1
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title(f'Kinematics for {robot.name}')
    ax.set_aspect('equal', adjustable='box')

    # Initialize plot elements
    links, = ax.plot(x_coords, y_coords, z_coords, 'b-', marker='o', label='Robot Links')
    joints = ax.scatter(x_coords, y_coords, z_coords, c='r', s=100, label='Joints')
    end_effector_trace, = ax.plot([], [], [], 'g--', label='End-Effector Path')

    # Plot the overall target path start and end points if provided
    if path_start_pos is not None:
        ax.scatter(path_start_pos[0], path_start_pos[1], path_start_pos[2],
                   color='green', marker='*', s=300, label='Path Start Target', edgecolors='black', linewidths=1)
        # Also plot the robot's actual initial end-effector position (from initial_angles)
        robot_initial_ee_pos = initial_positions[-1]
        ax.scatter(robot_initial_ee_pos[0], robot_initial_ee_pos[1], robot_initial_ee_pos[2],
                   color='blue', marker='o', s=200, label='Robot Initial EE Pos', edgecolors='black', linewidths=1)
    if path_end_pos is not None:
        ax.scatter(path_end_pos[0], path_end_pos[1], path_end_pos[2],
                   color='red', marker='*', s=300, label='Path End Target', edgecolors='black', linewidths=1)

    end_effector_positions = []

    # --- Table Setup ---
    col_labels = ['Position (x,y,z)', 'Angle (°)']
    # Use actual joint names for table rows, starting with the base link.
    row_labels = ['base_link'] + [joint['name'] for joint in robot.joints]
    
    # Initialize table with data from initial position
    cell_text = []
    for i, pos in enumerate(initial_positions):
        pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        angle_str = ""
        if i > 0: # Joint 0 (base) has no angle
            angle_deg = np.rad2deg(initial_angles[i-1])
            angle_str = f"{angle_deg:.1f}"
        cell_text.append([pos_str, angle_str])

    table = ax_table.table(cellText=cell_text, rowLabels=row_labels, colLabels=col_labels, loc='center', cellLoc='left')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.8)
    
    # Correctly gather the cell objects for updating.
    table_cells = []
    # The table has len(row_labels) data rows (from index 1 to len(row_labels))
    for i in range(1, len(row_labels) + 1):
        table_cells.append([table.get_celld()[(i, 0)], table.get_celld()[(i, 1)]])

    # Set blit=True for performance, but it can be tricky with text artists. False is safer.
    ani = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False)
    # Create the legend using the artists from the axes
    ax.legend()
    plt.show()
    return ani

def plot_workspace(robot, workspace_points):
    """
    Creates a 3D scatter plot of the robot's workspace.

    :param robot: An instance of the RobotArm class (for naming the plot).
    :param workspace_points: A NumPy array of 3D points (Nx3).
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    x = workspace_points[:, 0]
    y = workspace_points[:, 1]
    z = workspace_points[:, 2]

    ax.scatter(x, y, z, c=z, cmap='viridis', s=1, alpha=0.5)

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title(f'Workspace for {robot.name}')
    ax.set_aspect('equal', adjustable='box')
    plt.show()