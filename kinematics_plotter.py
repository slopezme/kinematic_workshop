import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


def animate_movement(robot, initial_angles, final_angles, frames=200, interval=50):
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
        end_effector_pos = joint_positions[-1]
        print(f"Frame {frame}: End Effector Position = [{end_effector_pos[0]:.2f}, {end_effector_pos[1]:.2f}, {end_effector_pos[2]:.2f}]")

        links.set_data_3d(x_coords, y_coords, z_coords)
        joints._offsets3d = (x_coords, y_coords, z_coords)

        # Update end-effector trace
        end_effector_positions.append(joint_positions[-1])
        trace_x = [p[0] for p in end_effector_positions]
        trace_y = [p[1] for p in end_effector_positions]
        trace_z = [p[2] for p in end_effector_positions]
        end_effector_trace.set_data_3d(trace_x, trace_y, trace_z)

        # Update joint position text
        for i, pos in enumerate(joint_positions):
            joint_texts[i].set_position((pos[0], pos[1]))
            if i > 0: # Joint 0 is the base, it has no angle
                angle_deg = np.rad2deg(current_angles[i-1])
                joint_texts[i].set_text(f"  J{i}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n  Angle: {angle_deg:.1f}°")


        return links, joints, end_effector_trace, *joint_texts
        
    fig = plt.figure(figsize=(12, 10)) # Set a bigger figure size
    ax = fig.add_subplot(111, projection='3d')

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

    # Initialize text annotations for joint positions
    joint_texts = []
    for i, pos in enumerate(initial_positions):
        if i > 0:
            angle_deg = np.rad2deg(initial_angles[i-1])
            text = ax.text(pos[0], pos[1], pos[2], f"  J{i}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n  Angle: {angle_deg:.1f}°", fontsize=8)
        else: # For the base (J0)
            text = ax.text(pos[0], pos[1], pos[2], f"  J{i}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})", fontsize=8)
        joint_texts.append(text)
    
    end_effector_positions = []

    # Set blit=True for performance, but it can be tricky with text artists. False is safer.
    ani = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False) 
    plt.legend()
    plt.show()