import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


def animate_movement(robot, initial_angles, final_angles, frames=200, interval=50):
    """
    Animates the robot's movement from an initial to a final joint configuration.
    """
    def update(frame):
        # Get joint angles for the current frame
        current_angles = joint_angles_over_time[frame]
        
        # Compute forward kinematics
        frame_transforms = robot.forward_kinematics(current_angles)
        joint_positions = [T[:3, 3] for T in frame_transforms]
        
        # Update link and joint positions
        x_coords = [p[0] for p in joint_positions]
        y_coords = [p[1] for p in joint_positions]
        z_coords = [p[2] for p in joint_positions]
        
        links.set_data_3d(x_coords, y_coords, z_coords)
        joints._offsets3d = (x_coords, y_coords, z_coords)

        # Update end-effector trace
        end_effector_positions.append(joint_positions[-1])
        trace_x = [p[0] for p in end_effector_positions]
        trace_y = [p[1] for p in end_effector_positions]
        trace_z = [p[2] for p in end_effector_positions]
        end_effector_trace.set_data_3d(trace_x, trace_y, trace_z)

        return links, joints, end_effector_trace
        
    fig = plt.figure()
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
    
    end_effector_positions = []

    ani = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False)
    plt.legend()
    plt.show()