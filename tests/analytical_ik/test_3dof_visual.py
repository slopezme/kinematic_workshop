import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def plot_robot_with_frames(ax, robot, joint_values, color='b', label=''):
    """
    Plot the robot with coordinate frames at each joint.
    """
    transforms = robot.forward_kinematics(joint_values)
    
    positions = [T[:3, 3] for T in transforms]
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    
    # Plot kinematic chain
    ax.plot(xs, ys, zs, 'o-', color=color, linewidth=4, markersize=12, 
            label=label, markerfacecolor=color, markeredgecolor='black', markeredgewidth=2)
    
    # Mark base
    ax.scatter(xs[0], ys[0], zs[0], color='black', s=200, marker='s', 
               edgecolors='white', linewidths=2, zorder=10, label='Base')
    
    # Mark end-effector
    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=250, marker='*', 
               edgecolors='black', linewidths=2, zorder=10, label='End-Effector')
    
    # Show coordinate frames
    frame_scale = 0.4
    for i, T in enumerate(transforms):
        origin = T[:3, 3]
        x_axis = T[:3, 0] * frame_scale
        y_axis = T[:3, 1] * frame_scale
        z_axis = T[:3, 2] * frame_scale
        
        # X-axis (red), Y-axis (green), Z-axis (blue)
        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], 
                 color='red', alpha=0.7, arrow_length_ratio=0.2, linewidth=2)
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], 
                 color='green', alpha=0.7, arrow_length_ratio=0.2, linewidth=2)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], 
                 color='blue', alpha=0.7, arrow_length_ratio=0.2, linewidth=2)
    
    # Print joint positions for debugging
    print(f"\n{label} Joint Positions:")
    for i, pos in enumerate(positions):
        print(f"  Joint {i}: [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}]")
    
    return positions[-1]


def test_forward_kinematics_visualization():
    """
    Test forward kinematics with simple configurations.
    """
    print("=" * 70)
    print("3DOF Robot - Forward Kinematics Visualization Test")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    # Test configurations
    configs = [
        {'q': [0, 0, 0], 'name': 'Zero Configuration'},
        {'q': [np.pi/4, 0, 0.5], 'name': 'q1=45°, q2=0°, q3=0.5m'},
        {'q': [0, np.pi/4, 1.0], 'name': 'q1=0°, q2=45°, q3=1.0m'},
        {'q': [np.pi/4, np.pi/4, 0.8], 'name': 'q1=45°, q2=45°, q3=0.8m'}
    ]
    
    fig = plt.figure(figsize=(16, 4))
    
    for i, config in enumerate(configs):
        ax = fig.add_subplot(1, 4, i+1, projection='3d')
        
        q = config['q']
        name = config['name']
        
        print(f"\n{'─' * 70}")
        print(f"Configuration: {name}")
        print(f"  q1 = {np.rad2deg(q[0]):7.2f}° ({q[0]:7.4f} rad)")
        print(f"  q2 = {np.rad2deg(q[1]):7.2f}° ({q[1]:7.4f} rad)")
        print(f"  q3 = {q[2]:7.4f} m")
        
        end_pos = plot_robot_with_frames(ax, robot, q, color='blue', label='Robot')
        
        ax.set_xlabel('X', fontsize=10)
        ax.set_ylabel('Y', fontsize=10)
        ax.set_zlabel('Z', fontsize=10)
        ax.set_title(name, fontsize=10, fontweight='bold')
        
        max_range = 2.5
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
        ax.set_box_aspect([1,1,1])
        
        # Add grid
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('3dof_forward_kinematics_test.png', dpi=150, bbox_inches='tight')
    print(f"\n{'=' * 70}")
    print("Visualization saved as '3dof_forward_kinematics_test.png'")
    print("=" * 70)
    plt.show()


def test_ik_visualization():
    """
    Test IK solutions with visualization.
    """
    print("\n" + "=" * 70)
    print("3DOF Robot - Inverse Kinematics Visualization")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    targets = [
        [0.5, 0.5, 1.5],
        [0.0, 1.0, 0.5],
        [1.0, 0.0, 2.0]
    ]
    
    fig = plt.figure(figsize=(15, 5))
    
    for i, target in enumerate(targets):
        ax = fig.add_subplot(1, 3, i+1, projection='3d')
        
        print(f"\n{'─' * 70}")
        print(f"Target {i+1}: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        
        # Analytical IK solution
        q_ik = robot.inverse_kinematics(target)
        success, error, pos_achieved = robot.verify_ik_solution(target, q_ik)
        
        print(f"\nAnalytical IK Solution:")
        print(f"  q1 = {np.rad2deg(q_ik[0]):7.2f}°, q2 = {np.rad2deg(q_ik[1]):7.2f}°, q3 = {q_ik[2]:.4f}m")
        print(f"  Error: {error:.2e} m - {'✓' if success else '✗'}")
        
        plot_robot_with_frames(ax, robot, q_ik, color='blue', label='Robot')
        
        # Plot target
        ax.scatter(*target, color='green', s=300, marker='*', 
                  label='Target', zorder=15, edgecolors='black', linewidths=2)
        
        ax.set_xlabel('X', fontsize=10)
        ax.set_ylabel('Y', fontsize=10)
        ax.set_zlabel('Z', fontsize=10)
        ax.set_title(f'Target: [{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}]', 
                    fontsize=10, fontweight='bold')
        ax.legend(fontsize=8)
        
        max_range = 2.5
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
        ax.set_box_aspect([1,1,1])
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('3dof_ik_visualization.png', dpi=150, bbox_inches='tight')
    print(f"\n{'=' * 70}")
    print("Visualization saved as '3dof_ik_visualization.png'")
    print("=" * 70)
    plt.show()


if __name__ == "__main__":
    test_forward_kinematics_visualization()
    test_ik_visualization()
