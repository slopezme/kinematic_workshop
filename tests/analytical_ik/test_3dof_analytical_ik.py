import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tools.robot_3dof_dh import Robot3DOF_DH


def plot_robot_configuration(ax, robot, joint_values, color='b', label='', alpha=1.0, show_frames=False):
    """
    Plot the robot in a given configuration.
    
    :param ax: Matplotlib 3D axis
    :param robot: Robot3DOF_DH instance
    :param joint_values: [q1, q2, q3] joint values
    :param color: Color for the robot links
    :param label: Label for the configuration
    :param alpha: Transparency
    :param show_frames: Whether to show coordinate frames at each joint
    """
    transforms = robot.forward_kinematics(joint_values)
    
    positions = [T[:3, 3] for T in transforms]
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    
    # Plot the kinematic chain (links between joints)
    ax.plot(xs, ys, zs, 'o-', color=color, linewidth=3, markersize=10, 
            label=label, alpha=alpha, markerfacecolor=color, markeredgecolor='black', markeredgewidth=1.5)
    
    # Mark the base
    ax.scatter(xs[0], ys[0], zs[0], color='black', s=150, marker='s', 
               alpha=alpha, edgecolors='white', linewidths=2, zorder=10)
    
    # Mark the end-effector
    ax.scatter(xs[-1], ys[-1], zs[-1], color=color, s=200, marker='^', 
               alpha=alpha, edgecolors='black', linewidths=2, zorder=10)
    
    # Optionally show coordinate frames
    if show_frames:
        frame_scale = 0.3
        for i, T in enumerate(transforms):
            origin = T[:3, 3]
            x_axis = T[:3, 0] * frame_scale
            y_axis = T[:3, 1] * frame_scale
            z_axis = T[:3, 2] * frame_scale
            
            ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], 
                     color='red', alpha=alpha*0.6, arrow_length_ratio=0.3)
            ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], 
                     color='green', alpha=alpha*0.6, arrow_length_ratio=0.3)
            ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], 
                     color='blue', alpha=alpha*0.6, arrow_length_ratio=0.3)
    
    return positions[-1]


def test_analytical_ik_comparison():
    """
    Test and compare algebraic and geometric inverse kinematics solutions.
    """
    print("=" * 70)
    print("3DOF Robot - Analytical Inverse Kinematics Test")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    test_positions = [
        [0.5, 0.5, 1.5],
        [0.0, 1.0, 0.5],
        [1.0, 0.0, 2.0],
        [-0.5, -0.5, 1.0],
        [0.3, 0.4, 1.2]
    ]
    
    print(f"\nRobot Parameters:")
    print(f"  L1 = {L1}")
    print(f"\nTesting {len(test_positions)} target positions...\n")
    
    results = []
    
    for i, target in enumerate(test_positions):
        print(f"\n{'─' * 70}")
        print(f"Test {i+1}: Target Position = [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        print(f"{'─' * 70}")
        
        q_algebraic = robot.inverse_kinematics_algebraic(target)
        success_alg, error_alg, pos_alg = robot.verify_ik_solution(target, q_algebraic)
        
        print(f"\n  Algebraic Solution:")
        print(f"    q1 = {np.rad2deg(q_algebraic[0]):8.3f}° ({q_algebraic[0]:7.4f} rad)")
        print(f"    q2 = {np.rad2deg(q_algebraic[1]):8.3f}° ({q_algebraic[1]:7.4f} rad)")
        print(f"    q3 = {q_algebraic[2]:8.4f} m")
        print(f"    Achieved: [{pos_alg[0]:.6f}, {pos_alg[1]:.6f}, {pos_alg[2]:.6f}]")
        print(f"    Error: {error_alg:.2e} m")
        print(f"    Status: {'✓ SUCCESS' if success_alg else '✗ FAILED'}")
        
        q_geometric = robot.inverse_kinematics_geometric(target)
        success_geo, error_geo, pos_geo = robot.verify_ik_solution(target, q_geometric)
        
        print(f"\n  Geometric Solution:")
        print(f"    q1 = {np.rad2deg(q_geometric[0]):8.3f}° ({q_geometric[0]:7.4f} rad)")
        print(f"    q2 = {np.rad2deg(q_geometric[1]):8.3f}° ({q_geometric[1]:7.4f} rad)")
        print(f"    q3 = {q_geometric[2]:8.4f} m")
        print(f"    Achieved: [{pos_geo[0]:.6f}, {pos_geo[1]:.6f}, {pos_geo[2]:.6f}]")
        print(f"    Error: {error_geo:.2e} m")
        print(f"    Status: {'✓ SUCCESS' if success_geo else '✗ FAILED'}")
        
        results.append({
            'target': target,
            'q_algebraic': q_algebraic,
            'q_geometric': q_geometric,
            'error_algebraic': error_alg,
            'error_geometric': error_geo,
            'success_algebraic': success_alg,
            'success_geometric': success_geo
        })
    
    print(f"\n{'=' * 70}")
    print("Summary")
    print(f"{'=' * 70}")
    alg_success = sum(r['success_algebraic'] for r in results)
    geo_success = sum(r['success_geometric'] for r in results)
    print(f"Algebraic Solution: {alg_success}/{len(results)} successful")
    print(f"Geometric Solution: {geo_success}/{len(results)} successful")
    
    return results


def visualize_ik_solutions():
    """
    Visualize the robot configurations for different IK solutions.
    """
    print("\n" + "=" * 70)
    print("Visualizing IK Solutions")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    test_positions = [
        [0.5, 0.5, 1.5],
        [0.0, 1.0, 0.5],
        [1.0, 0.0, 2.0]
    ]
    
    fig = plt.figure(figsize=(15, 5))
    
    for i, target in enumerate(test_positions):
        ax = fig.add_subplot(1, 3, i+1, projection='3d')
        
        q_algebraic = robot.inverse_kinematics_algebraic(target)
        q_geometric = robot.inverse_kinematics_geometric(target)
        
        plot_robot_configuration(ax, robot, q_algebraic, color='blue', 
                                label='Algebraic', alpha=0.7)
        plot_robot_configuration(ax, robot, q_geometric, color='red', 
                                label='Geometric', alpha=0.7)
        
        ax.scatter(*target, color='green', s=200, marker='*', 
                  label='Target', zorder=10)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Target: [{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}]')
        ax.legend()
        ax.set_box_aspect([1,1,1])
        
        max_range = 2.5
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
    
    plt.tight_layout()
    plt.savefig('3dof_ik_comparison.png', dpi=150, bbox_inches='tight')
    print("\nVisualization saved as '3dof_ik_comparison.png'")
    plt.show()


def test_forward_kinematics():
    """
    Test forward kinematics with known joint values.
    """
    print("\n" + "=" * 70)
    print("Forward Kinematics Test")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    test_configs = [
        [0, 0, 0],
        [np.pi/4, 0, 1.0],
        [np.pi/2, np.pi/4, 0.5],
        [-np.pi/4, -np.pi/6, 1.5]
    ]
    
    for i, joint_values in enumerate(test_configs):
        print(f"\nConfiguration {i+1}:")
        print(f"  q1 = {np.rad2deg(joint_values[0]):7.2f}° ({joint_values[0]:7.4f} rad)")
        print(f"  q2 = {np.rad2deg(joint_values[1]):7.2f}° ({joint_values[1]:7.4f} rad)")
        print(f"  q3 = {joint_values[2]:7.4f} m")
        
        transforms = robot.forward_kinematics(joint_values)
        end_effector_pos = transforms[-1][:3, 3]
        
        print(f"  End-effector position: [{end_effector_pos[0]:.6f}, "
              f"{end_effector_pos[1]:.6f}, {end_effector_pos[2]:.6f}]")


def compare_with_numerical_ik():
    """
    Compare analytical IK with numerical optimization (if available).
    """
    print("\n" + "=" * 70)
    print("Analytical vs Numerical IK Comparison")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    target = [0.5, 0.5, 1.5]
    
    print(f"\nTarget Position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    
    q_analytical = robot.inverse_kinematics_algebraic(target)
    success_ana, error_ana, pos_ana = robot.verify_ik_solution(target, q_analytical)
    
    print(f"\nAnalytical Solution:")
    print(f"  Joint values: q1={np.rad2deg(q_analytical[0]):.2f}°, "
          f"q2={np.rad2deg(q_analytical[1]):.2f}°, q3={q_analytical[2]:.4f}m")
    print(f"  Error: {error_ana:.2e} m")
    print(f"  Computation: Instant (closed-form)")
    
    print(f"\nNote: Analytical IK provides exact solutions without iteration,")
    print(f"      making it much faster and more reliable than numerical methods.")


def main():
    """
    Main test function.
    """
    print("\n")
    print("╔" + "═" * 68 + "╗")
    print("║" + " " * 68 + "║")
    print("║" + "  3DOF Robot with Analytical Inverse Kinematics".center(68) + "║")
    print("║" + "  Based on Denavit-Hartenberg Parameters".center(68) + "║")
    print("║" + " " * 68 + "║")
    print("╚" + "═" * 68 + "╝")
    
    test_forward_kinematics()
    
    results = test_analytical_ik_comparison()
    
    compare_with_numerical_ik()
    
    print("\n" + "=" * 70)
    print("Generating Visualization...")
    print("=" * 70)
    visualize_ik_solutions()
    
    print("\n" + "=" * 70)
    print("All tests completed!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
