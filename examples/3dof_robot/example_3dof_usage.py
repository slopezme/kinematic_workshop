import sys
import os
import numpy as np

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def example_basic_usage():
    """
    Basic usage example of the 3DOF robot with analytical IK.
    """
    print("\n" + "=" * 70)
    print("Example 1: Basic Usage")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    target_position = [0.5, 0.5, 1.5]
    print(f"\nTarget position: {target_position}")
    
    joint_values = robot.inverse_kinematics(target_position)
    print(f"\nJoint values:")
    print(f"  q1 = {np.rad2deg(joint_values[0]):.2f}° ({joint_values[0]:.4f} rad)")
    print(f"  q2 = {np.rad2deg(joint_values[1]):.2f}° ({joint_values[1]:.4f} rad)")
    print(f"  q3 = {joint_values[2]:.4f} m")
    
    transforms = robot.forward_kinematics(joint_values)
    achieved_position = transforms[-1][:3, 3]
    print(f"\nAchieved position: [{achieved_position[0]:.6f}, {achieved_position[1]:.6f}, {achieved_position[2]:.6f}]")
    print(f"Error: {np.linalg.norm(achieved_position - target_position):.2e} m")


def example_multiple_targets():
    """
    Test IK with multiple target positions.
    """
    print("\n" + "=" * 70)
    print("Example 2: Multiple Target Positions")
    print("=" * 70)
    
    robot = Robot3DOF_DH(L1=1.0)
    targets = [
        [0.7, 0.3, 1.8],
        [0.0, 0.5, 1.0],
        [1.0, 1.0, 2.0]
    ]
    
    for i, target in enumerate(targets):
        print(f"\nTarget {i+1}: {target}")
        q = robot.inverse_kinematics(target)
        success, error, achieved = robot.verify_ik_solution(target, q)
        
        print(f"  Joint values: q1={np.rad2deg(q[0]):.2f}°, q2={np.rad2deg(q[1]):.2f}°, q3={q[2]:.4f}m")
        print(f"  Error: {error:.2e} m - {'✓' if success else '✗'}")


def example_compare_methods():
    """
    Compare analytical and numerical IK methods.
    """
    print("\n" + "=" * 70)
    print("Example 3: Compare Analytical and Numerical IK Methods")
    print("=" * 70)
    
    robot = Robot3DOF_DH(L1=1.0)
    target = [0.5, 0.5, 1.5]
    
    print(f"\nTarget Position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    
    q_analytical = robot.inverse_kinematics(target)
    success_ana, error_ana, pos_ana = robot.verify_ik_solution(target, q_analytical)
    
    print(f"\nAnalytical IK Solution:")
    print(f"  Joint values: q1={np.rad2deg(q_analytical[0]):.2f}°, "
          f"q2={np.rad2deg(q_analytical[1]):.2f}°, q3={q_analytical[2]:.4f}m")
    print(f"  Error: {error_ana:.2e} m")
    print(f"  Computation: Instant (closed-form)")
    
    print(f"\nNote: Analytical IK provides exact solutions without iteration,")
    print(f"      making it much faster and more reliable than numerical methods.")


def example_forward_kinematics():
    """
    Example of forward kinematics computation.
    """
    print("\n" + "=" * 70)
    print("Example 4: Forward Kinematics")
    print("=" * 70)
    
    robot = Robot3DOF_DH(L1=1.0)
    
    joint_values = [np.pi/4, np.pi/6, 1.2]
    print(f"\nJoint configuration:")
    print(f"  q1 = {np.rad2deg(joint_values[0]):.2f}°")
    print(f"  q2 = {np.rad2deg(joint_values[1]):.2f}°")
    print(f"  q3 = {joint_values[2]:.4f} m")
    
    transforms = robot.forward_kinematics(joint_values)
    
    print(f"\nTransformation matrices:")
    for i, T in enumerate(transforms):
        print(f"\nFrame {i}:")
        print(f"  Position: [{T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f}]")


def example_ik_verification():
    """
    Example showing IK solution verification.
    """
    print("\n" + "=" * 70)
    print("Example 4: IK Solution Verification")
    print("=" * 70)
    
    robot = Robot3DOF_DH(L1=1.0)
    
    targets = [
        [0.5, 0.5, 1.5],
        [1.0, 0.0, 2.0],
        [0.0, 0.8, 0.7]
    ]
    
    for i, target in enumerate(targets):
        print(f"\n--- Target {i+1}: {target} ---")
        
        q = robot.inverse_kinematics(target)
        success, error, achieved = robot.verify_ik_solution(target, q)
        
        print(f"  Joint values: q1={np.rad2deg(q[0]):.2f}°, q2={np.rad2deg(q[1]):.2f}°, q3={q[2]:.4f}m")
        print(f"  Achieved: [{achieved[0]:.6f}, {achieved[1]:.6f}, {achieved[2]:.6f}]")
        print(f"  Error: {error:.2e} m")
        print(f"  Status: {'✓ SUCCESS' if success else '✗ FAILED'}")


def example_dh_parameters():
    """
    Display the DH parameters used by the robot.
    """
    print("\n" + "=" * 70)
    print("Example 5: DH Parameters")
    print("=" * 70)
    
    print("\nDenavit-Hartenberg Parameters:")
    print("\n┌───────┬──────────┬──────┬──────┬─────────┐")
    print("│   i   │  θ_i     │  d_i │  a_i │  α_i    │")
    print("├───────┼──────────┼──────┼──────┼─────────┤")
    print("│   1   │  q1*     │  L1  │   0  │  +90°   │")
    print("│   2   │  q2*     │   0  │   0  │  -90°   │")
    print("│   3   │   0°     │  q3* │   0  │   0°    │")
    print("└───────┴──────────┴──────┴──────┴─────────┘")
    print("\n* indicates variable joint parameter")
    
    print("\nInverse Kinematics Formulas:")
    print("\nAnalytical Solution:")
    print("  q1 = atan2(y, x)")
    print("  q2 = atan2(-√(x² + y²), z - L1)")
    print("  q3 = √(x² + y² + (z - L1)²)")


def main():
    """
    Run all examples.
    """
    print("\n╔" + "═" * 68 + "╗")
    print("║" + "  3DOF Robot - Analytical IK Examples".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    
    example_dh_parameters()
    example_basic_usage()
    example_multiple_targets()
    example_forward_kinematics()
    example_ik_verification()
    compare_with_numerical_ik()
    
    print("\n" + "=" * 70)
    print("All examples completed!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
