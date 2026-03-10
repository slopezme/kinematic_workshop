import sys
import os
import numpy as np

# Add parent directory to path to import tools module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from tools.robot_3dof_dh import Robot3DOF_DH


def test_analytical_ik():
    """
    Simple test of analytical inverse kinematics.
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
    
    for i, target in enumerate(test_positions):
        print(f"\n{'─' * 70}")
        print(f"Test {i+1}: Target Position = [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        print(f"{'─' * 70}")
        
        q_solution = robot.inverse_kinematics(target)
        success, error, pos_achieved = robot.verify_ik_solution(target, q_solution)
        
        print(f"\n  Analytical IK Solution:")
        print(f"    q1 = {np.rad2deg(q_solution[0]):8.3f}° ({q_solution[0]:7.4f} rad)")
        print(f"    q2 = {np.rad2deg(q_solution[1]):8.3f}° ({q_solution[1]:7.4f} rad)")
        print(f"    q3 = {q_solution[2]:8.4f} m")
        print(f"    Achieved: [{pos_achieved[0]:.6f}, {pos_achieved[1]:.6f}, {pos_achieved[2]:.6f}]")
        print(f"    Error: {error:.2e} m")
        print(f"    Status: {'✓ SUCCESS' if success else '✗ FAILED'}")
    
    print(f"\n{'=' * 70}")
    print("Test completed successfully!")
    print(f"{'=' * 70}\n")


if __name__ == "__main__":
    test_analytical_ik()
