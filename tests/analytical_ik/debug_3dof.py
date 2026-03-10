import numpy as np
from tools.robot_3dof_dh import Robot3DOF_DH


def debug_forward_kinematics():
    """
    Debug the forward kinematics to understand the robot structure.
    """
    print("=" * 70)
    print("3DOF Robot - Forward Kinematics Debug")
    print("=" * 70)
    
    L1 = 1.0
    robot = Robot3DOF_DH(L1=L1)
    
    print(f"\nRobot Parameters:")
    print(f"  L1 = {L1}")
    
    print("\nDH Parameters:")
    print("┌───────┬──────────┬──────┬──────┬─────────┐")
    print("│   i   │  θ_i     │  d_i │  a_i │  α_i    │")
    print("├───────┼──────────┼──────┼──────┼─────────┤")
    print("│   1   │  q1*     │  L1  │   0  │  +90°   │")
    print("│   2   │  q2*     │   0  │   0  │  -90°   │")
    print("│   3   │   0°     │  q3* │   0  │   0°    │")
    print("└───────┴──────────┴──────┴──────┴─────────┘")
    
    # Test configuration: all zeros
    print("\n" + "─" * 70)
    print("Test 1: Zero Configuration [q1=0, q2=0, q3=0]")
    print("─" * 70)
    
    q = [0, 0, 0]
    transforms = robot.forward_kinematics(q)
    
    print("\nTransformation Matrices:")
    for i, T in enumerate(transforms):
        print(f"\nFrame {i} (relative to base):")
        print(f"  Position: [{T[0,3]:8.4f}, {T[1,3]:8.4f}, {T[2,3]:8.4f}]")
        print(f"  Rotation matrix:")
        for row in T[:3, :3]:
            print(f"    [{row[0]:7.4f}, {row[1]:7.4f}, {row[2]:7.4f}]")
    
    # Test configuration: q1=45°
    print("\n" + "─" * 70)
    print("Test 2: q1=45°, q2=0°, q3=0.5m")
    print("─" * 70)
    
    q = [np.pi/4, 0, 0.5]
    transforms = robot.forward_kinematics(q)
    
    print("\nJoint Positions:")
    for i, T in enumerate(transforms):
        pos = T[:3, 3]
        print(f"  Frame {i}: [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]")
    
    # Test configuration: q2=45°
    print("\n" + "─" * 70)
    print("Test 3: q1=0°, q2=45°, q3=1.0m")
    print("─" * 70)
    
    q = [0, np.pi/4, 1.0]
    transforms = robot.forward_kinematics(q)
    
    print("\nJoint Positions:")
    for i, T in enumerate(transforms):
        pos = T[:3, 3]
        print(f"  Frame {i}: [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]")
    
    # Test IK
    print("\n" + "─" * 70)
    print("Test 4: Inverse Kinematics")
    print("─" * 70)
    
    target = [0.5, 0.5, 1.5]
    print(f"\nTarget Position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    
    q_ik = robot.inverse_kinematics_algebraic(target)
    print(f"\nIK Solution:")
    print(f"  q1 = {np.rad2deg(q_ik[0]):7.2f}° ({q_ik[0]:7.4f} rad)")
    print(f"  q2 = {np.rad2deg(q_ik[1]):7.2f}° ({q_ik[1]:7.4f} rad)")
    print(f"  q3 = {q_ik[2]:7.4f} m")
    
    transforms_ik = robot.forward_kinematics(q_ik)
    achieved = transforms_ik[-1][:3, 3]
    error = np.linalg.norm(achieved - target)
    
    print(f"\nForward Kinematics of IK Solution:")
    print(f"  Achieved: [{achieved[0]:.6f}, {achieved[1]:.6f}, {achieved[2]:.6f}]")
    print(f"  Error: {error:.2e} m")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    debug_forward_kinematics()
