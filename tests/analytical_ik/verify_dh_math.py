import numpy as np


def manual_forward_kinematics(q1, q2, q3, L1=1.0):
    """
    Manually compute forward kinematics step by step to verify.
    
    DH Parameters from image:
    Joint 1: theta=q1, d=L1, a=0, alpha=90°
    Joint 2: theta=q2, d=0,  a=0, alpha=-90°
    Joint 3: theta=0,  d=q3, a=0, alpha=0°
    """
    print(f"\nManual FK Computation:")
    print(f"  q1 = {np.rad2deg(q1):.2f}°, q2 = {np.rad2deg(q2):.2f}°, q3 = {q3:.4f}m, L1 = {L1}")
    
    # T_0_1: Joint 1
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    T_0_1 = np.array([
        [c1,  0,  s1, 0],
        [s1,  0, -c1, 0],
        [0,   1,  0,  L1],
        [0,   0,  0,  1]
    ])
    print(f"\n  T_0_1 position: [{T_0_1[0,3]:.4f}, {T_0_1[1,3]:.4f}, {T_0_1[2,3]:.4f}]")
    
    # T_1_2: Joint 2
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    T_1_2 = np.array([
        [c2,  0, -s2, 0],
        [s2,  0,  c2, 0],
        [0,  -1,  0,  0],
        [0,   0,  0,  1]
    ])
    
    T_0_2 = T_0_1 @ T_1_2
    print(f"  T_0_2 position: [{T_0_2[0,3]:.4f}, {T_0_2[1,3]:.4f}, {T_0_2[2,3]:.4f}]")
    
    # T_2_3: Joint 3 (prismatic)
    T_2_3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, q3],
        [0, 0, 0, 1]
    ])
    
    T_0_3 = T_0_2 @ T_2_3
    print(f"  T_0_3 position: [{T_0_3[0,3]:.4f}, {T_0_3[1,3]:.4f}, {T_0_3[2,3]:.4f}]")
    
    return T_0_3[:3, 3]


def derive_ik_from_fk():
    """
    Derive the IK equations from the FK to verify correctness.
    """
    print("\n" + "=" * 70)
    print("Deriving IK from FK")
    print("=" * 70)
    
    print("\nFrom the DH parameters, the end-effector position is:")
    print("  px = q3 * sin(q2) * cos(q1)")
    print("  py = q3 * sin(q2) * sin(q1)")
    print("  pz = L1 + q3 * cos(q2)")
    
    print("\nSolving for joint variables:")
    print("  q1 = atan2(py, px)")
    print("  From px² + py² = q3² * sin²(q2)")
    print("  From pz = L1 + q3 * cos(q2)")
    print("  Therefore: q3 = sqrt(px² + py² + (pz - L1)²)")
    print("  And: q2 = atan2(sqrt(px² + py²), pz - L1)")


def test_ik_formulas():
    """
    Test different IK formula variations.
    """
    print("\n" + "=" * 70)
    print("Testing IK Formulas")
    print("=" * 70)
    
    L1 = 1.0
    target = [0.5, 0.5, 1.5]
    px, py, pz = target
    
    print(f"\nTarget: [{px}, {py}, {pz}]")
    
    # Formula 1: From your image (algebraic)
    q1_v1 = np.arctan2(py, px)
    q2_v1 = np.arctan2(np.sqrt(px**2 + py**2), L1 - pz)
    q3_v1 = np.sqrt(px**2 + py**2 + (pz - L1)**2)
    
    print(f"\nFormula 1 (from image - algebraic):")
    print(f"  q1 = atan2(py, px) = {np.rad2deg(q1_v1):.2f}°")
    print(f"  q2 = atan2(sqrt(px²+py²), L1-pz) = {np.rad2deg(q2_v1):.2f}°")
    print(f"  q3 = sqrt(px²+py²+(pz-L1)²) = {q3_v1:.4f}m")
    
    pos_v1 = manual_forward_kinematics(q1_v1, q2_v1, q3_v1, L1)
    error_v1 = np.linalg.norm(pos_v1 - target)
    print(f"  FK result: [{pos_v1[0]:.4f}, {pos_v1[1]:.4f}, {pos_v1[2]:.4f}]")
    print(f"  Error: {error_v1:.6f}")
    
    # Formula 2: Corrected version
    q1_v2 = np.arctan2(py, px)
    q2_v2 = np.arctan2(np.sqrt(px**2 + py**2), pz - L1)
    q3_v2 = np.sqrt(px**2 + py**2 + (pz - L1)**2)
    
    print(f"\nFormula 2 (corrected):")
    print(f"  q1 = atan2(py, px) = {np.rad2deg(q1_v2):.2f}°")
    print(f"  q2 = atan2(sqrt(px²+py²), pz-L1) = {np.rad2deg(q2_v2):.2f}°")
    print(f"  q3 = sqrt(px²+py²+(pz-L1)²) = {q3_v2:.4f}m")
    
    pos_v2 = manual_forward_kinematics(q1_v2, q2_v2, q3_v2, L1)
    error_v2 = np.linalg.norm(pos_v2 - target)
    print(f"  FK result: [{pos_v2[0]:.4f}, {pos_v2[1]:.4f}, {pos_v2[2]:.4f}]")
    print(f"  Error: {error_v2:.6f}")


if __name__ == "__main__":
    derive_ik_from_fk()
    test_ik_formulas()
