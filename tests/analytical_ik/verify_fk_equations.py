import numpy as np


def dh_transform(theta, d, a, alpha):
    """Standard DH transformation."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])
    return T


def manual_fk(q1, q2, q3, L1=1.0):
    """Manually compute FK step by step."""
    print(f"\nComputing FK for q1={np.rad2deg(q1):.2f}°, q2={np.rad2deg(q2):.2f}°, q3={q3:.4f}m")
    
    # Joint 1: theta=q1, d=L1, a=0, alpha=90°
    T_0_1 = dh_transform(q1, L1, 0, np.pi/2)
    print(f"\nT_0_1:")
    print(T_0_1)
    print(f"Position after joint 1: [{T_0_1[0,3]:.4f}, {T_0_1[1,3]:.4f}, {T_0_1[2,3]:.4f}]")
    
    # Joint 2: theta=q2, d=0, a=0, alpha=-90°
    T_1_2 = dh_transform(q2, 0, 0, -np.pi/2)
    print(f"\nT_1_2:")
    print(T_1_2)
    
    T_0_2 = T_0_1 @ T_1_2
    print(f"\nT_0_2 = T_0_1 @ T_1_2:")
    print(T_0_2)
    print(f"Position after joint 2: [{T_0_2[0,3]:.4f}, {T_0_2[1,3]:.4f}, {T_0_2[2,3]:.4f}]")
    
    # Joint 3: theta=0, d=q3, a=0, alpha=0°
    T_2_3 = dh_transform(0, q3, 0, 0)
    print(f"\nT_2_3:")
    print(T_2_3)
    
    T_0_3 = T_0_2 @ T_2_3
    print(f"\nT_0_3 = T_0_2 @ T_2_3:")
    print(T_0_3)
    print(f"Final position: [{T_0_3[0,3]:.4f}, {T_0_3[1,3]:.4f}, {T_0_3[2,3]:.4f}]")
    
    return T_0_3[:3, 3]


def derive_fk_equations():
    """Derive the FK equations symbolically."""
    print("\n" + "="*70)
    print("Deriving FK Equations from DH Matrices")
    print("="*70)
    
    print("\nJoint 1: theta=q1, d=L1, a=0, alpha=90°")
    print("T_0_1 = [cos(q1)  0   sin(q1)  0  ]")
    print("        [sin(q1)  0  -cos(q1)  0  ]")
    print("        [  0      1     0      L1 ]")
    print("        [  0      0     0      1  ]")
    
    print("\nJoint 2: theta=q2, d=0, a=0, alpha=-90°")
    print("T_1_2 = [cos(q2)  0  -sin(q2)  0]")
    print("        [sin(q2)  0   cos(q2)  0]")
    print("        [  0     -1     0      0]")
    print("        [  0      0     0      1]")
    
    print("\nJoint 3: theta=0, d=q3, a=0, alpha=0°")
    print("T_2_3 = [1  0  0  0 ]")
    print("        [0  1  0  0 ]")
    print("        [0  0  1  q3]")
    print("        [0  0  0  1 ]")
    
    print("\nMultiplying T_0_1 @ T_1_2 @ T_2_3...")
    print("The final position [x, y, z] should be extracted from T_0_3[0:3, 3]")


def test_specific_cases():
    """Test specific cases to understand the FK."""
    print("\n" + "="*70)
    print("Testing Specific Cases")
    print("="*70)
    
    # Test 1: q1=45°, q2=0°, q3=1.0m
    print("\n" + "-"*70)
    print("Test 1: q1=45°, q2=0°, q3=1.0m")
    print("-"*70)
    pos1 = manual_fk(np.pi/4, 0, 1.0, L1=1.0)
    print(f"\nExpected: x should be positive, y should be positive")
    print(f"Got: [{pos1[0]:.4f}, {pos1[1]:.4f}, {pos1[2]:.4f}]")
    
    # Test 2: q1=0°, q2=45°, q3=1.0m
    print("\n" + "-"*70)
    print("Test 2: q1=0°, q2=45°, q3=1.0m")
    print("-"*70)
    pos2 = manual_fk(0, np.pi/4, 1.0, L1=1.0)
    print(f"\nExpected: x should be positive (tilted forward)")
    print(f"Got: [{pos2[0]:.4f}, {pos2[1]:.4f}, {pos2[2]:.4f}]")
    
    # Test 3: Try to reach [0.5, 0.5, 1.5]
    print("\n" + "-"*70)
    print("Test 3: Trying to reach target [0.5, 0.5, 1.5]")
    print("-"*70)
    
    target = [0.5, 0.5, 1.5]
    L1 = 1.0
    
    # Try the IK formula
    x, y, z = target
    q1_ik = np.arctan2(y, x)
    q2_ik = np.arctan2(np.sqrt(x**2 + y**2), z - L1)
    q3_ik = np.sqrt(x**2 + y**2 + (z - L1)**2)
    
    print(f"IK solution: q1={np.rad2deg(q1_ik):.2f}°, q2={np.rad2deg(q2_ik):.2f}°, q3={q3_ik:.4f}m")
    
    pos3 = manual_fk(q1_ik, q2_ik, q3_ik, L1)
    error = np.linalg.norm(pos3 - target)
    print(f"\nTarget:   [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")
    print(f"Achieved: [{pos3[0]:.4f}, {pos3[1]:.4f}, {pos3[2]:.4f}]")
    print(f"Error: {error:.6f}m")


if __name__ == "__main__":
    derive_fk_equations()
    test_specific_cases()
