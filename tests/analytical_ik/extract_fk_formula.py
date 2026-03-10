import numpy as np
import sympy as sp


def extract_position_formula():
    """Extract the position formula from DH matrices symbolically."""
    print("="*70)
    print("Extracting Position Formula from DH Matrices")
    print("="*70)
    
    # Define symbolic variables
    q1, q2, q3, L1 = sp.symbols('q1 q2 q3 L1', real=True)
    
    # T_0_1: theta=q1, d=L1, a=0, alpha=pi/2
    c1, s1 = sp.cos(q1), sp.sin(q1)
    T_0_1 = sp.Matrix([
        [c1, 0, s1, 0],
        [s1, 0, -c1, 0],
        [0, 1, 0, L1],
        [0, 0, 0, 1]
    ])
    
    # T_1_2: theta=q2, d=0, a=0, alpha=-pi/2
    c2, s2 = sp.cos(q2), sp.sin(q2)
    T_1_2 = sp.Matrix([
        [c2, 0, -s2, 0],
        [s2, 0, c2, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # T_2_3: theta=0, d=q3, a=0, alpha=0
    T_2_3 = sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, q3],
        [0, 0, 0, 1]
    ])
    
    print("\nComputing T_0_2 = T_0_1 @ T_1_2...")
    T_0_2 = T_0_1 * T_1_2
    T_0_2 = sp.simplify(T_0_2)
    
    print("\nComputing T_0_3 = T_0_2 @ T_2_3...")
    T_0_3 = T_0_2 * T_2_3
    T_0_3 = sp.simplify(T_0_3)
    
    print("\nFinal transformation matrix T_0_3:")
    sp.pprint(T_0_3)
    
    print("\n" + "="*70)
    print("End-Effector Position Formulas:")
    print("="*70)
    
    x = T_0_3[0, 3]
    y = T_0_3[1, 3]
    z = T_0_3[2, 3]
    
    print(f"\nx = {x}")
    print(f"y = {y}")
    print(f"z = {z}")
    
    # Simplify
    x_simplified = sp.simplify(x)
    y_simplified = sp.simplify(y)
    z_simplified = sp.simplify(z)
    
    print("\nSimplified:")
    print(f"x = {x_simplified}")
    print(f"y = {y_simplified}")
    print(f"z = {z_simplified}")
    
    return x_simplified, y_simplified, z_simplified


def verify_with_numerical():
    """Verify the formulas numerically."""
    print("\n" + "="*70)
    print("Numerical Verification")
    print("="*70)
    
    # Test case: q1=45°, q2=54.74°, q3=0.866m, L1=1.0
    q1_val = np.pi/4
    q2_val = np.arctan2(np.sqrt(2), 0.5)  # 54.74°
    q3_val = 0.8660254
    L1_val = 1.0
    
    print(f"\nTest values:")
    print(f"q1 = {np.rad2deg(q1_val):.2f}°")
    print(f"q2 = {np.rad2deg(q2_val):.2f}°")
    print(f"q3 = {q3_val:.6f}m")
    print(f"L1 = {L1_val}")
    
    # From symbolic derivation, the formulas should be:
    # x = -q3*sin(q2)*cos(q1)
    # y = -q3*sin(q2)*sin(q1)
    # z = L1 + q3*cos(q2)
    
    x = -q3_val * np.sin(q2_val) * np.cos(q1_val)
    y = -q3_val * np.sin(q2_val) * np.sin(q1_val)
    z = L1_val + q3_val * np.cos(q2_val)
    
    print(f"\nUsing formula: x = -q3*sin(q2)*cos(q1)")
    print(f"               y = -q3*sin(q2)*sin(q1)")
    print(f"               z = L1 + q3*cos(q2)")
    print(f"\nResult: [{x:.6f}, {y:.6f}, {z:.6f}]")
    
    # Now derive IK from these corrected FK equations
    print("\n" + "="*70)
    print("Deriving IK from Corrected FK")
    print("="*70)
    
    print("\nGiven:")
    print("  x = -q3*sin(q2)*cos(q1)")
    print("  y = -q3*sin(q2)*sin(q1)")
    print("  z = L1 + q3*cos(q2)")
    
    print("\nSolving for joint variables:")
    print("  From x and y: tan(q1) = y/x")
    print("  Therefore: q1 = atan2(y, x)")
    
    print("\n  From x² + y² = q3²*sin²(q2)")
    print("  And z - L1 = q3*cos(q2)")
    print("  Therefore: q3² = (x² + y²) + (z - L1)²")
    print("  So: q3 = sqrt(x² + y² + (z - L1)²)")
    
    print("\n  And: tan(q2) = sqrt(x² + y²) / (z - L1)")
    print("  But we need to account for the negative sign in FK")
    print("  So: q2 = atan2(-sqrt(x² + y²), -(z - L1))")
    print("  Or: q2 = atan2(sqrt(x² + y²), z - L1) + π")


if __name__ == "__main__":
    try:
        x_formula, y_formula, z_formula = extract_position_formula()
    except ImportError:
        print("sympy not installed, skipping symbolic derivation")
    
    verify_with_numerical()
