# 3DOF Cylindrical Robot - Implementation Notes

## Robot Structure

This is a **3-DOF cylindrical robot** with:
- 2 revolute joints (q1, q2) 
- 1 prismatic joint (q3)

The robot is **NOT planar** - it operates in 3D space with a cylindrical workspace.

## Key Implementation Details

### Forward Kinematics

The DH parameters produce the following end-effector position:

```
x = -q3 * sin(q2) * cos(q1)
y = -q3 * sin(q2) * sin(q1)  
z = L1 + q3 * cos(q2)
```

**Important**: Note the negative signs in x and y equations. These come from the DH convention with alpha angles of +90° and -90°.

### Inverse Kinematics

The corrected IK formulas that match the FK are:

```python
q1 = np.arctan2(y, x)
q2 = np.arctan2(-np.sqrt(x**2 + y**2), z - L1)
q3 = np.sqrt(x**2 + y**2 + (z - L1)**2)
```

**Critical**: The q2 formula uses `-sqrt(x² + y²)` (negative) to account for the negative signs in the FK equations.

### Derivation

From FK:
- `x² + y² = q3² * sin²(q2)`  (after squaring and adding)
- `z - L1 = q3 * cos(q2)`

Therefore:
- `q3 = sqrt(x² + y² + (z - L1)²)`
- `sin(q2) = -sqrt(x² + y²) / q3`  (negative from FK)
- `cos(q2) = (z - L1) / q3`
- `q2 = atan2(sin(q2), cos(q2)) = atan2(-sqrt(x² + y²), z - L1)`
- `q1 = atan2(y, x)`  (standard from x and y ratios)

## Testing Results

The implementation has been verified with multiple test cases:
- Target [0.5, 0.5, 1.5]: Error = 5.55e-17 m ✓
- Target [0.0, 1.0, 0.5]: Error = 1.60e-16 m ✓
- Target [1.0, 0.0, 2.0]: Error = 2.54e-17 m ✓
- Target [-0.5, -0.5, 1.0]: Error = 1.11e-16 m ✓
- Target [0.3, 0.4, 1.2]: Error = 0.00e+00 m ✓

All errors are at floating-point precision level, confirming the IK is exact.

## Common Pitfalls

1. **Sign errors**: The FK has negative signs in x and y that must be accounted for in IK
2. **Assuming planar robot**: This is a 3D cylindrical robot, not a 2D planar arm
3. **Using formulas from images directly**: The formulas in reference images may not account for specific DH conventions used in implementation

## Files

- `tools/robot_3dof_dh.py`: Main implementation
- `test_3dof_simple.py`: Simple test without visualization
- `test_3dof_visual.py`: Test with 3D visualization
- `debug_3dof.py`: Debug script showing FK matrices
- `example_3dof_usage.py`: Usage examples
