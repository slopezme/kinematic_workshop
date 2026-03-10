# Analytical Inverse Kinematics Pipeline

## Overview

This pipeline uses **closed-form mathematical solutions** to solve inverse kinematics for a specific 3-DOF cylindrical robot. The solution is derived algebraically from the robot's Denavit-Hartenberg (DH) parameters, providing exact, instant results.

## Method: Analytical Solution

The analytical IK approach derives explicit formulas for joint angles:

1. **Define robot with DH parameters**: Standard robotics notation
2. **Derive FK equations**: Express end-effector position as function of joint angles
3. **Solve algebraically**: Invert equations to get joint angles from position
4. **Compute instantly**: No iteration, no approximation

### Advantages
- ✓ **Exact solution**: Zero error (within floating-point precision)
- ✓ **Instant computation**: No iteration required
- ✓ **Always converges**: Deterministic solution
- ✓ **No initial guess needed**: Direct calculation
- ✓ **Predictable**: Same input always gives same output

### Disadvantages
- ✗ **Robot-specific**: Only works for this 3-DOF configuration
- ✗ **Requires derivation**: Must solve equations for each robot type
- ✗ **Limited flexibility**: Can't easily add constraints

## Robot Structure

**Type**: 3-DOF Cylindrical Robot

**Joints**:
- **q1**: Base revolute joint (rotates around vertical Z-axis)
- **q2**: Second revolute joint (rotates around vertical axis at height L1)
- **q3**: Prismatic joint (extends vertically)

**DH Parameters**:

| Joint | Type | θ | d | a | α |
|-------|------|---|---|---|---|
| 1 | Revolute | q1* | L1 | 0 | +90° |
| 2 | Revolute | q2* | 0 | 0 | -90° |
| 3 | Prismatic | 0° | q3* | 0 | 0° |

*\* indicates variable joint parameter*

## Implementation

**File**: `tools/robot_3dof_dh.py`

### Forward Kinematics

From DH transformations:
```
x = -q3 * sin(q2) * cos(q1)
y = -q3 * sin(q2) * sin(q1)
z = L1 + q3 * cos(q2)
```

### Inverse Kinematics

Analytical solution:
```python
q1 = atan2(y, x)
q2 = atan2(-√(x² + y²), z - L1)
q3 = √(x² + y² + (z - L1)²)
```

**Key Method**:
```python
def inverse_kinematics(self, target_position):
    """
    Compute inverse kinematics using analytical solution.
    
    Returns exact joint values instantly with zero error.
    """
    x, y, z = target_position
    
    q1 = np.arctan2(y, x)
    q2 = np.arctan2(-np.sqrt(x**2 + y**2), z - self.L1)
    q3 = np.sqrt(x**2 + y**2 + (z - self.L1)**2)
    
    return np.array([q1, q2, q3])
```

## Running Tests

### 1. Simple Test (No Visualization)

Quick test of analytical IK with multiple target positions.

```bash
python3 tests/analytical_ik/test_3dof_simple.py
```

**What it does:**
- Tests 5 different target positions
- Computes IK solution for each
- Verifies solution with forward kinematics
- Reports error (should be ~0)

**Expected output:**
```
======================================================================
3DOF Robot - Analytical Inverse Kinematics Test
======================================================================

Test 1: Target Position = [0.500, 0.500, 1.500]
──────────────────────────────────────────────────────────────────────

  Analytical IK Solution:
    q1 =   45.000° ( 0.7854 rad)
    q2 =  -54.736° (-0.9553 rad)
    q3 =   0.8660 m
    Achieved: [0.500000, 0.500000, 1.500000]
    Error: 5.55e-17 m
    Status: ✓ SUCCESS

...
```

**All errors should be < 1e-15 m** (floating-point precision level)

### 2. Visual Test (With 3D Plots)

Comprehensive test with 3D visualization of robot configurations.

```bash
python3 tests/analytical_ik/test_3dof_visual.py
```

**What it does:**
- Tests forward kinematics with 4 configurations
- Tests inverse kinematics with 3 target positions
- Shows 3D plots of robot with coordinate frames
- Saves visualizations as PNG files

**Expected output:**
- Console output showing joint positions and errors
- Two matplotlib windows:
  1. Forward kinematics test (4 subplots)
  2. Inverse kinematics test (3 subplots)
- Saved images:
  - `docs/3dof_forward_kinematics_test.png`
  - `docs/3dof_ik_visualization.png`

**Visualization features:**
- Robot kinematic chain (links and joints)
- Coordinate frames at each joint (RGB = XYZ)
- Base marked with black square
- End-effector marked with triangle
- Target positions marked with green stars

### 3. Debug/Verification Scripts

Additional scripts for debugging and verification:

```bash
# Debug forward kinematics step-by-step
python3 tests/analytical_ik/debug_3dof.py

# Verify FK equations match DH transformations
python3 tests/analytical_ik/verify_fk_equations.py
```

## Running Examples

### Usage Examples

Demonstrates various use cases:

```bash
python3 examples/3dof_robot/example_3dof_usage.py
```

**Examples included:**
1. Basic usage
2. Multiple target positions
3. Forward kinematics
4. IK solution verification
5. DH parameters display

## Typical Workflow

### 1. Quick Verification
```bash
python3 tests/analytical_ik/test_3dof_simple.py
```
Verify IK is working correctly (all tests should pass with ~0 error).

### 2. Visual Inspection
```bash
python3 tests/analytical_ik/test_3dof_visual.py
```
See the robot in action and verify kinematic chain looks correct.

### 3. Experiment with Examples
```bash
python3 examples/3dof_robot/example_3dof_usage.py
```
Learn how to use the API in your own code.

## Using in Your Code

```python
from tools.robot_3dof_dh import Robot3DOF_DH
import numpy as np

# Create robot (L1 = 1.0 meter)
robot = Robot3DOF_DH(L1=1.0)

# Inverse kinematics (analytical)
target = [0.5, 0.5, 1.5]
joint_values = robot.inverse_kinematics(target)

print(f"q1 = {np.rad2deg(joint_values[0]):.2f}°")
print(f"q2 = {np.rad2deg(joint_values[1]):.2f}°")
print(f"q3 = {joint_values[2]:.4f} m")

# Forward kinematics
transforms = robot.forward_kinematics(joint_values)
achieved_position = transforms[-1][:3, 3]

# Verify solution
error = np.linalg.norm(achieved_position - target)
print(f"Error: {error:.2e} m")  # Should be ~0
```

## Configuration

**File**: `configs/robot_3dof_dh_config.yaml`

```yaml
robot_name: "3DOF DH Robot with Analytical IK"

L1: 1.0  # Height of base link

dh_parameters:
  - joint_number: 1
    joint_type: revolute
    d: 1.0
    alpha_deg: 90
    
  - joint_number: 2
    joint_type: revolute
    d: 0
    alpha_deg: -90
    
  - joint_number: 3
    joint_type: prismatic
    theta: 0
    alpha_deg: 0
```

## Test Results

All tests should show **exact solutions** with errors at floating-point precision:

| Target | Error | Status |
|--------|-------|--------|
| [0.5, 0.5, 1.5] | 5.55e-17 m | ✓ |
| [0.0, 1.0, 0.5] | 1.60e-16 m | ✓ |
| [1.0, 0.0, 2.0] | 2.54e-17 m | ✓ |
| [-0.5, -0.5, 1.0] | 1.11e-16 m | ✓ |
| [0.3, 0.4, 1.2] | 0.00e+00 m | ✓ |

## Troubleshooting

### Import Errors
Make sure you're running from the repository root:
```bash
cd /path/to/kinematic_workshop
python3 tests/analytical_ik/test_3dof_simple.py
```

### Visualization Not Showing
- Check matplotlib is installed: `pip install matplotlib`
- Try saving to file instead of showing interactively

### Unexpected Results
- Verify L1 parameter matches your robot
- Check that target is within workspace
- Run debug script: `python3 tests/analytical_ik/debug_3dof.py`

## Comparison with Numerical IK

| Feature | Analytical IK | Numerical IK |
|---------|---------------|--------------|
| Speed | **Instant** | Iterative (slower) |
| Accuracy | **Exact (0 error)** | Approximate |
| Convergence | **Always** | May fail |
| Initial guess | **Not needed** | Required |
| Robot types | Specific only | Any robot |
| Implementation | Simple formulas | Complex optimizer |

## Mathematical Background

The analytical solution is derived by:

1. Writing forward kinematics from DH parameters
2. Expressing end-effector position as function of joints
3. Solving the system of equations algebraically
4. Obtaining closed-form expressions for each joint

**Key insight**: This robot's geometry allows equations to be decoupled and solved in closed form.

## References

- [Complete Technical Documentation](3DOF_ANALYTICAL_IK.md)
- [Implementation Notes](IMPLEMENTATION_NOTES.md)
- [Quick Start Guide](README_3DOF.md)

## Next Steps

- Compare with [Numerical IK Pipeline](NUMERICAL_IK_PIPELINE.md)
- Modify L1 parameter to match your robot dimensions
- Try different target positions
- Visualize the workspace boundary
