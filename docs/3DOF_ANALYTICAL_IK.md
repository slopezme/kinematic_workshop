# 3DOF Robot with Analytical Inverse Kinematics

## Overview

This implementation provides a 3-DOF (Degrees of Freedom) robot arm using **Denavit-Hartenberg (DH) parameters** with **analytical inverse kinematics** solutions. Unlike numerical optimization methods that iteratively search for solutions, analytical IK provides closed-form mathematical solutions that are:

- **Exact**: No approximation errors
- **Fast**: Instant computation without iterations
- **Reliable**: Always converges to a solution (if one exists)
- **Deterministic**: Same input always produces same output

## Robot Configuration

### DH Parameters

The robot is defined using the following Denavit-Hartenberg parameters:

| Joint | Type       | θ_i  | d_i | a_i | α_i   |
|-------|------------|------|-----|-----|-------|
| 1     | Revolute   | q1*  | L1  | 0   | +90°  |
| 2     | Revolute   | q2*  | 0   | 0   | -90°  |
| 3     | Prismatic  | 0°   | q3* | 0   | 0°    |

*\* indicates variable joint parameter*

### Robot Structure

This is a **cylindrical robot** (not planar):

- **Joint 1 (q1)**: Base revolute joint rotating around vertical Z-axis
- **Joint 2 (q2)**: Second revolute joint rotating around vertical Z-axis at height L1
- **Joint 3 (q3)**: Prismatic joint extending vertically along Z-axis
- **L1**: Fixed vertical offset to first joint (height of base)

## Forward Kinematics

From the DH transformations, the end-effector position is:

```
x = -q3 * sin(q2) * cos(q1)
y = -q3 * sin(q2) * sin(q1)
z = L1 + q3 * cos(q2)
```

Note: The negative signs in x and y come from the DH convention with the specific alpha angles used.

## Inverse Kinematics Solution

Given a target position **[x, y, z]**, the analytical solution is:

```
q1 = atan2(y, x)
q2 = atan2(-√(x² + y²), z - L1)
q3 = √(x² + y² + (z - L1)²)
```

This solution accounts for the negative signs in the forward kinematics equations and provides exact results with zero error (within floating-point precision).

## Forward Kinematics

The forward kinematics uses the standard DH transformation:

```
T_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
```

The transformation matrix for each joint is:

```
     [cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ)]
T =  [sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ)]
     [  0         sin(α)         cos(α)          d    ]
     [  0           0              0            1    ]
```

## Usage Examples

### Basic Usage

```python
from tools.robot_3dof_dh import Robot3DOF_DH
import numpy as np

# Create robot with L1 = 1.0 meter
robot = Robot3DOF_DH(L1=1.0)

# Define target position
target = [0.5, 0.5, 1.5]

# Compute inverse kinematics (algebraic method)
joint_values = robot.inverse_kinematics(target, method='algebraic')
print(f"q1 = {np.rad2deg(joint_values[0]):.2f}°")
print(f"q2 = {np.rad2deg(joint_values[1]):.2f}°")
print(f"q3 = {joint_values[2]:.4f} m")

# Verify solution with forward kinematics
transforms = robot.forward_kinematics(joint_values)
achieved_position = transforms[-1][:3, 3]
error = np.linalg.norm(achieved_position - target)
print(f"Position error: {error:.2e} m")
```

### Comparing Both IK Methods

```python
# Algebraic solution
q_alg = robot.inverse_kinematics(target, method='algebraic')

# Geometric solution
q_geo = robot.inverse_kinematics(target, method='geometric')

# Both should reach the same target with different joint configurations
```

### Verifying IK Solutions

```python
success, error, achieved_pos = robot.verify_ik_solution(target, joint_values)
if success:
    print(f"✓ Solution valid with error: {error:.2e} m")
else:
    print(f"✗ Solution failed with error: {error:.2e} m")
```

## Files

- **`tools/robot_3dof_dh.py`**: Main robot class implementation
- **`robot_3dof_dh_config.yaml`**: Configuration file with DH parameters
- **`test_3dof_analytical_ik.py`**: Comprehensive test suite with visualization
- **`test_3dof_simple.py`**: Simple test without visualization
- **`example_3dof_usage.py`**: Usage examples and demonstrations

## Running Tests

### Simple Test (No Visualization)
```bash
python3 test_3dof_simple.py
```

### Comprehensive Test (With Visualization)
```bash
python3 test_3dof_analytical_ik.py
```

### Usage Examples
```bash
python3 example_3dof_usage.py
```

## Advantages Over Numerical IK

| Feature | Analytical IK | Numerical IK |
|---------|---------------|--------------|
| Speed | Instant | Iterative (slower) |
| Accuracy | Exact | Approximate |
| Convergence | Always* | May fail |
| Deterministic | Yes | Depends on initial guess |
| Multiple solutions | Can find all | Finds one |

*\* Assuming target is within workspace*

## Mathematical Background

The analytical solutions are derived from:

1. **Forward kinematics equations**: Expressing end-effector position as function of joint variables
2. **Algebraic manipulation**: Solving the system of equations symbolically
3. **Geometric interpretation**: Using robot geometry to derive relationships

The key insight is that this particular robot configuration allows the equations to be decoupled and solved in closed form.

## Workspace

The robot's workspace is determined by:
- **q1 range**: [-π, π] (full rotation)
- **q2 range**: [-π/2, π/2] (±90°)
- **q3 range**: [0, 2·L1] (prismatic extension)

The workspace can be computed using:
```python
workspace_points = robot.compute_workspace(
    q1_range=[-np.pi, np.pi],
    q2_range=[-np.pi/2, np.pi/2],
    q3_range=[0, 2.0],
    resolution=20
)
```

## References

- Denavit, J., & Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms"
- Craig, J. J. (2005). "Introduction to Robotics: Mechanics and Control"
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). "Robot Modeling and Control"

## Notes

- The analytical solutions assume the target position is reachable
- For positions outside the workspace, the solutions may be mathematically valid but physically unrealizable
- The two methods (algebraic and geometric) represent different robot configurations (elbow-up vs elbow-down type solutions)
- Joint limits should be checked after computing IK solutions
