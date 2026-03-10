# Quick Start: 3DOF Analytical IK Robot

## What's New

A new 3-DOF robot implementation with **analytical inverse kinematics** based on Denavit-Hartenberg parameters. This provides exact, instant solutions without numerical optimization.

## Quick Run

```bash
# Simple test (no visualization)
python3 test_3dof_simple.py

# Full test with visualization
python3 test_3dof_analytical_ik.py

# Usage examples
python3 example_3dof_usage.py
```

## Quick Example

```python
from tools.robot_3dof_dh import Robot3DOF_DH
import numpy as np

# Create robot
robot = Robot3DOF_DH(L1=1.0)

# Solve IK for target position
target = [0.5, 0.5, 1.5]
q = robot.inverse_kinematics(target, method='algebraic')

print(f"Joint angles: q1={np.rad2deg(q[0]):.2f}°, q2={np.rad2deg(q[1]):.2f}°")
print(f"Prismatic: q3={q[2]:.4f}m")

# Verify with forward kinematics
transforms = robot.forward_kinematics(q)
achieved = transforms[-1][:3, 3]
error = np.linalg.norm(achieved - target)
print(f"Error: {error:.2e}m")
```

## Key Features

- ✓ **Analytical IK**: Exact closed-form solution (no iteration)
- ✓ **DH parameters**: Standard robotics formulation
- ✓ **Fast**: Instant computation
- ✓ **Reliable**: Always converges
- ✓ **Exact**: Zero error within floating-point precision

## Files Created

| File | Description |
|------|-------------|
| `tools/robot_3dof_dh.py` | Main robot class |
| `robot_3dof_dh_config.yaml` | Configuration with DH parameters |
| `test_3dof_simple.py` | Simple test script |
| `test_3dof_analytical_ik.py` | Full test with visualization |
| `example_3dof_usage.py` | Usage examples |
| `3DOF_ANALYTICAL_IK.md` | Complete documentation |

## DH Parameters

| Joint | Type | θ | d | a | α |
|-------|------|---|---|---|---|
| 1 | Revolute | q1* | L1 | 0 | +90° |
| 2 | Revolute | q2* | 0 | 0 | -90° |
| 3 | Prismatic | 0° | q3* | 0 | 0° |

## IK Formula

**Analytical Solution:**
```
q1 = atan2(y, x)
q2 = atan2(-√(x² + y²), z - L1)
q3 = √(x² + y² + (z - L1)²)
```

See `3DOF_ANALYTICAL_IK.md` for complete documentation.
