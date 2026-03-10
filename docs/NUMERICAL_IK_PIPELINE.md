# Numerical Inverse Kinematics Pipeline

## Overview

This pipeline uses **numerical optimization** to solve inverse kinematics by minimizing the error between the desired end-effector position and the position achieved by candidate joint angles. This is a general-purpose approach that works with any robot configuration.

## Method: Error Minimization

The numerical IK approach treats inverse kinematics as an optimization problem:

1. **Define an error function**: Distance between target position and current end-effector position
2. **Use optimization algorithm**: scipy's L-BFGS-B minimizes this error
3. **Iterate until convergence**: Find joint angles that minimize position error

### Advantages
- ✓ Works with any robot configuration (no analytical solution needed)
- ✓ Flexible - can add constraints (joint limits, collision avoidance)
- ✓ Can optimize for orientation as well as position
- ✓ Easy to extend with additional objectives

### Disadvantages
- ✗ Requires initial guess (may converge to local minimum)
- ✗ Slower than analytical solutions (iterative process)
- ✗ May not converge for some targets
- ✗ Solution depends on initial guess

## Implementation

**File**: `tools/robot_arm.py`

**Key Method**:
```python
def inverse_kinematics(self, target_position, initial_guess_angles, 
                      tolerance=1e-3, max_iterations=100):
    """
    Computes inverse kinematics using numerical optimization.
    
    Uses scipy.optimize.minimize with L-BFGS-B method to find
    joint angles that minimize position error.
    """
```

**Error Function**:
```python
def error_function(angles):
    transforms = self.forward_kinematics(angles)
    current_position = transforms[-1][:3, 3]
    error = np.linalg.norm(current_position - target_position)
    return error
```

## Robot Configuration

Robots are defined using YAML files with a TF tree structure:

**File**: `configs/robot_config.yaml`

```yaml
robot_name: "Sample 3-DOF Arm"

joints:
  - name: joint1
    parent: base_link
    child: link1
    origin: {xyz: [0, 0, 0.5], rpy: [0, 0, 0]}
    axis: {xyz: [0, 0, 1]}  # Rotation axis
    limit: {lower: 0, upper: 180}  # degrees
```

## Running Tests

### 1. Forward Kinematics Test

Tests forward kinematics by animating robot movement between two configurations.

```bash
python3 tests/numerical_ik/test_forward_kinematics.py
```

**What it does:**
- Loads robot from `configs/robot_config.yaml`
- Animates movement from initial to final joint configuration
- Visualizes the robot arm and end-effector trajectory
- Shows forward kinematics in action

**Expected output:**
- Matplotlib animation window showing robot movement
- End-effector traces a path in 3D space

### 2. Inverse Kinematics Test

Tests numerical IK by making the robot follow a straight-line path.

```bash
python3 tests/numerical_ik/test_inverse_kinematics.py
```

**What it does:**
- Defines a straight-line path in 3D space
- Uses numerical IK to compute joint angles for each point
- Animates the robot following the path
- Demonstrates IK solving capability

**Expected output:**
- Console output showing IK convergence for each point
- Animation of robot following the target path
- End-effector should closely track the desired trajectory

**Example output:**
```
Calculating Inverse Kinematics for the path...
Point 1/100: Error = 2.34e-05 m
Point 2/100: Error = 1.87e-05 m
...
Animation complete.
```

### 3. Workspace Computation Test

Computes and visualizes the robot's reachable workspace.

```bash
python3 tests/numerical_ik/test_workspace.py
```

**With custom resolution:**
```bash
python3 tests/numerical_ik/test_workspace.py --res 10
```

**What it does:**
- Samples all possible joint angle combinations
- Computes forward kinematics for each configuration
- Collects all reachable end-effector positions
- Visualizes the workspace as a 3D point cloud

**Parameters:**
- `--res`: Resolution in degrees (default: 5°)
  - Lower values = more detailed but slower
  - Higher values = faster but coarser

**Expected output:**
- Console showing progress (e.g., "calculated 10,000 of 50,000 points")
- 3D plot showing the workspace boundary
- Point cloud representing all reachable positions

**Performance note:** Workspace computation is computationally expensive. For a 3-DOF robot with 5° resolution, it computes ~50,000 points. Higher DOF robots require coarser resolution.

## Typical Workflow

### 1. Define Your Robot
Edit `configs/robot_config.yaml` to define your robot structure:
- Add/remove joints
- Set joint limits
- Define link lengths and orientations

### 2. Test Forward Kinematics
```bash
python3 tests/numerical_ik/test_forward_kinematics.py
```
Verify your robot moves as expected.

### 3. Test Inverse Kinematics
```bash
python3 tests/numerical_ik/test_inverse_kinematics.py
```
Verify IK can reach desired positions.

### 4. Analyze Workspace
```bash
python3 tests/numerical_ik/test_workspace.py --res 10
```
Understand your robot's reachable space.

## Using in Your Code

```python
from tools.robot_arm import RobotArm
import numpy as np

# Load robot
robot = RobotArm('configs/robot_config.yaml')

# Forward kinematics
joint_angles = [0.5, 0.3, 0.2]  # radians
transforms = robot.forward_kinematics(joint_angles)
end_effector_pos = transforms[-1][:3, 3]

# Inverse kinematics
target_position = [1.0, 0.5, 1.5]
initial_guess = [0, 0, 0]
angles, position, error = robot.inverse_kinematics(
    target_position, 
    initial_guess,
    tolerance=1e-3,
    max_iterations=100
)

print(f"Solution: {np.rad2deg(angles)} degrees")
print(f"Error: {error:.6f} m")
```

## Troubleshooting

### IK Not Converging
- Try different initial guesses
- Increase max_iterations
- Relax tolerance
- Check if target is within workspace

### Slow Performance
- Reduce workspace resolution
- Use fewer sample points for IK path
- Consider analytical IK if available for your robot

### Robot Looks Wrong
- Check joint axis definitions in YAML
- Verify origin transforms (xyz, rpy)
- Test with simple configurations first

## Comparison with Analytical IK

| Feature | Numerical IK | Analytical IK |
|---------|--------------|---------------|
| Speed | Slower (iterative) | Instant (closed-form) |
| Accuracy | Approximate | Exact |
| Generality | Any robot | Specific robot only |
| Reliability | Depends on initial guess | Always converges* |
| Implementation | Complex (optimizer) | Simple (formulas) |

*if target is reachable

## References

- **Optimization**: scipy.optimize.minimize with L-BFGS-B
- **Forward Kinematics**: Transform tree approach (similar to ROS TF)
- **Configuration**: YAML-based (similar to URDF)

## Next Steps

- Try the [Analytical IK Pipeline](ANALYTICAL_IK_PIPELINE.md) for the 3DOF robot
- Modify `configs/robot_config.yaml` to create your own robot
- Experiment with different IK targets and paths
