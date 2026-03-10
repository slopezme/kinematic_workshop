# Kinematic Workshop

Educational tools for robot kinematics using Python. This repository contains implementations of forward and inverse kinematics for different robot configurations.

## 📁 Repository Structure

```
kinematic_workshop/
├── tools/                      # Core implementations
│   ├── robot_arm.py           # Numerical IK (error minimization)
│   ├── robot_3dof_dh.py       # Analytical IK (closed-form solution)
│   └── transforms.py          # Transformation utilities
├── configs/                    # Robot configurations
│   ├── robot_config.yaml      # For numerical IK
│   ├── robot2_config.yaml     # Alternative configuration
│   └── robot_3dof_dh_config.yaml  # For analytical IK (DH parameters)
├── tests/
│   ├── numerical_ik/          # Tests for error minimization approach
│   │   ├── test_forward_kinematics.py
│   │   ├── test_inverse_kinematics.py
│   │   └── test_workspace.py
│   └── analytical_ik/         # Tests for closed-form solution
│       ├── test_3dof_simple.py
│       ├── test_3dof_visual.py
│       └── debug_3dof.py
├── examples/
│   └── 3dof_robot/            # Analytical IK examples
│       └── example_3dof_usage.py
├── docs/                       # Documentation
│   ├── NUMERICAL_IK_PIPELINE.md    # 📊 Numerical IK guide
│   ├── ANALYTICAL_IK_PIPELINE.md   # 🎯 Analytical IK guide
│   ├── 3DOF_ANALYTICAL_IK.md       # Technical details
│   └── README_3DOF.md              # Quick reference
└── kinematics_plotter.py      # Visualization utilities
```

## 🤖 Two Approaches to Inverse Kinematics

This repository demonstrates **two fundamentally different approaches** to solving inverse kinematics:

### 1. 📊 Numerical IK (Error Minimization)
**Method**: Iterative optimization that minimizes position error

- **Implementation**: `tools/robot_arm.py`
- **Approach**: Uses scipy optimizer to find joint angles
- **Pros**: Works with any robot configuration
- **Cons**: Slower, approximate, needs initial guess
- **Documentation**: **[📖 Numerical IK Pipeline Guide](docs/NUMERICAL_IK_PIPELINE.md)**

**Quick Start:**
```bash
# Forward kinematics animation
python3 tests/numerical_ik/test_forward_kinematics.py

# Inverse kinematics path following
python3 tests/numerical_ik/test_inverse_kinematics.py

# Workspace computation
python3 tests/numerical_ik/test_workspace.py --res 10

# To test with another robot configuration, run:
python3 tests/numerical_ik/test_forward_kinematics.py --config robot2_config.yaml
python3 tests/numerical_ik/test_inverse_kinematics.py --config robot2_config.yaml
python3 tests/numerical_ik/test_workspace.py --res 40 --config robot2_config.yaml
```

### 2. 🎯 Analytical IK (Closed-Form Solution)
**Method**: Direct mathematical formulas derived from DH parameters

- **Implementation**: `tools/robot_3dof_dh.py`
- **Approach**: Algebraic solution for 3DOF cylindrical robot
- **Pros**: Exact (zero error), instant, always converges
- **Cons**: Robot-specific, requires mathematical derivation
- **Documentation**: **[📖 Analytical IK Pipeline Guide](docs/ANALYTICAL_IK_PIPELINE.md)**

**Quick Start:**
```bash
# 3D visualization with coordinate frames
python3 tests/analytical_ik/test_3dof_visual.py
```

### 📊 Comparison

| Feature | Numerical IK | Analytical IK |
|---------|--------------|---------------|
| **Speed** | Iterative (~ms) | Instant (~μs) |
| **Accuracy** | Approximate (mm) | Exact (0 error) |
| **Generality** | Any robot ✓ | Specific robot only |
| **Convergence** | May fail | Always succeeds* |
| **Initial guess** | Required | Not needed |
| **Implementation** | Complex | Simple formulas |

*if target is within workspace

## 🚀 Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd kinematic_workshop
   ```

2. **Create virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies:**
   ```bash
   pip install numpy pyyaml matplotlib scipy
   ```

## 📚 Documentation

### Pipeline Guides (Start Here!)
- **[📊 Numerical IK Pipeline](docs/NUMERICAL_IK_PIPELINE.md)** - Complete guide to error minimization approach
- **[🎯 Analytical IK Pipeline](docs/ANALYTICAL_IK_PIPELINE.md)** - Complete guide to closed-form solution approach

### Additional Documentation
- **[3DOF Robot Quick Start](docs/README_3DOF.md)** - Quick reference for 3DOF robot
- **[3DOF Technical Details](docs/3DOF_ANALYTICAL_IK.md)** - Detailed mathematical documentation
- **[Implementation Notes](docs/IMPLEMENTATION_NOTES.md)** - Technical implementation details

## 🎯 Features

### General Robot Arm
- ✓ YAML-based robot configuration
- ✓ Forward kinematics with TF tree
- ✓ Numerical inverse kinematics (optimization-based)
- ✓ Workspace computation and visualization
- ✓ Animation support

### 3DOF Cylindrical Robot
- ✓ Analytical inverse kinematics (exact solution)
- ✓ DH parameter-based forward kinematics
- ✓ Zero error (floating-point precision)
- ✓ Instant computation (no iteration)
- ✓ 3D visualization with coordinate frames

## 📖 Usage Examples

### General Robot Arm
```python
from tools.robot_arm import RobotArm

robot = RobotArm('configs/robot_config.yaml')

# Forward kinematics
joint_angles = [0.5, 0.3, 0.2]
transforms = robot.forward_kinematics(joint_angles)

# Inverse kinematics (numerical)
target = [1.0, 0.5, 1.5]
initial_guess = [0, 0, 0]
angles, position, error = robot.inverse_kinematics(target, initial_guess)
```

### 3DOF Robot with Analytical IK
```python
from tools.robot_3dof_dh import Robot3DOF_DH

robot = Robot3DOF_DH(L1=1.0)

# Analytical inverse kinematics
target = [0.5, 0.5, 1.5]
joint_values = robot.inverse_kinematics(target)

# Forward kinematics
transforms = robot.forward_kinematics(joint_values)
```

## 🧪 Testing (Visual Output)

### Numerical IK Tests (Error Minimization)
```bash
# Forward kinematics animation (default robot)
python3 tests/numerical_ik/test_forward_kinematics.py

# Forward kinematics with alternative robot configuration
python3 tests/numerical_ik/test_forward_kinematics.py --config robot2_config.yaml

# Inverse kinematics path following animation
python3 tests/numerical_ik/test_inverse_kinematics.py

# Inverse kinematics with alternative robot
python3 tests/numerical_ik/test_inverse_kinematics.py --config robot2_config.yaml

# Workspace visualization
python3 tests/numerical_ik/test_workspace.py --res 10

# Workspace with alternative robot
python3 tests/numerical_ik/test_workspace.py --res 10 --config robot2_config.yaml
```

### Analytical IK Tests (Closed-Form Solution)
```bash
# 3D visualization with coordinate frames
python3 tests/analytical_ik/test_3dof_visual.py
```

**See the pipeline guides for all tests and detailed explanations:**
- [Numerical IK Pipeline Guide](docs/NUMERICAL_IK_PIPELINE.md)
- [Analytical IK Pipeline Guide](docs/ANALYTICAL_IK_PIPELINE.md)

## 📊 Visualization

The repository includes visualization tools for:
- Robot configuration and joint positions
- End-effector trajectories
- Workspace boundaries
- Coordinate frames at each joint

## 🤝 Contributing

This is an educational project. Feel free to extend it with:
- New robot configurations
- Additional IK methods
- More visualization options
- Performance optimizations

## 📝 License

Educational use - see individual files for details.

## 🔗 Related Documentation

- [Denavit-Hartenberg Parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
- [Robot Kinematics](https://en.wikipedia.org/wiki/Robot_kinematics)
- [Forward and Inverse Kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics)
