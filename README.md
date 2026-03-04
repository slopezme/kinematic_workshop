# kinematic_workshop

This project is a simple and educational tool to demonstrate the principles of robot kinematics using Python. It visualizes a robot arm based on a configuration file that defines its structure using a transform (TF) tree, similar to URDF.

## Features

- **Intuitive Robot Configuration**: Define your robot's kinematic chain in a simple `robot_config.yaml` file by specifying joints and their parent/child links.
- **Forward Kinematics**: Calculates the position and orientation of each joint and the end-effector.
- **3D Visualization**: Plots the robot arm in its given configuration using `matplotlib`.
- **Animation**: Animates the robot's movement between two joint configurations and traces the path of the end-effector.
- **Workspace Computation**: Computes and visualizes the reachable workspace of the robot's end-effector.

## Getting Started

### Prerequisites

You need to have Python 3 installed.

### Installation

1.  Clone this repository or download the files.
2.  Navigate into the project directory:
    ```bash
    cd kinematic_workshop
    ```
3.  Create and activate a Python virtual environment. This creates an isolated environment for the project's dependencies.
    ```bash
    # Create the virtual environment (e.g., named 'venv')
    python3 -m venv venv
    # Activate it
    source venv/bin/activate
    ```

4.  Install the required Python libraries inside the virtual environment:
    ```bash
    pip install numpy pyyaml matplotlib scipy
    ```

### How to Run

To see an animation of the robot moving between two configurations, run `test_forward_kinematics.py`:
```bash
python3 test_forward_kinematics.py
```

This will open a `matplotlib` window showing an animation of the robot arm moving from a start to an end configuration.

### Customizing the Robot

You can define your own robot by modifying the `kinematic_workshop/robot_config.yaml` file. Simply add, remove, or modify entries in the `joints` list. Each joint connects a parent to a child link and has a defined origin (translation and rotation) and an axis of motion.
