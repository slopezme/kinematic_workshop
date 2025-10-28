# kinematic_workshop

This project is a simple and educational tool to demonstrate the principles of robot kinematics using Python. It visualizes a robot arm based on a configuration file that defines its structure using a transform (TF) tree, similar to URDF.

## Features

- **Intuitive Robot Configuration**: Define your robot's kinematic chain in a simple `robot_config.yaml` file by specifying joints and their parent/child links.
- **Forward Kinematics**: Calculates the position and orientation of each joint and the end-effector.
- **3D Visualization**: Plots the robot arm in its given configuration using `matplotlib`.
- **Animation**: Animates the robot's movement between two joint configurations and traces the path of the end-effector.

## Getting Started

### Prerequisites

You need to have Python 3 installed.

### Installation

1.  Clone this repository or download the files.
2.  Install the required Python libraries:
    ```bash
    pip install numpy pyyaml matplotlib scipy
    ```

### How to Run

The main entry point for testing is `test_robot.py`. This script loads the robot configuration, sets some example joint angles, and generates a 3D plot of the robot.

To run the simulation:

```bash
python kinematic_workshop/test_robot.py
```

This will open a `matplotlib` window showing an animation of the robot arm moving from a start to an end configuration.

### Customizing the Robot

You can define your own robot by modifying the `kinematic_workshop/robot_config.yaml` file. Simply add, remove, or modify entries in the `joints` list. Each joint connects a parent to a child link and has a defined origin (translation and rotation) and an axis of motion.
