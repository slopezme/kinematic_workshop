# kinematic_workshop

This project is a simple and educational tool to demonstrate the principles of robot kinematics using Python. It visualizes a robot arm based on a configuration file that defines its structure using the Denavit-Hartenberg (DH) convention.

## Features

- **Configurable Robot Structure**: Define your robot's kinematic chain in a simple `robot_config.yaml` file.
- **Denavit-Hartenberg Convention**: Uses the standard DH parameterization for robot modeling.
- **Forward Kinematics**: Calculates the position and orientation of each joint and the end-effector.
- **3D Visualization**: Plots the robot arm in its given configuration using `matplotlib`.

## Getting Started

### Prerequisites

You need to have Python 3 installed.

### Installation

1.  Clone this repository or download the files.
2.  Install the required Python libraries:
    ```bash
    pip install numpy pyyaml matplotlib
    ```

### How to Run

The main entry point for testing is `test_robot.py`. This script loads the robot configuration, sets some example joint angles, and generates a 3D plot of the robot.

To run the simulation:

```bash
python kinematic_workshop/test_robot.py
```

This will open a `matplotlib` window showing the 3D plot of the robot arm defined in `robot_config.yaml` with the joint angles specified in `test_robot.py`.

### Customizing the Robot

You can define your own robot by modifying the `kinematic_workshop/robot_config.yaml` file. Simply add or remove entries in the `dh_parameters` list. Each entry corresponds to a joint and requires the four DH parameters: `a`, `alpha`, `d`, and `theta` (which is used as an initial value).
