# lumen_bringup

Lightweight bringup package for starting the core nodes and hardware interfaces required to run the Lumen robot.

## Overview
This package provides launch files and configuration to bring up the robot stack (drivers, core nodes, parameter configuration). The README focuses on the primary use case: starting the robot for development or deployment.

## Quick start
1. Install ROS 2 and dependencies.
2. From the workspace root:
   - colcon build
   - source install/setup.bash
3. Launch the bringup stack:
   - Mapping : 
   - `ros2 launch lumen_bringup bringup.launch.py`
   - Navigation :
   - `ros2 launch lumen_bringup navigation.launch.py`

Adjust any launch arguments or parameters as needed.

## Configuration
Edit the package's launch and config files to change robot-specific parameters, namespaces, or enabled components.

## Support
If you need help or want to contribute, open an issue or a pull request in this repository.

## License
This project is licensed under the MIT License — see the bundled LICENSE file for details.
