# bullet_panda

![PyPI pyversions](https://img.shields.io/pypi/pyversions/ansicolortags.svg)

Panda robot simulation using [PyBullet](https://www.pybullet.org) and a Python interface class to control and monitor the robot in the simulation. The interface is written in the structure similar to the [_panda_robot_](https://github.com/justagist/panda_robot) ROS package used for controlling the real robot. This allows for direct transfer of code to the real robot (when using the [_panda_robot_](https://github.com/justagist/panda_robot) ROS package).

Although, this package is structured as a ROS package, it can be used without ROS.

## Dependencies

- [PyBullet](https://www.pybullet.org) (`pip install pybullet`)

## Related Packages

- [_mujoco_panda_](https://github.com/justagist/mujoco_panda) : Simulation using Mujoco Physics Engine (pymujoco) with exposed controllers and state feedback. Also provides a Python interface class to control and monitor the robot in the simulation. The interface is written in the structure similar to the [_panda_robot_](https://github.com/justagist/panda_robot) ROS package used for controlling the real robot. This allows for direct transfer of code to the real robot (when using the [_panda_robot_](https://github.com/justagist/panda_robot) ROS package).

- [_panda_simulator_](https://github.com/justagist/panda_simulator) : Simulation in Gazebo with exposed controllers and state feedback using ROS topics and services. The simulated robot uses the same ROS topics and services as the real robot when using the [_franka_ros_interface_](https://github.com/justagist/franka_ros_interface).
- [_franka_ros_interface_](https://github.com/justagist/franka_ros_interface) : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [_panda_robot_](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).
