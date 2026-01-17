# IKViz

**IKViz** is a custom-built robotics visualization and inverse kinematics sandbox written in C++ using OpenGL. It is designed for interactive robot inspection, kinematic debugging, and real-time IK experimentation. No manual IK required.

## Demo

### Interactive IK & Visualization

![IKViz Demo](Docs/IKVIZ DEMO.gif)

## Features

* **Real-time inverse kinematics** with interactive end-effector manipulation
* **Dual-robot visualization** for comparing reference vs. driven/jogged states
* **URDF-based robot loading** with proper link hierarchy and transforms
* **Clean edge-rendered geometry** for clear kinematic structure visibility
* **Custom gizmos** for position and orientation control
* **Orbit camera system** for intuitive 3D navigation
* **ImGui-based UI** for fast iteration and debugging

## Purpose

IKViz was built to support the development of precision robotic arms and motion planners, enabling rapid iteration on kinematics, visualization, and control logic without the overhead of full simulation environments. It serves as both a debugging tool and a foundation for future trajectory planning and control extensions.

## Built With

* **C++**
* **OpenGL**
* **GLFW / GLAD**
* **GLM**
* **ImGui**
* **CMake**


