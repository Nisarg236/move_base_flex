[![Jazzy CI](https://github.com/naturerobots/move_base_flex/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/naturerobots/move_base_flex/actions/workflows/jazzy.yaml)
[![Humble CI](https://github.com/naturerobots/move_base_flex/actions/workflows/humble.yaml/badge.svg)](https://github.com/naturerobots/move_base_flex/actions/workflows/humble.yaml)

# Move Base Flex: A Highly Flexible Navigation Framework

**Move Base Flex (MBF)** is a **modular, map-agnostic navigation framework** for ROS that provides **well-defined interfaces and action servers** for **path planning**, **control**, and **recovery behaviors**.
Rather than being a complete navigation stack, MBF serves as an **interface layer** that enables developers to design and integrate their own navigation systems using arbitrary map representations and custom plugin implementations.

## Key Features

* **Map-Agnostic Interface Design**
  
  MBF's interfaces are independent of any particular map representation (e.g., 2D costmaps, [meshes](https://github.com/naturerobots/mesh_navigation), or voxel grids), enabling seamless integration, scientific comparison, and context-aware selection of both navigation implementations and map types.

* **Modular Action-Based Architecture**
  
  Separate action servers for *path planning*, *control*, and *recovery* enable external [deliberation software](https://github.com/ros-wg-delib/awesome-ros-deliberation) (e.g., Behavior Trees, SMACH, or custom logic) to coordinate complex navigation strategies.

* **Extensible Plugin Framework**
  
  Multiple planners, controllers, and recovery behaviors can be loaded simultaneously, selected at runtime, or executed in parallel using different concurrency slots.

* **Rich Feedback and Diagnostics**
  
  All actions expose detailed feedback, results, and error codes, providing transparent runtime information for better debugging and system supervision.

* **Clear Separation of Interfaces and Implementations**
  
  MBF's design facilitates reuse, experimentation, and the rapid development of new navigation approaches independent of any particular mapping or planning framework.

## Concepts & Architecture

We have created Move Base Flex for a larger target group besides the standard developers and users of move_base and 2D navigation based on costmaps, as well as addressed move_base's limitations. Since robot navigation can be separated into planning and controlling in many cases, even for outdoor scenarios without the benefits of flat terrain, we designed MBF based on abstract planner-, controller- and recovery behavior-execution classes. To accomplish this goal, we created abstract base classes for the nav core BaseLocalPlanner, BaseGlobalPlanner and RecoveryBehavior plugin interfaces, extending the API to provide a richer and more expressive interface without breaking the current move_base plugin API. The new abstract interfaces allow plugins to return valuable information in each execution cycle, e.g. why a valid plan or a velocity command could not be computed. This information is then passed to the external executive logic through MBF planning, navigation or recovering actions’ feedback and result. The planner, controller and recovery behavior execution is implemented in the abstract execution classes without binding the software implementation to 2D costmaps. In our framework, MoveBase is just a particular implementation of a navigation system: its execution classes implement the abstract ones, bind the system to the costmaps. Thereby, the system can easily be used for other approaches, e.g. navigation on meshes or 3D occupancy grid maps. However, we provide a SimpleNavigationServer class without a binding to costmaps.

MBF architecture:
![MBF architecture](doc/images/move_base_flex.png)

## History

MBF was originally developed for **ROS 1** by [Magazino](https://www.magazino.eu/en/) (see [noetic](https://github.com/naturerobots/move_base_flex/tree/noetic) or [master](https://github.com/naturerobots/move_base_flex/tree/master) branch) as a **backwards-compatible replacement for `move_base`**, providing a more flexible and transparent architecture when no modular alternative was available.
It has been successfully deployed in production environments, for example at **[Magazino](https://www.magazino.eu/?lang=en)**, to control **TORU robots** operating in dynamic warehouse scenarios.

Compared to `move_base`, MBF introduced:

* Separate action servers for path planning, control, and recovery
* Detailed feedback and error reporting
* Runtime selection of multiple plugin implementations
* Map-agnostic interface definitions

With the advent of **ROS 2** and newer navigation frameworks such as **Nav2**, MBF continues to serve as a **lightweight, interface-oriented foundation** for research, prototyping, and customized navigation systems.

## Future Work
MBF is an ongoing project. Some of the improvements that we have planned for the near future are:

* Release MBF Mesh Navigation, see [mesh_navigation](https://github.com/uos/mesh_navigation).
* Auto select the active controller when having concurrently running controllers
* Add Ackermann steering API
* Multi Goal API + Action
* Add new navigation server and core packages using [grid_map](https://wiki.ros.org/grid_map).
* Constraints-based goal (see issue https://github.com/naturerobots/move_base_flex/issues/8)

But, of course you are welcome to propose new fancy features and help make them a reality! Pull Requests are always welcome!

## Credit

### [<img width="25" height="25" src="doc/images/logos/magazino_icon.png"> Magazino](https://www.magazino.eu/) 
Move Base Flex was initially developed at Magazino.

### [<img width="25" height="25" src="doc/images/logos/nature_robots_icon.jpg"> Nature Robots](https://naturerobots.com/)
The latest version (ROS2) is developed and maintained by Nature Robots.

## Further Resources

* [Move Base Flex Documentation and Tutorials](https://wiki.ros.org/move_base_flex)
* [Example TurtleBot3 Configuration](https://github.com/Rayman/turtlebot3_mbf)
* [MBF Deliberation Examples](https://github.com/amock/mbf_deliberation)
* [MBF Tutorials](https://github.com/naturerobots/mbf_tutorials)
* [Mesh Navigation](https://github.com/naturerobots/mesh_navigation)
