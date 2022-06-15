---
title: Example Unity-ROS2 project
subtitle: README
author: a.whit ([email](mailto:nml@whit.contact))
date: January 2022
documentclass: scrartcl
colorlinks: True
---

This repository contains an example [Unity3D][unity] project that has been 
prepared for use with Git and [ROS2][ros2]. The example consists of a 3D scene 
containing a sphere that can be manipulated via ROS messages. This packages 
makes use of the ROS2-Unity interface created by Unity Technologies, and made 
available via the [Unity Robotics Hub][robotics_hub].


## Getting started

This project is intended as an introduction to ROS-Unity integration, but it 
might also be used as a template or starting point for other projects. The 
installation and initial use are designed to be relatively straightforward. 
We recommend running through these steps before attempting to modify the 
project.

The repository itself is derived from the 
[empty Unity3D project repository][empty_unity_project].


## Installation

The following procedure has been tested with Ubuntu 20.04. The procedure will 
be slightly different, but very similar, for Windows 10. It is assumed that all 
steps are executed in a command terminal, or the Unity Editor, unless otherwise 
specified.

1. Before proceeding, please ensure that both ROS2 and Unity3D are correctly 
installed and working.
2. In a terminal, [clone][git_clone] this repository to a local path:[^lfs]
   ```
   git clone https://github.com/ricmua/example_ros_unity_3d_project.git /path/to/repository
   ```
3. Initialize the git [submodules][git_submodules]:
   ```
   cd /path/to/repository
   git submodule update --init
   ```
[^lfs]: Assumes git [LFS](https://github.com/git-lfs/git-lfs/blob/main/docs/spec.md) is installed. On Ubuntu: ``sudo apt install git-lfs``.


### Unity installation

4. Open the [Unity Editor][unity_editor].
5. Install the Unity plugin for the [ROS-TCP-Connector][ros_tcp_connector] by 
   following the [instructions][ros_tcp_connector_unity] provided for 
   [ROS-Unity integration][ros_unity_integration] via the 
   [Unity Robotics Hub][robotics_hub].
6. Optional: Use the Unity 
   [message generation plugin][unity_message_generation] to initialize C# 
   scripts for any relevant messages. The following built-in message types have 
   already been initialized for this example:
   * ``example_interfaces.msg.Float64``
   * ``geometry_msgs.msg.Point``
   * ``std_msgs.msg.ColorRGBA``
7. Open the cloned project repository (e.g., ``/path/to/repository``) using 
   the ``File->Open Project`` menu 
   item in the Unity Editor, or [add][unity_hub_add] the project via 
   [Unity Hub][unity_hub] and then launch a new copy of the editor.
8. [Build][unity_build] the Unity project to create a binary, if desired. 
   Alternatively -- as described in a later section -- the project can be run 
   without building by entering [Play Mode][unity_play_mode] in the Unity 
   Editor.



### ROS installation

8. Navigate to the ROS workspace included in the project:
   ```
   cd /path/to/repository/ros_workspace
   ```
9. [Source][ros_environment] the ROS2 environment:
   ```
   source /path/to/ros/setup.bash
   ```
10. [Build][ros_build] the workspace:
   ```
   colcon build
   ```

## Running the example

1. Run the Unity3D binary -- if built during installation -- or open the 
   project in the Unity editor and [enter play mode][unity_play_mode]. This 
   starts the GUI. A sphere should be displayed in 3D space.
2. In a command terminal, navigate to the ROS workspace included in the 
   project:
   ```
   cd /path/to/repository/ros_workspace
   ```
3. [Source][ros_environment] the ROS2 environment:
   ```
   source /path/to/ros/setup.bash
   ```
4. [Source][ros_overlay] the ROS2 overlay workspace environment:
   ```
   source install/local_setup.bash
   ```
5. Launch the example:
   ```
   ros2 launch launch/example.launch.py
   ```
6. If everything functions properly, the sphere should move to random positions 
   in the 3D scene every quarter second.
7. Type ``Ctrl-C`` in the command terminal to terminate the ROS2 application.
8. Shut down the Unity scene by exiting the application or exiting Play Mode.


## Forking

**TODO: Expand documentation related to modification of this example.**

### ROS topics

The C# script in this example initializes three ROS [topic] subscriptions; each 
for manipulating a different property of the sphere object. Each topic name 
consists of a relative [namespace] -- determined by the name of the 
sphere game object -- followed by a property identifier. Since it is 
used to determine the topic namespace, the game object must have a name 
that agrees with ROS [naming conventions]. For example, given a Unity 
Sphere primitive with the name ``Sphere1``, the following ROS topics would 
be initialized:

* ``Sphere1/position``
* ``Sphere1/color``
* ``Sphere1/radius``

The message types for the topics are as follows:

* position: [geometry_msgs/Point]
* color: [std_msgs/ColorRGBA]
* radius: [example_interfaces/Float64]


### Coordinate systems

When a [position message] is received via the position topic subscription, 
the 3D vector data contained in the message is first transformed, before 
applying the new posiiton to the sphere object. This transformation is 
necessary because ROS and Unity use different coordinate systems, by 
default. As noted in the documentation for the [ROSGeometry] component of 
the [ROS-TCP-Connector] package:

> In Unity, the X axis points right, Y up, and Z forward. ROS, on the 
> other hand, supports various coordinate frames: in the most 
> commonly-used one, X points forward, Y left, and Z up. In ROS 
> terminology, this frame is called "FLU" (forward, left, up), whereas 
> the Unity coordinate frame would be "RUF" (right, up, forward).

Instances of this class assume that incoming data uses the FLU coordinate 
system. Vectors are converted the data to the RUF coordinate system before 
use in the Unity scene.


### SphereNode

The [C# script](Assets/Scripts/SphereNode.cs) provided in this example is meant 
to be [attached] to a Unity [game object] -- generally by dragging and 
dropping in the Unity Editor. Here, it is assumed that the game object is a 
sphere, but that is not strictly necessary.

To manipulate the game object, position, color, and size modifications are 
requested via ROS topics.

[game object]: https://docs.unity3d.com/Manual/class-GameObject.html

[attached]: https://docs.unity3d.com/Manual/CreatingAndUsingScripts.html

License
-------

Copyright 2021 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.





[unity]: https://en.wikipedia.org/wiki/Unity_(game_engine)

[ros2]: https://docs.ros.org/en/galactic/index.html

[robotics_hub]: https://github.com/Unity-Technologies/Unity-Robotics-Hub

[git_clone]: https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository

[git_submodules]: https://git-scm.com/book/en/v2/Git-Tools-Submodules#_cloning_submodules

[unity_editor]: https://unity.com/developer-tools

[ros_tcp_connector]: https://github.com/Unity-Technologies/ROS-TCP-Connector

[ros_tcp_connector_unity]: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-unity-setup

[ros_unity_integration]: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md

[unity_message_generation]: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/MessageGeneration.md

[unity_build]: https://docs.unity3d.com/Manual/PublishingBuilds.html

[unity_play_mode]: https://docs.unity3d.com/Manual/GameView.html

[unity_hub]: https://unity.com/unity-hub

[unity_hub_add]: https://docs.unity3d.com/hub/manual/AddProject.html#add-an-existing-project-from-your-disk

[topic]: https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html

[namespace]: https://design.ros2.org/articles/topic_and_service_names.html#namespaces

[naming conventions]: https://design.ros2.org/articles/topic_and_service_names.html

[geometry_msgs/Point]: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg

[std_msgs/ColorRGBA]: https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ColorRGBA.msg

[example_interfaces/Float64]: https://github.com/ros2/example_interfaces/blob/master/msg/Float64.msg

[ros_overlay]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#source-the-setup-file

[ros_environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html#source-the-setup-files

[ros_build]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package

[empty_unity_project]: https://github.com/ricmua/empty_ros_unity_3d_project

[position message]: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg

[RosGeometry]: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md

[ROS-TCP-Connector]: https://github.com/Unity-Technologies/ROS-TCP-Connector
 

