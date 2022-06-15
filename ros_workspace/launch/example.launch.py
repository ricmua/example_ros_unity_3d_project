""" Launch file for a ROS2-Unity3D example.

A `launch file`_ for the ROS2 `launch system`_ that runs a ROS2-Unity example.
See the architecture_ documentation for further details.

.. _launch system: https://docs.ros.org/en/galactic/Tutorials/Launch-system.html
.. _launch file: https://docs.ros.org/en/galactic/Tutorials/Launch-Files
                 /Creating-Launch-Files.html
.. _architecture: https://github.com/ros2/launch/blob/master/launch/doc
                  /source/architecture.rst
"""

# Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
# 
# Created by: a. whit. (nml@whit.contact)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    unity_node \
        = Node(package='ros_tcp_endpoint',
               executable='default_server_endpoint',
               parameters=['config/unity.yaml'],
              )
    
    example_node \
        = Node(package='simple_ros2_unity_example',
               executable='example_node',
              )
    
    
    return LaunchDescription([unity_node,
                              example_node,
                             ]
                            )
    

    

