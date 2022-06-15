""" A simple example that illustrates the interface

This module contains a ROS2 entry point function that creates a node to publish 
random Unity3D game object positions.

"""

# Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
# 
# Created by: a. whit. (nml@whit.contact)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.


# Imports.
from numpy.random import randn
import rclpy
import rclpy.node
from geometry_msgs.msg import Point


# Entry point for ROS2.
def main():
    """ A simple function for creating a ROS2 node that publishes random 
        3D position messages intended for a Unity3D GameObject.
    """
    
    # Initialize ROS.
    rclpy.init()
    
    # Create a new ROS2 node.
    node = rclpy.node.Node('example')
    
    # Create a publisher for setting the position of the Unity3D GameObject.
    position_publisher = node.create_publisher(Point, "MySphere/position", 5)
    
    # Create a callback that publishes a random position.
    # The random position is generated using a standard Gaussian distribution.
    make_msg = lambda: Point(x=randn(), y=randn(), z=randn())
    publish = lambda: position_publisher.publish(make_msg())
    
    # Create a timer to set the period for position updates.
    # The period defaults to one quarter second.
    timer = node.create_timer(0.250, publish)
    
    # Pass control to the ROS2 event loop until the user requests shutdown via
    # a keyboard interrupt.
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()
    
    # Shutdown ROS.
    rclpy.shutdown()
    
  

if __name__ == '__main__': main()

