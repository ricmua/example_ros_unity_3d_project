/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


// Import standard collections.
using System.Collections;
using System.Collections.Generic;

// Import Unity.
using UnityEngine;

// Import the interface to ROS2.
using Unity.Robotics.ROSTCPConnector;

// Import the ROSGeometry package to facilitation coordinate frame conversion.
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// Import the relevant ROS2 message types.
using PositionMessage = RosMessageTypes.Geometry.PointMsg;//MVector3;
using ColorMessage    = RosMessageTypes.Std.ColorRGBAMsg;
using RadiusMessage   = RosMessageTypes.ExampleInterfaces.Float64Msg;



/** This class allows remote ROS nodes to manipulate the position, color, and 
 *  size of a spherical Unity game object.
 *  
 *  #### Topics
 *  
 *  This class initializes three ROS [topic] subscriptions; each for 
 *  manipulating a different property of the [sphere] object. Each topic name 
 *  consists of a relative [namespace] -- determined by the name of the 
 *  [sphere] game object -- followed by a property identifier. Since it is 
 *  used to determine the topic namespace, the game object must have a name 
 *  that agrees with ROS [naming conventions]. For example, given a Unity 
 *  Sphere primitive with the name ``Sphere1``, the following ROS topics would 
 *  be initialized:
 *  
 *  * ``Sphere1/position``
 *  * ``Sphere1/color``
 *  * ``Sphere1/radius``
 *  
 *  The message types for the topics are as follows:
 *  
 *  * position: [geometry_msgs/Point]
 *  * color: [std_msgs/ColorRGBA]
 *  * radius: [example_interfaces/Float64]
 *  
 *  #### Coordinate systems
 *  
 *  When a [position message] is received via the position topic subscription, 
 *  the 3D vector data contained in the message is first transformed, before 
 *  applying the new posiiton to the [sphere] object. This transformation is 
 *  necessary because ROS and Unity use different coordinate systems, by 
 *  default. As noted in the documentation for the [ROSGeometry] component of 
 *  the [ROS-TCP-Connector] package:
 *  
 *  > In Unity, the X axis points right, Y up, and Z forward. ROS, on the 
 *  > other hand, supports various coordinate frames: in the most 
 *  > commonly-used one, X points forward, Y left, and Z up. In ROS 
 *  > terminology, this frame is called "FLU" (forward, left, up), whereas 
 *  > the Unity coordinate frame would be "RUF" (right, up, forward).
 *  
 *  Instances of this class assume that incoming data uses the FLU coordinate 
 *  system. Vectors are converted the data to the RUF coordinate system before 
 *  use in the Unity scene.
 *  
 *  [position message]: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg
 *  [sphere]: @ref sphere
 *  [RosGeometry]: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md
 *  [ROS-TCP-Connector]: https://github.com/Unity-Technologies/ROS-TCP-Connector
 *  [topic]: https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html
 *  [namespace]: https://design.ros2.org/articles/topic_and_service_names.html#namespaces
 *  [naming conventions]: https://design.ros2.org/articles/topic_and_service_names.html
 *  [sphere]: @ref sphere
 *  [geometry_msgs/Point]: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg
 *  [std_msgs/ColorRGBA]: https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ColorRGBA.msg
 *  [example_interfaces/Float64]: https://github.com/ros2/example_interfaces/blob/master/msg/Float64.msg
 *  
 *  
 */
public class SphereNode : MonoBehaviour
{
    /** Connection to ROS2 provided by the ROS-TCP-Connector package.
     *  
     *  See the [ROS-TCP-Connector] documentation for further details.
     *  
     *  [ROS-TCP-Connector]: https://github.com/Unity-Technologies/ROS-TCP-Connector
     */
    private ROSConnection ros;
    
    /** 3D Sphere object that this object is associated with.
     *  
     *  Position, color, and size manipulations are requested via ROS topics 
     *  and are applied to this [game object]. The game object is generally 
     *  [attached] to this class by dragging and dropping in the Unity Editor.
     *  
     *  [game object]: https://docs.unity3d.com/Manual/class-GameObject.html
     *  [attached]: https://docs.unity3d.com/Manual/CreatingAndUsingScripts.html
     */
    //private GameObject sphere;
    
    /** Initialize ROS connection and create subscriptions for receiving 
     *  position, color, and size commands.
     *  
     *  The [Start] method is inherited from [MonoBehavior], and is called 
     *  before the first frame update in a Unity scene.
     *  Here, this method initializes three ROS [topic] subscriptions; each for 
     *  manipulating a property of the [sphere] object. Each topic name 
     *  consists of a relative [namespace] -- determined by the name of the 
     *  [sphere] game object -- followed by a property identifier. Since it is 
     *  used to determine the topic namespace, the game object must have a name 
     *  that agrees with ROS [naming conventions]. For example, 
     *  the color topic might have the name ``Sphere1/color``. 
     *  
     *  [Start]: https://docs.unity3d.com/ScriptReference/MonoBehaviour.Start.html
     *  [MonoBehavior]: https://docs.unity3d.com/Manual/class-MonoBehaviour.html
     *  [topic]: https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html
     *  [namespace]: https://design.ros2.org/articles/topic_and_service_names.html#namespaces
     *  [naming conventions]: https://design.ros2.org/articles/topic_and_service_names.html
     *  [sphere]: @ref sphere
     *  
     */
    void Start()
    {
        GameObject sphere = gameObject; // Inherited from MonoBehavior
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ColorMessage>(sphere.name + "/color", ColorChange);
        ros.Subscribe<PositionMessage>(sphere.name + "/position", 
                                       PositionChange);
        ros.Subscribe<RadiusMessage>(sphere.name + "/radius", RadiusChange);
    }
    
    /** ROS callback for color change messages.
     *  
     *  This method is invoked when this ROS2 node receives a message via the 
     *  color topic subscription. 
     *  The data in the [color message] is applied directly to the [sphere] 
     *  object that this class instance is associated with.
     *  
     *  [color message]: https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ColorRGBA.msg
     *  [sphere]: @ref sphere
     */
    void ColorChange(ColorMessage color_message)
    {
        GameObject sphere = gameObject; // Inherited from MonoBehavior
        Color32 color = new Color32((byte)color_message.r, 
                                    (byte)color_message.g, 
                                    (byte)color_message.b, 
                                    (byte)color_message.a);
        sphere.GetComponent<Renderer>().material.color = color;
    }
    
    /** ROS callback for position change messages.
     *  
     *  This method is invoked when this ROS2 node receives a message via the 
     *  position topic subscription. 
     *  The data in the [position message] is applied to the [sphere] 
     *  object that this class instance is associated with, after a applying 
     *  a transform. As noted in the documentation for the [ROSGeometry] 
     *  component of the [ROS-TCP-Connector] package:
     *  
     *  > In Unity, the X axis points right, Y up, and Z forward. ROS, on the 
     *  > other hand, supports various coordinate frames: in the most 
     *  > commonly-used one, X points forward, Y left, and Z up. In ROS 
     *  > terminology, this frame is called "FLU" (forward, left, up), whereas 
     *  > the Unity coordinate frame would be "RUF" (right, up, forward).
     *  
     *  This function assumes that the incoming data uses the FLU coordinate 
     *  system, and converts the data to the RUF coordinate system.
     *  
     *  [position message]: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg
     *  [sphere]: @ref sphere
     *  [RosGeometry]: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md
     *  [ROS-TCP-Connector]: https://github.com/Unity-Technologies/ROS-TCP-Connector
     */
    void PositionChange(PositionMessage position_message)
    {
        GameObject sphere = gameObject; // Inherited from MonoBehavior
        Vector3<FLU> position     = new Vector3<FLU>((float)position_message.x, 
                                                     (float)position_message.y, 
                                                     (float)position_message.z);
        sphere.transform.position = (Vector3) position.To<RUF>();
    }
    
     /** ROS callback for size change messages.
     *  
     *  This method is invoked when this ROS2 node receives a message via the 
     *  radius topic subscription. 
     *  The data in the [radius message] is applied directly to the [sphere] 
     *  object that this class instance is associated with.
     *  
     *  [radius message]: https://github.com/ros2/example_interfaces/blob/master/msg/Float64.msg
     *  [sphere]: @ref sphere
     */
   void RadiusChange(RadiusMessage radius_message)
    {
        GameObject sphere = gameObject; // Inherited from MonoBehavior
        float diameter = 2 * (float) radius_message.data;
        sphere.transform.localScale = new Vector3(diameter, diameter, diameter);
    }
}
