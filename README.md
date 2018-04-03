# Vicon control

This repository includes:

* A GUI and command line tools to connect to and get frames from a computer running the propietary [Vicon Tracker](https://www.vicon.com/products/software/tracker). 
* A GUI and command line tools to connect to and receive sensor data from a peer running the provided robot software.
* Template software that can be used to close the control loop using Vicon Tracker data to create a reference for the robot.
* Template software to be run on the robot which listens to a reference and sends sensor data.

The user software is designed for ROS Lunar running on Ubuntu 16.04. The software on the robot is designed for a [BeagleBone Black](https://beagleboard.org/black)

## Overview

The recommend setup scheme is shown below:

![General Scheme](/images/general_scheme.png)

In this scheme, the components are set up in a modular fashion so that components can be changed without influencing the others.

* The vicon software creates a client using the [Vicon datastream SDK](https://www.vicon.com/products/software/datastream-sdk) and generates an array of objects over the ROS topic "object_update". A replacement would only have to satisfy publishing the same data over the same topic.
* The vision controller listens to the "object_update" topic and should generate a suitable reference from the detected objects. This reference should be an array of floats, sent over the ROS topic "reference_update".
* The robot software creates a peer which listens to the "reference_update" topic and converts the message into and UDP message to be send. This peer also listens to the robot and visualizes sensor data.
* The robot controller listens to the UDP reference message and should should control the plant accordingly. It also sends periodic data to the other end of the socket.

This results in the cascaded control loop as shown below:

![Control Loop](/images/control_loop.png)

# Installation

Good practice is to always keep your system up to date before installing any software:

```
sudo apt-get update
sudo apt-get dist-upgrade
```

## Prerequisites

Since the applications are build on the ROS framework, it is required to install and setup ROS to be able to run them. To do so, please follow the instructions described in the [ROS Lunar installation guide](http://wiki.ros.org/lunar/Installation) for your operating system. Another ROS version might still be able to run the software, but is not supported.

## Vicon workspace installation

The Vicon software can be installed with:

`. vicon_control/vicon_workspace/install`

*Note the dot and space before the script location. This script makes sure ROS discovers the packages in vicon_workspace for future bash sessions (e.g. terminals). The dot ensures that this is also the case for the current terminal.*

The next step is to properly set the connection to the Vicon Tracker PC. Currently, this PC has an IP address of 192.168.10.1 and is sending messages over port 801 on subnet (with netmask) 255.255.254.0. Consequently, the user PC has to have an IP address of 192.168.10.x (1 < x < 255) and must reside in the same subnet of 255.255.254.0. Example settings in the standard ubuntu desktop environment are shown below:

![Connection Settings](/images/connection_settings.png)

*When revising the settings, the GUI converted 255.255.254.0 to 23. This is completely normal and means that it is properly set up.*

## Robot workspace installation

## Setting up the controller

## Setting up the robot

# Communication

As shown in the general scheme, communication is an integral part connecting all the subsystems. Communication from the cameras to the Vicon Tracker, and from the Vicon Tracker to the user pc is predefined by Vicon. Therefore, four communication instances are defined.

## ROS: object_update

To communicate between the Vicon software and the vision controller, a *ros_object_array* message is send over topic *object_update*. This message consist of and array of *ros_object*:

```
vicon_control/vicon_workspace/msg/ros_object_array.msg

ros_object[] objects
```

Where a *ros_object* is defined by a name, three translational values and three rotational values:

```
vicon_control/vicon_workspace/msg/ros_object.msg 

string name
float64 x
float64 y
float64 z
float64 rx
float64 ry
float64 rz
```

The name will either be the defined name in the Vicon Tracker software for objects or "Marker &lt; id &gt; " for markers. The ROS data type of *float64* coincides with the *c++* datatype of *double*. Additionally for markers, the rotational values will always be 0, since it is impossible to deduce rotation from a single marker.

## ROS: reference_update

## UDP: reference
 
## UDP: sensor data

# To be added

This repository is not yet complete. The following components are still in development and will be added in the future:

* Workspace for robot GUI and command line tools to run on 
* Software that can be used to control the robot using Vicon Tracker data
* Sofware that is run on the BeagleBone Black
