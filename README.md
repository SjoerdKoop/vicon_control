# Vicon control

This repository includes:

* A GUI and command line tools to connect to and get frames from a computer running the propietary [Vicon Tracker](https://www.vicon.com/products/software/tracker)
* A GUI and command line tools to connect to and receive sensor data from a peer running the provided robot software
* Template software that can be used to close the control loop using Vicon Tracker data to create a reference for the robot
* Template software to be run on the robot which listens to a reference and sends sensor data

To achieve modularity of each package, several open source files have been added from third parties:

* *vicon_workspace/vicon_datastream_sdk*: Holds relevant headers and libraries from the [Vicon Datastream SDK](https://www.vicon.com/downloads/utilities-and-sdk/datastream-sdk)
* *robot_control/pru-cgt*: Holds binaries, headers and libraries generated by [Texas Instruments PRU Code Generation Tools](http://software-dl.ti.com/codegen/non-esd/downloads/download.htm#PRU)
* *robot_control/pru-icss*: Holds the PRU linker command file and relevant headers of the PRU-ICSS generated by the [Linux Processor SDK for AM335x](http://software-dl.ti.com/processor-sw/esd/PROCESSOR-SDK-LINUX-AM335X/latest/index_FDS.html)

## Target systems

The four components have been designed for ROS Lunar running on Ubuntu 16.04. The *robot_control* package has scripts and generates binaries and firmware to be used on a [BeagleBone Black](https://beagleboard.org/black)

## Overview

The recommend setup scheme is shown below:

![General Scheme](/images/general_scheme.png)

In this scheme, the components are set up in a modular fashion so that components can be changed without influencing the others.

* The vicon software creates a client using the [Vicon datastream SDK](https://www.vicon.com/products/software/datastream-sdk) and generates an array of objects over the ROS topic [*object_update*](https://github.com/SjoerdKoop/vicon_control#ros-object_update). A replacement would only have to satisfy publishing the same data over the same topic.
* The vision controller listens to the [*object_update*](https://github.com/SjoerdKoop/vicon_control#ros-object_update) topic and should generate a suitable reference from the detected objects. This reference should be an array of floats, sent over the ROS topic [*reference_update*](https://github.com/SjoerdKoop/vicon_control#ros-reference_update).
* The robot software creates a peer which listens to the [*reference_update*](https://github.com/SjoerdKoop/vicon_control#ros-reference_update) topic and converts the message into an [UDP message](https://github.com/SjoerdKoop/vicon_control#udp-reference) to be send. This peer also listens to the robot and visualizes [sensor data](https://github.com/SjoerdKoop/vicon_control#udp-sensor-data).
* The robot controller listens to the [UDP reference message](https://github.com/SjoerdKoop/vicon_control#udp-reference) and should should control the plant accordingly. It also sends [periodic data](https://github.com/SjoerdKoop/vicon_control#udp-sensor-data) to the other end of the socket.

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

To cross compile ARM binaries on Ubuntu 16.04, the appropriate toolchains have to be installed. The toolchains for C and C++ can be installed with:

```
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

## Installion of ROS workspaces

Each workspace can be installed by running the corresponding *installation* script:

```
. vicon_control/robot_workspace/installation
. vicon_control/vicon_workspace/installation 
. vicon_control/vision_control/installation 
```

*Before the installation scripts, there is a dot followed by a space before the actual file. This scripts makes sure ROS discovers the packages in the corresponding workspace for future bash sessions (e.g. terminals). The dot ensures that this is also the case for the current terminal.*

These scripts calls *catkin_make*. If *catkin_make* can be run (i.e. ROS has been properly installed), the scripts will source the package path to the user's *~/.bashrc*.

## Setting up the connection

### User PC

Currently, the Vicon Tracker PC has an IP address of 192.168.10.1 and is sending messages over port 801 on subnet (with netmask) 255.255.254.0. Consequently, the user PC has to have an IP address of 192.168.10.x (1 < x < 255) and must reside in the same subnet of 255.255.254.0. Example settings in the standard ubuntu desktop environment are shown below:

![Connection Settings](/images/connection_settings.png)

*When revising the settings, the GUI converted 255.255.254.0 to 23. This is normal and means that it is properly set up.*

### Robot

On the BeagleBoneBlack, network configuration is done by editing the */etc/network/interfaces* file. A proper static IP address on the same subnet can be set by appending the settings below to the file. The placeholder &lt;device&gt; should be set to the communication device that connects to the user's network (can be checked with *ifconfig*). The value *x* should be set so that it is not the same IP address as the user PC or Vicon Tracker PC.

```
## Set static <device> ip address
auto <device>                           # Enables device on startup
iface <device> inet static              # Sets device to have a static IP address
        address 192.168.10.x            # Set static IP address
        netmask 255.255.254.0           # Set netmask
        gateway 192.168.10.254          # Set gateway
        dns-nameservers 8.8.8.8         # Set DNS nameserver 1
        dns-nameserver 8.8.4.4          # Set DNS nameserver 2
```

# Usage

For documentation on running nodes from a terminal, please refer to [Executables](https://github.com/SjoerdKoop/vicon_control#executables)

## Vision control design

Controllers should inherit from *VisionController* and should override the *objectsToReference* function. An example implementation can be seen as the example controller in the package *vision_control*.


# Communication

As shown in the general scheme, communication is an integral part connecting all the subsystems. Communication from the cameras to the Vicon Tracker, and from the Vicon Tracker to the user pc is predefined by Vicon. Therefore, four additional communication instances are defined.

## ROS: object_update

To communicate from the Vicon workspace to the vision controller, a *ros_object_array* message is send over topic *object_update*. This message consist of and array of *ros_object*:

```
vicon_tools/ros_object_array.msg
--------------------------------
ros_object[] objects
```

Where a *ros_object* is defined by a name, three translational values and three rotational values:

```
vicon_tools/ros_object.msg 
--------------------------
string name
float64 x
float64 y
float64 z
float64 rx
float64 ry
float64 rz
```

The name will either be the defined name in the Vicon Tracker software for objects or "marker&lt;id&gt;" for markers. The ROS data type of *float64* coincides with the *c++* datatype of *double*. Since object data from the Vicon Tracker arrives as doubles, this type will also be used for the ROS message. Additionally for markers, the rotational values will always be 0, since it is impossible to deduce rotation from a single marker.

## ROS: reference_update

To communicate from the vision controller to the robot workspace, a standard *[Float32MultiArray](https://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html)* message is send over topic *reference_update*. This message can be found in the *std_msgs* package and is defined as:

```
std_msgs/Float32MultiArray.msg
------------------------------
MultiArrayLayout layout
float32[] data
```

Since we only use one dimension, the *layout* variable is not relevant. A *float32* in ROS coincides with a *float* in *c++*, this is used instead of a double to effectively halve the magnitude of communication messages without losing relevant accuracy. Reference values should be added (*data.push_back(value)*) in the same order as they are to be read by the robot.

## UDP: reference

Communication of the reference generated by the vision controller to the robot is done over an UDP socket. The message consists of the number of total reference variables and each of those variables, as shown below:

![Reference UDP message](/images/reference_udp_message.png)

## UDP: sensor data

Communication of sensor data to the user is done over the same UDP socket. The message consists of the number of total sensor data, and the names and values of each sensor, as shown below:

![Sensor Data UDP message](/images/sensor_data_udp_message.png)

Since a string can have variable length, a consensus has to be made between the server and the client. Currently, the consensus is that the name string is 16 bytes long.

# Executables

The software packets create several useful command line tools for debugging or running without a GUI. Usage of these tools is described below:

* Vicon workspace tools (vicon_tools): These tools are executables that connect to a Vicon datastream at a given IP address and port.
	* `rosrun vicon_tools dual <Vicon datastream IP address> <Vicon datastream port> <number of markers>`
	* `rosrun vicon_tools markers <Vicon datastream IP address> <Vicon datastream port> <number of markers>`
	* `rosrun vicon_tools objects <Vicon datastream IP address> <Vicon datastream port>`
* Vision control tools (vision_control_tools): These tools print object updates and send reference updates.
	* `rosrun vision_control_tools object_subscriber`
	* `rosrun vision_control_tools reference_publisher` 
* Robot Workspace tools (robot_tools): These tools consist of executables that communicate with or allows the user to send a set reference the robot at a given IP address and port and an executable that prints data updates.
	* `rosrun robot_tools communicate <robot IP address> <robot port>`
	* `rosrun robot_tools data_subscriber`
	* `rosrun robot_tools send_reference <robot IP address> <robot port>`
* Robot control tools: These tools are to be run on the robot. They consist of a server of data updates and a listener to reference updates from a specific host at a given IP address and port and a tool that samples the shared memory with the PRU and prints the current data at an index.
	* `data_server <user IP address> <user port>`
	* `sudo read_shared_memory <index>`\
	  *(This executable has to be run as a super user, since it involves opening a global memory map.)*
	* `reference_client <user IP address> <user port>`
	* `sudo write_shared_memory <index> <value>`\
          *(This executable has to be run as a super user, since it involves opening a global memory map.)*
