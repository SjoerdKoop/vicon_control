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
* The robot software creates a peer which listens to the "reference_update" topic and converts the message into and UDP message to be send. This peer also listens to the robot and visualizer sensor data.
* The robot controller listens to the UDP reference message and should should control the plant accordingly. It also sends periodic data to the other end of the socket.

This results in the cascaded control loop as shown below:

![Control Loop](/images/control_loop.png){ height=100% }

# Installation

Good practice is to always keep your system up to date before installing any software:

```
sudo apt-get update
sudo apt-get dist-upgrade
```

## Prerequisites

Since the applications are build on the ROS framework, it is required to install and setup ROS to be able to run them. To do so, please follow the instructions described in the [ROS Lunar installation guide](http://wiki.ros.org/lunar/Installation) for your operating system. Another ROS version might still be able to run the software, but is not supported.

## Vicon software installation

First of all, the Vicon datastream SDK has to be installed. This is achieved by running the corresponding bash script:

`sudo vicon_control/install_vicon_datastream_sdk`

This script has be to be run as superuser, since it involves copying files to system directories (/usr/lib and /usr/include). If the Vicon datastream SDK is successfully installed, the Vicon software can be installed with:

`. vicon_control/install_vicon`

*Note the dot and space before the script location. This script makes sure ROS discovers the packages in vicon_workspace for future bash sessions (e.g. terminals). The dot ensures that this is also the case for the current terminal.*

## Robot software installation

TO BE ADDED

## Setting up the controller

TO BE ADDED

## Setting up the robot

TO BE ADDED

# To be added

This repository is not yet complete. The following components are still in development and will be added in the future:

* Workspace for robot GUI and command line tools to run on 
* Software that can be used to control the robot using Vicon Tracker data
* Sofware that is run on the BeagleBone Black
