# Vicon control

This repository includes:

Markup :	* A GUI and command line tools to connect to and get frames from a computer running the propietary Vicon Tracker [software](https://www.vicon.com/products/software/tracker). 
		* A GUI and command line tools to connect to and receive sensor data from a peer running the provided robot software.

The software is designed for ROS Lunar running on Ubuntu 16.04.

## Pre-installation

Since the applications are build on the ROS framework, it is required to install and setup ROS to be able to run them. To do so, follow the instructions for your operating system in the [ROS Lunar installation guide](http://wiki.ros.org/lunar/Installation).

## Vicon installation

First of all, the Vicon datastream SDK has to be installed. This is achieved by running the corresponding bash script:

`sudo vicon_control/install_vicon_datastream_sdk`

**If** the Vicon datastream SDK is successfully installed, the Vicon software can be installed with:

'. vicon_control/install_vicon

*Note the dot before the script location. This script makes sure ROS discovers the packages in vicon_workspace for future bash sessions (e.g. terminals). The dot ensures that it is also the case for the current terminal.*

## Robot installation

### User computer

**TO BE ADDED**

### Robot

**TO BE ADDED**

## To be added

This repository is not yet complete. The following components are still in development and will be added in the future:

Markup :	* Workspace for robot GUI and command line tools to run on 
		* Sofware that is run on the BeagleBone Black
