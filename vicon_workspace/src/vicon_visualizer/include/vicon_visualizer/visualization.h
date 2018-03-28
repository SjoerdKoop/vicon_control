#ifndef VICON_VISUALIZER_VISUALIZATION_H
#define VICON_VISUALIZER_VISUALIZATION_H

// ROS
#include "ros/ros.h"							// ros::*
#include "visualization_msgs/Marker.h"			// visualization_msgs::Marker

// Vicon tools
#include "vicon_tools/remove_objects.h"			// vicon_tools::remove_objects
#include "vicon_tools/ros_object_array.h"		// vicon_tools::ros_object_array

// Ball size
#define BALL_SIZE 500

namespace Visualization {
	static ros::Publisher marker_pub;		// Publisher to marker_update

	// Initializes the namespace
	void init();

	// Creates a marker
	visualization_msgs::Marker createMarker(const vicon_tools::ros_object object);

	// Creates a marker that removes it from the visualizer
	visualization_msgs::Marker createRemovalMarker(int id);

	// Handle for when a remove message has arrived
	void onObjectRemove(const vicon_tools::remove_objects::ConstPtr& msg);

	// Handle for when an update message has arrived
	void onObjectUpdate(const vicon_tools::ros_object_array::ConstPtr& msg);

	// Set the default marker
	void setDefaultMarkerSettings(visualization_msgs::Marker* marker);

	// Terminates the namespace
	void terminate();
}

#endif // VICON_VISUALIZER_VISUALIZATION_H