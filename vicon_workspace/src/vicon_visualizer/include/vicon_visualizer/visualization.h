#ifndef VICON_VISUALIZER_VISUALIZATION_H
#define VICON_VISUALIZER_VISUALIZATION_H

// ROS
#include "ros/ros.h"							// ros::*
#include "visualization_msgs/Marker.h"			// visualization_msgs::Marker

// System
#include <map>									// std::map
#include <string>								// std::string

// Vicon tools
#include "vicon_tools/ros_object_array.h"		// vicon_tools::ros_object_array

namespace Visualization {
	static std::map<std::string, int> id_map;		// Mapping of name to ID
	static ros::Publisher marker_pub;				// Publisher to marker_update
	static int shape_;								// The shape used for visualization
	static int scale_x_;							// Scale in the X dimension
	static int scale_y_;							// Scale in the Y dimension
	static int scale_z_;							// Scale in the Z dimension

	// Initializes the namespace
	void init();

	// Creates a marker
	visualization_msgs::Marker createMarker(const vicon_tools::ros_object object, int id);

	// Gets the ID of the object
	int getObjectID(std::string name);

	// Handle for when an update message has arrived
	void onObjectUpdate(const vicon_tools::ros_object_array::ConstPtr& msg);

	// Sets the default marker
	void setDefaultMarkerSettings(visualization_msgs::Marker* marker);

	// Sets X scale
	void setScaleX(int scale_x);

	// Sets Y scale
	void setScaleY(int scale_y);

	// Sets Z scale
	void setScaleZ(int scale_z);

	// Sets the default shape
	void setShape(int index);

	// Terminates the namespace
	void terminate();
}

#endif // VICON_VISUALIZER_VISUALIZATION_H