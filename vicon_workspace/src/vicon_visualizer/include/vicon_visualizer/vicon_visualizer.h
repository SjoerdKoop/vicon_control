#ifndef VICON_VISUALIZER_VICON_VISUALIZER_H
#define VICON_VISUALIZER_VICON_VISUALIZER_H

// Qt
#include <QWidget>														// QWidget

// ROS
#include "ros/ros.h"													// ros::*
#include "visualization_msgs/Marker.h"									// visualization_msgs::Marker

// RViz
#include "rviz/display.h"												// rviz::Display
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"	// rviz::OrbitViewController

// Vicon visualizer
#include "vicon_visualizer/ui_vicon_visualizer.h"						// Ui::ViconVisualizer

class ViconVisualizer : public QWidget {
	Q_OBJECT

	public:
		// Constructor
		ViconVisualizer(QWidget* parent);

		// Destructor
		~ViconVisualizer();

	private:
		bool isAcceptingMarkers;						// Holds whether the marker dispaly is accepting marker messages
		rviz::Display* markers_;						// Marker display
		visualization_msgs::Marker defaultMarker;		// Default marker
		ros::Subscriber objectSub;     					// Subscriber to object_update
		ros::Subscriber removeSub;     					// Subscriber to object_remove
		Ui::ViconVisualizer	ui_;						// Ui::ViconVisualizer


	private slots:
		// Fires when the pause button is pressed
		void pauseButtonPressed();

		// Fires when the reset button is pressed
		void resetButtonPressed();
};

#endif  // VICON_VISUALIZER_VICON_VISUALIZER_H
