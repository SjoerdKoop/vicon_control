// Declarations
#include "vicon_visualizer/vicon_visualizer.h"

// Rviz
#include "rviz/view_manager.h"						// rviz::ViewManager
#include "rviz/visualization_manager.h"				// rviz::VisualizationManager

// System
#include <math.h>									// M_PI

// Vicon visualizer
#include "vicon_visualizer/remove_objects.h"		// vicon_visualizer::remove_objects
#include "vicon_visualizer/ros_object_array.h"		// vicon_visualizer::ros_object_array
#include "vicon_visualizer/visualization.h"			// Visualization::*

// Constructor
ViconVisualizer::ViconVisualizer(QWidget* parent) : QWidget(parent) {
	// Set object name
	setObjectName("Vicon visualizer");

	// Initializer visualization namespace
	Visualization::init();

	// Set up subscribers
	ros::NodeHandle n;
	objectSub = n.subscribe<vicon_visualizer::ros_object_array>("object_update", 1, Visualization::onObjectUpdate);
	removeSub = n.subscribe<vicon_visualizer::remove_objects>("object_remove", 1, Visualization::onObjectRemove);

	// Extend the widget with all attributes and children from UI file
	ui_.setupUi(this);

	// Create RViz visualization manager
	rviz::VisualizationManager* visMan = new rviz::VisualizationManager(ui_.panel_);
	visMan->setFixedFrame("/object_frame");
	ui_.panel_->initialize(visMan->getSceneManager(), visMan);
	visMan->initialize();
	visMan->startUpdate();

	// Create grid display
	rviz::Display* grid = visMan->createDisplay("rviz/Grid", "grid", true);
	grid->subProp("Cell Size")->setValue(1000);

	// Create markers display
	markers_ = visMan->createDisplay("rviz/Marker", "markers", true);
	markers_->setTopic("object_marker", "visualization_msgs::Marker");
	isAcceptingMarkers = true;

	// Set up a proper initial view
	// Positive X to the left, positive Y in the distance, positive Z up
	rviz::OrbitViewController* currentViewController = (rviz::OrbitViewController*) visMan->getViewManager()->getCurrent();
	currentViewController->pitch(M_PI / 8.0f);
	currentViewController->yaw(3.0f * M_PI / 4.0f);
	currentViewController->zoom(-10000.0f);

	// Connect UI signals to slots
	connect(ui_.pause_button_, SIGNAL(pressed()), this, SLOT(pauseButtonPressed()));
	connect(ui_.reset_button_, SIGNAL(pressed()), this, SLOT(resetButtonPressed()));
}

// Destructor
ViconVisualizer::~ViconVisualizer() {
	// Shutdown subscribers
	objectSub.shutdown();
	removeSub.shutdown();

	// Terminate Visualization namespace
	Visualization::terminate();
}

// Fires when the pause button is pressed
void ViconVisualizer::pauseButtonPressed() {
	if (isAcceptingMarkers) {
		markers_->setTopic("", NULL);
		isAcceptingMarkers = false;
		ui_.pause_button_->setText("Resume");
	}
	else {
		markers_->setTopic("object_marker", "visualization_msgs::Marker");
		isAcceptingMarkers = true;
		ui_.pause_button_->setText("Pause");
	}
}

// Fires when the reset button is pressed
void ViconVisualizer::resetButtonPressed() {
	markers_->reset();
}