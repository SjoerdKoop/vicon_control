// Declarations
#include "vicon_visualizer/vicon_visualizer.h"

// Rviz
#include "rviz/view_manager.h"						// rviz::ViewManager
#include "rviz/visualization_manager.h"				// rviz::VisualizationManager

// System
#include <math.h>									// M_PI

// Vicon tools
#include "vicon_tools/ros_object_array.h"			// vicon_tools::ros_object_array

// Vicon visualizer
#include "vicon_visualizer/visualization.h"			// Visualization::*

// Constructor
ViconVisualizer::ViconVisualizer(QWidget* parent) : QWidget(parent)
{
	// Set object name
	setObjectName("Vicon visualizer");

	// Extend the widget with all attributes and children from UI file
	ui_.setupUi(this);

	// Initialize visualization namespace
	Visualization::init();

	// Set parameters of visualization namespace
	Visualization::setShape(ui_.shape_combo_->currentIndex());
	Visualization::setScaleX(ui_.scale_x_edit_->value());
	Visualization::setScaleY(ui_.scale_y_edit_->value());
	Visualization::setScaleZ(ui_.scale_z_edit_->value());

	// Set up subscribers
	ros::NodeHandle n;
	object_sub_ = n.subscribe<vicon_tools::ros_object_array>("object_update", 1, Visualization::onObjectUpdate);

	// Create RViz visualization manager
	rviz::VisualizationManager* vis_man = new rviz::VisualizationManager(ui_.panel_);
	vis_man->setFixedFrame("/object_frame");
	ui_.panel_->initialize(vis_man->getSceneManager(), vis_man);
	vis_man->initialize();
	vis_man->startUpdate();

	// Create grid display
	rviz::Display* grid = vis_man->createDisplay("rviz/Grid", "grid", true);
	grid->subProp("Cell Size")->setValue(1000);

	// Create markers display
	markers_ = vis_man->createDisplay("rviz/Marker", "markers", true);
	markers_->setTopic("object_marker", "visualization_msgs::Marker");
	isAcceptingMarkers_ = true;

	// Set up a proper initial view
	// Positive X to the left, positive Y in the distance, positive Z up
	rviz::OrbitViewController* current_view_vontroller = (rviz::OrbitViewController*) vis_man->getViewManager()->getCurrent();
	current_view_vontroller->pitch(M_PI / 8.0f);
	current_view_vontroller->yaw(3.0f * M_PI / 4.0f);
	current_view_vontroller->zoom(-10000.0f);

	// Button signals
	connect(ui_.pause_button_, SIGNAL(pressed()), this, SLOT(pauseButtonPressed()));
	connect(ui_.reset_button_, SIGNAL(pressed()), this, SLOT(resetButtonPressed()));

	// Combo box signal
	connect(ui_.shape_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(shapeComboCurrentIndexChanged(int)));

	// Edit box signals
	connect(ui_.scale_x_edit_, SIGNAL(editingFinished()), this, SLOT(scaleXEditingFinished()));
	connect(ui_.scale_y_edit_, SIGNAL(editingFinished()), this, SLOT(scaleYEditingFinished()));
	connect(ui_.scale_z_edit_, SIGNAL(editingFinished()), this, SLOT(scaleZEditingFinished()));
}

// Destructor
ViconVisualizer::~ViconVisualizer()
{
	// Shutdown subscribers
	object_sub_.shutdown();

	// Terminate Visualization namespace
	Visualization::terminate();
}

/* Button slots */

// Fires when the pause button is pressed
void ViconVisualizer::pauseButtonPressed()
{
	if (isAcceptingMarkers_) 
	{
		markers_->setTopic("", NULL);
		isAcceptingMarkers_ = false;
		ui_.pause_button_->setText("Resume");
	}
	else 
	{
		markers_->setTopic("object_marker", "visualization_msgs::Marker");
		isAcceptingMarkers_ = true;
		ui_.pause_button_->setText("Pause");
	}
}

// Fires when the reset button is pressed
void ViconVisualizer::resetButtonPressed()
{
	// Clear stored ID map
	Visualization::id_map.clear();

	// Reset markers
	markers_->reset();
}

/* Combo box slot */

// Fires when shape combo box has changed
void ViconVisualizer::shapeComboCurrentIndexChanged(int index)
{
	// Update shape
	Visualization::setShape(ui_.shape_combo_->currentIndex());

	// Reset markers
	markers_->reset();
}

/* Edit box slots */

// Fires when X scale editiing is finished
void ViconVisualizer::scaleXEditingFinished()
{
	// Clear focus
	ui_.scale_x_edit_->clearFocus();

	// Set value
	Visualization::setScaleX(ui_.scale_x_edit_->value());

	// Reset markers
	markers_->reset();
}

// Fires when Y scale editiing is finished
void ViconVisualizer::scaleYEditingFinished()
{	
	// Clear focus
	ui_.scale_y_edit_->clearFocus();

	// Set value
	Visualization::setScaleY(ui_.scale_y_edit_->value());

	// Reset markers
	markers_->reset();
}

// Fires when Z scale editiing is finished
void ViconVisualizer::scaleZEditingFinished()
{	
	// Clear focus
	ui_.scale_z_edit_->clearFocus();

	// Set value
	Visualization::setScaleZ(ui_.scale_z_edit_->value());

	// Reset markers
	markers_->reset();
}