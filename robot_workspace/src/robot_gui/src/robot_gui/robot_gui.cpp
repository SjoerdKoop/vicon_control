// Declarations
#include "robot_gui/robot_gui.h"

// ROS
#include <pluginlib/class_list_macros.h>	// PLUGINLIB_EXPORT_CLASS

namespace robot_gui
{

// Constructor
RobotGUI::RobotGUI(): rqt_gui_cpp::Plugin(), widget_(0)
{
  // Set object name
  setObjectName("GUI");
}

// Initializes the plugin
void RobotGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // Access standalone command line arguments
  QStringList argv = context.argv();
  
  // Create QWidget
  widget_ = new QWidget();

  // Extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // Set window title
  widget_->setWindowTitle("Robot GUI");

  // Add widget to the user interface
  context.addWidget(widget_);
}

}  // namespace robot_gui
PLUGINLIB_EXPORT_CLASS(robot_gui::RobotGUI, rqt_gui_cpp::Plugin)