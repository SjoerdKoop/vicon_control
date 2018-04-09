// Declarations
#include "vicon_gui/vicon_gui.h"



// ROS
#include <pluginlib/class_list_macros.h>	// PLUGINLIB_EXPORT_CLASS

namespace vicon_gui
{

// Constructor
ViconGUI::ViconGUI() : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Set object name
  setObjectName("GUI");
}

// Initializes the plugin
void ViconGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // Access standalone command line arguments
  QStringList argv = context.argv();
  
  // Create QWidget
  widget_ = new QWidget();

  // Extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // Set window title
  widget_->setWindowTitle("Vicon GUI");

  // Add widget to the user interface
  context.addWidget(widget_);

  // Connect signals to slots
  connect(ui_.vicon_peer_, SIGNAL(log(const QString&)), ui_.vicon_logger_, SLOT(log(const QString&)));
}

}  // namespace vicon_gui
PLUGINLIB_EXPORT_CLASS(vicon_gui::ViconGUI, rqt_gui_cpp::Plugin)