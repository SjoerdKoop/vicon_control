// Declarations
#include "vicon_gui/vicon_gui.h"

// Qt
#include <QStringList>
#include <QVBoxLayout>

// ROS
#include <pluginlib/class_list_macros.h>

namespace vicon_gui
{

ViconGUI::ViconGUI()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Set object name
  setObjectName("GUI");
}

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

void ViconGUI::shutdownPlugin()
{
  // unregister all publishers here
}

void ViconGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void ViconGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace vicon_gui
PLUGINLIB_EXPORT_CLASS(vicon_gui::ViconGUI, rqt_gui_cpp::Plugin)