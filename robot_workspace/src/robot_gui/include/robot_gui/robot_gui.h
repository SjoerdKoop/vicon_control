#ifndef ROBOT_GUI_ROBOT_GUI_H
#define ROBOT_GUI_ROBOT_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <robot_gui/ui_robot_gui.h>

// Qt
#include <QWidget>

namespace robot_gui
{

class RobotGUI
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  RobotGUI();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:

  Ui::RobotGUI ui_;
  QWidget* widget_;
};
}  // namespace robot_gui
#endif  // ROBOT_GUI_ROBOT_GUI_H
