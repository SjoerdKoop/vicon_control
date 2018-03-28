#ifndef VICON_GUI_VICON_GUI_H
#define VICON_GUI_VICON_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <vicon_gui/ui_vicon_gui.h>

// Qt
#include <QWidget>

namespace vicon_gui
{

class ViconGUI
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  ViconGUI();
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

  Ui::ViconGUI ui_;
  QWidget* widget_;
};
}  // namespace vicon_gui
#endif  // VICON_GUI_VICON_GUI_H
