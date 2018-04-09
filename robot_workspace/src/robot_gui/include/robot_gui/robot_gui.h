#ifndef ROBOT_GUI_ROBOT_GUI_H
#define ROBOT_GUI_ROBOT_GUI_H

// Robot GUI
#include <robot_gui/ui_robot_gui.h>			// Ui::RobotGUI

// rqt
#include <rqt_gui_cpp/plugin.h>				// rqt_gui_cpp::Plugin

// Qt
#include <QWidget>							// QWidget

namespace robot_gui
{

class RobotGUI : public rqt_gui_cpp::Plugin
{
	Q_OBJECT

	public:
		// Constructor
		RobotGUI();

		// Initializes the plugin
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	private:
		Ui::RobotGUI ui_;		// The UI
		QWidget* widget_;		// The main widget
	};
} // namespace robot_gui

#endif // ROBOT_GUI_ROBOT_GUI_H
