#ifndef VICON_GUI_VICON_GUI_H
#define VICON_GUI_VICON_GUI_H

// rqt
#include <rqt_gui_cpp/plugin.h>				// rqt_gui_cpp::Plugin

// Vicon GUI
#include <vicon_gui/ui_vicon_gui.h>			// Ui::ViconGUI

// Qt
#include <QWidget>							// QWidget

namespace vicon_gui
{

class ViconGUI : public rqt_gui_cpp::Plugin
{
	Q_OBJECT

	public:
		// Constructor
		ViconGUI();

		// Initializes the plugin
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	private:
		Ui::ViconGUI ui_;		// The UI
		QWidget* widget_;		// The main widget
	};
}  // namespace vicon_gui

#endif  // VICON_GUI_VICON_GUI_H
