// Declarations
#include "logger/logger.h"

// Constructor
Logger::Logger(QWidget* parent) : QWidget(parent) {
	// Set object name
	setObjectName("Vicon visualizer");

	// Extend the widget with all attributes and children from UI file
	ui_.setupUi(this);

	// Set read only
	ui_.log_->setReadOnly(true);

	// Connect UI signals to slots
	connect(ui_.clear_button_, SIGNAL(pressed()), ui_.log_, SLOT(clear()));
	connect(this, SIGNAL(logMessage(const QString&)), ui_.log_, SLOT(appendPlainText(const QString&)));
}

// Fires when log request is acquired
void Logger::log(const QString& msg) {
	// Emit logging of message in the text box
	emit logMessage(msg);
}