#ifndef LOGGER_LOGGER_H
#define LOGGER_LOGGER_H

// Qt
#include <QWidget>			// QWidget

// Vicon visualizer
#include "logger/ui_logger.h"

class Logger : public QWidget {
	Q_OBJECT

	public:
		// Constructor
		Logger(QWidget* parent);

	private:
		Ui::Logger	ui_;	// Ui::Logger

	signals:
		// Fired when message has been acquired that has to be logged
		void logMessage(const QString& msg);

	public slots:
		// Fires when log request is acquired
		void log(const QString& msg);
};

#endif  // LOGGER_LOGGER_H
