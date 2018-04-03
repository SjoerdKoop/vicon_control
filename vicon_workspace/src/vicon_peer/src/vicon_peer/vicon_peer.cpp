// Declarations
#include "vicon_peer/vicon_peer.h"

// Export
#include <pluginlib/class_list_macros.h>		// PLUGINLIB_EXPORT_CLASS

// Qt
#include <QHostAddress>							// QHostAddress

// Constructor
ViconPeer::ViconPeer(QWidget* parent) : QWidget(parent)
{
	// Extend the widget with all attributes and children from UI file
	ui_.setupUi(this);

	// Set window title
	setWindowTitle("Robot peer");

	// Set object name
	setObjectName("Robot peer");

	// Button signal
	connect(ui_.button_, SIGNAL(pressed()), this, SLOT(buttonPressed()));

	// Check box signals
	connect(ui_.markers_check_, SIGNAL(stateChanged(int)), this, SLOT(markersCheckBoxStateChanged(int)));
	connect(ui_.objects_check_, SIGNAL(stateChanged(int)), this, SLOT(objectsCheckBoxStateChanged(int)));

	// Edit box signals
	connect(ui_.ip_edit_, SIGNAL(returnPressed()), this, SLOT(ipReturnPressed()));
	connect(ui_.ip_edit_, SIGNAL(textChanged(const QString&)), this, SLOT(ipTextChanged(const QString&)));
	connect(ui_.markers_edit_, SIGNAL(editingFinished()), this, SLOT(markersEditingFinished()));
	connect(ui_.markers_edit_, SIGNAL(valueChanged(int)), this, SLOT(markersValueChanged(int)));
	connect(ui_.port_edit_, SIGNAL(editingFinished()), this, SLOT(portEditingFinished()));

	// Process signals
	connect(&this->ping_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(connectIfActiveHost(int, QProcess::ExitStatus)));
	connect(&this->client_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(onClientDisconnect(int, QProcess::ExitStatus)));
	
	// Set programs for processes
	client_process_.setProgram("rosrun");
	ping_process_.setProgram("ping");

	updateButton();

	// Disable markers edit box if needed
	if (!ui_.markers_check_->isChecked())
	{
		ui_.markers_edit_->setEnabled(false);
	}

	//ui_.markers_edit_->setKeyboardTracking(false);
}

// Destructor
ViconPeer::~ViconPeer()
{
	// Kill processes
	client_process_.kill();
	ping_process_.kill();

	// Wait for processes
	client_process_.waitForFinished();
	ping_process_.waitForFinished();
}

// Fires when the button is pressed
void ViconPeer::buttonPressed()
{
	if (client_process_.state() == QProcess::NotRunning)
	{
		// If a valid IPv4 address is set
		if (hasValidIPAddress())
		{
			// Connect
			connectToVicon();
		}
	}
	else
	{
		disconnectFromVicon();
	}
}

// Connects to the robot
void ViconPeer::connectToVicon() 
{
	if (client_process_.state() != QProcess::NotRunning) 
	{
		disconnectFromVicon();
	}

	// Setup ping process and start
	ping_process_.setArguments(QStringList() << "-w1" << ui_.ip_edit_->text());
	ping_process_.start();

	// Log connection message
	log("Connecting to " + ui_.ip_edit_->text() + " at port " + QString::number(ui_.port_edit_->value()) + "...");

	// Update button state
	updateButton();

	// Disable input
	disableInput();
}

// Disables the GUI
void ViconPeer::disableInput()
{
	ui_.ip_edit_->setEnabled(false);
	ui_.markers_check_->setEnabled(false);
	ui_.markers_edit_->setEnabled(false);
	ui_.objects_check_->setEnabled(false);
	ui_.port_edit_->setEnabled(false);
}

// Disconnects from Vicon
void ViconPeer::disconnectFromVicon()
{
	// Disconnect -> kill client process
	client_process_.kill();
	client_process_.waitForFinished();

	// Change button text
	ui_.button_->setText("Connect");

	// Enable input
	enableInput();

	// Disable button if needed
	if (!hasValidIPAddress())
	{
		ui_.button_->setEnabled(false);
	}
}

// Enables input
void ViconPeer::enableInput()
{
	ui_.ip_edit_->setEnabled(true);
	ui_.markers_check_->setEnabled(true);

	if (ui_.markers_check_->isChecked()) {
		ui_.markers_edit_->setEnabled(true);
	}

	ui_.objects_check_->setEnabled(true);
	ui_.port_edit_->setEnabled(true);
}

// Check whether the IP edit box has a valid IPv4 address
bool ViconPeer::hasValidIPAddress()
{
	// Convert input to IP address
	QHostAddress ipAddress(ui_.ip_edit_->text());

	//Return whether it is a valid IPv4 address
	return QAbstractSocket::IPv4Protocol == ipAddress.protocol();
}


/* Button slot */

// Updates button
void ViconPeer::updateButton()
{
	// If there is no client running
	if (client_process_.state() == QProcess::NotRunning && ping_process_.state() == QProcess::NotRunning)
	{
		// Set text to "Connect"
		ui_.button_->setText("Connect");

		// If a valid mode is selected
		if ((ui_.markers_check_->isChecked() && ui_.markers_edit_->value() > 0) || ui_.objects_check_->isChecked())
		{
			// If a valid IP address is given
			if (hasValidIPAddress())
			{	
				// Enable button
				ui_.button_->setEnabled(true);
			}
			// If no valid IP address is given
			else
			{	
				// Disable button
				ui_.button_->setEnabled(false);
			}
		}
		// If no valid mode is selected
		else 
		{
			// Disable button
			ui_.button_->setEnabled(false);
		}
	}
	// If a client is running
	else
	{
		// Set text to "Disconnect"
		ui_.button_->setText("Disconnect");

		// If a host is not currently being pinged
		if (ping_process_.state() == QProcess::NotRunning)
		{
			// Enable button
			ui_.button_->setEnabled(true);
		}
		// If a host is being pinged
		else
		{
			// Disable button
			ui_.button_->setEnabled(false);
		}
	}
}

/* Check box slots */

// Fires when state of the markers checkbox has changed
void ViconPeer::markersCheckBoxStateChanged(int state) 
{
	// Enables setting amount of markers
	ui_.markers_edit_->setEnabled(ui_.markers_check_->isChecked());

	// Update button state
	updateButton();
}

// Fires when state of the objects checkbox has changed
void ViconPeer::objectsCheckBoxStateChanged(int state) 
{
	// Update button state
	updateButton();
}

/* Edit box slots */

// Fires when return is pressed in the IP edit box
void ViconPeer::ipReturnPressed()
{
	// Clear focus
	ui_.ip_edit_->clearFocus();

	// If a valid IPv4 address is set
	if (hasValidIPAddress())
	{
		// Connect
		connectToVicon();
	}
}

// Fires when text has changed in the IP edit box
void ViconPeer::ipTextChanged(const QString& newText)
{
	// If there is a no robot connected
	if (!isViconConnected)
	{
		// Update button state
		updateButton();
	}
}
// Fires when markers editing is finished
void ViconPeer::markersEditingFinished()
{
	// Clear focus
	ui_.markers_edit_->clearFocus();
}

// Fires when markers edit value changed
void ViconPeer::markersValueChanged(int new_value)
{
	// Update button state
	updateButton();
}

// Fires when port editing is finished
void ViconPeer::portEditingFinished()
{
	// Clear focus
	ui_.port_edit_->clearFocus();
}

/* Process slots */

// Starts client if host is active
void ViconPeer::connectIfActiveHost(int ping_exit_code, QProcess::ExitStatus ping_exit_status) 
{
	// If host has been found
	if (ping_exit_code == 0) 
	{
		// Create argument list
		QStringList args = QStringList() << "vicon_tools";

		// Set appropriate arguments
		if (ui_.markers_check_->isChecked()) 
		{
			if (ui_.objects_check_->isChecked()) 
			{
				args << "dual" << ui_.ip_edit_->text() << QString::number(ui_.port_edit_->value()) << QString::number(ui_.markers_edit_->value());
			}
			else 
			{
				args << "markers" << ui_.ip_edit_->text() << QString::number(ui_.port_edit_->value()) << QString::number(ui_.markers_edit_->value());
			}
		}
		else 
		{
			if (ui_.objects_check_->isChecked()) 
			{
				args << "objects" << ui_.ip_edit_->text() << QString::number(ui_.port_edit_->value());
			}
			else 
			{
				// Update button state
				updateButton();

				// Enable input
				enableInput();

				return;
			}
		}

		// Start client process
		client_process_.setArguments(args);
		client_process_.start();
		

		// Log connection message
		log("Connected!");

		// Update button state
		updateButton();

		// Disable input
		disableInput();
	}
	else 
	{
		log("Could not connect: No active host.");

		// Update button state
		updateButton();

		// Enable input
		enableInput();
	}
}

// Fires when client disconnects
void ViconPeer::onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status)
{
	// Disconnect -> kill client process
	client_process_.kill();
	client_process_.waitForFinished();

	// Update button state
	updateButton();

	// Enable input
	enableInput();
}