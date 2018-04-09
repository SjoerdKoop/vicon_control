// Declarations
#include "robot_peer/robot_peer.h"

// Export
#include <pluginlib/class_list_macros.h>		// PLUGINLIB_EXPORT_CLASS

// Qt
#include <QHostAddress>							// QHostAddress

// Constructor
RobotPeer::RobotPeer(QWidget* parent) : QWidget(parent)
{
	// Extend the widget with all attributes and children from UI file
	ui_.setupUi(this);

	// Set window title
	setWindowTitle("Robot peer");

	// Set object name
	setObjectName("Robot peer");

	// Button signal
	connect(ui_.button_, SIGNAL(pressed()), this, SLOT(buttonPressed()));

	// Edit box signals
	connect(ui_.ip_edit_, SIGNAL(returnPressed()), this, SLOT(ipReturnPressed()));
	connect(ui_.ip_edit_, SIGNAL(textChanged(const QString&)), this, SLOT(ipTextChanged(const QString&)));
	connect(ui_.port_edit_, SIGNAL(editingFinished()), this, SLOT(portEditingFinished()));

	// Process signals
	connect(&this->client_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(onClientDisconnect(int, QProcess::ExitStatus)));

	// Set programs for client process
	client_process_.setProgram("rosrun");

	// Update button state
	updateButton();
}

// Destructor
RobotPeer::~RobotPeer()
{
	
}

// Fires when the button is pressed
void RobotPeer::buttonPressed()
{
	if (client_process_.state() == QProcess::NotRunning)
	{
		// If a valid IPv4 address is set
		if (hasValidIPAddress())
		{
			// Connect
			connectToRobot();
		}
	}
	else
	{
		disconnectFromRobot();
	}
}

// Connects to the robot
void RobotPeer::connectToRobot() 
{
	// Disconnect if already connected
	if (client_process_.state() != QProcess::NotRunning) 
	{
		disconnectFromRobot();
	}

	// Create argument list
	// Start with package
	QStringList args = QStringList() << "robot_tools";

	// Append executable name
	args << "communicate";

	// Append connection data
	args << ui_.ip_edit_->text();
	args << QString::number(ui_.port_edit_->value());

	// Start client process
	client_process_.setArguments(args);
	client_process_.start();

	// Update button state
	updateButton();

	// Disable input
	disableInput();
}

// Disables input
void RobotPeer::disableInput()
{
	ui_.ip_edit_->setEnabled(false);
	ui_.port_edit_->setEnabled(false);
}

// Disconnects from the robot
void RobotPeer::disconnectFromRobot()
{
	// Disconnect -> kill client process
	client_process_.kill();
	client_process_.waitForFinished();

	// Enable input
	enableInput();

	// Update button state
	updateButton();
}

// Enables input
void RobotPeer::enableInput()
{
	ui_.ip_edit_->setEnabled(true);
	ui_.port_edit_->setEnabled(true);
}

// Check whether the IP edit box has a valid IPv4 address
bool RobotPeer::hasValidIPAddress()
{
	// Convert input to IP address
	QHostAddress ipAddress(ui_.ip_edit_->text());

	//Return whether it is a valid IPv4 address
	return QAbstractSocket::IPv4Protocol == ipAddress.protocol();
}

/* Button slot */

// Updates button
void RobotPeer::updateButton()
{
	// If there is no client running
	if (client_process_.state() == QProcess::NotRunning)
	{
		// Set text to "Connect"
		ui_.button_->setText("Connect");

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
	// If a client is running
	else
	{
		// Set text to "Disconnect"
		ui_.button_->setText("Disconnect");

		// Enable button
		ui_.button_->setEnabled(true);
	}
}

/* Edit box slots */

// Fires when return is pressed in the IP edit box
void RobotPeer::ipReturnPressed()
{
	// Clear focus
	ui_.ip_edit_->clearFocus();

	// If a valid IPv4 address is set
	if (hasValidIPAddress())
	{
		// Connect
		connectToRobot();
	}
}

// Fires when text has changed in the IP edit box
void RobotPeer::ipTextChanged(const QString& newText)
{
	// Update button state
	updateButton();
}

// Fires when port editing is finished
void RobotPeer::portEditingFinished()
{
	// Clear focus
	ui_.port_edit_->clearFocus();
}

/* Process slot */

// Fires when client disconnects
void RobotPeer::onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status)
{
	// Disconnect -> kill client process
	client_process_.kill();
	client_process_.waitForFinished();

	// Update button state
	updateButton();

	// Enable input
	enableInput();
}