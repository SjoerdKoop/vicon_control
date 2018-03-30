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

	// Connect UI signals to slots
	connect(ui_.button_, SIGNAL(pressed()), this, SLOT(buttonPressed()));
	connect(ui_.ip_edit_, SIGNAL(returnPressed()), this, SLOT(ipReturnPressed()));
	connect(ui_.ip_edit_, SIGNAL(textChanged(const QString&)), this, SLOT(ipTextChanged(const QString&)));
	connect(ui_.markers_check_, SIGNAL(stateChanged(int)), this, SLOT(markersCheckBoxStateChanged(int)));
	connect(ui_.port_edit_, SIGNAL(editingFinished()), this, SLOT(portEditingFinished()));

	// Connect other signals to slots
	connect(&this->ping_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(connectIfActiveHost(int, QProcess::ExitStatus)));
	connect(&this->client_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(onClientDisconnect(int, QProcess::ExitStatus)));
	
	// Set programs for processes
	client_process_.setProgram("rosrun");
	ping_process_.setProgram("ping");

	// Set initial state
	ui_.button_->setText("Connect");

	// Disable button if needed
	if (!hasValidIPAddress())
	{
		ui_.button_->setEnabled(false);
	}

	// Disable markers edit box if needed
	if (!ui_.markers_check_->isChecked())
	{
		ui_.markers_edit_->setEnabled(false);
	}
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
	
	// Disable GUI
	disableGUI();

	// Setup ping process and start
	ping_process_.setArguments(QStringList() << "-w1" << ui_.ip_edit_->text());
	ping_process_.start();

	// Log connection message
	log("Connecting to " + ui_.ip_edit_->text() + " at port " + QString::number(ui_.port_edit_->value()) + "...");
}

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
				// Enable GUI
				enableGUI();

				return;
			}
		}

		// Start client process
		client_process_.setArguments(args);
		client_process_.start();
		
		// Change button text
		ui_.button_->setText("Disconnect");

		// Log connection message
		log("Connected!");

		// Reenable disconnect button
		ui_.button_->setEnabled(true);

		// Disable some GUI functionality while connected
		ui_.ip_edit_->setEnabled(false);
		ui_.markers_check_->setEnabled(false);
		ui_.markers_edit_->setEnabled(false);
		ui_.objects_check_->setEnabled(false);
		ui_.port_edit_->setEnabled(false);
	}
	else 
	{
		log("Could not connect: No active host.");

		// Enable GUI
		enableGUI();
	}
}

// Disables the GUI
void ViconPeer::disableGUI()
{
	ui_.button_->setEnabled(false);
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

	// Enable GUI
	enableGUI();

	// Disable button if needed
	if (!hasValidIPAddress())
	{
		ui_.button_->setEnabled(false);
	}
}

// Enables the GUI
void ViconPeer::enableGUI()
{
	ui_.button_->setEnabled(true);
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
		// If addres is a valid IPv4 address
		if (hasValidIPAddress())
		{
			// Enable connect button
			ui_.button_->setEnabled(true);
		}
		else
		{
			// Disable connect button
			ui_.button_->setEnabled(false);
		}
	}
}


// Fires when state of the markers checkbox has changed
void ViconPeer::markersCheckBoxStateChanged(int state) 
{
	ui_.markers_edit_->setEnabled(ui_.markers_check_->isChecked());
}

// Fires when client disconnects
void ViconPeer::onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status)
{
	std::cout << "Exit code: " << client_exit_code << std::endl;
	disconnectFromVicon();// Fires when client disconnects
}


// Fires when port editing is finished
void ViconPeer::portEditingFinished()
{
	// Clear focus
	ui_.port_edit_->clearFocus();
	
	// If the port edit box is enabled
	if (ui_.port_edit_->isEnabled())
	{
		// If a valid IPv4 address is set
		if (hasValidIPAddress())
		{
			// Connect
			connectToVicon();
		}
	}
}