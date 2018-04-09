#ifndef VICON_PEER_VICON_PEER_H
#define VICON_PEER_VICON_PEER_H

// Qt
#include <QProcess>								// QProcess

// Robot peer
#include "robot_peer/ui_robot_peer.h"			// UI::RobotPeer


class RobotPeer : public QWidget {
	Q_OBJECT
	
	public:
		// Constructor
		RobotPeer(QWidget* parent);

		// Destructor
		~RobotPeer();

	private:
		QProcess client_process_;			// Client process
		Ui::RobotPeer ui_;					// The UI
		QWidget* widget_;					// The main widget

		// Disables input
		void disableInput();

		// Enables input
		void enableInput();

		// Updates button
		void updateButton();

		// Connects to the robot
		void connectToRobot();

		// Disconnects from the robot
		void disconnectFromRobot();

		// Check whether the IP edit box has a valid IPv4 address
		bool hasValidIPAddress();

	private slots:
		/* Button slot */

		// Fires when the button is pressed
		void buttonPressed();

		/* Check box slots */

		// Fires when state of the markers checkbox has changed
		void markersCheckBoxStateChanged(int state);

		// Fires when state of the objects checkbox has changed
		void objectsCheckBoxStateChanged(int state);

		/* Edit box slots */

		// Fires when return is pressed in the IP edit box
		void ipReturnPressed();

		// Fires when text has changed in the IP edit box
		void ipTextChanged(const QString& newText);

		// Fires when markers editing is finished
		void markersEditingFinished();

		// Fires when markers edit value changed
		void markersValueChanged(int new_value);

		// Fires when port editing is finished
		void portEditingFinished();

		/* Process slot */

		// Fires when client disconnects
		void onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status);
};

#endif // VICON_PEER_VICON_PEER_H