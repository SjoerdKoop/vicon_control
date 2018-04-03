#ifndef VICON_PEER_VICON_PEER_H
#define VICON_PEER_VICON_PEER_H

// Qt
#include <QProcess>								// QProcess

// Vicon peer
#include "vicon_peer/ui_vicon_peer.h"			// UI::ViconPeer


class ViconPeer : public QWidget {
	Q_OBJECT
	
	public:
		// Constructor
		ViconPeer(QWidget* parent);

		// Destructor
		~ViconPeer();

	private:
		QWidget* widget_;					// The main widget
		Ui::ViconPeer ui_;					// The UI
		bool isViconConnected;				// Holds whether a client is connected to the Vicon datastream
		QProcess client_process_;			// Client process
		QProcess ping_process_;				// Ping process
		bool trackObjects_;					// Whether to track objects
		int number_of_markers_;				// How many markers to track

		// Disables input
		void disableInput();

		// Enables input
		void enableInput();

		// Updates button
		void updateButton();

		// Connects to Vicon
		void connectToVicon();

		// Disconnects from Vicon
		void disconnectFromVicon();

		// Check whether the IP edit box has a valid IPv4 address
		bool hasValidIPAddress();

		// Checks whether host is active
		void checkHost();
		
	signals:
		// Signals logging of a message
		void log(const QString& msg);

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

		/* Process slots */
		
		// Starts client if host is active
		void connectIfActiveHost(int ping_exit_code, QProcess::ExitStatus ping_exit_status);

		// Fires when client disconnects
		void onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status);

};

#endif // VICON_PEER_VICON_PEER_H