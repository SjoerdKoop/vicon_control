#ifndef VICON_PEER_VICON_PEER_WIDGET_H
#define VICON_PEER_VICON_PEER_WIDGET_H

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
		bool isViconConnected;
		QProcess client_process_;
		QProcess ping_process_;
		bool trackObjects_;
		int number_of_markers_;

		// Disables the GUI
		void disableGUI();

		// Enables the GUI
		void enableGUI();

	signals:
		// Signals logging of a message
		void log(const QString& msg);

	private slots:
		// Fires when the button is pressed
		void buttonPressed();

		// Checks whether host is active
		void checkHost();

		// Calls connectToVicon if host is active
		void connectIfActiveHost(int ping_exit_code, QProcess::ExitStatus ping_exit_status);

		// Connects to Vicon
		void connectToVicon();

		// Disconnects from Vicon
		void disconnectFromVicon();

		// Check whether the IP edit box has a valid IPv4 address
		bool hasValidIPAddress();

		// Fires when return is pressed in the IP edit box
		void ipReturnPressed();

		// Fires when text has changed in the IP edit box
		void ipTextChanged(const QString& newText);

		// Fires when state of the markers checkbox has changed
		void markersCheckBoxStateChanged(int state);

		// Fires when client disconnects
		void onClientDisconnect(int client_exit_code, QProcess::ExitStatus client_exit_status);

		// Fires when port editing is finished
		void portEditingFinished();

		void test(QProcess::ProcessError error);
};

#endif // VICON_PEER_VICON_PEER_WIDGET_H