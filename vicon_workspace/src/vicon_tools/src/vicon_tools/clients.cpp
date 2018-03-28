// Declarations
#include "vicon_tools/clients.h"

// ROS
#include "ros/ros.h"						// ros::*, ROS_*

// System
#include <string>							// std::string
#include <vector>							// std::vector

// Vicon tools
#include "vicon_tools/ros_object.h"			// vicon_tools::ros_object

// Constructor
ViconClient::ViconClient() 
{
	// Initialize publisher
	ros::NodeHandle n;
	pub_ = n.advertise<vicon_tools::ros_object_array>("object_update", 1);

	// Create datastream client
	client_ = new Client();
}

// Destructor
ViconClient::~ViconClient() 
{
	// Shutdown publisher
	pub_.shutdown();

	// Delete datastream client
	delete client_;
}

// Connects to the datastream
bool ViconClient::connect(char* ip, char* port)
{
	// Create SDK compatible host name
    std::string host_name = std::string() + ip + ':' + port; 

	// Connect to host
	ROS_INFO("Connecting to %s at port %s ...", ip, port);
	Result::Enum result = client_->Connect(host_name).Result;

	if (result == Result::Success) {
		// Show info
		ROS_INFO("Datastream Client connected!");

		// Set general client options
    	client_->SetStreamMode(StreamMode::ServerPush);		// Stream mode
		client_->SetBufferSize(BUFFER_SIZE);				// Buffer size

		// Return success
		return true;
	}
	else {
		// Show fatal error
		ROS_FATAL("ERROR: Datastream client could not connect!");

		// Return failure
		return false;
	}
}

// Extracts desired data from the client
// To be overwritten by children
vicon_tools::ros_object_array ViconClient::extractData()
{

}

// Runs the client
void ViconClient::run()
{
	// Main loop
	while (true) {
		// Wait for new frame
		client_->GetFrame();

		// Extract desired data from the client
		object_array = extractData();

		// Publish objectArray over topic "object_update"
		pub_.publish(object_array);

		// Spin once
		ros::spinOnce();
	}
}
// Constructor
MarkerClient::MarkerClient(int number_of_markers) : ViconClient()
{
    // Populate objects_
    for (int i = 0; i < number_of_markers; i++) {
        objects_.push_back(TrackedObject());
    }
}

// Runs the client
void MarkerClient::run()
{
	// Enable unlabeled markers
	client_->EnableUnlabeledMarkerData();
	
	// Run main loop
	ViconClient::run();
}

// Extracts desired data from the client
vicon_tools::ros_object_array MarkerClient::extractData()
{
	vicon_tools::ros_object_array object_array;	// Holds ros_object_array message

   // Holds time between this frame and the one before
    double delta_time = 1 / client_->GetFrameRate().FrameRateHz;

    // Excluded markers, start with no elements. Fills up when object and markers have been paired
    std::vector<int> excluded_markers;

    // Get unlabeled marker count
	int marker_count = client_->GetUnlabeledMarkerCount().MarkerCount;

	// If there are markers
    if (marker_count > 0) {
        // For each object
        for (int i = 0; i < objects_.size(); i++) {
            // Most likely marker's ID and position
            int best_marker_id;
            struct Vector3D best_marker_position;

            // Euclidian norm of predicted object position and the most like marker's position
            double best_norm = DBL_MAX;

            for (int j = 0; j < marker_count; j++) {
                // Look for marker in excluded markers
                auto it = std::find(excluded_markers.begin(), excluded_markers.end(), j);

                // If marker is not excluded
                if (it == excluded_markers.end()) {
                    // Get coordinates
                    Output_GetUnlabeledMarkerGlobalTranslation umgtOutput = client_->GetUnlabeledMarkerGlobalTranslation(i);

                    // Calculate norm
                    double dx = objects_[i].pos_.x_ - umgtOutput.Translation[0];
                    double dy = objects_[i].pos_.y_ - umgtOutput.Translation[1];
                    double dz = objects_[i].pos_.z_ - umgtOutput.Translation[2];
                    double norm = std::sqrt(dx * dx + dy * dy + dz * dz);

                    // If norm is smaller, set this marker to be the best fit for current object
                    if (norm < best_norm) {
                        best_marker_id = j;
                        best_marker_position = Vector3D(umgtOutput.Translation);
                        best_norm = norm;
                    }

                    // If this is the first detection of the object, the first detected marker is assumed to be the object
                    if (!objects_[i].isInitialized_) {
                        break;
                    }
                }
            }

            if (best_norm < NORM_THRESHOLD || !objects_[i].isInitialized_) {
                // Update objects to most likely marker
                objects_[i].updatePosition(best_marker_position, delta_time);

                // Exclude marker from future pairings
                excluded_markers.push_back(best_marker_id);
            }
        }
    }

	// Dummy zero variable;
	double zero = 0;

	// For each object
	for (unsigned short i = 0; i < objects_.size(); i++) {
		// Create new ros_object message
		vicon_tools::ros_object object;

		// Copy data to ros_object message
		memcpy(&object.id, &i, sizeof(unsigned short));
		memcpy(&object.x, &objects_[i].pos_.x_, sizeof(double));
		memcpy(&object.y, &objects_[i].pos_.y_, sizeof(double));
		memcpy(&object.z, &objects_[i].pos_.z_, sizeof(double));
		memcpy(&object.rx, &zero, sizeof(double));
		memcpy(&object.ry, &zero, sizeof(double));
		memcpy(&object.rz, &zero, sizeof(double));

		// Add object to array
		object_array.objects.push_back(object);
	}

	return object_array;
}

// Runs the client
void ObjectClient::run()
{
	// Enable segments
	client_->EnableSegmentData();
	
	// Run main loop
	ViconClient::run();
}

// Extracts desired data from the client
vicon_tools::ros_object_array ObjectClient::extractData()
{
    Output_GetSegmentGlobalRotationEulerXYZ o_gsgr;	// Holds rotation output
    Output_GetSegmentGlobalTranslation o_gsgt;		// Holds translation output
    vicon_tools::ros_object_array object_array;		// Holds ros_object_array message
    std::string root_segment_name;					// Holds current segment name
    std::string subject_name;						// Holds current subject name

    // For each subject
    for (int i = 0; i < client_->GetSubjectCount().SubjectCount; i++)
	{
			// Get subject name
			subject_name = client_->GetSubjectName(i).SubjectName;	

		if (client_->GetObjectQuality(subject_name).Quality > QUALITY_THRESHOLD)
		{
			// Create new ros_object message
			vicon_tools::ros_object object;

			// Get root segment name
			root_segment_name = client_->GetSubjectRootSegmentName(subject_name).SegmentName;

			// Get root segment data
			o_gsgt = client_->GetSegmentGlobalTranslation(subject_name, root_segment_name);
			o_gsgr = client_->GetSegmentGlobalRotationEulerXYZ(subject_name, root_segment_name);

			// Copy data to ros_object message
			memcpy(&object.id, &i, sizeof(unsigned short));
			memcpy(&object.x, &o_gsgt.Translation[0], sizeof(double));
			memcpy(&object.y, &o_gsgt.Translation[1], sizeof(double));
			memcpy(&object.z, &o_gsgt.Translation[2], sizeof(double));
			memcpy(&object.rx, &o_gsgr.Rotation[0], sizeof(double));
			memcpy(&object.ry, &o_gsgr.Rotation[1], sizeof(double));
			memcpy(&object.rz, &o_gsgr.Rotation[2], sizeof(double));
			std::cout << "X: " << o_gsgt.Result << std::endl;
			// Add object to array
			object_array.objects.push_back(object);
		}
    }

    return object_array;
}