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
ViconClient::ViconClient(int number_of_markers) 
{
	// Initialize publisher
	ros::NodeHandle n;
	pub_ = n.advertise<vicon_tools::ros_object_array>("object_update", 1);

    // Populate objects_ if needed (markers only)
    for (int i = 0; i < number_of_markers; i++)
	{
        objects_.push_back(new TrackedObject());
    }

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

	if (result == Result::Success)
	{
		// Show info
		ROS_INFO("Datastream Client connected!");

		// Set general client options
    	client_->SetStreamMode(StreamMode::ServerPush);		// Stream mode
		client_->SetBufferSize(BUFFER_SIZE);				// Buffer size

		// Return success
		return true;
	}
	else
	{
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
	while (true)
	{
		// Wait for new frame
		client_->GetFrame();

		// Extract desired data from the client
		object_array = extractData();

		// If an object update is acquired
		if (object_array.objects.size() > 0)
		{
			// Publish objectArray over topic "object_update"
			pub_.publish(object_array);
		}

		// Spin once
		ros::spinOnce();
	}
}


// Generates ros_object_array from markers
vicon_tools::ros_object_array ViconClient::getMarkers()
{
	vicon_tools::ros_object_array object_array;							// Holds ros_object_array message
	double delta_time = 1 / client_->GetFrameRate().FrameRateHz;		// Elapsed time
	std::vector<int> excluded_markers;									// Excluded markers
	int marker_count = client_->GetUnlabeledMarkerCount().MarkerCount;	// Unlabeled marker count

	// Loop variables
	int best_marker_id;						// Best marker ID
	struct Vector3D best_marker_position;	// Best marker position
	double best_norm;						// Best norm
	double dx;								// X difference
	double dy;								// Y difference
	double dz;								// Z difference
	double norm;							// Norm

	// If there are markers
	if (marker_count > 0)
	{
		// For each object
		for (TrackedObject* object : objects_)
		{
			// Initialize norm to maximum
			best_norm = DBL_MAX;

			for (int j = 0; j < marker_count; j++)
			{
				// Look for marker in excluded markers
				auto it = std::find(excluded_markers.begin(), excluded_markers.end(), j);

				// If marker is not excluded
				if (it == excluded_markers.end())
				{
					// Get coordinates
					Output_GetUnlabeledMarkerGlobalTranslation umgtOutput = client_->GetUnlabeledMarkerGlobalTranslation(j);

					// Calculate norm
					dx = object->pos_.x_ - umgtOutput.Translation[0];
					dy = object->pos_.y_ - umgtOutput.Translation[1];
					dz = object->pos_.z_ - umgtOutput.Translation[2];
					norm = std::sqrt(dx * dx + dy * dy + dz * dz);

					// If norm is smaller, set this marker to be the best fit for current object
					if (norm < best_norm)
					{
						best_marker_id = j;
						best_marker_position = Vector3D(umgtOutput.Translation);
						best_norm = norm;
					}

					// If this is the first detection of the object, the first detected marker is assumed to be the object
					if (!object->is_initialized_)
					{
						break;
					}
				}
			}

			if (best_norm < NORM_THRESHOLD || !object->is_initialized_)
			{
				// Update objects to most likely marker
				object->updatePosition(best_marker_position, delta_time);
				std::cout << "Updating object" << std::endl;
				// Exclude marker from future pairings
				excluded_markers.push_back(best_marker_id);
			}
		}
	}

	// For each object
	for (int i = 0; i < objects_.size(); i++)
	{
		// If the object has an update pending
		if (objects_[i]->has_update_)
		{
			// Create new ros_object message
			vicon_tools::ros_object ros_object;

			// Copy data to ros_object message
			ros_object.name = "marker" + std::to_string(i);
			ros_object.x = objects_[i]->pos_.x_;
			ros_object.y = objects_[i]->pos_.y_;
			ros_object.z = objects_[i]->pos_.z_;
			ros_object.rx = 0;
			ros_object.ry = 0;
			ros_object.rz = 0;

			// Add object to array
			object_array.objects.push_back(ros_object);
		}
	}
	std::cout << "Marker objects: " << object_array.objects.size() << std::endl;
	// Return ROS object array
	return object_array;
}

// Generates ros_object_array from objects
vicon_tools::ros_object_array ViconClient::getObjects()
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
			object.name = subject_name;
			object.x = o_gsgt.Translation[0];
			object.y = o_gsgt.Translation[1];
			object.z = o_gsgt.Translation[2];
			object.rx = o_gsgr.Rotation[0];
			object.ry = o_gsgr.Rotation[1];
			object.rz = o_gsgr.Rotation[2];
			
			// Add object to array
			object_array.objects.push_back(object);
		}
    }

    return object_array;
}

// Constructor
DualClient::DualClient(int number_of_markers) : ViconClient(number_of_markers)
{
}

// Runs the client
void DualClient::run()
{	
	// Enable segments and unlabeled markers
	client_->EnableSegmentData();
	client_->EnableUnlabeledMarkerData();

	// Run main loop as a MarkerClient
	ViconClient::run();
}


// Extracts desired data from the client
vicon_tools::ros_object_array DualClient::extractData()
{
	// Get data
	vicon_tools::ros_object_array objects1 = ViconClient::getMarkers();
	vicon_tools::ros_object_array objects2 = ViconClient::getObjects();

	// Merge arrays
	for (int i = 0; i < objects2.objects.size(); i++)
	{
		objects1.objects.push_back(objects2.objects[i]);
	}
	
	// Return message with merged arrays
	return objects1;
}

// Constructor
MarkerClient::MarkerClient(int number_of_markers) : ViconClient(number_of_markers)
{
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
	return getMarkers();
}

// Constructor
ObjectClient::ObjectClient() : ViconClient(0)
{
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
	return getObjects();
}