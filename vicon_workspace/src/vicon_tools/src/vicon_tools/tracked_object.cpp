// Declarations
#include "vicon_tools/tracked_object.h"

// Constructor
TrackedObject::TrackedObject()
{
	// Start uninitialized
    is_initialized_ = false;

	// Start without update
	has_update_ = false;
}

// Getter for predicted position delta_time seconds from last know position
struct Vector3D TrackedObject::predictPosition(double delta_time)
{
    struct Vector3D prediction;		// Prediction vector

	// Calculate prediction
    prediction.x_ = pos_.x_ + delta_time * vel_.x_;
    prediction.y_ = pos_.y_ + delta_time * vel_.y_;
    prediction.z_ = pos_.z_ + delta_time * vel_.z_;

	// Return prediction
    return prediction;
}

// Updates position and velocity
void TrackedObject::updatePosition(struct Vector3D new_pos, double delta_time)
{
	// If the object has already been initialized
    if (is_initialized_)
	{
		// Update velocity
        vel_.x_ = (new_pos.x_ - pos_.x_) * delta_time;
        vel_.y_ = (new_pos.y_ - pos_.y_) * delta_time;
        vel_.z_ = (new_pos.z_ - pos_.z_) * delta_time;
    }
	// If the object has not yet been initialized
    else
	{
		// Set initialized
        is_initialized_ = true;
    }

	// Set new position
    pos_.x_ = new_pos.x_;
    pos_.y_ = new_pos.y_;
    pos_.z_ = new_pos.z_;

	// Notify update is available
	has_update_ = true;
}

