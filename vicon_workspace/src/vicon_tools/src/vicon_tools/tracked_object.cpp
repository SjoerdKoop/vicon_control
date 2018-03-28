// Declarations
#include "vicon_tools/tracked_object.h"

// Constructor
TrackedObject::TrackedObject()
{
    isInitialized_ = false;
}

// Getter for predicted position delta_time seconds from last know position
struct Vector3D TrackedObject::predictPosition(double delta_time)
{
    struct Vector3D prediction;

    prediction.x_ = pos_.x_ + delta_time * vel_.x_;
    prediction.y_ = pos_.y_ + delta_time * vel_.y_;
    prediction.z_ = pos_.z_ + delta_time * vel_.z_;

    return prediction;
}

// Updates position and velocity
void TrackedObject::updatePosition(struct Vector3D new_pos, double delta_time)
{
    if (isInitialized_)
	{
        vel_.x_ = (new_pos.x_ - pos_.x_) * delta_time;
        vel_.y_ = (new_pos.y_ - pos_.y_) * delta_time;
        vel_.z_ = (new_pos.z_ - pos_.z_) * delta_time;
    }
    else
	{
        isInitialized_ = true;
    }

    pos_.x_ = new_pos.x_;
    pos_.y_ = new_pos.y_;
    pos_.z_ = new_pos.z_;
}

