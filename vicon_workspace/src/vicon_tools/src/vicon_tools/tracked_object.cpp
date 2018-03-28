// Declarations
#include "vicon_tools/tracked_object.h"

// Constructor
TrackedObject::TrackedObject() {
    isInitialized = false;
}

// Getter for predicted position deltaTime seconds from last know position
struct Vector3D TrackedObject::predictPosition(double deltaTime) {
    struct Vector3D prediction;

    prediction.x_ = pos.x_ + deltaTime * vel.x_;
    prediction.y_ = pos.y_ + deltaTime * vel.y_;
    prediction.z_ = pos.z_ + deltaTime * vel.z_;

    return prediction;
}

// Updates position and velocity
void TrackedObject::updatePosition(struct Vector3D newPos, double deltaTime) {
    if (isInitialized) {
        vel.x_ = (newPos.x_ - pos.x_) * deltaTime;
        vel.y_ = (newPos.y_ - pos.y_) * deltaTime;
        vel.z_ = (newPos.z_ - pos.z_) * deltaTime;
    }
    else {
        isInitialized = true;
    }

    pos.x_ = newPos.x_;
    pos.y_ = newPos.y_;
    pos.z_ = newPos.z_;
}

