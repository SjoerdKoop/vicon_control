#ifndef TRACKED_OBJECT_H
#define TRACKED_OBJECT_H

// Vicon tools
#include "vicon_tools/vector3D.h"         // Vector3D

class TrackedObject {
    public:
        // Constructor
        TrackedObject();

        // Getter for predicted position deltaTime seconds from last know position
        struct Vector3D predictPosition(double deltaTime);

        // Updates position and velocity
        void updatePosition(struct Vector3D newPos, double deltaTime);

        // Holds whether object is initialized
        bool isInitialized;

        // Position
        struct Vector3D pos;
    private:

        // Velocity
        struct Vector3D vel;
};

#endif