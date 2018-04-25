#ifndef VICON_TOOLS_TRACKED_OBJECT_H
#define VICON_TOOLS_TRACKED_OBJECT_H

// Vicon tools
#include "vicon_tools/vector3D.h"         // Vector3D

// Class defining a tracked object (using single markers)
class TrackedObject
{
	public:
		// Constructor
		TrackedObject();

		// Getter for predicted position delta_time seconds from last know position
		struct Vector3D predictPosition(double delta_time);

		// Updates position and velocity
		void updatePosition(struct Vector3D new_pos, double delta_time);
	
		bool is_initialized_;		// Holds whether object is initialized
		bool has_update_;			// Holds whether object has an update pending
		struct Vector3D pos_;		// Position

	private:	
		struct Vector3D vel_;		// Velocity
};

#endif // VICON_TOOLS_TRACKED_OBJECT_H