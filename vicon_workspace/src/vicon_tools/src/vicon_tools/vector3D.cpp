// Declarations
#include "vicon_tools/vector3D.h"

// Constructors
Vector3D::Vector3D()
{

}

Vector3D::Vector3D(double components[3])
{
	// Set coordinates
	x_ = components[0];
	y_ = components[1];
	z_ = components[2];
}