#ifndef VECTOR3D_H
#define VECTOR3D_H

// Struct holding 3D components
struct Vector3D {
	// Constructors
	Vector3D();
	Vector3D(double components[3]);

	// Components
	double x_;
	double y_;
	double z_;
};

#endif