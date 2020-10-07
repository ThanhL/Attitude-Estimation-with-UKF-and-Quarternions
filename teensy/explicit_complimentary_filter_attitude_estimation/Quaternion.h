#ifndef QUATERNION_H
#define QUATERNION_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include <vector> 
#include <math.h>

class Quaternion
{
public:
	// Scalar part of Quaternion
	float s;

	// Orthogonal complex number (vector part of Quaternion) 
	float v_1, v_2, v_3;
	//vector<float> v;

	Quaternion();
	Quaternion(float s, float v_1, float v_2, float v_3);
	~Quaternion();


	/*** Quaternion Operators ***/
	Quaternion operator+ (const Quaternion q2); // q3 = q1+q2
	Quaternion operator- (const Quaternion q2); // q3 = q1-q2
	Quaternion operator* (const Quaternion q2); // q3 = q1 * q2
	Quaternion operator/ (const Quaternion q2); // q3 = q1 / q2

	/*** Inverse/Conjugate ***/
	Quaternion conjugate();

	/*** Vector rotated by quaternion ***/
	// Note: v is pure quaternion i.e v = 0<v> 
	Quaternion vector_rotation_by_quaternion(const Quaternion v);


};





#endif