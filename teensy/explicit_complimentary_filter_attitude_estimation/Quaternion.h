#ifndef QUATERNION_H
#define QUATERNION_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
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
	Quaternion(float roll, float pitch, float yaw);
	~Quaternion();


	/*** Quaternion Operators ***/
	Quaternion operator+ (const Quaternion q2); // q3 = q1+q2
	Quaternion operator- (const Quaternion q2); // q3 = q1-q2
	Quaternion operator* (const Quaternion q2); // q3 = q1 * q2
	Quaternion operator/ (const Quaternion q2); // q3 = q1 / q2

	/*** Inverse/Conjugate ***/
	Quaternion conjugate();
	Quaternion inverse();

	/*** Vector rotated by quaternion ***/
	// Note: v is pure quaternion i.e v = 0<v> 
	Quaternion vector_rotation_by_quaternion(const Quaternion v);

	/*** Normalize quaternion so that it becomes UnitQuaternion ***/
	float norm2();
	void normalize();

	/*** Quaternion to RPY ***/
	float get_roll();
	float get_pitch();
	float get_yaw();

	/*** To vector method ***/
	BLA::Matrix<4> to_vector();
};

class UnitQuaternion : public Quaternion
{
public:
	UnitQuaternion();
	UnitQuaternion(float s, float v_1, float v_2, float v_3);
	static UnitQuaternion omega(float wx, float wy, float wz);

	/*** UnitQuaternion operators ***/
	UnitQuaternion operator+ (const UnitQuaternion q2); // q3 = q1+q2
	UnitQuaternion operator- (const UnitQuaternion q2); // q3 = q1-q2
	UnitQuaternion operator* (const UnitQuaternion q2); // q3 = q1 * q2
	UnitQuaternion operator/ (const UnitQuaternion q2); // q3 = q1 / q2

	/*** Inverse/Conjugate ***/
	UnitQuaternion conjugate();
	UnitQuaternion inverse();

	/*** Vector rotated by quaternion ***/
	// Note: v is pure quaternion i.e v = 0<v> 
	BLA::Matrix<3> vector_rotation_by_quaternion(BLA::Matrix<3> v);

};



#endif