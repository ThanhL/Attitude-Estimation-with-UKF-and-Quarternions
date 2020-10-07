#include "Quaternion.h"


/*** Constructors ***/
Quaternion::Quaternion()
{
	this->s = 1;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
}

Quaternion::Quaternion(float s, float v_1, float v_2, float v_3)
{
	this->s = s;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
}

/*** Destructors ***/
Quaternion::~Quaternion()
{
	// Destructor
}


/*** Quaternion Operators ***/
Quaternion Quaternion::operator+(const Quaternion q2)
{
	Quaternion quaternion_result;
	quaternion_result.s = this->s + q2.s;
	quaternion_result.v_1 = this->v_1 + q2.v_1;
	quaternion_result.v_2 = this->v_2 + q2.v_2;
	quaternion_result.v_3 = this->v_3 + q2.v_3;
	return quaternion_result;
}



Quaternion Quaternion::operator-(const Quaternion q2)
{
	Quaternion quaternion_result;
	quaternion_result.s = this->s - q2.s;
	quaternion_result.v_1 = this->v_1 - q2.v_1;
	quaternion_result.v_2 = this->v_2 - q2.v_2;
	quaternion_result.v_3 = this->v_3 - q2.v_3;
	return quaternion_result;
}


Quaternion Quaternion::operator*(const Quaternion q2)
{
	Quaternion quaternion_result;
	quaternion_result.s = s*q2.s - v_1*q2.v_1 - v_2*q2.v_2 - v_3*q2.v_3;
	quaternion_result.v_1 = v_1*q2.s + s*q2.v_1 + v_2*q2.v_3 - v_3*q2.v_2;
	quaternion_result.v_2 = s*q2.v_2 - v_1*q2.v_3 + v_2*q2.s + v_3*q2.v_1;
	quaternion_result.v_3 = s*q2.v_3 + v_1*q2.v_2 - v_2*q2.v_1 + v_3*q2.s;
	return quaternion_result;
}

/*** Quaternion Inverse/Conjugate (since inverse of quaternion is its conjugate) ***/
Quaternion Quaternion::conjugate()
{
  Quaternion quat_conjugate(s,-v_1,-v_2,-v_3);
  return quat_conjugate;
}

/*** Quaternion rotation by vector (v' = dot(q,v,q_conj)) ***/
Quaternion Quaternion::vector_rotation_by_quaternion(const Quaternion v)
{
	// Robotics vision and control quaternion vector rotation (p45)
	return (*this) * v * conjugate();

}

