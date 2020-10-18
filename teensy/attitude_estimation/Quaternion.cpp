#include "Quaternion.h"

/*** Constructors ***/
// -- Quaternion --
Quaternion::Quaternion()
{
	this->s = 1;
	this->v_1 = 0.0;
	this->v_2 = 0.0;
	this->v_3 = 0.0;
}

Quaternion::Quaternion(float s, float v_1, float v_2, float v_3)
{
	this->s = s;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
}


// Create a quaternion from roll,pitch,yaw angle
Quaternion::Quaternion(float roll, float pitch, float yaw)
{
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);

	this->s = cr * cp * cy + sr * sp * sy;
	this->v_1 = sr * cp * cy - cr * sp * sy;
	this->v_2 = cr * sp * cy + sr * cp * sy;
	this->v_3 = cr * cp * sy - sr * sp * cy;
}


// -- UnitQuaternion --
UnitQuaternion::UnitQuaternion()
{
	this->s = 1;
	this->v_1 = 0.0;
	this->v_2 = 0.0;
	this->v_3 = 0.0;
}

UnitQuaternion::UnitQuaternion(float s, float v_1, float v_2, float v_3)
{
	// Constructs a UnitQuaternion from angle times rotation vector
	this->s = s;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
	(*this).normalize();
}


UnitQuaternion UnitQuaternion::omega(float wx, float wy, float wz)
{
	// Constructs a UnitQuaternion from angle times rotation vector
	// EQN referral: https://math.stackexchange.com/questions/39553/how-do-i-apply-an-angular-velocity-vector3-to-a-unit-quaternion-orientation
	UnitQuaternion uq;

	float theta = sqrt(wx*wx + wy*wy + wz*wz);
	uq.s = cos(theta/2.0);
	uq.v_1 = sin(theta/2.0) * (wx / theta);
	uq.v_2 = sin(theta/2.0) * (wy / theta);
	uq.v_3 = sin(theta/2.0) * (wz / theta);

	return uq;
}

/*** Destructors ***/
Quaternion::~Quaternion()
{
	// Destructor
}


/*** Quaternion Operators ***/
// -- Quaternions --
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



UnitQuaternion UnitQuaternion::operator+(const UnitQuaternion q2)
{
	UnitQuaternion quaternion_result;
	quaternion_result.s = this->s + q2.s;
	quaternion_result.v_1 = this->v_1 + q2.v_1;
	quaternion_result.v_2 = this->v_2 + q2.v_2;
	quaternion_result.v_3 = this->v_3 + q2.v_3;
	return quaternion_result;
}


// -- Unit Quaternions --
UnitQuaternion UnitQuaternion::operator-(const UnitQuaternion q2)
{
	UnitQuaternion quaternion_result;
	quaternion_result.s = this->s - q2.s;
	quaternion_result.v_1 = this->v_1 - q2.v_1;
	quaternion_result.v_2 = this->v_2 - q2.v_2;
	quaternion_result.v_3 = this->v_3 - q2.v_3;
	return quaternion_result;
}


UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion q2)
{
	UnitQuaternion quaternion_result;
	quaternion_result.s = s*q2.s - v_1*q2.v_1 - v_2*q2.v_2 - v_3*q2.v_3;
	quaternion_result.v_1 = v_1*q2.s + s*q2.v_1 + v_2*q2.v_3 - v_3*q2.v_2;
	quaternion_result.v_2 = s*q2.v_2 - v_1*q2.v_3 + v_2*q2.s + v_3*q2.v_1;
	quaternion_result.v_3 = s*q2.v_3 + v_1*q2.v_2 - v_2*q2.v_1 + v_3*q2.s;
	return quaternion_result;
}


/*** Quaternion Inverse/Conjugate (since inverse of quaternion is its conjugate) ***/
// -- Quaternions --
Quaternion Quaternion::conjugate()
{
	Quaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

Quaternion Quaternion::inverse()
{
	Quaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

// -- Unit Quaternions --
UnitQuaternion UnitQuaternion::conjugate()
{
	UnitQuaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

UnitQuaternion UnitQuaternion::inverse()
{
	UnitQuaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

/*** Quaternion rotation by vector (v' = dot(q,v,q_conj)) ***/
Quaternion Quaternion::vector_rotation_by_quaternion(const Quaternion v)
{
	// Robotics vision and control quaternion vector rotation (p45)
	return (*this) * v * conjugate();

}


BLA::Matrix<3> UnitQuaternion::vector_rotation_by_quaternion(BLA::Matrix<3> v)
{
	// Robotics vision and control quaternion vector rotation (p45) however returns a vector
	// instead of a pure quaternion 0<v>
	UnitQuaternion v_unit_quat = UnitQuaternion(0, v(0), v(1), v(2));
	UnitQuaternion quat_result =  (*this) * v_unit_quat * conjugate();
	BLA::Matrix<3> rotated_vec = {quat_result.v_1, quat_result.v_2, quat_result.v_3};
	return rotated_vec;
}

/*** Normalize Quaternion ***/
float Quaternion::norm2()
{
	float norm2 = s*s + v_1*v_1 + v_2*v_2 + v_3*v_3;
	return sqrt(norm2);
}

void Quaternion::normalize()
{
	float q_magnitude = norm2();
	s /= q_magnitude;
	v_1 /= q_magnitude;
	v_2 /= q_magnitude;
	v_3 /= q_magnitude;
}


/*** Quaternion to RPY ***/
float Quaternion::get_roll()
{
	float roll = atan2(2*(s*v_1 + v_2*v_3), 1 - 2*(v_1*v_1 + v_2*v_2));
	return roll;
}

float Quaternion::get_pitch()
{
	float sinp = 2*(s*v_2 - v_3*v_1);
	return asin(sinp);
}

float Quaternion::get_yaw()
{
	float yaw = atan2(2*(s*v_3 + v_1*v_2), 1 - 2*(v_2*v_2 + v_3*v_3));
	return yaw;
}


BLA::Matrix<3> Quaternion::to_rpy()
{
	// Returns roll, pitch, yaw from quaternion
	BLA::Matrix<3> rpy;
	float test = v_1*v_2 + v_3*s;
	// Serial.println(s);
	// Serial.println(v_1);
	// Serial.println(v_2);
	// Serial.println(v_3);
	Serial.println(test);

	if (test > 0.499)
	{
		Serial.println("yess");
		rpy(0) = 0;						// roll
		rpy(1) = M_PI / 2.0;			// pitch
		rpy(2) = 2 * atan2(v_1, v_3);	// yaw
		return rpy;
	}

	if (test < -0.499)
	{
		rpy(0) = 0;						// roll
		rpy(1) = -M_PI / 2.0;			// pitch
		rpy(2) = -2 * atan2(v_1, v_3);	// yaw
		return rpy;
	}

	float sqx = v_1 * v_1;
	float sqy = v_2 * v_2;
	float sqz = v_3 * v_3;

	rpy(0) = atan2(2*v_1*s - 2*v_2*v_3, 1 - 2*sqx - 2*sqz);
	rpy(1) = asin(2*test);
	rpy(2) = atan2(2*v_2*s - 2*v_1*v_3, 1 - 2*sqy - 2*sqz);
	return rpy;
}

/*** To vector method ***/
// Converts quaternion to a BLA::Matrix 4x1 vector
BLA::Matrix<4> Quaternion::to_vector()
{
	BLA::Matrix<4> quat_vect = {this->s, this->v_1, this->v_2, this->v_3};
	return quat_vect;	
}
