#include "Vector3.h"

// ------------------------------------------------------------
// Vector3 functions
// ------------------------------------------------------------

// constructors

Vector3::Vector3()
{
	x = y = z = 0.0f;
}

Vector3::Vector3(const FT* args)
{
	x = args[0];
	y = args[1];
	z = args[2];
}

Vector3::Vector3(FT _x, FT _y, FT _z) 
{
	x = _x; 
	y = _y; 
	z = _z;
}

Vector3::Vector3(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

// set

void Vector3::set(const FT* args)
{
	x = args[0];
	y = args[1];
	z = args[2];
}

void Vector3::set(FT _x, FT _y, FT _z) 
{
	x = _x; 
	y = _y; 
	z = _z;
}

void Vector3::set(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

void Vector3::set(const Vector3* v)
{
	x = v->x;
	y = v->y;
	z = v->z;
}

// operators

Vector3 Vector3::operator *(FT f) const
{ 
	Vector3 tmp(*this);
	tmp *= f;
	return tmp;
}

Vector3 Vector3::operator /(FT f) const
{ 
	Vector3 tmp(*this);
	tmp /= f;
	return tmp;
}

Vector3 Vector3::operator +(const Vector3& v) const
{
	Vector3 tmp(*this);
	tmp += v;
	return tmp;
}

Vector3 Vector3::operator -(const Vector3& v) const
{
	Vector3 tmp(*this);
	tmp -= v;
	return tmp;
}

Vector3 Vector3::operator -() const
{
	Vector3 tmp(*this);
	tmp.x = -tmp.x;
	tmp.y = -tmp.y;
	tmp.z = -tmp.z;
	return tmp;
}

Vector3& Vector3::operator *=(FT f) 
{ 
	x *= f; 
	y *= f; 
	z *= f; 
	return *this;
}

Vector3& Vector3::operator /=(FT f) 
{ 
	FT d = 1.0f / f;
	x *= d; 
	y *= d; 
	z *= d; 
	return *this;
}

Vector3& Vector3::operator +=(const Vector3& v) 
{
	x += v.x; 
	y += v.y; 
	z += v.z; 
	return *this;
}

Vector3& Vector3::operator -=(const Vector3& v) 
{
	x -= v.x; 
	y -= v.y; 
	z -= v.z; 
	return *this;
}

int Vector3::operator==(const Vector3& v) 
{ 
	return ((x == v.x) && (y == v.y) && (z == v.z));
}

int Vector3::operator!=(const Vector3& v) 
{ 
	return ((x != v.x) || (y != v.y) || (z != v.z));
}

int operator==(const Vector3& v1, const Vector3& v2)
{
  return (v1.v[0] == v2.v[0] && v1.v[1] == v2.v[1] && v1.v[2] == v2.v[2]);
}

Vector3& Vector3::operator=(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

Vector3& Vector3::operator&(const Vector3& v)
{
	x = -v.x;
	y = -v.y;
	z = -v.z;
	return *this;
}

// linear algebra

FT Vector3::sqauremagnitude() const
{
	return x * x + y * y + z * z;
}

FT Vector3::magnitude() const
{
	return sqrt(sqauremagnitude());
}

FT Vector3::sqauredistance(const Vector3& v) const
{
	Vector3 tmp = Vector3(v.x - x, v.y - y, v.z - z);
	return tmp.sqauremagnitude();
}

FT Vector3::distance(const Vector3& v) const
{
	Vector3 tmp = Vector3(v.x - x, v.y - y, v.z - z);
	return tmp.magnitude();
}

void Vector3::normalize()
{
	FT m = magnitude();
	if(IsZero(m)) return;
	*this /= m;
}

void Vector3::normalize(const Vector3& v)
{
	*this = v;
	normalize();
}
FT Vector3::dot(const Vector3& v) const 
{ 
	return x * v.x + y * v.y + z * v.z; 
}

Vector3 Vector3::cross(const Vector3& v)
{ 
	return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); 
}
