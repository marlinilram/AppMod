#ifndef Vector3_H
#define Vector3_H

#include <math.h>

typedef float                       FT;

class Vector3
{
public:

	// members

	union 
	{
		struct { FT x; FT y; FT z; };
		struct { FT v[3]; };
	};

	// constructors

	Vector3();
	Vector3(const FT* args);
	Vector3(FT _x, FT _y, FT _z);
	Vector3(const Vector3& v);

	// set

	void set(const FT* args);
	void set(FT _x, FT _y, FT _z);
	void set(const Vector3& v);
	void set(const Vector3* v);
	// Per coordinate (explicit inline functions)
	void setx(FT newX) { v[0] = newX; }
	void sety(FT newY) { v[1] = newY; }
	void setz(FT newZ) { v[2] = newZ; }

	// Data access using indices
	FT&       operator[](int i)       { return (v[i]); }
	const FT& operator[](int i) const { return (v[i]); }

	// operators

	Vector3 operator *(FT f) const;
	Vector3 operator /(FT f) const;
	Vector3 operator +(const Vector3& v) const;
	Vector3 operator -(const Vector3& v) const;
	Vector3 operator -() const;
	//Matrix3 operator *(const Vector3& v) const;
	Vector3& operator *=(FT f);
	Vector3& operator /=(FT f);
	Vector3& operator +=(const Vector3& v);
	Vector3& operator -=(const Vector3& v);
	int operator==(const Vector3& v);
	int operator!=(const Vector3& v);
	friend int operator==(const Vector3& v1, const Vector3& v2);
	friend int operator!=(const Vector3& v1, const Vector3& v2) { return !(v1 == v2); }
	Vector3& operator=(const Vector3& v);
	Vector3& operator&(const Vector3& v); // reverse copy

	// linear algebra
	FT sqauremagnitude() const;
	FT magnitude() const;
	FT sqauredistance(const Vector3& v) const;
	FT distance(const Vector3& v) const;
	void normalize(); 
	void normalize(const Vector3& v);
	FT dot(const Vector3& v) const;
	Vector3 cross(const Vector3& v);
  inline bool IsZero(FT f){ return fabs(f) < 0.000100f;};
	//Matrix3 square() const;
};

#endif