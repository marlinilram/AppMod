#ifndef Bound_H
#define Bound_H

class Bound
{
public:
    Bound() : minX(0), maxX(0), minY(0), maxY(0), minZ(0), maxZ(0) {};
    ~Bound() {};
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
};

#endif