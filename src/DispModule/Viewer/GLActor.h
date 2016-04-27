#ifndef GLActor_H
#define GLActor_H

#include <GL/glew.h>
#include <vector>
#include <string>
#include "Eigen\Eigen"

enum DrawbleType
{
    ML_POINT,
    ML_LINE,
    ML_MESH
};

class GLActor
{
    typedef Eigen::Vector3f Vector3f;

public:
    GLActor(){};
    GLActor(DrawbleType drawType, float pointSize) :drawable_type(drawType), point_size(pointSize){};
    ~GLActor(){};

    void draw();
    void addElement(float x, float y, float z, float r, float g, float b);
    void setPointSize(float pointSize);
    void clearElement();

private:
    std::vector<Vector3f> points;
    std::vector<int> topology;
    std::vector<Vector3f> colors;
    DrawbleType drawable_type;
    GLfloat point_size;
};

#endif