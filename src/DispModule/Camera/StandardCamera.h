#ifndef StandardCamera_H
#define StandardCamera_H

#include "QGLViewer/camera.h"

class StandardCamera : public qglviewer::Camera
{
public:
    StandardCamera();

    void computeProjectionMatrix() const;
};

#endif