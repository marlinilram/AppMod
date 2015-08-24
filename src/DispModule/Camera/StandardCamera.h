#ifndef StandardCamera_H
#define StandardCamera_H

#include "QGLViewer/camera.h"

class StandardCamera : public qglviewer::Camera
{
public:
    StandardCamera();

    virtual qreal zNear() const;
    virtual qreal zFar() const;

    void toggleMode() { standard = !standard; }
    bool isStandard() { return standard; }

private:
    bool standard;
};

#endif