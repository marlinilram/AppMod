#include "StandardCamera.h"

StandardCamera::StandardCamera()
{
    standard = true;
}

qreal StandardCamera::zNear() const
{
    if (standard)
        return 0.1;
    else
        return qglviewer::Camera::zNear();
}

qreal StandardCamera::zFar() const
{
    if (standard)
        return 1000.0;
    else
        return qglviewer::Camera::zFar();
}