#ifndef Viewer_H
#define Viewer_H

#include <glew-1.11.0/include/GL/glew.h>
#include <QGLViewer/qglviewer.h>
#include <QGLShader>
#include <QGLBuffer>
#include "Bound.h"
#include "Model.h"
#include "GLActor.h"

class Model;

class Viewer : public QGLViewer
{
public:
    Viewer(QWidget *widget);
    Viewer();
    ~Viewer();

public:
    void getModel(Model *model);
    void getSnapShot(Model *model);
    void fixCamera();
    
    void addActors();
    void addDrawablePoint(float x, float y, float z, float r, float g, float b);
    void addDrawableTri(float v0[3], float v1[3], float v2[3], float c0[3], float c1[3], float c2[3]);
    void addDrawableLine(float v0[3], float v1[3], float c0[3], float c1[3]);

    void checkVisibleVertices(Model *model);
    void resetScreen();
    inline void setShowModel(bool status) { show_model = status; };

protected:
    virtual void draw();
    virtual void init();
    virtual void postDraw();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual QString helpString() const;

    void setBuffer();
    void setFBO();

private:
    void drawCornerAxis();
    void setWheelandMouse();
    void setKeyShortcut();

private:
    QGLShaderProgram *shaderProgram;
    QGLBuffer *vertices_buffer;
    QGLBuffer *faces_buffer;
    QGLBuffer *colors_buffer;
    Bound scene_bounds;
    float scene_radius;

    QVector<QVector3D> vertices;
    QVector<QVector3D> colors;
    GLuint *faces;

    GLenum num_faces;
    GLenum num_vertices;

    GLuint offscr_color;
    GLuint offscr_depth;
    GLuint offscr_fbo;

    std::vector<GLActor> actors;

private:
    bool wireframe_, flatShading_;
    bool showCornerAxis_;
    bool camera_fixed;
    bool show_model;
    bool show_other_drawable;
};

#endif