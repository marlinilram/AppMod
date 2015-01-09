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
    void getModelWithTexture(Model *model, cv::Mat &rho_img);
    void getSnapShot(Model *model);
    void fixCamera();
    
    void addActors();
    void addDrawablePoint(float x, float y, float z, float r, float g, float b);
    void addDrawableTri(float v0[3], float v1[3], float v2[3], float c0[3], float c1[3], float c2[3]);
    void addDrawableLine(float v0[3], float v1[3], float c0[3], float c1[3]);

	void setCheckVisbStatus(bool on);
    void checkVisibleVertices(Model *model);
	void checkVertexVisbs(int pt_id, Model *model, Eigen::VectorXf &visb);
	bool checkVertexVisbs(int pt_id, Model *model, Eigen::Vector3f &view_dir);
	void checkModelVisbs(Model *model, std::vector<std::vector<bool>> &model_visbs);
    void resetScreen();
    inline void setShowModel(bool status) { show_model = status; };
	inline void getViewDirection(float view[3]) { view[0] = -camera()->viewDirection().x; view[1] = -camera()->viewDirection().y; view[2] = -camera()->viewDirection().z; };

protected:
    virtual void draw();
    virtual void init();
    virtual void postDraw();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual QString helpString() const;

    void setBuffer();
    void setFBO();
    void setRealisticRenderBuffer();

private:
    void drawCornerAxis();
    void setWheelandMouse();
    void setKeyShortcut();

private:
    QGLShaderProgram *shaderProgram;
    QGLBuffer *vertices_buffer;
    QGLBuffer *faces_buffer;
    QGLBuffer *colors_buffer;
    QGLBuffer *rhod_irradiance_buffer;
    QGLBuffer *rhos_specular_buffer;
    Bound scene_bounds;
    float scene_radius;
	qglviewer::Vec scene_center;

    QVector<QVector3D> vertices;
    QVector<QVector3D> colors;
    QVector<QVector3D> rhod_irradiance;
    QVector<QVector3D> rhos_specular;
    GLuint *faces;

    GLenum num_faces;
    GLenum num_vertices;

    GLuint offscr_color;
    GLuint offscr_depth;
    GLuint offscr_fbo;

    std::vector<GLActor> actors;

	GLuint visible_query;
	GLuint visible_result;

    QGLShaderProgram *shaderProgramTexture;
    QVector<QVector2D> text_coords;
    GLuint text_ogl;

private:
    bool wireframe_, flatShading_;
    bool showCornerAxis_;
    bool camera_fixed;
    bool show_model;
    bool show_other_drawable;
    bool render_with_texture;
};

#endif