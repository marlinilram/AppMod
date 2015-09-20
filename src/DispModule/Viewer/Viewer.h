#ifndef Viewer_H
#define Viewer_H

#include <glew-1.11.0/include/GL/glew.h>
#include <QGLViewer/qglviewer.h>
#include <QGLShader>
#include <QGLBuffer>
#include <Eigen\Eigen>
#include <Eigen\Sparse>
#include <cv.h>
#include <highgui.h>

class Model;
class Bound;
class GLActor;

class Viewer : public QGLViewer
{
public:
    Viewer(QWidget *widget);
    Viewer();
    ~Viewer();

public:
    void getModel(Model *model);
    void updateModelShape(Model *model);
    void getModelWithTexture(Model *model, cv::Mat &rho_img);
    void getSnapShot(Model *model, bool save_to_file = true);
    void renderImage(Model *model);
    void renderBGRAImage(Model *model);
    void renderNImage(Model *model);
    void fixCamera();
    void UpdateGLOutside();
    
    void addActors();
    void addDrawablePoint(float x, float y, float z, float r, float g, float b);
    void addDrawableTri(float v0[3], float v1[3], float v2[3], float c0[3], float c1[3], float c2[3]);
    void addDrawableLine(float v0[3], float v1[3], float c0[3], float c1[3]);
    void setBackGroundImage(QString fname);
    inline void setLighting(double x, double y, double z) { lighting = QVector3D(x, y, z); };
    inline void getLighting(double l[3]) { l[0] = lighting.x(); l[1] = lighting.y(); l[2] = lighting.z(); };


	void setCheckVisbStatus(bool on);
    void checkVisibleVertices(Model *model);
	void checkVertexVisbs(int pt_id, Model *model, Eigen::VectorXf &visb);
	bool checkVertexVisbs(int pt_id, Model *model, Eigen::Vector3f &view_dir);
	void checkModelVisbs(Model *model, std::vector<std::vector<bool>> &model_visbs);
    void resetScreen();
    void resetCamera(Model *model);
    void getCameraInfo(Model *model);
    inline void setShowModel(bool status) { show_model = status; };
    inline void setRenderNormImg(bool status) { render_normal_img = status; render_mode = status ? 1 : 0; };
    inline void setRenderMode(int mode) { render_mode = mode; };
    inline void setShowBackground(bool status) { show_background_img = status; };
    inline void getViewDirection(float view[3]) { view[0] = -camera()->viewDirection().x; view[1] = -camera()->viewDirection().y; view[2] = -camera()->viewDirection().z; };

	void drawTrackBall();

public:
	std::vector<CvPoint3D32f> objpts;
	std::vector<CvPoint3D32f> pts3d;
	std::vector<CvPoint2D32f> pts2d;
	CvPoint3D32f objectCentroid;
	double objectRadius;
	Viewer* viewer2;
	

protected:
    virtual void draw();
    virtual void drawBackground();
    virtual void init();
    virtual void postDraw();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual QString helpString() const;
	virtual void postSelection(const QPoint& point);
	virtual void drawWithNames();
	virtual void mouseMoveEvent(QMouseEvent *e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	void drawModel();
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
    QGLBuffer *normals_buffer;
    QGLBuffer *rhod_irradiance_buffer;
    QGLBuffer *rhos_specular_buffer;
    Bound* scene_bounds;
    float scene_radius;
	qglviewer::Vec scene_center;

    QVector<QVector3D> vertices;
    QVector<QVector3D> colors;
    QVector<QVector3D> normals;
    QVector<QVector3D> rhod_irradiance;
    QVector<QVector3D> rhos_specular;
    GLuint *faces;
    QVector3D lighting;

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
    float ratio, u_max, v_max;
    bool show_background_img;

    bool wireframe_, flatShading_;
    bool showCornerAxis_;
    bool camera_fixed;
    bool show_model;
    bool show_other_drawable;
    bool render_with_texture;
    bool render_normal_img;
    int render_mode;
	qglviewer::Vec orig, dir, selectedPoint;

	bool sycncamera;
};

#endif