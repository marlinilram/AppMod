#include "Viewer.h"
#include "Coarse.h"

#include <QMenu>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMap>
#include <QCursor>
#include <QGLViewer/manipulatedFrame.h>
#include <cv.h>
#include <highgui.h>

#include <math.h>
#include "LOG.h"

Viewer::Viewer(QWidget *widget) : QGLViewer(widget), wireframe_(false), flatShading_(false)
{
    shaderProgram = new QGLShaderProgram;
    vertices_buffer = new QGLBuffer;
    colors_buffer = new QGLBuffer;
    faces_buffer = new QGLBuffer(QGLBuffer::IndexBuffer);

    wireframe_ = false;
    flatShading_ = true;
    showCornerAxis_ = false;
    camera_fixed = false;
    show_model = true;
    show_other_drawable = true;
}

Viewer::Viewer()
{
    QGLViewer::QGLViewer();
}

Viewer::~Viewer()
{
    delete shaderProgram;
    delete vertices_buffer;
    delete faces_buffer;
    delete colors_buffer;
}

void Viewer::draw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (show_model)
    {
        shaderProgram->bind();

        shaderProgram->setUniformValue("fMeshSize", GLfloat(num_faces));

        colors_buffer->bind();
        shaderProgram->setAttributeBuffer("color", GL_FLOAT, 0, 3, 0);
        shaderProgram->enableAttributeArray("color");
        colors_buffer->release();

        vertices_buffer->bind();
        //shaderProgram->setAttributeArray("vertex", vertices.constData());
        shaderProgram->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
        shaderProgram->enableAttributeArray("vertex");
        vertices_buffer->release();

        //glDrawElements(GL_TRIANGLES, 3 * num_faces, GL_UNSIGNED_INT, faces);

        //glDrawArrays(GL_TRIANGLES, 0, vertices.size());

        faces_buffer->bind();
        glDrawElements(GL_TRIANGLES, num_faces * 3, GL_UNSIGNED_INT, (void*)0);
        faces_buffer->release();

        shaderProgram->disableAttributeArray("vertex");
        shaderProgram->release();
    }

    // draw actors
    if (show_other_drawable)
    {
        for (decltype(actors.size()) i = 0; i < actors.size(); ++i)
        {
            actors[i].draw();
        }
    }
}

void Viewer::init()
{
    // Restore previous viewer state.
    //restoreStateFromFile();
    makeCurrent();

    glewExperimental = TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cout << "glewInit failed, aborting." << endl;
    }

    GLint major = 0;
    GLint minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    std::cout << major << "." << minor << std::endl;

    // Set shader
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    //glEnable(GL_CULL_FACE);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_FLAT);

    qglClearColor(QColor(Qt::white));
    setBackgroundColor(QColor(Qt::white));
    setForegroundColor(QColor(Qt::white));

    shaderProgram->addShaderFromSourceFile(QGLShader::Vertex, "shader/vertexShader.vsh");
    shaderProgram->addShaderFromSourceFile(QGLShader::Fragment, "shader/fragmentShader.fsh");
    shaderProgram->link();

    num_vertices = 0;
    vertices.clear();
    num_faces = 0;
    faces = nullptr;

    setBuffer();

    actors.push_back(GLActor(ML_POINT, 5.0f));
    actors.push_back(GLActor(ML_MESH, 1.0f));
    actors.push_back(GLActor(ML_LINE, 1.0f));
    //addDrawableLine(Eigen::Vector3f(0, 0, 0).data(), Eigen::Vector3f(10, 10, 10).data(),
    //    Eigen::Vector3f(0, 0, 1).data(), Eigen::Vector3f(0, 0, 1).data());

    setSceneCenter(qglviewer::Vec(0, 0, 0));
    setSceneRadius(50);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 5);
    
    setKeyShortcut();
    setWheelandMouse();

    // Display the help window. The help window tabs are automatically updated when you define new
    // standard key or mouse bindings (as is done above). Custom bindings descriptions are added using
    // setKeyDescription() and setMouseBindingDescription().
    //help();

    doneCurrent();
}

void Viewer::setKeyShortcut()
{
    /////////////////////////////////////////////////////
    //       Keyboard shortcut customization           //
    //      Changes standard action key bindings       //
    /////////////////////////////////////////////////////

    // Set 'Control+F' as the FPS toggle state key.
    setShortcut(DISPLAY_FPS, Qt::CTRL + Qt::Key_F);

    // Add custom key description (see keyPressEvent).
    setKeyDescription(Qt::Key_W, "Toggles wire frame display");
    setKeyDescription(Qt::Key_F, "Toggles flat shading display");
}

void Viewer::setWheelandMouse()
{
    /////////////////////////////////////////////////////
    //         Mouse bindings customization            //
    //     Changes standard action mouse bindings      //
    /////////////////////////////////////////////////////

    //
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);

    // Left and right buttons together make a camera zoom : emulates a mouse third button if needed.
    setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::LeftButton, CAMERA, ZOOM);

    setMouseBinding(Qt::ControlModifier | Qt::ShiftModifier, Qt::RightButton, SELECT);
    setWheelBinding(Qt::NoModifier, CAMERA, MOVE_FORWARD);
    setMouseBinding(Qt::AltModifier, Qt::LeftButton, CAMERA, TRANSLATE);

    // Add custom mouse bindings description (see mousePressEvent())
    setMouseBindingDescription(Qt::NoModifier, Qt::RightButton, "Opens a camera path context menu");
}

///////////////////////////////////////////////
//      Define new key bindings : F & W      //
///////////////////////////////////////////////

void Viewer::keyPressEvent(QKeyEvent *e)
{
    // Get event modifiers key
    const Qt::KeyboardModifiers modifiers = e->modifiers();

    // A simple switch on e->key() is not sufficient if we want to take state key into account.
    // With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
    // That's why we use imbricated if...else and a "handled" boolean.
    bool handled = false;
    if ((e->key() == Qt::Key_W) && (modifiers == Qt::NoButton))
    {
        wireframe_ = !wireframe_;
        if (wireframe_)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        handled = true;
        updateGL();
    }
    else
        if ((e->key() == Qt::Key_F) && (modifiers == Qt::NoButton))
        {
        flatShading_ = !flatShading_;
        if (flatShading_)
            glShadeModel(GL_FLAT);
        else
            glShadeModel(GL_SMOOTH);
        handled = true;
        updateGL();
        }
        else
            if ((e->key() == Qt::Key_1) && (modifiers == Qt::NoButton))
            {
        showCornerAxis_ = !showCornerAxis_;
        handled = true;
        updateGL();
            }
    // ... and so on with other else/if blocks.

    if (!handled)
        QGLViewer::keyPressEvent(e);
}


///////////////////////////////////////////////////////////
//             Define new mouse bindings                 //
//   A camera viewpoint menu binded on right button      //
///////////////////////////////////////////////////////////

void Viewer::mousePressEvent(QMouseEvent* e)
{
    if ((e->button() == Qt::RightButton) && (e->modifiers() == Qt::NoButton))
    {
        QMenu menu(this);
        menu.addAction("Camera positions");
        menu.addSeparator();
        QMap<QAction*, int> menuMap;

        bool atLeastOne = false;
        // We only test the 20 first indexes. This is a limitation.
        for (unsigned short i = 0; i < 20; ++i)
            if (camera()->keyFrameInterpolator(i))
            {
            atLeastOne = true;
            QString text;
            if (camera()->keyFrameInterpolator(i)->numberOfKeyFrames() == 1)
                text = "Position " + QString::number(i);
            else
                text = "Path " + QString::number(i);

            menuMap[menu.addAction(text)] = i;
            }

        if (!atLeastOne)
        {
            menu.addAction("No position defined");
            menu.addAction("Use to Alt+Fx to define one");
        }

        QAction* action = menu.exec(e->globalPos());

        if (atLeastOne && action)
            camera()->playPath(menuMap[action]);
    }
    else
        QGLViewer::mousePressEvent(e);
}

QString Viewer::helpString() const
{
    QString text("<h2>K e y b o a r d A n d M o u s e</h2>");
    text += "This example illustrates the mouse and key bindings customization.<br><br>";
    text += "Use <code>setShortcut()</code> to change standard action key bindings (display of axis, grid or fps, exit shortcut...).<br><br>";
    text += "Use <code>setMouseBinding()</code> and <code>setWheelBinding()</code> to change standard action mouse bindings ";
    text += "(camera rotation, translation, object selection...).<br><br>";
    text += "If you want to define <b>new</b> key or mouse actions, overload <code>keyPressEvent()</code> and/or ";
    text += "<code>mouse(Press|Move|Release)Event()</code> to define and bind your own new actions. ";
    text += "Use <code>setKeyDescription()</code> and <code>setMouseBindingDescription()</code> to add a description of your bindings in the help window.<br><br>";
    text += "In this example, we defined the <b>F</b> and <b>W</b> keys and the right mouse button opens a popup menu. ";
    text += "See the keyboard and mouse tabs in this help window for the complete bindings description.<br><br>";
    text += "By the way, exit shortcut has been binded to <b>Ctrl+Q</b>.";
    return text;
}

void Viewer::getModel(Model *model)
{
    std::vector<float> model_vertices;
    std::vector<unsigned int> model_faces;
    std::vector<float> model_colors;
    model->passData(model_vertices, model_faces, model_colors);

    vertices.clear();
    num_vertices = (GLenum)model_vertices.size() / 3;
    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size() / 3; ++i)
    {
        vertices.push_back(QVector3D(model_vertices[3 * i + 0], model_vertices[3 * i + 1], model_vertices[3 * i + 2]));
    }

    colors.clear();
    for (decltype(model_colors.size()) i = 0; i < model_colors.size() / 3; ++i)
    {
        colors.push_back(QVector3D(model_colors[3 * i + 0], model_colors[3 * i + 1], model_colors[3 * i + 2]));
    }

    if (faces != nullptr) delete faces;
    faces = new GLuint[model_faces.size()];
    num_faces = (GLenum)model_faces.size() / 3;
    for (decltype(model_faces.size()) i = 0; i < model_faces.size(); ++i)
    {
        faces[i] = model_faces[i];
    }

    setBuffer();

    scene_bounds = model->getBounds();

    scene_center = qglviewer::Vec((scene_bounds.minX + scene_bounds.maxX) / 2,
                                (scene_bounds.minY + scene_bounds.maxY) / 2,
                                (scene_bounds.minZ + scene_bounds.maxZ) / 2);
    
    setSceneCenter(scene_center);

    float x_span = (scene_bounds.maxX - scene_bounds.minX) / 2;
    float y_span = (scene_bounds.maxY - scene_bounds.minY) / 2;
    float z_span = (scene_bounds.maxZ - scene_bounds.minZ) / 2;
    scene_radius = x_span>y_span ? (x_span > z_span ? x_span : z_span) : (y_span > z_span ? y_span : z_span);
	scene_radius *= 1.5;

    setSceneRadius(scene_radius);
    camera()->fitSphere(scene_center, scene_radius);

    

    setStateFileName(QString((model->getDataPath()+"/camera_info.xml").c_str()));
    if (restoreStateFromFile())
        std::cout << "Load camera info successes...\n";
}

void Viewer::setBuffer()
{
    makeCurrent();

    if (vertices_buffer->isCreated())
        vertices_buffer->destroy();
    vertices_buffer->create();
    vertices_buffer->bind();
    vertices_buffer->allocate(num_vertices * 3 * sizeof(GLfloat));

    vertices_buffer->write(0, vertices.constData(), num_vertices * 3 * sizeof(GLfloat));

    vertices_buffer->release();

    if (faces_buffer->isCreated())
        faces_buffer->destroy();
    faces_buffer->create();
    faces_buffer->bind();
    faces_buffer->allocate(num_faces * 3 * sizeof(GLuint));

    faces_buffer->write(0, faces, num_faces * 3 * sizeof(GLuint));

    faces_buffer->release();

    if (colors_buffer->isCreated()) colors_buffer->destroy();
    colors_buffer->create();
    colors_buffer->bind();
    colors_buffer->allocate(num_vertices * 3 * sizeof(GLfloat));

    colors_buffer->write(0, colors.constData(), num_vertices * 3 * sizeof(GLfloat));

    colors_buffer->release();

    doneCurrent();
}

void Viewer::setFBO()
{
    // set frame buffer object

    int height = QPaintDevice::height();
    int width = QPaintDevice::width();

    glGenFramebuffers(1, &offscr_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

    glGenRenderbuffers(1, &offscr_color);
    glBindRenderbuffer(GL_RENDERBUFFER, offscr_color);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA32F, width, height);

    glGenRenderbuffers(1, &offscr_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, offscr_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, width, height);

    // attach color and depth textures to fbo
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, offscr_color);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, offscr_depth);

    static const GLenum draw_buffers[] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, draw_buffers);

    GLenum framebuffer_status;
    framebuffer_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (framebuffer_status != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "set frame buffer object failed\n";

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Viewer::drawCornerAxis()
{
    int viewport[4];
    int scissor[4];

    // The viewport and the scissor are changed to fit the lower left
    // corner. Original values are saved.
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetIntegerv(GL_SCISSOR_BOX, scissor);

    // Axis viewport size, in pixels
    const int size = 150;
    glViewport(0, 0, size, size);
    glScissor(0, 0, size, size);

    // The Z-buffer is cleared to make the axis appear over the
    // original image.
    glClear(GL_DEPTH_BUFFER_BIT);

    // Tune for best line rendering
    glDisable(GL_LIGHTING);
    glLineWidth(3.0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMultMatrixd(camera()->orientation().inverse().matrix());

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);

    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);

    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_LIGHTING);

    // The viewport and the scissor are restored.
    glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void Viewer::postDraw()
{
    QGLViewer::postDraw();
    if (showCornerAxis_) drawCornerAxis();
}

void Viewer::getSnapShot(Model *model)
{
    makeCurrent();

    int height = QPaintDevice::height();
    int width = QPaintDevice::width();
    float *primitive_buffer = new float[height*width];
    float *z_buffer = new float[height*width];
    float *image_buffer = new float[3*height*width];

    setFBO();

    glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

    draw();

    glReadBuffer(GL_COLOR_ATTACHMENT0);

    glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, primitive_buffer);
    glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, image_buffer);
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, z_buffer);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // get camera info, matrix is column major
    GLfloat modelview[16];
    GLfloat projection[16];
    GLint viewport[4];
    camera()->getModelViewMatrix(modelview);
    camera()->getProjectionMatrix(projection);
    camera()->getViewport(viewport);

    //cv::Mat pixel_img = cv::Mat(height, width, CV_64FC1, 0.0);

    //// opengl buffer image stores in row major from left bottom
    //for (int i = 0; i < height; ++i)
    //{
    //    for (int j = 0; j < width; ++j)
    //    {
    //        pixel_img.at<double>(height - 1 - i, j) = primitive_ID[i*width + j];
    //    }
    //}

    //cv::Mat imgCV = cv::Mat(height, width, CV_32FC3, img);
    //cv::flip(imgCV, imgCV, 0);
    //cv::imwrite("img.png", imgCV * 255);

    cv::Mat primitive_ID_img(height, width, CV_32FC1, primitive_buffer);
    cv::flip(primitive_ID_img, primitive_ID_img, 0);
    cv::Mat z_img(height, width, CV_32FC1, z_buffer);
    cv::flip(z_img, z_img, 0);
    cv::Mat r_img(height, width, CV_32FC3, image_buffer);
    cv::flip(r_img, r_img, 0);

    cv::Mat primitive_ID(height, width, CV_32S, cv::Scalar(-1));

    //cv::FileStorage fs("z_depth.yml", cv::FileStorage::WRITE);
    //fs << "mat" << primitive_ID_img;
    //fs.release();

    //cv::Mat test = primitive_ID_img.clone();
    //cv::threshold(primitive_ID_img, test, 0, 1, cv::THRESH_BINARY);

    //std::string a = "PrimitiveID:\n";
    for (int j = 0; j < width; ++j)
        for (int i = 0; i < height; ++i)
        {
            float fPrimitive = primitive_ID_img.at<float>(i, j);

            fPrimitive = fPrimitive*num_faces;
            int iPrimitive = (int)(fPrimitive < 0 ? (fPrimitive - 0.5) : (fPrimitive + 0.5));

        //if (iPrimitive == 169)
        //{
        //    test.at<float>(i, j) = 1;
        //}
        //else test.at<float>(i, j) = 0;
            if (iPrimitive < (int)num_faces) primitive_ID.at<int>(i, j) = iPrimitive;
        //{
            //if (iPrimitive == 168 || iPrimitive == 169 || iPrimitive == 170)
            //    a += std::to_string(i) + " " + std::to_string(j) + " " + std::to_string(iPrimitive) + "\n";
        //}
        }
    //LOG::Instance()->OutputMisc(a.c_str());
    //cv::imwrite("primitiveImg.png", primitive_ID_img * 255);

    model->passRenderImgInfo(z_img, primitive_ID, r_img);
    model->passCameraPara(modelview, projection, viewport);

    std::string data_path = model->getDataPath();
    cv::imwrite(data_path + "/matched.png", r_img*255);

    delete primitive_buffer;
    delete z_buffer;
    delete image_buffer;

    doneCurrent();
}

void Viewer::fixCamera()
{
    if (camera_fixed)
    {
        camera_fixed = false;
        setWheelandMouse();
    }
    else
    {
        camera_fixed = true;
        clearMouseBindings();
    }
}

void Viewer::addDrawablePoint(float x, float y, float z, float r, float g, float b)
{
    actors[0].addElement(x, y, z, r, g, b);
}

void Viewer::addDrawableTri(float v0[3], float v1[3], float v2[3], float c0[3], float c1[3], float c2[3])
{
    actors[1].addElement(v0[0], v0[1], v0[2], c0[0], c0[1], c0[2]);
    actors[1].addElement(v1[0], v1[1], v1[2], c1[0], c1[1], c1[2]);
    actors[1].addElement(v2[0], v2[1], v2[2], c2[0], c2[1], c2[2]);
}

void Viewer::addDrawableLine(float v0[3], float v1[3], float c0[3], float c1[3])
{
    actors[2].addElement(v0[0], v0[1], v0[2], c0[0], c0[1], c0[2]);
    actors[2].addElement(v1[0], v1[1], v1[2], c1[0], c1[1], c1[2]);
}

void Viewer::checkVisibleVertices(Model *model)
{
    makeCurrent();

    glGenQueries(1, &visible_query);
    std::vector<bool> vertices_visible_state;

    draw();

    for (int i = 0; i < num_vertices; ++i)
    {
        glBeginQuery(GL_SAMPLES_PASSED, visible_query);
        glPointSize(5.0f);

        glBegin(GL_POINTS);

        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(GLfloat(vertices[i][0]), GLfloat(vertices[i][1]), GLfloat(vertices[i][2]));

        glEnd();

        glEndQuery(GL_SAMPLES_PASSED);
        glGetQueryObjectuiv(visible_query, GL_QUERY_RESULT, &visible_result);
        if (visible_result == 0)
            vertices_visible_state.push_back(false);
        else
        {
            vertices_visible_state.push_back(true);
            addDrawablePoint(vertices[i][0], vertices[i][1], vertices[i][2], 1.0f, 0.0f, 0.0f);
        }
    }

    // need to return query
    glDeleteQueries(1, &visible_query);

    model->passVerticesVisbleStatus(vertices_visible_state);

    doneCurrent();

    updateGL();
}

void Viewer::resetScreen()
{
    makeCurrent();

    for (auto &i : actors)
    {
        i.clearElement();
    }

    updateGL();

    doneCurrent();
}

void Viewer::checkVertexVisbs(int pt_id, Model *model, Eigen::VectorXf &visb)
{
	makeCurrent();
	// change camera type to orthographic projection
	camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);

	// turn on off screen rendering
	setFBO();
	glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

	// for each view direction
	Eigen::MatrixX3f &S_mat = model->getModelLightObj()->getSampleMatrix();
	qglviewer::Vec view_dir;

	GLuint visible_query;
	glGenQueries(1, &visible_query);
	GLuint visible_result;

	visb = Eigen::VectorXf::Ones(S_mat.rows());

	//int height = QPaintDevice::height();
	//int width = QPaintDevice::width();
	//float *image_buffer = new float[3*height*width];

	for (int i = 0; i < S_mat.rows(); ++i)
	{
		view_dir.x = -S_mat(i, 0);
		view_dir.y = -S_mat(i, 1);
		view_dir.z = -S_mat(i, 2);
		camera()->setViewDirection(view_dir);
		camera()->fitSphere(scene_center, scene_radius);
		

		//gluLookAt(1.5*scene_radius*view_dir.x, 1.5*scene_radius*view_dir.y, 1.5*scene_radius*view_dir.z, 
		//	scene_center.x, scene_center.y, scene_center.z, 0, 0, 1);	

		preDraw();
		draw();

		glBeginQuery(GL_ANY_SAMPLES_PASSED, visible_query);
		glPointSize(1.5f);

		glBegin(GL_POINTS);

		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(GLfloat(vertices[pt_id][0]), GLfloat(vertices[pt_id][1]), GLfloat(vertices[pt_id][2]));
		//std::cout<<vertices[pt_id][0]<<"\t"<<vertices[pt_id][1]<<"\t"<<vertices[pt_id][2]<<"\n";

		glEnd();

		glEndQuery(GL_ANY_SAMPLES_PASSED);
		glGetQueryObjectuiv(visible_query, GL_QUERY_RESULT, &visible_result);
		if (visible_result == 0)
			visb(i) = 0.0f;
		//else
		//{
		//	visb(i) = 1.0f;
		//	//std::cout <<"visible\n";
		//}

		//glReadBuffer(GL_COLOR_ATTACHMENT0);
		//glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, image_buffer);
		//cv::Mat r_img(height, width, CV_32FC3, image_buffer);
		//cv::flip(r_img, r_img, 0);
		//cv::imwrite(model->getDataPath() + "/imgs/" + std::to_string(i) + "vd.png", r_img*255);

		//std::cout<<"ith view direction: " << i << "\n";
	}


	glDeleteQueries(1, &visible_query);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	restoreStateFromFile();

	doneCurrent();
	std::cout<<"visib test finished...\n";
	//
}

void Viewer::checkModelVisbs(Model *model, std::vector<std::vector<bool>> &model_visbs)
{
	makeCurrent();
	// change camera type to orthographic projection
	camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);

	// turn on off screen rendering
	setFBO();
	glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

	// for each view direction
	Eigen::MatrixX3f &S_mat = model->getModelLightObj()->getSampleMatrix();
	qglviewer::Vec view_dir;

	GLuint visible_query;
	glGenQueries(1, &visible_query);
	GLuint visible_result;



}

bool Viewer::checkVertexVisbs(int pt_id, Model *model, Eigen::Vector3f &view_dir)
{

	//makeCurrent();

	

	camera()->setViewDirection(qglviewer::Vec(-view_dir(0),-view_dir(1),-view_dir(2)));
	camera()->fitSphere(scene_center, scene_radius);


	//gluLookAt(1.5*scene_radius*view_dir.x, 1.5*scene_radius*view_dir.y, 1.5*scene_radius*view_dir.z, 
	//	scene_center.x, scene_center.y, scene_center.z, 0, 0, 1);	

	preDraw();
	draw();

	glBeginQuery(GL_ANY_SAMPLES_PASSED, visible_query);
	//glPointSize(1.5f);

	glBegin(GL_POINTS);

	//glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(GLfloat(vertices[pt_id][0]), GLfloat(vertices[pt_id][1]), GLfloat(vertices[pt_id][2]));
	//std::cout<<vertices[pt_id][0]<<"\t"<<vertices[pt_id][1]<<"\t"<<vertices[pt_id][2]<<"\n";

	glEnd();

	glEndQuery(GL_ANY_SAMPLES_PASSED);
	glGetQueryObjectuiv(visible_query, GL_QUERY_RESULT, &visible_result);

	//doneCurrent();
	//std::cout << visible_result<<"\n";
	if (visible_result == 0)
		return false;

	return true;
}

void Viewer::setCheckVisbStatus(bool on)
{
	if (on == true)
	{
		// change camera type to orthographic projection
		camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);

		// turn on off screen rendering
		setFBO();
		glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

		glGenQueries(1, &visible_query);
	}
	else
	{
		glDeleteQueries(1, &visible_query);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		restoreStateFromFile();
	}
}