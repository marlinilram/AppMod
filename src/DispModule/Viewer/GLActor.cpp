#include "GLActor.h"

void GLActor::draw()
{
    if (drawable_type == ML_POINT)
    {
        GLfloat ori_point_size;
        glGetFloatv(GL_POINT_SIZE, &ori_point_size);

        glPointSize(point_size);

        glBegin(GL_POINTS);

        for (decltype(points.size()) i = 0; i < points.size(); ++i)
        {
            glColor3f(GLfloat(colors[i](0)), GLfloat(colors[i](1)), GLfloat(colors[i](2)));
            glVertex3f(GLfloat(points[i](0)), GLfloat(points[i](1)), GLfloat(points[i](2)));
        }

        glEnd();
        glPointSize(ori_point_size);
    }

    if (drawable_type == ML_MESH)
    {
        GLfloat ori_point_size;
        glGetFloatv(GL_POINT_SIZE, &ori_point_size);

        glPointSize(point_size);

        glBegin(GL_TRIANGLES);

        for (decltype(points.size()) i = 0; i < points.size(); ++i)
        {
            glColor3f(GLfloat(colors[i](0)), GLfloat(colors[i](1)), GLfloat(colors[i](2)));
            glVertex3f(GLfloat(points[i](0)), GLfloat(points[i](1)), GLfloat(points[i](2)));
        }

        glEnd();
    }

    if (drawable_type == ML_LINE)
    {
        GLfloat ori_point_size;
        GLfloat ori_line_width;
        glGetFloatv(GL_POINT_SIZE, &ori_point_size);
        glGetFloatv(GL_LINE_WIDTH, &ori_line_width);

        glPointSize(point_size);
        glLineWidth(point_size);

        glBegin(GL_LINES);

        for (decltype(points.size()) i = 0; i < points.size(); ++i)
        {
            glColor3f(GLfloat(colors[i](0)), GLfloat(colors[i](1)), GLfloat(colors[i](2)));
            glVertex3f(GLfloat(points[i](0)), GLfloat(points[i](1)), GLfloat(points[i](2)));
        }

        glEnd();

        glPointSize(ori_point_size);
        glLineWidth(ori_line_width);
    }
}

void GLActor::addElement(float x, float y, float z, float r, float g, float b)
{
    if (drawable_type == ML_POINT)
    {
        points.push_back(Vector3f(x, y, z));
        colors.push_back(Vector3f(r, g, b));
        topology.push_back(topology.size());
    }

    if (drawable_type == ML_MESH)
    {
        points.push_back(Vector3f(x, y, z));
        colors.push_back(Vector3f(r, g, b));
        topology.push_back(topology.size());
    }

    if (drawable_type == ML_LINE)
    {
        points.push_back(Vector3f(x, y, z));
        colors.push_back(Vector3f(r, g, b));
        topology.push_back(topology.size());
    }
}

void GLActor::setPointSize(float pointSize)
{
    point_size = GLfloat(pointSize);
}

void GLActor::clearElement()
{
    points.clear();
    topology.clear();
    colors.clear();
}