#include "viewer_selector.h"
#include "QGLViewer/qglviewer.h"
#include <QPoint>
Viewer_Selector::Viewer_Selector()
{
	m_gl_viewer_ = NULL;
};
Viewer_Selector::~Viewer_Selector()
{

};
void Viewer_Selector::set_viewer(QGLViewer* v)
{
	this->m_gl_viewer_ = v;
};
QGLViewer*  Viewer_Selector::get_viewer()
{
	return this->m_gl_viewer_;
};

bool Viewer_Selector::is_point_in_polygon(const qglviewer::Vec& P, const std::vector<QPoint>& polygon, QGLViewer* v)
{
	if (v==NULL)
	{
		return false;
	}

	qglviewer::Vec  proc = v->camera()->projectedCoordinatesOf(P);
	return Viewer_Selector::is_point_in_polygon(QPoint(proc.x, proc.y), polygon);
};
bool Viewer_Selector::delete_points_in_polygon(std::vector<qglviewer::Vec>& points, std::vector<int>& ids, const std::vector<QPoint>& polygon, QGLViewer* v)
{
	if (v == NULL)
	{
		return false;
	}
	for (unsigned int i = 0; i < points.size(); i++)
	{
		bool in = Viewer_Selector::is_point_in_polygon(points[i], polygon, v);
		if (in)
		{
			for (unsigned int j = i; j < points.size() - 1; j++)
			{
				points[j] = points[j + 1];
				ids[j] = ids[j + 1];
			}
			points.resize(points.size() - 1);
			ids.resize(points.size());
			i--;
		}
	};
	return true;
};
bool Viewer_Selector::is_point_in_polygon(const QPoint& P, const std::vector<QPoint>& polygon)
{
	bool bIsInside = false;

	int N = (int)polygon.size();

	int i = 0;
	int j = N - 1;

	for (; i < N; j = i, ++i)
	{
		const QPoint& U0 = polygon[i];
		const QPoint& U1 = polygon[j];  // current edge

		if (((U0.y() <= P.y()) && (P.y() < U1.y())) ||  // U1 is above the ray, U0 is on or below the ray
			((U1.y() <= P.y()) && (P.y() < U0.y())))    // U0 is above the ray, U1 is on or below the ray
		{
			// find x-intersection of current edge with the ray. 
			// Only consider edge crossings on the ray to the right of P.
			float x = U0.x() + (P.y() - U0.y()) * (U1.x() - U0.x()) / (U1.y() - U0.y());
			if (x > P.x())
				bIsInside = !bIsInside;
		}
	}

	return bIsInside;
};