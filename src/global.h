#ifndef GLOBAL_H
#define GLOBAL_H
#include <vector>
#include <opencv2/core/core.hpp>
#include <QGLViewer/qglviewer.h>
namespace GLOBAL
{
	static cv::Mat m_mat_source_mask0_;
	static std::vector<int> m_selected_faces_;
	static qglviewer::Vec m_scence_center_;
	static double	m_scene_radius_;
	static double	m_zClippingCoefficient_;
	static double	m_FieldOfView_;
}

#endif