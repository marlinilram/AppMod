#ifndef MiniTexture_H
#define MiniTexture_H
#include <QLabel>
#include <QObject>
#include <QString>
#include <QImage>
#include <QWidget>
#include <QMouseEvent>
#include "AppearanceModel.h"
class AppearanceModel;
class MiniTexture : public QLabel
{
  Q_OBJECT

public:
	MiniTexture(QWidget * parent = 0, Qt::WindowFlags f = 0);
	~MiniTexture();
	
public:
	void set_file(QString);
	QString get_file_name();
	static void generate_mask(const QImage& im, QImage& mask);
private:
	void set_image(const QImage&);
	void load_texture();


public:
	void mouseDoubleClickEvent(QMouseEvent * event);
public slots:
	void clear();


private:
	QString m_image_file_;
	QImage  m_image_;
	QImage  m_image_mask_;
	void* m_texture_;
	bool    m_with_image_;
	bool    m_new_image_;
	AppearanceModel* m_appearance_model_;
	cv::Mat		m_show_images_;
};
#endif // !DispModuleHandler_H
