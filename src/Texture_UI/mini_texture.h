#ifndef MiniTexture_H
#define MiniTexture_H
#include <QLabel>
#include <QObject>
#include <QString>
#include <QImage>
#include <QWidget>
#include <QMouseEvent>
class MiniTexture : public QLabel
{
  Q_OBJECT

public:
	MiniTexture(QWidget * parent = 0, Qt::WindowFlags f = 0);
	~MiniTexture();
	
public:
	void set_file(QString);
	QString get_file_name();
private:
	void set_image(const QImage&);
	void load_texture();
public slots:
	void clear();
private:
	QString m_image_file_;
	QImage  m_image_;
	void* m_texture_;
	bool    m_with_image_;
	bool    m_new_image_;
};
#endif // !DispModuleHandler_H
