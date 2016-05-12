#ifndef texture_label_H
#define texture_label_H
#include <QLabel>
#include <QObject>
#include <QString>
#include <QImage>
#include <QWidget>
#include <QMouseEvent>
class MiniTexture;
class MainWindow_Texture;
class Texture_Label : public QLabel
{
  Q_OBJECT

public:
	Texture_Label(QWidget * parent = 0, Qt::WindowFlags f = 0);
	~Texture_Label();


	
public:
	void set_file(QString);
	void mousePressEvent(QMouseEvent * event);
	void mouseReleaseEvent(QMouseEvent * event);
	void mouseDoubleClickEvent(QMouseEvent * event);

	void set_mainwindow(MainWindow_Texture*);
	MainWindow_Texture* get_mainwindow();
private:
	void set_image(const QImage&);
public slots:
	void clear();
	void select_cut(MiniTexture*);
private:

	MainWindow_Texture* m_mainWindow_;

	QString m_image_file_;
	QImage  m_image_;
	QLabel* m_full_image_label_;
	bool    m_with_image_;
	bool    m_new_image_;
signals:
	void cut_selected( MiniTexture*);
};
#endif // !DispModuleHandler_H
