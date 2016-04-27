#ifndef ShapeItem_H
#define ShapeItem_H
#include <QListWidgetItem>
#include "../Model/Shape.h"
class QListWidget;
class MiniTexture;
class ShapeItem : public QListWidgetItem
{
 // Q_OBJECT

public:
	ShapeItem(QListWidget * parent = 0, int type = Type);
	~ShapeItem();
	
public:
	Render_Shape* get_shape();
	void set_shape(Render_Shape* s);
	void set_texture(MiniTexture*);
	MiniTexture* get_texture();
private:
	Render_Shape*	m_shape_;
	MiniTexture*	m_mini_texture_;

};
#endif // !DispModuleHandler_H
