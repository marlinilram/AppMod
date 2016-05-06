#ifndef ShapeList_H
#define ShapeList_H
#include <QListWidget>
#include <string>
class ShapeItem;
class MiniTexture;
class ShapeList : public QListWidget
{
  Q_OBJECT

public:
	ShapeList(QWidget * parent = 0);
	~ShapeList();
	
public:
	void clear();
	void connect_signal();
	void setParent(QWidget * parent);
	void setGeometry(int x, int y, int w, int h);
	QListWidget* texture_list();
	void show();
	void set_texture(ShapeItem * item, MiniTexture* texture);
    std::string getTexturePath(int item_id);
	MiniTexture* get_mini_texture(int item_id);
private slots:
	void item_changed(QListWidgetItem * current);
	void texture_item_clicked(QListWidgetItem *);
private:
	std::vector<ShapeItem*>	 m_items_;
	QListWidget*			 m_texture_list_;

};
#endif // !DispModuleHandler_H
