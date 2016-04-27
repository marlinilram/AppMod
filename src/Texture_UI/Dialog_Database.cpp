#include "Dialog_Database.h"

Dialog_Database::Dialog_Database(QWidget * parent, Qt::WindowFlags flags)
:QDialog(parent, flags)
{
	this->setupUi(this);
	this->setFixedSize(this->width(), this->height());
}
Dialog_Database::~Dialog_Database()
{
}
