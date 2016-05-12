#ifndef Dialog_Database_H
#define Dialog_Database_H

#include <QDialog>
#include "ui_Dialog_Database.h"


class Dialog_Database : public QDialog, public Ui::Dialog_Database
{
    Q_OBJECT

public:
	Dialog_Database(QWidget*, Qt::WindowFlags); 
	~Dialog_Database();

private:

private slots:

private:


};

#endif