#include <glew-1.11.0/include/GL/glew.h>
#include "MainWindow.h"
#include <QtWidgets/QApplication>
#include "MainWindow_Texture.h"
int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  MainWindow *window = new MainWindow();
  window->show();
  //window->hide();
  return a.exec();
}