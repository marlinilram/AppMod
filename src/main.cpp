#include <glew-1.11.0/include/GL/glew.h>
#include "MainWindow.h"
#include <QtWidgets/QApplication>
#include "MainWindow_Texture.h"
#include <time.h>
int main(int argc, char **argv)
{
	srand(time(NULL));
  QApplication a(argc, argv);

  MainWindow *window = new MainWindow();
  window->show();

  return a.exec();
}

//#include<windows.h>
//#include <iostream>
//int main(void)
//{
//	// 获取一个可供画图的DC，我这里就直接用桌面算了
//	HDC hdc = GetWindowDC(GetDesktopWindow());
//
//	// 创建红色1像素宽度的实线画笔
//	HPEN hpen1 = CreatePen(PS_SOLID, 1, RGB(255, 0, 0));
//	// 创建绿色5像素宽度的破折画笔，如果你想创建其他种类的画笔请参阅MSDN
//	HPEN hpen2 = CreatePen(PS_DASH, 5, RGB(0, 255, 0));
//	// 创建一个实体蓝色画刷
//	HBRUSH hbrush1 = CreateSolidBrush(RGB(0, 0, 255));
//	// 创造一个透明的画刷，如果你想创建其他种类的画刷请参阅MSDN
//	HBRUSH hbrush2 = (HBRUSH)GetStockObject(NULL_BRUSH);
//
//	// 将hpen1和hbrush1选进HDC，并保存HDC原来的画笔和画刷
//	HPEN hpen_old = (HPEN)SelectObject(hdc, hpen1);
//	HBRUSH hbrush_old = (HBRUSH)SelectObject(hdc, hbrush1);
//
//// 	// 在(40,30)处画一个宽200像素，高50像素的矩形
//// 	Rectangle(hdc, 40, 30, 40 + 200, 30 + 50);
//
//	// 换hpen1和hbrush1，然后在(40,100)处也画一个矩形，看看有何差别
//	SelectObject(hdc, hpen2);
//	SelectObject(hdc, hbrush2);
//// 	Rectangle(hdc, 40, 100, 40 + 200, 100 + 50);
//// 
//// 	// 画个椭圆看看
//// 	Ellipse(hdc, 40, 200, 40 + 200, 200 + 50);
//
//	// 画个(0,600)到(800,0)的直线看看
//	MoveToEx(hdc, 0, 600, NULL);
//	LineTo(hdc, 800, 0);
//// 
//// 	// 在(700,500)处画个黄点，不过这个点只有一像素大小，你细细的看才能找到
//// 	SetPixel(hdc, 700, 500, RGB(255, 255, 0));
//
////	// 恢复原来的画笔和画刷
//	SelectObject(hdc, hpen_old);
//	SelectObject(hdc, hbrush_old);
//	int a;
//	std::cin >> a;
//	return 0;
//}