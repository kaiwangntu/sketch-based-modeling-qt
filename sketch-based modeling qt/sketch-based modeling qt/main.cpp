#include "sketchinterface.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	SketchInterface w;
	w.show();
	return a.exec();
}
