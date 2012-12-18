#ifndef SKETCHINTERFACE_H
#define SKETCHINTERFACE_H

#include <QtGui/QMainWindow>
#include "ui_sketchinterface.h"


class SketchInterface : public QMainWindow
{
	Q_OBJECT

public:
	SketchInterface(QWidget *parent = 0, Qt::WFlags flags = 0);
	~SketchInterface();

private:
	Ui::SketchInterfaceClass ui;
};

#endif // SKETCHINTERFACE_H
