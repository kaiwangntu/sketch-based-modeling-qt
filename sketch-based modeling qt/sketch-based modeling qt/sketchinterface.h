#ifndef SKETCHINTERFACE_H
#define SKETCHINTERFACE_H

#include <QtGui/QMainWindow>
#include "ui_sketchinterface.h"
#include "SketchDoc.h"


class SketchInterface : public QMainWindow
{
	Q_OBJECT

public:
	SketchInterface(QWidget *parent = 0, Qt::WFlags flags = 0);
	~SketchInterface();

	SketchDoc* GetDoc(){return this->pDoc;}

private:
	Ui::SketchInterfaceClass ui;
	
	SketchDoc* pDoc;

};

#endif // SKETCHINTERFACE_H
