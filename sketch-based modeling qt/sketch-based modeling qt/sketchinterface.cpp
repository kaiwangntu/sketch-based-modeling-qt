#include "sketchinterface.h"


SketchInterface::SketchInterface(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	ui.toolBox->setCurrentWidget(ui.Basics);
	ui.statusBar->showMessage("status bar");
}

SketchInterface::~SketchInterface()
{

}
