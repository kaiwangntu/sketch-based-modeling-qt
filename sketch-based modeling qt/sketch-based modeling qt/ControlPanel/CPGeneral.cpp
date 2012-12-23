#include "../sketchinterface.h"

void SketchInterface::ConnectCPGeneral()
{
	QObject::connect(ui.ViewSelectMode, SIGNAL(clicked()), this, SLOT(OnViewSelectMode()));
	QObject::connect(ui.SketchMode, SIGNAL(clicked()), this, SLOT(OnSketchMode()));
}

void SketchInterface::CPGeneralInit()
{
	//view/select or manipu mode
	if (this->pDoc->GetManipMode()==VIEW_SELECTION_MODE)
	{
		this->ui.ViewSelectMode->setChecked(true);
	}
	else if (this->pDoc->GetManipMode()==SKETCH_MODE)
	{
		this->ui.SketchMode->setChecked(true);
	}
}

void SketchInterface::OnViewSelectMode()
{
	this->pDoc->SetManipMode(VIEW_SELECTION_MODE);
}

void SketchInterface::OnSketchMode()
{
	this->pDoc->SetManipMode(SKETCH_MODE);
}