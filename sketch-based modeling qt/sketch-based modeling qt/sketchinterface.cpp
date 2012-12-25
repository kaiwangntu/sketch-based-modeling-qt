#include "sketchinterface.h"


SketchInterface::SketchInterface(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	//no variables related to the view class is used in the init function of the doc class
	//but some variables related to the doc class is used in the init function of the view class
	//so establish the doc first
	this->pDoc=new SketchDoc(this);

	ui.setupUi(this);
	ui.toolBox->setCurrentWidget(ui.Basics);
	ui.statusBar->showMessage("status bar");

	//connect signals to slots on general control panel
	connect(ui.toolBox, SIGNAL(currentChanged(int)), this, SLOT(OnSwitchPanel(int)));
	ConnectCPGeneral();
	ConnectCPCreation();
	ConnectCPEditing();

	//initialize control panel
	CPGeneralInit();
	CPCreationInit();
	CPEditingInit();
}

SketchInterface::~SketchInterface()
{
	delete this->pDoc;
}

void SketchInterface::OnSwitchPanel(int iIndex)
{
	if (this->ui.toolBox->widget(iIndex)==ui.Creation)
	{
		this->pDoc->OnModeCreation();
	}
	else if (this->ui.toolBox->widget(iIndex)==ui.Edit)
	{
		this->pDoc->OnModeEditing();
	}
	else if (this->ui.toolBox->widget(iIndex)==ui.Test)
	{
		this->pDoc->OnModeTest();
	}
}