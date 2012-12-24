#include "../sketchinterface.h"
#include <QFileDialog>

void SketchInterface::ConnectCPCreation()
{
	QObject::connect(ui.CRBluePlane, SIGNAL(clicked()), this, SLOT(OnShowCRBluePlane()));
	QObject::connect(ui.CRGreenPlane, SIGNAL(clicked()), this, SLOT(OnShowCRGreenPlane()));
	QObject::connect(ui.CRRedPlane, SIGNAL(clicked()), this, SLOT(OnShowCRRedPlane()));
	QObject::connect(ui.CR_CN, SIGNAL(clicked()), this, SLOT(OnShowCRCN()));
	QObject::connect(ui.CR_US, SIGNAL(clicked()), this, SLOT(OnShowCROnlyUserSketch()));
	QObject::connect(ui.CR_SS, SIGNAL(clicked()), this, SLOT(OnShowCRSS()));
	QObject::connect(ui.CR_AUTOROT, SIGNAL(clicked()), this, SLOT(OnCRAutoRot()));
	QObject::connect(ui.CRReadCtr, SIGNAL(clicked()), this, SLOT(OnCRReadContour()));
	QObject::connect(ui.CRWriteCtr, SIGNAL(clicked()), this, SLOT(OnCRWriteContour()));
	QObject::connect(ui.CRAdjustCtr, SIGNAL(clicked()), this, SLOT(OnCRAdjustContour()));
	QObject::connect(ui.CRGenerateMesh, SIGNAL(clicked()), this, SLOT(OnCRGenerateMesh()));
}

void SketchInterface::CPCreationInit()
{
	//render ref plane in creation mode
	ui.CRBluePlane->setChecked(pDoc->GetMeshCreation().GetRenderRefPlane()[0]);
	ui.CRGreenPlane->setChecked(pDoc->GetMeshCreation().GetRenderRefPlane()[1]);
	ui.CRRedPlane->setChecked(pDoc->GetMeshCreation().GetRenderRefPlane()[2]);
	ui.CR_CN->setChecked(pDoc->GetMeshCreation().GetRenderCN());
	ui.CR_US->setChecked(pDoc->GetMeshCreation().GetRenderOnlyUserSketch());
	if (pDoc->GetMeshCreation().GetCS2Surf()!=NULL)
	{
		ui.CR_SS->setChecked(pDoc->GetMeshCreation().GetCS2Surf()->GetRenderSS());
	}
	ui.CR_AUTOROT->setChecked(pDoc->GetMeshCreation().GetAutoRotState());
}

void SketchInterface::OnShowCRBluePlane()
{
	this->pDoc->GetMeshCreation().GetRenderRefPlane()[0]=ui.CRBluePlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowCRGreenPlane()
{
	this->pDoc->GetMeshCreation().GetRenderRefPlane()[1]=ui.CRGreenPlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowCRRedPlane()
{
	this->pDoc->GetMeshCreation().GetRenderRefPlane()[2]=ui.CRRedPlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowCRCN()
{
	this->pDoc->GetMeshCreation().SetRenderCN(ui.CR_CN->isChecked());
	ui.widget->updateGL();
}

void SketchInterface::OnShowCROnlyUserSketch()
{
	pDoc->GetMeshCreation().SetRenderOnlyUserSketch(ui.CR_US->isChecked());
	ui.widget->updateGL();
}

void SketchInterface::OnShowCRSS()
{
	if (pDoc->GetMeshCreation().GetCS2Surf()!=NULL)
	{
		pDoc->GetMeshCreation().GetCS2Surf()->SetRenderSS(ui.CR_SS->isChecked());
	}
	else//roll back to previous state
	{
		if (ui.CR_SS->isChecked())
		{
			ui.CR_SS->setChecked(false);
		}
		else
		{
			ui.CR_SS->setChecked(true);
		}
	}
	ui.widget->updateGL();
}

void SketchInterface::OnCRAutoRot()
{
	pDoc->GetMeshCreation().SetAutoRotState(ui.CR_AUTOROT->isChecked());
	ui.widget->updateGL();
}

void SketchInterface::OnCRReadContour()
{
	// TODO: Add your control notification handler code here
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"*.contour","Contour File (*.contour)");	
	if(!fileName.isEmpty())
	{
		pDoc->GetMeshCreation().ReadContourFromFile(fileName.toUtf8().constData());
	}
	ui.widget->updateGL();
}

void SketchInterface::OnCRWriteContour()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"*.contour","Contour File (*.contour)");
	if(!fileName.isEmpty())
	{
		pDoc->GetMeshCreation().WriteContourToFile(fileName.toUtf8().constData());
	}
}

void SketchInterface::OnCRAdjustContour()
{
	if (!pDoc->GetMesh().empty())
	{
		return;
	}
	pDoc->GetMeshCreation().AdjustContourView();
	ui.widget->updateGL();
}

void SketchInterface::OnCRGenerateMesh()
{
	//disable the preview checkbox
	pDoc->SetRenderPreMesh(MESH_EXIST_VIEW);

	//do not display the reference planes
	pDoc->GetMeshCreation().GetRenderRefPlane()[0]=false;
	ui.CRBluePlane->setChecked(false);
	pDoc->GetMeshCreation().GetRenderRefPlane()[1]=false;
	ui.CRGreenPlane->setChecked(false);
	pDoc->GetMeshCreation().GetRenderRefPlane()[2]=false;
	ui.CRRedPlane->setChecked(false);
	
	this->update();
}