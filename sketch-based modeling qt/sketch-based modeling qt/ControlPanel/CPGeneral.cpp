#include "../sketchinterface.h"
#include "../OBJHandle.h"
#include <QFileDialog>

void SketchInterface::ConnectCPGeneral()
{
	QObject::connect(ui.ViewSelectMode, SIGNAL(clicked()), this, SLOT(OnViewSelectMode()));
	QObject::connect(ui.SketchMode, SIGNAL(clicked()), this, SLOT(OnSketchMode()));
	QObject::connect(ui.SketchMode, SIGNAL(clicked()), this, SLOT(OnSketchMode()));
	QObject::connect(ui.FileNew, SIGNAL(clicked()), this, SLOT(OnFileNew()));
	QObject::connect(ui.FileOpen, SIGNAL(clicked()), this, SLOT(OnFileOpen()));
	QObject::connect(ui.FileSave, SIGNAL(clicked()), this, SLOT(OnFileSave()));
	QObject::connect(ui.FileSaveAs, SIGNAL(clicked()), this, SLOT(OnFileSaveAs()));
	QObject::connect(ui.LightX, SIGNAL(valueChanged(int)), this, SLOT(OnSetLightX(int)));
	QObject::connect(ui.LightY, SIGNAL(valueChanged(int)), this, SLOT(OnSetLightY(int)));
	QObject::connect(ui.LightZ, SIGNAL(valueChanged(int)), this, SLOT(OnSetLightZ(int)));
	QObject::connect(ui.MeshClrR,SIGNAL(valueChanged(int)), this, SLOT(OnSetMeshClrR(int)));
	QObject::connect(ui.MeshClrG,SIGNAL(valueChanged(int)), this, SLOT(OnSetMeshClrG(int)));
	QObject::connect(ui.MeshClrB,SIGNAL(valueChanged(int)), this, SLOT(OnSetMeshClrB(int)));
	QObject::connect(ui.ViewWireframe, SIGNAL(clicked()), this, SLOT(OnViewWireframe()));
	QObject::connect(ui.ViewSmooth, SIGNAL(clicked()), this, SLOT(OnViewSmooth()));
	QObject::connect(ui.ViewHybrid, SIGNAL(clicked()), this, SLOT(OnViewHybrid()));
	QObject::connect(ui.ViewPoints, SIGNAL(clicked()), this, SLOT(OnViewPoints()));
	QObject::connect(ui.ShowAxis, SIGNAL(clicked()), this, SLOT(OnShowAxis()));
	QObject::connect(ui.PreviewMesh, SIGNAL(clicked()), this, SLOT(OnPreviewMesh()));
	QObject::connect(ui.Subdivide, SIGNAL(clicked()), this, SLOT(OnSubdivide()));
	QObject::connect(ui.SnapShot, SIGNAL(clicked()), this, SLOT(OnSnapShot()));
	QObject::connect(ui.ExpSce, SIGNAL(clicked()), this, SLOT(OnExpSce()));
	QObject::connect(ui.ImpSce, SIGNAL(clicked()), this, SLOT(OnImpSce()));
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

	//light settings
	float* LightPos=pDoc->GetLightPos();
	ui.LightX->setRange(0,1000);
	ui.LightX->setValue(LightPos[0]*250+500);
	ui.LightY->setRange(0,1000);
	ui.LightY->setValue(LightPos[1]*250+500);
	ui.LightZ->setRange(0,1000);
	ui.LightZ->setValue(LightPos[2]*250+500);
	//mesh color settings
	vector<double> MeshColor=pDoc->GetDefaultColor();
	ui.MeshClrR->setRange(0,1000);
	ui.MeshClrR->setValue(MeshColor[0]*1000);
	ui.MeshClrG->setRange(0,1000);
	ui.MeshClrG->setValue(MeshColor[1]*1000);
	ui.MeshClrB->setRange(0,1000);
	ui.MeshClrB->setValue(MeshColor[2]*1000);
	//set view style
	switch (pDoc->GetViewStyle())
	{
	case WIREFRAME_VIEW:
		ui.ViewWireframe->setChecked(true);
		break;
	case SMOOTH_VIEW: 
		ui.ViewSmooth->setChecked(true);
		break;
	case HYBRID_VIEW: 
		ui.ViewHybrid->setChecked(true);
		break;
	case POINTS_VIEW: 
		ui.ViewPoints->setChecked(true);
		break;
	default:
		break;
	}
	//show axis or not
	if (pDoc->IsAxisOn())
	{
		ui.ShowAxis->setChecked(true);
	}
	else
	{
		ui.ShowAxis->setChecked(false);
	}
	//show previewed mesh or not
	if (pDoc->GetRenderPreMesh()==MESH_PREVIEW)
	{
		ui.PreviewMesh->setChecked(true);
	}
	else if (pDoc->GetRenderPreMesh()==MESH_NOT_PREVIEW)
	{
		ui.PreviewMesh->setChecked(false);
	}

}

void SketchInterface::OnViewSelectMode()
{
	this->pDoc->SetManipMode(VIEW_SELECTION_MODE);
	this->ui.widget->SetViewSelectCursor();
}

void SketchInterface::OnSketchMode()
{
	this->pDoc->SetManipMode(SKETCH_MODE);
	this->ui.widget->SetSketchCursor();
}

void SketchInterface::OnFileNew()
{
	this->pDoc->NewDocument();
	this->ui.widget->Reset();
	CPGeneralInit();
	CPCreationInit();
}

void SketchInterface::OnFileOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"*.obj","Obj File (*.obj)");	
	if(!fileName.isEmpty())
	{
		this->pDoc->OpenDocument(fileName);
	}
}

void SketchInterface::OnFileSave()
{
	if (pDoc->GetOpenedFileName().isEmpty())
	{
		OnFileSaveAs();
		return;
	}
	pDoc->SaveDocument("");
}

void SketchInterface::OnFileSaveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"*.obj","Obj File (*.obj)");
	if (!fileName.isEmpty())
	{
		pDoc->SaveDocument(fileName);
	}
}

void SketchInterface::OnSetLightX(int iInput)
{
	float* LightPos=pDoc->GetLightPos();
	LightPos[0]=(float)(iInput-500)/250.0;
	pDoc->SetLightPos(LightPos);
	ui.widget->updateGL();
}

void SketchInterface::OnSetLightY(int iInput)
{
	float* LightPos=pDoc->GetLightPos();
	LightPos[1]=(float)(iInput-500)/250.0;
	pDoc->SetLightPos(LightPos);
	ui.widget->updateGL();
}

void SketchInterface::OnSetLightZ(int iInput)
{
	float* LightPos=pDoc->GetLightPos();
	LightPos[2]=(float)(iInput-500)/250.0;
	pDoc->SetLightPos(LightPos);
	ui.widget->updateGL();
}

void SketchInterface::OnSetMeshClrR(int iInput)
{
	vector<double> vecNewColor;
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor=pDoc->GetDefaultColor();
	}
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor.at(0)=(float)iInput/1000.0;
		pDoc->SetDefaultColor(vecNewColor);
	}
	ui.widget->updateGL();
}

void SketchInterface::OnSetMeshClrG(int iInput)
{
	vector<double> vecNewColor;
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor=pDoc->GetDefaultColor();
	}
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor.at(1)=(float)iInput/1000.0;
		pDoc->SetDefaultColor(vecNewColor);
	}
	ui.widget->updateGL();

}

void SketchInterface::OnSetMeshClrB(int iInput)
{
	vector<double> vecNewColor;
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor=pDoc->GetDefaultColor();
	}
	if (!pDoc->GetMesh().empty())
	{
		vecNewColor.at(2)=(float)iInput/1000.0;
		pDoc->SetDefaultColor(vecNewColor);
	}
	ui.widget->updateGL();
}

void SketchInterface::OnViewWireframe()
{
	pDoc->SetViewStyle(WIREFRAME_VIEW);
	ui.widget->updateGL();
}

void SketchInterface::OnViewSmooth()
{
	pDoc->SetViewStyle(SMOOTH_VIEW);
	ui.widget->updateGL();
}

void SketchInterface::OnViewHybrid()
{
	pDoc->SetViewStyle(HYBRID_VIEW);
	ui.widget->updateGL();
}

void SketchInterface::OnViewPoints()
{
	pDoc->SetViewStyle(POINTS_VIEW);
	ui.widget->updateGL();
}

void SketchInterface::OnShowAxis()
{
	if (ui.ShowAxis->isChecked())
	{
		pDoc->RenderAxis(true);
	}
	else 
	{
		pDoc->RenderAxis(false);
	}
	ui.widget->updateGL();
}

void SketchInterface::OnPreviewMesh()
{
	if (ui.PreviewMesh->isChecked())
	{
		pDoc->SetRenderPreMesh(MESH_PREVIEW);
	}
	else 
	{
		pDoc->SetRenderPreMesh(MESH_NOT_PREVIEW);
	}
	ui.widget->updateGL();
}

void SketchInterface::OnSubdivide()
{
	if (!pDoc->GetMesh().empty())
	{
		//OBJHandle::LoopSubDivision(pDoc->GetMesh());
		OBJHandle::CCSubDivision(pDoc->GetMesh());
	}
	ui.widget->updateGL();
}

void SketchInterface::OnSnapShot()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"*.bmp","BMP File (*.bmp)");
	if (!fileName.isEmpty())
	{
		ui.widget->saveSceneImage(fileName);
	}
}

void SketchInterface::OnExpSce()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"*.sce","Scene File (*.sce)");
	if (!fileName.isEmpty())
	{
		this->pDoc->ExportScePara(fileName);
	}

}

void SketchInterface::OnImpSce()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"*.sce","Scene File (*.sce)");
	if (!fileName.isEmpty())
	{
		this->pDoc->ImportScePara(fileName);
	}
	//don't forget to update the light position on the control panel
	float* LightPos=pDoc->GetLightPos();
	ui.LightX->setValue(LightPos[0]*250+500);
	ui.LightY->setValue(LightPos[1]*250+500);
	ui.LightZ->setValue(LightPos[2]*250+500);
	this->update();
}