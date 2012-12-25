#include "../sketchinterface.h"
#include <QFileDialog>

void SketchInterface::ConnectCPEditing()
{
	QObject::connect(ui.DefBluePlane, SIGNAL(clicked()), this, SLOT(OnShowDefBluePlane()));
	QObject::connect(ui.DefGreenPlane, SIGNAL(clicked()), this, SLOT(OnShowDefGreenPlane()));
	QObject::connect(ui.DefYellowPlane, SIGNAL(clicked()), this, SLOT(OnShowDefYellowPlane()));
	QObject::connect(ui.DefSphere, SIGNAL(clicked()), this, SLOT(OnShowDefSphere()));
	QObject::connect(ui.DefHandle, SIGNAL(clicked()), this, SLOT(OnShowDefHandle()));
	QObject::connect(ui.DefROI, SIGNAL(clicked()), this, SLOT(OnShowDefROI()));
	QObject::connect(ui.DefAnchor, SIGNAL(clicked()), this, SLOT(OnShowDefAnchor()));
	QObject::connect(ui.DefSetROI, SIGNAL(valueChanged(int)), this, SLOT(OnDefSetROI(int)));
	QObject::connect(ui.DefClearROI, SIGNAL(clicked()), this, SLOT(OnDefClearROI()));
	QObject::connect(ui.DefIter, SIGNAL(valueChanged(int)), this, SLOT(OnDefSetIter(int)));
	QObject::connect(ui.DefLambda, SIGNAL(valueChanged(double)), this, SLOT(OnDefSetLambda(double)));
	QObject::connect(ui.DefDeform, SIGNAL(clicked()), this, SLOT(OnDefDeform()));
	QObject::connect(ui.ExtExtrude, SIGNAL(clicked()), this, SLOT(OnExExtrude()));
}

void SketchInterface::CPEditingInit()
{
	//render ref plane in deformation mode
	ui.DefBluePlane->setChecked(pDoc->GetMeshDeformation().bRenderRefPlane[0]);
	ui.DefGreenPlane->setChecked(pDoc->GetMeshDeformation().bRenderRefPlane[1]);
	ui.DefYellowPlane->setChecked(pDoc->GetMeshDeformation().bRenderRefPlane[2]);
	ui.DefSphere->setChecked(pDoc->GetMeshDeformation().bRenderSphere);
	ui.DefHandle->setChecked(pDoc->GetMeshDeformation().bRenderHandleNb);
	ui.DefROI->setChecked(pDoc->GetMeshDeformation().bRenderROI);
	ui.DefAnchor->setChecked(pDoc->GetMeshDeformation().bRenderAnchor);
	//slider for ROI
	ui.DefSetROI->setRange(0,100);
	ui.DefSetROI->setTickInterval(1);
	ui.DefSetROI->setValue(pDoc->GetMeshDeformation().GetSelectionRange()*100);
	//iteration and lambda value
	int iIterNum=0;
	double dLambda=0;
	pDoc->GetMeshDeformation().GetFlexibleDeformPara(iIterNum,dLambda);
	ui.DefIter->setRange(0,20);
	ui.DefIter->setValue(iIterNum);
	ui.DefLambda->setRange(0,1);
	ui.DefLambda->setSingleStep(0.1);
	ui.DefLambda->setValue(dLambda);
}

void SketchInterface::OnShowDefBluePlane()
{
	pDoc->GetMeshDeformation().bRenderRefPlane[0]=ui.DefBluePlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefGreenPlane()
{
	pDoc->GetMeshDeformation().bRenderRefPlane[1]=ui.DefGreenPlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefYellowPlane()
{
	pDoc->GetMeshDeformation().bRenderRefPlane[2]=ui.DefYellowPlane->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefSphere()
{
	pDoc->GetMeshDeformation().bRenderSphere=ui.DefSphere->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefHandle()
{
	pDoc->GetMeshDeformation().bRenderHandleNb=ui.DefHandle->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefROI()
{
	pDoc->GetMeshDeformation().bRenderROI=ui.DefROI->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnShowDefAnchor()
{
	pDoc->GetMeshDeformation().bRenderAnchor=ui.DefAnchor->isChecked();
	ui.widget->updateGL();
}

void SketchInterface::OnDefSetROI(int iInput)
{
	pDoc->GetMeshDeformation().SetSelectionRange(iInput*0.01,pDoc->GetMesh());
	ui.widget->updateGL();
}

void SketchInterface::OnDefClearROI()
{
	ui.DefSetROI->setValue(0);
	pDoc->GetMeshDeformation().SetSelectionRange(0,pDoc->GetMesh());
	ui.widget->updateGL();
}

void SketchInterface::OnDefSetIter(int iInput)
{
	int iIterNum=0;
	double dLambda=0;
	pDoc->GetMeshDeformation().GetFlexibleDeformPara(iIterNum,dLambda);
	pDoc->GetMeshDeformation().SetFlexibleDeformPara(iInput,dLambda);
}

void SketchInterface::OnDefSetLambda(double dInput)
{
	int iIterNum=0;
	double dLambda=0;
	pDoc->GetMeshDeformation().GetFlexibleDeformPara(iIterNum,dLambda);
	pDoc->GetMeshDeformation().SetFlexibleDeformPara(iIterNum,dInput);
}

void SketchInterface::OnDefDeform()
{
	vector<Point_3> emptyVec;
	pDoc->SetTestPoints(emptyVec);
	pDoc->GetMeshDeformation().SetModifiedPointsPos(pDoc->GetMesh(),pDoc->GetTestPointsRef());
	pDoc->OnModeEditing();
}

void SketchInterface::OnExExtrude()
{
	pDoc->GetMeshExtrusion().ExtrudeMesh(pDoc->GetMesh(),pDoc->GettestvecvecNewEdgeVertexPos());
	pDoc->GettestvecvecNewEdgeVertexPos().clear();
	pDoc->OnModeEditing();
}