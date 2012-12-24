#ifndef SKETCHINTERFACE_H
#define SKETCHINTERFACE_H

#include <QtGui/QMainWindow>
#include "ui_sketchinterface.h"
#include "SketchDoc.h"


class SketchInterface : public QMainWindow
{
	Q_OBJECT

public slots:
	//control panels
	//basic panel
	void OnViewSelectMode();
	void OnSketchMode();
	void OnFileNew();
	void OnFileOpen();
	void OnFileSave();
	void OnFileSaveAs();
	void OnSetLightX(int iInput);
	void OnSetLightY(int iInput);
	void OnSetLightZ(int iInput);
	void OnSetMeshClrR(int iInput);
	void OnSetMeshClrG(int iInput);
	void OnSetMeshClrB(int iInput);
	void OnViewWireframe();
	void OnViewSmooth();
	void OnViewHybrid();
	void OnViewPoints();
	void OnShowAxis();
	void OnPreviewMesh();
	void OnSubdivide();
	void OnSnapShot();
	void OnExpSce();
	void OnImpSce();
	//creation panel
	void OnShowCRBluePlane();
	void OnShowCRGreenPlane();
	void OnShowCRRedPlane();
	void OnShowCRCN();
	void OnShowCROnlyUserSketch();
	void OnShowCRSS();
	void OnCRAutoRot();
	void OnCRReadContour();
	void OnCRWriteContour();
	void OnCRAdjustContour();
	void OnCRGenerateMesh();
	//editing panel
	void OnShowDefBluePlane();
	void OnShowDefGreenPlane();
	void OnShowDefYellowPlane();
	void OnShowDefSphere();
	void OnShowDefHandle();
	void OnShowDefROI();
	void OnShowDefAnchor();
	void OnDefSetROI(int iInput);
	void OnDefClearROI();


signals:

public:
	SketchInterface(QWidget *parent = 0, Qt::WFlags flags = 0);
	~SketchInterface();

	SketchDoc* GetDoc(){return this->pDoc;}

	SketchViewer* GetSketchViewer() {return this->ui.widget;}

	//control panels
	void ConnectCPGeneral();
	void CPGeneralInit();
	void ConnectCPCreation();
	void CPCreationInit();
	void ConnectCPEditing();
	void CPEditingInit();


private:
	Ui::SketchInterfaceClass ui;
	
	SketchDoc* pDoc;
	


};

#endif // SKETCHINTERFACE_H
