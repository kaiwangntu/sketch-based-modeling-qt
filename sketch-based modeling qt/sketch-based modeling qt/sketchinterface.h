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

signals:

public:
	SketchInterface(QWidget *parent = 0, Qt::WFlags flags = 0);
	~SketchInterface();

	SketchDoc* GetDoc(){return this->pDoc;}

	SketchViewer* GetSketchViewer() {return this->ui.widget;}

	//control panels
	void ConnectCPGeneral();
	void CPGeneralInit();


private:
	Ui::SketchInterfaceClass ui;
	
	SketchDoc* pDoc;
	


};

#endif // SKETCHINTERFACE_H
