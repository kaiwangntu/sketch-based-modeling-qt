#include "MeshEditing.h"
#include "sketchviewer.h"
#include "sketchinterface.h"
#include "PaintingOnMesh.h"
//#include "ControlPanel/ControlPanel.h"
#include <QMessageBox>

CMeshEditing::CMeshEditing(void)
{
}

CMeshEditing::~CMeshEditing(void)
{
}

void CMeshEditing::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;

	this->UserInput2DProfile.clear();
}

void CMeshEditing::Input2DProfilePoint(QPoint ProfilePoint)
{
	this->UserInput2DProfile.push_back(ProfilePoint);
}

void CMeshEditing::Convert2DProfileTo3D()
{
	if (this->UserInput2DProfile.empty())
	{
		return;
	}

	//judge the type of curve: open/closed according to the distance of the first and last point
	//get the average length
	double dTotalLen=0;
	for (unsigned int i=0;i<this->UserInput2DProfile.size()-1;i++)
	{
		Point_2 CurrentPoint(this->UserInput2DProfile.at(i).x(),this->UserInput2DProfile.at(i).y());
		Point_2 NextPoint(this->UserInput2DProfile.at(i+1).x(),this->UserInput2DProfile.at(i+1).y());
		dTotalLen=dTotalLen+sqrt(CGAL::squared_distance(CurrentPoint,NextPoint));
	}
	double dAverageLen=dTotalLen/(double)(this->UserInput2DProfile.size()-1);

	//get length between the first and last point
	Point_2 FirstPoint(this->UserInput2DProfile.front().x(),this->UserInput2DProfile.front().y());
	Point_2 FinalPoint(this->UserInput2DProfile.back().x(),this->UserInput2DProfile.back().y());
	double dFinalLen=sqrt(CGAL::squared_distance(FirstPoint,FinalPoint));

	//judge
	if (dFinalLen>dAverageLen*3)//open curve
	{
		SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
		GLdouble* modelview=pView->GetModelview();
		GLdouble* projection=pView->GetProjection();
		GLint* viewport=pView->GetViewport();

		Point_3 StartPoint,EndPoint;
		Facet_handle FhPaintFacet;
		CPaintingOnMesh Painting;
		bool bResultStart=Painting.PaintingScrPointOnFrontalMesh(pDoc->GetMesh(),this->UserInput2DProfile.front(),StartPoint,
			FhPaintFacet,modelview,projection,viewport);
		bool bResultEnd=Painting.PaintingScrPointOnFrontalMesh(pDoc->GetMesh(),this->UserInput2DProfile.back(),EndPoint,
			FhPaintFacet,modelview,projection,viewport);

		if (bResultStart&&bResultEnd)//two points on mesh,deformation
		{
			cout<<"Deformation Handle Curve..."<<endl;
			pDoc->GetMeshDeformation().SetDrawingCurveType(DEFORMATION_HANDLE_CURVE);//handle curve, if more than one point
			pDoc->GetMeshDeformation().SetCurvePoint2D(this->UserInput2DProfile);
			pDoc->GetMeshDeformation().Conver2DCurveTo3D(pDoc->GetMesh());
			pDoc->OnModeDeformation();
		}
		else if (!bResultStart && !bResultEnd)//two points not on mesh,cutting
		{
			cout<<"Cutting Curve..."<<endl;
			//pDoc->GetMeshCutting().SetDrawingCurveType(CUTTING_ACROSS_CURVE);
			//pDoc->GetMeshCutting().SetCurvePoint2D(this->UserInput2DProfile);
			//pDoc->GetMeshCutting().Conver2DCurveTo3D(pDoc->GetMesh());
			//pDoc->OnModeCutting();
		}
		else
		{
			QMessageBox msgBox;
			msgBox.setText("Invalid Input!");
			msgBox.exec();
		}
	}
	else//closed curve
	{
		cout<<"Extrusion Curve..."<<endl;
		//pDoc->GetMeshExtrusion().SetDrawingCurveType(EXTRUSION_CLOSED_CURVE);
		//pDoc->GetMeshExtrusion().SetCurvePoint2D(this->UserInput2DProfile);
		//pDoc->GetMeshExtrusion().Conver2DCurveTo3D(pDoc->GetMesh());
		//pDoc->OnModeExtrusion();
	}
	this->UserInput2DProfile.clear();
}

void CMeshEditing::Render(int mode)
{
	if (mode==GL_SELECT)
	{
	}
	else if (mode==GL_RENDER)
	{
		Render2DProfile();
	}
}

void CMeshEditing::Render2DProfile()
{
	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	//this is important! it makes windows coordinates and opengl coordinates consistent
	gluOrtho2D( 0, viewport[2],viewport[3], 0);	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glLineWidth(3);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	if (!this->UserInput2DProfile.empty())
	{
		for (unsigned int i=0;i<this->UserInput2DProfile.size()-1;i++)
		{
			glBegin(GL_LINES);
			glColor3f(1,0,0);
			glVertex2d(this->UserInput2DProfile.at(i).x(),this->UserInput2DProfile.at(i).y());
			glVertex2d(this->UserInput2DProfile.at(i+1).x(),this->UserInput2DProfile.at(i+1).y());
			glEnd();
		}
	} 

	glLineWidth(1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}