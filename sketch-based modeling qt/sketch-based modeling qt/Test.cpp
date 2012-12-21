#include "Test.h"
#include "GeometryAlgorithm.h"
//#include "ControlPanel/ControlPanel.h"

CTest::CTest(void)
{
}

CTest::~CTest(void)
{
}

void CTest::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;
	this->UserInput2DProfile.clear();
	////init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPTest()!=NULL)
	//{
	//	pCP->GetCPTest()->Init();
	//}
}


void CTest::InputCurvePoint2D(QPoint ProfilePoint)
{
	this->UserInput2DProfile.push_back(ProfilePoint);
}

void CTest::Conver2DCurveTo3D(GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	GeometryAlgorithm compute;
	//	compute.ProcessCurverPoint(this->UserInput2DProfile,15.0,SAMPLE_POINT_NUM);//20
	compute.ProcessCurverPoint(this->UserInput2DProfile,10.0,10);//20
	//	compute.MakeClosedStroke2d(this->UserInput2DProfile);
}

void CTest::LineFilter()
{
	GeometryAlgorithm compute;
	compute.FilterCurvePoint(this->UserInput2DProfile,10.0);
}

void CTest::LineResample()
{
	GeometryAlgorithm compute;
	compute.ResampleCurvePoint(this->UserInput2DProfile,10);
}

void CTest::LineSmooth()
{
	GeometryAlgorithm compute;
	compute.SmoothCurvePoint(this->UserInput2DProfile);
}

void CTest::Render(bool bSmoothView,GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	Render2DProfile(modelview,projection,viewport);
}

void CTest::Render2DProfile(GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	//this is important! it makes windows coordinates and opengl coordinates consistent
	gluOrtho2D( 0, viewport[2],viewport[3], 0);	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glLineWidth(2);
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

			glPointSize(4.0);
			glBegin(GL_POINTS);
			glColor3f(0,0,1);
			glVertex2d(this->UserInput2DProfile.at(i).x(),this->UserInput2DProfile.at(i).y());
			glVertex2d(this->UserInput2DProfile.at(i+1).x(),this->UserInput2DProfile.at(i+1).y());
			glEnd();
			glPointSize(1.0);
		}
	} 

	glLineWidth(1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();



	//glLineWidth(2.0);
	//glDisable(GL_LIGHTING);
	////draw cutting stroke
	//if (!this->UserInput2DProfile.empty())
	//{
	//	for (unsigned int i=0;i<this->UserInput2DProfile.size()-1;i++)
	//	{
	//		GLdouble  winX, winY, winZ; 
	//		GLdouble posX, posY, posZ; 

	//		winX =this->UserInput2DProfile.at(i).x;
	//		winY = viewport[3] - (float)this->UserInput2DProfile.at(i).y;
	//		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	//		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	//		Point_3 CurrentPoint(posX,posY,posZ);

	//		winX =this->UserInput2DProfile.at(i+1).x;
	//		winY = viewport[3] - (float)this->UserInput2DProfile.at(i+1).y;
	//		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	//		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	//		Point_3 NextPoint(posX,posY,posZ);

	//		glBegin(GL_LINES);
	//		glColor3f(1,0,0);
	//		glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
	//		glVertex3d(NextPoint.x(),NextPoint.y(),NextPoint.z());
	//		glEnd();

	//		glPointSize(4.0);
	//		glBegin(GL_POINTS);
	//		glColor3f(0,0,1);
	//		glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
	//		glVertex3d(NextPoint.x(),NextPoint.y(),NextPoint.z());
	//		glEnd();
	//		glPointSize(1.0);
	//	}
	//}
	//glEnable(GL_LIGHTING);
	//glLineWidth(1.0);
}