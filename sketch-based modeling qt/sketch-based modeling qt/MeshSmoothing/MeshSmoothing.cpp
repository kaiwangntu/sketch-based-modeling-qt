#include "MeshSmoothing.h"
#include "SmoothingAlgorithm.h"
#include "../sketchviewer.h"
#include "../sketchinterface.h"
#include "../PaintingOnMesh.h"
#include "../OBJHandle.h"

CMeshSmoothing::CMeshSmoothing(void)
{
}

CMeshSmoothing::~CMeshSmoothing(void)
{
}

void CMeshSmoothing::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;
	this->CurvePoint2D.clear();
	this->ROIVertices.clear();
	////init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPSmoothing()!=NULL)
	//{
	//	pCP->GetCPSmoothing()->Init();
	//}
}

void CMeshSmoothing::InputCurvePoint2D(QPoint Point2D)
{
	this->CurvePoint2D.push_back(Point2D);
}

void CMeshSmoothing::PaintROIVertices(KW_Mesh& Mesh,GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//convert UCP into opengl coordinate(on Znear) 
	GLdouble  winX, winY, winZ; 
	GLdouble posX, posY, posZ; 

	winX =this->CurvePoint2D.back().x();
	winY = viewport[3] - (float)this->CurvePoint2D.back().y();
	glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	Point_3 CurrentPoint(posX,posY,posZ);

	Facet_handle CurrentFacet;
	CPaintingOnMesh Painting;
	bool bResult=Painting.PaintingPointOnFrontalMesh(Mesh,CurrentPoint,CurrentFacet,modelview);
	if (bResult)
	{
		vector<Vertex_handle> vecVertexToSmooth;
		Halfedge_around_facet_circulator k = CurrentFacet->facet_begin();
		do 
		{
			vector<Vertex_handle>::iterator pVer=find(this->ROIVertices.begin(),this->ROIVertices.end(),
				k->vertex());
			if (pVer==this->ROIVertices.end())
			{
				this->ROIVertices.push_back(k->vertex());
			}
			vecVertexToSmooth.push_back(k->vertex());
		} while(++k != CurrentFacet->facet_begin());
		//smooth
		GeometryAlgorithm::LaplacianSmooth(2,0.5,vecVertexToSmooth);//0.3
//		GeometryAlgorithm::LaplacianSmooth(5,1,vecVertexToSmooth);//0.3
		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
		Mesh.SetRenderInfo(true,true,false,false,false);
//		GeometryAlgorithm::TaubinLambdaMuSmooth(5,0.3,-0.33,vecVertexToSmooth);
	}
	this->CurvePoint2D.clear();
}

void CMeshSmoothing::ClearROI()
{
	this->ROIVertices.clear();
}

void CMeshSmoothing::BilateralSmooth(KW_Mesh& Mesh)
{
	vector<Vertex_handle> vecVertexToSmooth;
	for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)
	{
		vecVertexToSmooth.push_back(i);
	}

	for (int iIter=0;iIter<1;iIter++)
	{
		CSmoothingAlgorithm::BilateralSmooth(Mesh,vecVertexToSmooth);
	}
}

void CMeshSmoothing::Render(bool bSmoothView,GLdouble* modelview,GLdouble* projection,GLint* viewport,GLenum mode)
{
	if (mode==GL_RENDER)
	{
		RenderCurvePoint2D(modelview,projection,viewport);
		RenderROI();
	}
}

void CMeshSmoothing::RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//draw user's stroke
	if (this->CurvePoint2D.empty())//&&!this->Mesh.empty()&&this->bDeformationHandleStrokeDrawing
	{
		return;
	}
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

	for (unsigned int i=0;i<this->CurvePoint2D.size()-1;i++)
	{
		glBegin(GL_LINES);
		glColor3f(1,0,0);
		glVertex2d(this->CurvePoint2D.at(i).x(),this->CurvePoint2D.at(i).y());
		glVertex2d(this->CurvePoint2D.at(i+1).x(),this->CurvePoint2D.at(i+1).y());
		glEnd();
	}

	glLineWidth(1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void CMeshSmoothing::RenderROI()
{
	if (!this->ROIVertices.empty())
	{
		glDisable(GL_LIGHTING);
		for (unsigned int i=0;i<this->ROIVertices.size();i++)
		{
			glPointSize(5);
			glBegin(GL_POINTS);
			glColor3f(1,0,0);
			glVertex3f(this->ROIVertices.at(i)->point().x(),this->ROIVertices.at(i)->point().y(),
				this->ROIVertices.at(i)->point().z());
			glEnd();
			glPointSize(1);
		}
		glEnable(GL_LIGHTING);
	}
}