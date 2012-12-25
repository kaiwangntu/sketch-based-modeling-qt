#include "MeshCutting.h"
#include "../sketchviewer.h"
#include "../sketchinterface.h"
#include "../PaintingOnMesh.h"
#include "../OBJHandle.h"

CMeshCutting::CMeshCutting(void)
{
}

CMeshCutting::~CMeshCutting(void)
{
}

void CMeshCutting::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;
	this->CurvePoint2D.clear();
	this->iDrawingCurveType=NONE_SELECTED;
	this->hCuttingClosedCurveVertex3d.clear();
	this->SidePoint=Point_3(0,0,0);

	this->vecTestPoint.clear();

	//reference plane
	this->Plane_spin=0.0;
	this->dAccumulatedAngleX=0.0;
	//this->Tunel.clear();

	////init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPCutting()!=NULL)
	//{
	//	pCP->GetCPCutting()->Init();
	//}
}

void CMeshCutting::InputCurvePoint2D(QPoint Point2D)
{
	this->CurvePoint2D.push_back(Point2D);
}

void CMeshCutting::SetDrawingCurveType(int iType)
{
	this->iDrawingCurveType=iType;
}

int CMeshCutting::GetDrawingCurveType()
{
	return this->iDrawingCurveType;
}

void CMeshCutting::Conver2DCurveTo3D(KW_Mesh& Mesh)
{
	if (this->CurvePoint2D.empty())
	{
		return;
	}

	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (!this->hCuttingClosedCurveVertex3d.empty())
	{
		this->iDrawingCurveType=CUTTING_BITE_CURVE;
	}
	else
	{
		this->iDrawingCurveType=CUTTING_ACROSS_CURVE;
	}

	if (iDrawingCurveType==CUTTING_ACROSS_CURVE)
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D,10.0);

		//convert UCP into opengl coordinate(on Znear) 
		vector<Point_3> UserCurvePoint;
		for (unsigned int i=0;i<this->CurvePoint2D.size();i++)
		{
			GLdouble  winX, winY, winZ; 
			GLdouble posX, posY, posZ; 

			winX =this->CurvePoint2D.at(i).x();
			winY = viewport[3] - (float)this->CurvePoint2D.at(i).y();
			glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
			gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
			Point_3 CurrentPoint(posX,posY,posZ);
			UserCurvePoint.push_back(CurrentPoint);
		}

		vector<Vertex_handle> CuttingClosedCurveVertex3d;
		CPaintingOnMesh Painting;
		if(Painting.PaintingClosedStrokeOnFrontalAndRearMesh(Mesh,UserCurvePoint,modelview,
			CuttingClosedCurveVertex3d))
		{
			this->hCuttingClosedCurveVertex3d=CuttingClosedCurveVertex3d;
			//get the plane for judging which part to cut
			Point_3 Point0=UserCurvePoint.front();
			Point_3 Point1=this->hCuttingClosedCurveVertex3d.front()->point();
			Point_3 Point2=this->hCuttingClosedCurveVertex3d.back()->point();
			this->SidePlane=Plane_3(Point0,Point1,Point2);
		}
	}
	else if (iDrawingCurveType==CUTTING_BITE_CURVE)
	{
		if (this->hCuttingClosedCurveVertex3d.empty())
		{
			this->CurvePoint2D.clear();
			return;
		}
		//convert UCP into opengl coordinate(on Znear) 
		GLdouble  winX, winY, winZ; 
		GLdouble posX, posY, posZ; 
		winX =this->CurvePoint2D.front().x();
		winY = viewport[3] - (float)this->CurvePoint2D.front().y();
		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
		this->SidePoint=Point_3(posX,posY,posZ);

		CPaintingOnMesh Painting;
		Facet_handle temp;
		if(Painting.PaintingPointOnFrontalMesh(Mesh,this->SidePoint,temp,modelview))
		{
			CutMesh(Mesh);
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			Mesh.SetRenderInfo(true,true,true,true,true);
			this->hCuttingClosedCurveVertex3d.clear();
			//added for qt interface by kw
			pDoc->OnModeEditing();
		}
		else
		{
			QMessageBox msgBox;
			msgBox.setText("Invalid Curve,could not judge which part to cut!");
			msgBox.exec();
		}
	}
	//else if (iDrawingCurveType==3)
	//{
	//	if (this->Tunel.GetTunVer().size()<2)
	//	{
	//		GeometryAlgorithm compute;
	//		compute.ProcessCurverPoint(this->CurvePoint2D);
	//		compute.MakeClosedStroke2d(this->CurvePoint2D);

	//		//convert UCP into opengl coordinate(on Znear) 
	//		vector<Point_3> UserCurvePoint;
	//		for (unsigned int i=0;i<this->CurvePoint2D.size();i++)
	//		{
	//			GLdouble  winX, winY, winZ; 
	//			GLdouble posX, posY, posZ; 

	//			winX =this->CurvePoint2D.at(i).x;
	//			winY = viewport[3] - (float)this->CurvePoint2D.at(i).y;
	//			glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	//			gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	//			Point_3 CurrentPoint(posX,posY,posZ);
	//			UserCurvePoint.push_back(CurrentPoint);
	//		}

	//		vector<Vertex_handle> TunnelCurve3d;
	//		CPaintingOnMesh Painting;
	//		if(Painting.PaintingClosedStrokeOnFrontalMesh(Mesh,UserCurvePoint,modelview,TunnelCurve3d))
	//		{
	//			if (this->Tunel.GetTunVer().size()==0)
	//			{
	//				this->Tunel.InputTunVer(TunnelCurve3d);
	//			}
	//			else if (this->Tunel.GetTunVer().size()==1)//remesh to get the two sketched circles
	//			{
	//				//the result curve is always ccw on screen,so reverse the second curve
	//				//and make it the same direction with the first one
	//				reverse(TunnelCurve3d.begin(),TunnelCurve3d.end());
	//				this->Tunel.InputTunVer(TunnelCurve3d);
	//				this->Tunel.CorrespondTunCircles();
	//				this->Tunel.SetRefPlane(this->BestFittingPlane,this->PlaneBoundaryPoints,
	//					this->RotateXAxisStartPoint,this->RotateXAxisEndPoint);
	//			}
	//			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//			Mesh.SetRenderInfo(true,true,true,true,true);
	//		}
	//	}
	//	else if (this->Tunel.GetTunVer().size()==2)//make tunnel along the sketched curve
	//	{
	//		GeometryAlgorithm compute;
	//		compute.ProcessCurverPoint(this->CurvePoint2D,15.0);

	//		vector<Point_3> vecTempPoints;
	//		CPaintingOnMesh PaintingOnMesh;
	//		int iResult=PaintingOnMesh.PaintingOnBFPlane(this->BestFittingPlane,modelview,projection,viewport,
	//			this->CurvePoint2D,vecTempPoints);
	//		if (iResult)
	//		{
	//			this->Tunel.SetTunnelDirectCurve(vecTempPoints);
	//		}

	//		//dig the hole
	//		this->Tunel.Tunnel(pDoc->GetMesh(),this->vecTestPoint);
	//		this->Tunel.clear();

	//		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//		Mesh.SetRenderInfo(true,true,true,true,true);
	//	}
	//}
	else
	{
		//do nothing
	}

	this->iDrawingCurveType=NONE_SELECTED;
	this->CurvePoint2D.clear();
}
//////////////////////////////////////////////////////////
//void CMeshCutting::SetPlaneSpin(double dStep)
//{
//	this->Plane_spin=dStep;
//}
//
//double CMeshCutting::GetPlaneSpin()
//{
//	return this->Plane_spin;
//}

//void CMeshCutting::RotateTunnelingPlaneX()
//{
	//if (this->Tunel.GetTunVer().size()!=2)
	//{
	//	return;
	//}
	//this->dAccumulatedAngleX=this->dAccumulatedAngleX+this->Plane_spin;
	//if (this->dAccumulatedAngleX>90||this->dAccumulatedAngleX<-90)
	//{
	//	this->dAccumulatedAngleX=this->dAccumulatedAngleX-Plane_spin;
	//	return;
	//}
	//GeometryAlgorithm compute;
	//compute.ComputeRotatedPlane(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,
	//	this->Plane_spin,this->BestFittingPlane);
	////rotate the tunnel direction curve
	//if(!this->Tunel.GetTunnelDirectCurve().empty())
	//{
	//	vector<Point_3> vecTempCurve=this->Tunel.GetTunnelDirectCurve();
	//	compute.ComputeRotatedCurve(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,this->Plane_spin,
	//		vecTempCurve);
	//	this->Tunel.SetTunnelDirectCurve(vecTempCurve);
	//}

	//compute.GetPlaneBoundaryPoints(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,this->BestFittingPlane,
	//	PlaneBoundaryPoints);
//}

//void CMeshCutting::AdjustPlaneBoundary(int iIncrease)
//{
	//if (this->Tunel.GetTunVer().size()!=2)
	//{
	//	return;
	//}
	//GeometryAlgorithm compute;
	//compute.AdjustPlaneBoundary(iIncrease,this->PlaneBoundaryPoints);
//}
////////////////////////////////////////////////////////////
void CMeshCutting::SetCuttingClosedCurveVertex3d(vector<Vertex_handle> hCuttingClosedCurveVertex3dIn)
{
	this->hCuttingClosedCurveVertex3d=hCuttingClosedCurveVertex3dIn;
}

vector<Vertex_handle> CMeshCutting::GetCuttingClosedCurveVertex3d()
{
	return this->hCuttingClosedCurveVertex3d;
}

bool CMeshCutting::CutMesh(KW_Mesh& Mesh)
{
	if (this->hCuttingClosedCurveVertex3d.empty())
	{
		return false;
	}
	vector<Halfedge_handle> hhClosedCurve;
	GetClosedStrokeHH(hhClosedCurve);
	DeleteFacets(Mesh,hhClosedCurve);
	FillHole(Mesh,hhClosedCurve);
//	this->hCuttingClosedCurveVertex3d.clear();

	return true;
}

int CMeshCutting::GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurve)
{
	//judge if hhClosedCurve needs to be reversed
	Halfedge_around_vertex_circulator testEdge=this->hCuttingClosedCurveVertex3d.front()->vertex_begin();
	Vertex_handle testVer1=this->hCuttingClosedCurveVertex3d.at(1);
	do 
	{
		if (testEdge->opposite()->vertex()==testVer1)
		{
			break;
		}
		else
		{
			testEdge++;
		}
	} while(testEdge!=this->hCuttingClosedCurveVertex3d.front()->vertex_begin());
	Point_3 SidePointOnMesh=testEdge->opposite()->next()->vertex()->point();

	if ((this->SidePlane.has_on_positive_side(SidePoint)&&this->SidePlane.has_on_negative_side(SidePointOnMesh))
		||
		(this->SidePlane.has_on_negative_side(SidePoint)&&this->SidePlane.has_on_positive_side(SidePointOnMesh)))
	{
		reverse(this->hCuttingClosedCurveVertex3d.begin(),this->hCuttingClosedCurveVertex3d.end());
	}

	for (unsigned int i=0;i<this->hCuttingClosedCurveVertex3d.size();i++)
	{
		Vertex_handle vhCurrent,vhNext;
		if (i==this->hCuttingClosedCurveVertex3d.size()-1)
		{
			vhCurrent=this->hCuttingClosedCurveVertex3d.back();
			vhNext=this->hCuttingClosedCurveVertex3d.front();
		}
		else
		{
			vhCurrent=this->hCuttingClosedCurveVertex3d.at(i);
			vhNext=this->hCuttingClosedCurveVertex3d.at(i+1);
		}
		Halfedge_around_vertex_circulator Havc=vhCurrent->vertex_begin();
		do 
		{
			if (Havc->opposite()->vertex()==vhNext)
			{
				hhClosedCurve.push_back(Havc->opposite());
				break;
			}
			else
			{
				Havc++;
			}
		} while(Havc!=vhCurrent->vertex_begin());
	}
	assert(hhClosedCurve.size()==this->hCuttingClosedCurveVertex3d.size());
	return hhClosedCurve.size();
}

int CMeshCutting::DeleteFacets(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve)
{
	//triangle incident to hhClosedCurve->opposite(outside the closed curve)
	vector<Facet_handle> fhCurveOutTri;
	//triangle incident to hhClosedCurve(inside the closed curve)
	vector<Facet_handle> fhCurveInTri;
	for (unsigned int i=0;i<hhClosedCurve.size();i++)
	{
		Halfedge_handle hhCurrent=hhClosedCurve.at(i);
		Halfedge_handle hhOpp=hhCurrent->opposite();
		fhCurveInTri.push_back(hhCurrent->facet());
		fhCurveOutTri.push_back(hhOpp->facet());
	}
	assert(fhCurveInTri.size()==fhCurveOutTri.size());

	int test10=Mesh.size_of_facets();
	//delete fhCurveInTri first
	for (unsigned int i=0;i<fhCurveInTri.size();i++)
	{
		Mesh.erase_facet(fhCurveInTri.at(i)->halfedge());
	}
	Mesh.normalize_border();
	int test11=Mesh.size_of_facets();
	assert(fhCurveInTri.size()+test11==test10);

	vector<Facet_handle> fhWithBorderEdge;
	while (GetFacetsWithBorderEdge(Mesh,fhWithBorderEdge,fhCurveOutTri))
	{
		for (unsigned int i=0;i<fhWithBorderEdge.size();i++)
		{
			Mesh.erase_facet(fhWithBorderEdge.at(i)->halfedge());
		}
		fhWithBorderEdge.clear();
		Mesh.normalize_border();
	}
	return 0;
}

int CMeshCutting::GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,
											vector<Facet_handle> ExemptionTri)
{
	for (Facet_iterator FIter=Mesh.facets_begin();FIter!=Mesh.facets_end();FIter++)
	{
		Halfedge_around_facet_circulator Hafc=FIter->facet_begin();
		do 
		{
			if (Hafc->is_border_edge())
			{
				vector<Facet_handle>::iterator FhIter=find(ExemptionTri.begin(),ExemptionTri.end(),FIter);
				if (FhIter==ExemptionTri.end())
				{
					fhWithBorderEdge.push_back(FIter);
				}
				break;
			}
			else
			{
				Hafc++;
			}
		} while(Hafc!=FIter->facet_begin());
	}
	int iResult=fhWithBorderEdge.size();
	return iResult;
}

int CMeshCutting::FillHole(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve)
{
	//fill the hole of hhClosedCurve 
	Mesh.fill_hole(hhClosedCurve.front());
	Mesh.normalize_border();
	int test00=Mesh.size_of_border_edges();
	assert(test00==0);

	//create a center vertex
	vector<Point_3> EdgePoints;
	for (unsigned int i=0;i<this->hCuttingClosedCurveVertex3d.size();i++)
	{
		EdgePoints.push_back(this->hCuttingClosedCurveVertex3d.at(i)->point());
	}
	Point_3 CentroidPoint=CGAL::centroid(EdgePoints.begin(),EdgePoints.end());

	Halfedge_handle hhToCenterPoint=Mesh.create_center_vertex(hhClosedCurve.front());
	hhToCenterPoint->vertex()->point()=CentroidPoint;

	//Halfedge_handle hhStart=hhClosedCurve.front();
	//while (true)
	//{
	//	int iFacetDegree=hhStart->facet()->facet_degree();
	//	if (iFacetDegree==3)
	//	{
	//		break;
	//	}
	//	Halfedge_handle hhNewStart;
	//	if (iFacetDegree%2==0)
	//	{
	//		hhNewStart=Mesh.split_facet(hhStart->next(),hhStart->prev());
	//	} 
	//	else
	//	{
	//		hhNewStart=Mesh.split_facet(hhStart,hhStart->prev()->prev());
	//	}
	//	hhStart=hhNewStart->opposite();
	//}

	Mesh.normalize_border();
	assert(Mesh.size_of_border_edges()==0);

	return 0;
}

void CMeshCutting::Render(bool bSmoothView,GLenum mode)
{
	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (mode==GL_RENDER)
	{
		RenderCurvePoint2D(modelview,projection,viewport);
		RenderCuttingCurve3D();
		RenderTunelCurve3D();
		RenderTunnelDirectCurve();
		RenderRefPlane(bSmoothView);
		RenderTestPoint();
	}
	else if (mode==GL_SELECT)
	{
	}
}

void CMeshCutting::RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport)
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

void CMeshCutting::RenderCuttingCurve3D()
{
	if (this->hCuttingClosedCurveVertex3d.empty())
	{
		return;
	}
	glLineWidth(6.0);
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->hCuttingClosedCurveVertex3d.size();i++)
	{
		glBegin(GL_LINES);
		glColor3f(0,0,1);
		{
			if (i!=this->hCuttingClosedCurveVertex3d.size()-1)
			{
				glVertex3d(this->hCuttingClosedCurveVertex3d.at(i)->point().x(),
					this->hCuttingClosedCurveVertex3d.at(i)->point().y(),
					this->hCuttingClosedCurveVertex3d.at(i)->point().z());
				glVertex3d(this->hCuttingClosedCurveVertex3d.at(i+1)->point().x(),
					this->hCuttingClosedCurveVertex3d.at(i+1)->point().y(),
					this->hCuttingClosedCurveVertex3d.at(i+1)->point().z());
			} 
			else
			{
				glVertex3d(this->hCuttingClosedCurveVertex3d.back()->point().x(),
					this->hCuttingClosedCurveVertex3d.back()->point().y(),
					this->hCuttingClosedCurveVertex3d.back()->point().z());
				glVertex3d(this->hCuttingClosedCurveVertex3d.front()->point().x(),
					this->hCuttingClosedCurveVertex3d.front()->point().y(),
					this->hCuttingClosedCurveVertex3d.front()->point().z());
			}
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
	glLineWidth(1.0);		
}

void CMeshCutting::RenderTunelCurve3D()
{
	//if (this->Tunel.GetTunVer().empty())
	//{
	//	return;
	//}
	//glLineWidth(6.0);
	//glDisable(GL_LIGHTING);
	//for (unsigned int i=0;i<this->Tunel.GetTunVer().size();i++)
	//{
	//	int iCurveLen=Tunel.GetTunVer().at(i).size();
	//	for (int j=0;j<iCurveLen;j++)
	//	{
	//		glBegin(GL_LINES);
	//		glColor3f(0,0,1);
	//		//((j+iCurveLen)-1)%iCurveLen is the previous one of j
	//		glVertex3d(this->Tunel.GetTunVer().at(i).at(j)->point().x(),
	//			this->Tunel.GetTunVer().at(i).at(j)->point().y(),
	//			this->Tunel.GetTunVer().at(i).at(j)->point().z());
	//		glVertex3d(this->Tunel.GetTunVer().at(i).at(((j+iCurveLen)-1)%iCurveLen)->point().x(),
	//			this->Tunel.GetTunVer().at(i).at(((j+iCurveLen)-1)%iCurveLen)->point().y(),
	//			this->Tunel.GetTunVer().at(i).at(((j+iCurveLen)-1)%iCurveLen)->point().z());
	//		glEnd();
	//	}
	//}
	//glEnable(GL_LIGHTING);
	//glLineWidth(1.0);		
}

void CMeshCutting::RenderTunnelDirectCurve()
{
//	if (this->Tunel.GetTunnelDirectCurve().empty())
//	{
//		return;
//	}
//	glLineWidth(6.0);
//	glDisable(GL_LIGHTING);
//
//	glClear(GL_DEPTH_BUFFER_BIT);
////	glDisable(GL_DEPTH_TEST);
//	glDepthMask(FALSE);
//
//	glPointSize(5);
//	glBegin(GL_POINTS);
//	glColor3f(1,0,0);
//	glVertex3f(this->Tunel.GetTunnelDirectCurve().front().x(),this->Tunel.GetTunnelDirectCurve().front().y(),
//		this->Tunel.GetTunnelDirectCurve().front().z());
//	glEnd();
//	glPointSize(1);
//
//	for (unsigned int i=0;i<this->Tunel.GetTunnelDirectCurve().size()-1;i++)
//	{
//		glBegin(GL_LINES);
//		glColor3f(0,1,0);
//		glVertex3d(this->Tunel.GetTunnelDirectCurve().at(i).x(),this->Tunel.GetTunnelDirectCurve().at(i).y(),
//			this->Tunel.GetTunnelDirectCurve().at(i).z());
//		glVertex3d(this->Tunel.GetTunnelDirectCurve().at(i+1).x(),this->Tunel.GetTunnelDirectCurve().at(i+1).y(),
//			this->Tunel.GetTunnelDirectCurve().at(i+1).z());
//		glEnd();
//
//	}
//
//	glDepthMask(TRUE);
////	glEnable(GL_DEPTH_TEST);
//
//	glEnable(GL_LIGHTING);
//	glLineWidth(1.0);		
}

void CMeshCutting::RenderRefPlane(bool bSmoothView)
{
//	if (this->Tunel.GetTunVer().size()!=2)
//	{
//		return;
//	}
//	glLineWidth(2);
//
//	glClear(GL_DEPTH_BUFFER_BIT);
////	glDisable(GL_DEPTH_TEST);
//	glDepthMask(FALSE);
//
//	glDisable(GL_LIGHTING);
//	//draw the frame of the plane
//	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
//	glColor3f(0,0,1);
//	glBegin(GL_QUADS);
//	glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
//	glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
//	glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
//	glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);
//	glEnd();
//
//	glEnable(GL_LIGHTING);
//
//	//draw the transparent face of the plane(2 faces)
//	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
//
//	const  GLfloat blue_color[] = {0.0f, 0.0f, 1.0f, 0.2f};
//	const GLfloat mat_blue_emission[] = {0.0f, 0.0f, 1.0f, 1.0f};
//	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , blue_color);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_blue_emission);
//
//
//	//	glColor4f(0.0f,0.0f,1.0f,0.2f);			// Full Brightness, 50% Alpha
//
//	glBegin(GL_QUADS);
//	//				glColor3f(0,1,0);
//	glNormal3f(this->BestFittingPlane.orthogonal_vector().x(),
//		this->BestFittingPlane.orthogonal_vector().y(),
//		this->BestFittingPlane.orthogonal_vector().z());
//	glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
//	glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
//	glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
//	glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);
//
//	glNormal3f(this->BestFittingPlane.opposite().orthogonal_vector().x(),
//		this->BestFittingPlane.opposite().orthogonal_vector().y(),
//		this->BestFittingPlane.opposite().orthogonal_vector().z());
//	glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);
//	glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
//	glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
//	glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
//	glEnd();
//
//	glDepthMask(TRUE);
////	glEnable(GL_DEPTH_TEST);
//
//	if (bSmoothView)
//	{
//		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
//	} 
//	else
//	{
//		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
//	}
//	glLineWidth(1);
}

void CMeshCutting::RenderTestPoint()
{
	if (this->vecTestPoint.empty())
	{
		return;
	}
	float fStep=1.0/this->vecTestPoint.size();
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->vecTestPoint.size();i++)
	{
		glPointSize(5);
		glBegin(GL_POINTS);
		//glColor3f(0,0,1);
		glColor3f(0,0+(i+1)*fStep,0+(i+1)*fStep);
		glVertex3f(this->vecTestPoint.at(i).x(),this->vecTestPoint.at(i).y(),
			this->vecTestPoint.at(i).z());
		glEnd();
		glPointSize(1);
	}
	glEnable(GL_LIGHTING);
}