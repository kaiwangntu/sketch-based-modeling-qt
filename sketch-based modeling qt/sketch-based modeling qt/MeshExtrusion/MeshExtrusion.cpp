#include "MeshExtrusion.h"
#include "../sketchviewer.h"
#include "../sketchinterface.h"
#include "../PaintingOnMesh.h"
#include "../OBJHandle.h"

CMeshExtrusion::CMeshExtrusion(void)
{
}

CMeshExtrusion::~CMeshExtrusion(void)
{
}

void CMeshExtrusion::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;
	this->CurvePoint2D.clear();
	this->iDrawingCurveType=NONE_SELECTED;
	this->Plane_spin=0.0;
	this->hExtrusionClosedCurveVertex3d.clear();
	this->ExtrusionSilhPoints.clear();
	this->dAccumulatedAngleX=0.0;
	this->vecvecExtrudedVertex.clear();
	this->ClosedCurveProj.clear();

	this->Tunel.clear();

	this->vecTestPoints.clear();
	////init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPExtrusion()!=NULL)
	//{
	//	pCP->GetCPExtrusion()->Init();
	//}
}

void CMeshExtrusion::InputCurvePoint2D(QPoint Point2D)
{
	this->CurvePoint2D.push_back(Point2D);
}

void CMeshExtrusion::SetDrawingCurveType(int iType)
{
	this->iDrawingCurveType=iType;
}

int CMeshExtrusion::GetDrawingCurveType()
{
	return this->iDrawingCurveType;
}

void CMeshExtrusion::SetSelectedItem()
{
	int iSelectedName=this->pDoc->GetRBSelName();
	if (iSelectedName==EXTRUSION_REF_PLANE)
	{
		this->iDrawingCurveType=EXTRUSION_SILH_CURVE;
	}
	else
	{
		this->iDrawingCurveType=NONE_SELECTED;
	}
}

void CMeshExtrusion::ManipSelItem(int g_iStepX, int g_iStepY)
{
	if (this->pDoc->GetRBSelName()!=this->pDoc->GetLBSelName())
	{
		return;
	}
	int iSelectedName=this->pDoc->GetRBSelName();

	if (iSelectedName==EXTRUSION_REF_PLANE)
	{
		if (abs(g_iStepX)>=10.0*abs(g_iStepY))
		{
			this->Plane_spin=g_iStepX;
			if (this->Plane_spin> 90.0 || this->Plane_spin< -90.0)
			{
				this->Plane_spin=0.0;
			}
			RotateExtrudeSilhPlaneY();
		}
		else if (abs(g_iStepY)>=10.0*abs(g_iStepX))
		{
			this->Plane_spin=g_iStepY;
			if (this->Plane_spin> 90.0 || this->Plane_spin< -90.0)
			{
				this->Plane_spin=0.0;
			}
			RotateExtrudeSilhPlaneX();
		}
	}
}

void CMeshExtrusion::Conver2DCurveTo3D(KW_Mesh& Mesh)
{
	if (this->CurvePoint2D.empty())
	{
		return;
	}

	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (this->iDrawingCurveType==EXTRUSION_CLOSED_CURVE)//closed curve
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D);
		compute.MakeClosedStroke2d(this->CurvePoint2D);

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

		vector<Vertex_handle> ExtrusionClosedCurveVertex3d;
		CPaintingOnMesh Painting;
		if(Painting.PaintingClosedStrokeOnFrontalMesh(Mesh,UserCurvePoint,modelview,
			ExtrusionClosedCurveVertex3d))
		{
			this->hExtrusionClosedCurveVertex3d=ExtrusionClosedCurveVertex3d;
			GetExtrudeSilhPlane(this->BestFittingPlane,this->PlaneBoundaryPoints,
				modelview,projection,viewport);
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			Mesh.SetRenderInfo(true,true,true,true,true);
		}

	}
	else if(this->iDrawingCurveType==EXTRUSION_SILH_CURVE)//silhouette curve
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D,20.0);

		if (!this->hExtrusionClosedCurveVertex3d.empty())
		{
			vector<Point_3> ExtrusionSilhPoints;
			CPaintingOnMesh PaintingOnMesh;
			int iResult=PaintingOnMesh.PaintingOnBFPlane(this->BestFittingPlane,modelview,projection,viewport,
				this->CurvePoint2D,ExtrusionSilhPoints);
			if (iResult)
			{
				this->ExtrusionSilhPoints=ExtrusionSilhPoints;
			}
		}
	}
	else
	{
		//do nothing
	}
	//this->iDrawingCurveType=NONE_SELECTED;
	this->CurvePoint2D.clear();
}

void CMeshExtrusion::GetExtrudeSilhPlane(Plane_3& BestFittingPlane,Point3D* PlaneBoundaryPoints,
										 GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//compute the best-fitting plane of the closed curve
	vector<Point_3> ClosedCurvePoints;
	Plane_3 ClosedCurvePlane;
	vector<Point_2> ClosedCurveScreenPoints;
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		Point_3 CurrentPoint=this->hExtrusionClosedCurveVertex3d.at(i)->point();
		ClosedCurvePoints.push_back(CurrentPoint);

		//compute its projection point on screen
		GLdouble  winX, winY, winZ; 
		gluProject(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z(), modelview, projection, viewport, 
					&winX, &winY, &winZ); 
		Point_2 ClosedCurveScreenPoint(winX,winY);
		ClosedCurveScreenPoints.push_back(ClosedCurveScreenPoint);
	}
	Point_3 ClosedCurveCentroidPoint;
	double result=linear_least_squares_fitting_3(ClosedCurvePoints.begin(),ClosedCurvePoints.end(),
		ClosedCurvePlane,ClosedCurveCentroidPoint,CGAL::Dimension_tag<0>());

	//find a point as the RotateAxisStartPoint
	//try to make the line between RotateAxisStartPoint and RotateAxisEndPoint
	//(i.e. the line between RotateAxisStartPoint and Centroid Point Projection)
	//parallel to the X axis of the screen
	GLdouble  winX, winY, winZ; 
	gluProject(ClosedCurveCentroidPoint.x(),ClosedCurveCentroidPoint.y(),ClosedCurveCentroidPoint.z(),
		modelview, projection, viewport, &winX, &winY, &winZ); 
	Point_2 ClosedCurveScreenCentroidPoint(winX,winY);
	Vector_2 VecTo=Point_2(0,winY)-ClosedCurveScreenCentroidPoint;
	int iBest=0;
	double dMinAngle=180;
	GeometryAlgorithm compute;
	for (unsigned int i=0;i<ClosedCurveScreenPoints.size();i++)
	{
		Vector_2 VecFrom=ClosedCurveScreenPoints.at(i)-ClosedCurveScreenCentroidPoint;
		double dAngle=compute.GetAngleBetweenTwoVectors2d(VecFrom,VecTo);
		if (dAngle<dMinAngle)
		{
			dMinAngle=dAngle;
			iBest=i;
		}
	}

	//compute the plane which the extrusion silhoette curve lies on
	//it is perpendicular to the above plane
	Point_3 StartPointProj=ClosedCurvePlane.projection(ClosedCurvePoints.at(iBest));
	Point_3 CentroidPointProj=ClosedCurvePlane.projection(ClosedCurveCentroidPoint);
	Line_3  PerpLineThroughCPProj=ClosedCurvePlane.perpendicular_line(CentroidPointProj);
	BestFittingPlane=Plane_3(PerpLineThroughCPProj,StartPointProj);

	//the axis perpendicular to ClosedCurvePlane and passing through CentroidPointProj
	this->RotateYAxisStartPoint=CentroidPointProj;
	this->RotateYAxisEndPoint=this->RotateYAxisStartPoint+ClosedCurvePlane.orthogonal_vector();

	//compute the two boundary points of the plane
	vector<Point_3> PointsProj,InterSectionPoints;
	for (unsigned int i=0;i<ClosedCurvePoints.size();i++)
	{
		PointsProj.push_back(ClosedCurvePlane.projection(ClosedCurvePoints.at(i)));
	}
	this->ClosedCurveProj=PointsProj;

	for (unsigned int i=0;i<PointsProj.size();i++)
	{
		Point_3 SegStartPoint,SegEndPoint;
		if (i==PointsProj.size()-1)
		{
			SegStartPoint=PointsProj.back();
			SegEndPoint=PointsProj.front();
		}
		else
		{
			SegStartPoint=PointsProj.at(i);
			SegEndPoint=PointsProj.at(i+1);
		}

		Segment_3 Seg(SegStartPoint,SegEndPoint);
		CGAL::Object result = CGAL::intersection(BestFittingPlane,Seg);
		Point_3 CurrentIP;
		if (CGAL::assign(CurrentIP, result)) 
		{
			InterSectionPoints.push_back(CurrentIP);
		}
	}
	assert(InterSectionPoints.size()>1);

	double dMinDistance=9999.0;
	double dMaxDistance=0.0;
	for (unsigned int i=0;i<InterSectionPoints.size();i++)
	{
		double dDistance=CGAL::squared_distance(StartPointProj,InterSectionPoints.at(i));
		if (dDistance<=dMinDistance)
		{
			this->RotateXAxisStartPoint=InterSectionPoints.at(i);
			dMinDistance=dDistance;
		}
		if (dDistance>=dMaxDistance)
		{
			this->RotateXAxisEndPoint=InterSectionPoints.at(i);
			dMaxDistance=dDistance;
		}
	}
	assert(this->RotateXAxisStartPoint!=this->RotateXAxisEndPoint);
	
	compute.GetPlaneBoundaryPoints(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,BestFittingPlane,
		PlaneBoundaryPoints);
}

void CMeshExtrusion::RotateExtrudeSilhPlaneX()
{
	this->dAccumulatedAngleX=this->dAccumulatedAngleX+this->Plane_spin;
	if (this->dAccumulatedAngleX>90||this->dAccumulatedAngleX<-90)
	{
		this->dAccumulatedAngleX=this->dAccumulatedAngleX-Plane_spin;
		return;
	}
	GeometryAlgorithm compute;
	compute.ComputeRotatedPlane(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,
		this->Plane_spin,this->BestFittingPlane);
	if(!this->ExtrusionSilhPoints.empty())
	{
		compute.ComputeRotatedCurve(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,this->Plane_spin,
			this->ExtrusionSilhPoints);
	}

	compute.GetPlaneBoundaryPoints(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,this->BestFittingPlane,
		PlaneBoundaryPoints);
}

void CMeshExtrusion::RotateExtrudeSilhPlaneY()
{
	//rotate the current plane back to make it perpendicular to the closed curve
	GeometryAlgorithm compute;
	if (this->dAccumulatedAngleX!=0)
	{
		compute.ComputeRotatedPlane(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,
									-this->dAccumulatedAngleX,BestFittingPlane);
		if(!this->ExtrusionSilhPoints.empty())
		{
			compute.ComputeRotatedCurve(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,
									-this->dAccumulatedAngleX,this->ExtrusionSilhPoints);
		}
		this->dAccumulatedAngleX=0;
	}

	//rotate the plane and curve around Y axis
	compute.ComputeRotatedPlane(this->RotateYAxisStartPoint,this->RotateYAxisEndPoint,
								Plane_spin,BestFittingPlane);
	if(!this->ExtrusionSilhPoints.empty())
	{
		compute.ComputeRotatedCurve(this->RotateYAxisStartPoint,this->RotateYAxisEndPoint,
								Plane_spin,this->ExtrusionSilhPoints);
	}

	//find the new RotateXAxisStartPoint and RotateXAxisEndPoint
	vector<Point_3> InterSectionPoints;
	for (unsigned int i=0;i<this->ClosedCurveProj.size();i++)
	{
		Point_3 SegStartPoint,SegEndPoint;
		if (i==this->ClosedCurveProj.size()-1)
		{
			SegStartPoint=this->ClosedCurveProj.back();
			SegEndPoint=this->ClosedCurveProj.front();
		}
		else
		{
			SegStartPoint=this->ClosedCurveProj.at(i);
			SegEndPoint=this->ClosedCurveProj.at(i+1);
		}

		Segment_3 Seg(SegStartPoint,SegEndPoint);
		CGAL::Object result = CGAL::intersection(BestFittingPlane,Seg);
		Point_3 CurrentIP;
		if (CGAL::assign(CurrentIP, result)) 
		{
			InterSectionPoints.push_back(CurrentIP);
		}
	}
	assert(InterSectionPoints.size()>1);

	Point_3 StartPointProj=this->RotateXAxisStartPoint;
	double dMinDistance=9999.0;
	for (unsigned int i=0;i<InterSectionPoints.size();i++)
	{
		double dDistance=CGAL::squared_distance(StartPointProj,InterSectionPoints.at(i));
		if (dDistance<=dMinDistance)
		{
			this->RotateXAxisStartPoint=InterSectionPoints.at(i);
			dMinDistance=dDistance;
		}
	}
	StartPointProj=this->RotateXAxisStartPoint;
	double dMaxDistance=0.0;
	for (unsigned int i=0;i<InterSectionPoints.size();i++)
	{
		double dDistance=CGAL::squared_distance(StartPointProj,InterSectionPoints.at(i));
		if (dDistance>=dMaxDistance)
		{
			this->RotateXAxisEndPoint=InterSectionPoints.at(i);
			dMaxDistance=dDistance;
		}
	}
	assert(this->RotateXAxisStartPoint!=this->RotateXAxisEndPoint);

	compute.GetPlaneBoundaryPoints(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,BestFittingPlane,
		PlaneBoundaryPoints);
}

void CMeshExtrusion::AdjustPlaneBoundary(int iIncrease)
{
	if (this->hExtrusionClosedCurveVertex3d.empty())
	{
		return;
	}
	GeometryAlgorithm compute;
	compute.AdjustPlaneBoundary(iIncrease,this->PlaneBoundaryPoints);
}

bool CMeshExtrusion::ExtrudeMesh(KW_Mesh& Mesh,vector<vector<Point_3> >& testvecvecNewEdgeVertexPos)
{
	if (this->hExtrusionClosedCurveVertex3d.empty()||this->ExtrusionSilhPoints.empty())
	{
		return false;
	}
	
	//judge if extrusion or tunneling
	//if intersect,tunneling
	if (GeometryAlgorithm::JudgeMeshOpenCurveIntersec(Mesh,this->ExtrusionSilhPoints))
	{
		this->Tunel.InputTunVer(this->hExtrusionClosedCurveVertex3d);
		vector<Point_3> SkeletonCurve;
		int iTunnelDirectPointNum=99;
		this->Tunel.ExtractSkelFromSilhCurve(this->ExtrusionSilhPoints,iTunnelDirectPointNum,SkeletonCurve);
		//make sure the extracted skeleton "inside" the mesh
		while(GeometryAlgorithm::JudgeMeshOpenCurveIntersec(Mesh,SkeletonCurve))
		{
			SkeletonCurve.pop_back();
		}
		while(SkeletonCurve.size()>=6)
		{
			SkeletonCurve.erase(SkeletonCurve.begin());
			SkeletonCurve.pop_back();
		}
		Vector_3 CurveDirection(SkeletonCurve.front(),SkeletonCurve.back());

		vector<Vertex_handle> hRearCurveVertex3d;
		CPaintingOnMesh Painting;
		if (Painting.PaintingFrontalCurveOnRearMesh(Mesh,this->hExtrusionClosedCurveVertex3d,CurveDirection,hRearCurveVertex3d,this->vecTestPoints))
		{
			this->Tunel.InputTunVer(hRearCurveVertex3d);
			this->Tunel.CorrespondTunCircles();
			this->Tunel.SetTunnelDirectCurve(SkeletonCurve);
			this->Tunel.Tunnel(Mesh,this->vecTestPoints);

			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			Mesh.SetRenderInfo(true,true,true,true,true);

			this->vecTestPoints.clear();

			this->Tunel.clear();
			this->hExtrusionClosedCurveVertex3d.clear();
			this->ExtrusionSilhPoints.clear();
			this->ClosedCurveProj.clear();
		}
		this->iDrawingCurveType=NONE_SELECTED;
		return true;
	}

	vector<Halfedge_handle> hhClosedCurve;
	GetClosedStrokeHH(hhClosedCurve);
	DeleteFacetsInClosedCurve(Mesh,hhClosedCurve);
	testvecvecNewEdgeVertexPos.clear();

	vector<Facet_handle> fhExtrudedFacet;
	ExtrudeClosedCurve(Mesh,hhClosedCurve,this->BestFittingPlane,testvecvecNewEdgeVertexPos,fhExtrudedFacet);

	//local refine
	//GeometryAlgorithm::ExtrusionLocalRefine(Mesh,fhExtrudedFacet);
	vector<Vertex_handle> vecVertexToSmooth;
	
	//put only sewing vertices and their neighbors in
	//GetVerticesToSmooth(vecVertexToSmooth);
	//put all extruded vertices in
	for (unsigned int i=0;i<this->vecvecExtrudedVertex.size();i++)
	{
		vecVertexToSmooth.insert(vecVertexToSmooth.end(),this->vecvecExtrudedVertex.at(i).begin(),
			this->vecvecExtrudedVertex.at(i).end());
	}


	GeometryAlgorithm::LaplacianSmooth(10,0.1,vecVertexToSmooth);
//	GeometryAlgorithm::TaubinLambdaMuSmooth(10,0.5,-0.53,vecVertexToSmooth);

	//this->vecTestPoints.clear();
	//for (unsigned int i=0;i<vecVertexToSmooth.size();i++)
	//{
	//	this->vecTestPoints.push_back(vecVertexToSmooth.at(i)->point());
	//}

	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	Mesh.SetRenderInfo(true,true,true,true,true);

	this->hExtrusionClosedCurveVertex3d.clear();
	this->ExtrusionSilhPoints.clear();
	this->ClosedCurveProj.clear();
	this->iDrawingCurveType=NONE_SELECTED;

	return true;
}

int CMeshExtrusion::GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurve)
{
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		Vertex_handle vhCurrent,vhNext;
		if (i==this->hExtrusionClosedCurveVertex3d.size()-1)
		{
			vhCurrent=this->hExtrusionClosedCurveVertex3d.back();
			vhNext=this->hExtrusionClosedCurveVertex3d.front();
		}
		else
		{
			vhCurrent=this->hExtrusionClosedCurveVertex3d.at(i);
			vhNext=this->hExtrusionClosedCurveVertex3d.at(i+1);
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
	assert(hhClosedCurve.size()==this->hExtrusionClosedCurveVertex3d.size());
	return hhClosedCurve.size();
}

int CMeshExtrusion::DeleteFacetsInClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve)
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

int CMeshExtrusion::GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,
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

int CMeshExtrusion::ExtrudeClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle>& hhClosedCurve,
									   Plane_3 BestFittingPlane,vector<vector<Point_3> >& testvecvecNewEdgeVertexPos,
									   vector<Facet_handle>& fhExtrudedFacet)
{
	//fill the hole of hhClosedCurve 
	Mesh.fill_hole(hhClosedCurve.front());
	Mesh.normalize_border();
	int test00=Mesh.size_of_border_edges();
	assert(test00==0);
	
	//create a center vertex
	std::list<Point_3> EdgePoints;
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		EdgePoints.push_back(this->hExtrusionClosedCurveVertex3d.at(i)->point());
	}
	Halfedge_handle hhToCenterPoint=Mesh.create_center_vertex(hhClosedCurve.front());


	vector<Halfedge_handle> hhNewToCenter;//halfedge pointing to the center vertex
	for (unsigned int i=0;i<hhClosedCurve.size();i++)
	{
		hhNewToCenter.push_back(hhClosedCurve.at(i)->prev()->opposite());
	}

	vector<vector<Point_3> > vecvecNewEdgeVertexPos;
	GetExtrusionPointsPos(BestFittingPlane,vecvecNewEdgeVertexPos);
	testvecvecNewEdgeVertexPos=vecvecNewEdgeVertexPos;

	Point_3 TopPoint;
	if (ExtrusionSilhPoints.size()%2==0)
	{
		TopPoint=CGAL::centroid(vecvecNewEdgeVertexPos.back().begin(),vecvecNewEdgeVertexPos.back().end());
	}
	else
	{
		TopPoint=ExtrusionSilhPoints.at(ExtrusionSilhPoints.size()/2);
	}

	hhToCenterPoint->vertex()->point()=TopPoint;

	//put the initial vertices in
	this->vecvecExtrudedVertex.push_back(this->hExtrusionClosedCurveVertex3d);
	for (unsigned int i=0;i<vecvecNewEdgeVertexPos.size();i++)
	{
		vector<Point_3> NewEdgeVertexPos=vecvecNewEdgeVertexPos.at(i);
		ExtrudeNewLayer(Mesh,NewEdgeVertexPos,hhNewToCenter);
		
		vector<Vertex_handle> vecExtrudedVertex;
		for (unsigned int j=0;j<hhNewToCenter.size();j++)
		{
			vecExtrudedVertex.push_back(hhNewToCenter.at(j)->opposite()->vertex());
		}
		this->vecvecExtrudedVertex.push_back(vecExtrudedVertex);

		//store extruded facets for local refine
		for (unsigned int j=0;j<hhNewToCenter.size();j++)
		{
			Halfedge_handle hhtemp=hhNewToCenter.at(j)->opposite()->next()->opposite();
			Halfedge_handle hhtemp2=hhtemp->next()->opposite();
			fhExtrudedFacet.push_back(hhtemp->facet());
			fhExtrudedFacet.push_back(hhtemp2->facet());
		}
	}
	assert(vecvecNewEdgeVertexPos.size()==this->vecvecExtrudedVertex.size()-1);

	//store the last extruded facets for local refine
	for (unsigned int i=0;i<hhNewToCenter.size();i++)
	{
		fhExtrudedFacet.push_back(hhNewToCenter.at(i)->facet());
	}
	
	
	Mesh.normalize_border();
	assert(Mesh.size_of_border_edges()==0);

	return 0;
}

int CMeshExtrusion::ExtrudeNewLayer(KW_Mesh& Mesh,vector<Point_3> NewEdgeVertexPos,
									vector<Halfedge_handle>& hhNewToCenter)
{
	//is this necessary?Does hhNewToCenter change?
	vector<Halfedge_handle> hhNewToCenterTemp;

	//split each edge
	vector<Halfedge_handle> hhvecToNewEdgeVertex;
	for (unsigned int i=0;i<hhNewToCenter.size();i++)
	{
//		Point_3 oripoint=hhNewToCenter.at(i)->opposite()->vertex()->point();
		Halfedge_handle hhToNewEdgeVertex=Mesh.split_edge(hhNewToCenter.at(i)->opposite());
//		Point_3 newpoint=Point_3(oripoint.x(),oripoint.y()-1,oripoint.z());
		hhToNewEdgeVertex->vertex()->point()=NewEdgeVertexPos.at(i);
		hhvecToNewEdgeVertex.push_back(hhToNewEdgeVertex);
		hhNewToCenterTemp.push_back(hhToNewEdgeVertex->opposite());
	}

	//split un-planar facet&quad
	for (unsigned int i=0;i<hhvecToNewEdgeVertex.size();i++)
	{
		Halfedge_handle hhnewTri=Mesh.split_facet(hhvecToNewEdgeVertex.at(i),
											hhvecToNewEdgeVertex.at(i)->next()->next()->next());
		Halfedge_handle hhnewQuad=hhnewTri->opposite();
		Mesh.split_facet(hhnewQuad,hhnewQuad->next()->next());
	}

	hhNewToCenter=hhNewToCenterTemp;

	return hhNewToCenter.size();
}

int CMeshExtrusion::GetExtrusionPointsPos(Plane_3 BestFittingPlane,vector<vector<Point_3> >& vecvecNewEdgeVertexPos)
{
	//reverse ExtrusionSilhPoints if needed
	if (CGAL::has_larger_distance_to_point(this->RotateXAxisStartPoint,this->ExtrusionSilhPoints.front(),
		this->ExtrusionSilhPoints.back()))
	{
		std::reverse(this->ExtrusionSilhPoints.begin(),this->ExtrusionSilhPoints.end());
	}

	//get the projection points
	Plane_3 ClosedCurvePlane;
	vector<Point_3> ClosedCurvePoints3d;
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		ClosedCurvePoints3d.push_back(this->hExtrusionClosedCurveVertex3d.at(i)->point());
	}
	Point_3 ClosedCurveCentroidPoint;
	double result=linear_least_squares_fitting_3(ClosedCurvePoints3d.begin(),ClosedCurvePoints3d.end(),
		ClosedCurvePlane,ClosedCurveCentroidPoint,CGAL::Dimension_tag<0>());

	vector<Point_3> ClosedCurvePointsProj3d;
	for (unsigned int i=0;i<ClosedCurvePoints3d.size();i++)
	{
		ClosedCurvePointsProj3d.push_back(ClosedCurvePlane.projection(ClosedCurvePoints3d.at(i)));
	}

	//rotate ClosedCurvePointsProj3d along axis first
	GeometryAlgorithm compute;
	compute.ComputeRotatedCurve(this->RotateXAxisStartPoint,this->RotateXAxisEndPoint,this->dAccumulatedAngleX,
								ClosedCurvePointsProj3d);

	//translate,rotate,scale in one plane
	int iGroupNum=(int)(ExtrusionSilhPoints.size()/2);
	Point_3 OldMiddlePoint=CGAL::midpoint(this->RotateXAxisEndPoint,this->RotateXAxisStartPoint);
	for (int i=0;i<iGroupNum;i++)
	{
		Point_3 NewAxisStartPoint=this->ExtrusionSilhPoints.at(i);
		Point_3 NewAxisEndPoint=this->ExtrusionSilhPoints.at(this->ExtrusionSilhPoints.size()-1-i);
		Point_3 NewMiddlePoint=CGAL::midpoint(NewAxisStartPoint,NewAxisEndPoint);
		Vector_3 vec=NewMiddlePoint-OldMiddlePoint;
		vector<Point_3> NewLayerPoints;
		//move to the height of each layer
		for (unsigned int j=0;j<ClosedCurvePointsProj3d.size();j++)
		{
			Point_3 OldPoint=ClosedCurvePointsProj3d.at(j);
			Point_3 NewPoint=OldPoint+vec;
			NewLayerPoints.push_back(NewPoint);
		}

		//rotate along axis through middle point&perpendicular to plane
		Point_3 TransStartPoint=this->RotateXAxisStartPoint+vec;
		Vector_3 vecFrom=TransStartPoint-NewMiddlePoint;
		Vector_3 vecTo=NewAxisStartPoint-NewMiddlePoint;
		Vector_3 vecCrossResult;
		double angle=compute.GetAngleBetweenTwoVectors3d(vecFrom,vecTo,vecCrossResult);
		Point_3 AxisEndPoint=NewMiddlePoint+vecCrossResult;

		Point_3 testRotatePoint0=NewLayerPoints.front();
		Point_3 testRotatePoint1=testRotatePoint0;
		compute.ComputeRotatedPoint(NewMiddlePoint,AxisEndPoint,angle,testRotatePoint0);
		compute.ComputeRotatedPoint(AxisEndPoint,NewMiddlePoint,angle,testRotatePoint1);
		if (CGAL::has_larger_distance_to_point(NewAxisStartPoint,testRotatePoint0,testRotatePoint1))
		{
			compute.ComputeRotatedCurve(AxisEndPoint,NewMiddlePoint,angle,NewLayerPoints);
		}
		else
		{
			compute.ComputeRotatedCurve(NewMiddlePoint,AxisEndPoint,angle,NewLayerPoints);
		}
	
		
		//scale
		double dLen=sqrt(vecTo.squared_length());
		for (unsigned int j=0;j<NewLayerPoints.size();j++)
		{
			Vector_3 vec=NewLayerPoints.at(j)-NewMiddlePoint;
			vec=vec/sqrt(vec.x()*vec.x()+vec.y()*vec.y()+vec.z()*vec.z());
			vec=vec*dLen;
			NewLayerPoints.at(j)=NewMiddlePoint+vec;
		}

		vecvecNewEdgeVertexPos.push_back(NewLayerPoints);
	}
	return vecvecNewEdgeVertexPos.size();
}

int CMeshExtrusion::GetExtrudedVertices(vector<vector<Vertex_handle> >& OutvecvecExtrudedVertex)
{
	OutvecvecExtrudedVertex=this->vecvecExtrudedVertex;
	return OutvecvecExtrudedVertex.size();
}

int CMeshExtrusion::GetVerticesToSmooth(vector<Vertex_handle>& vecVertexToSmooth)
{
	vecVertexToSmooth=this->hExtrusionClosedCurveVertex3d;
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=vecVertexToSmooth.at(i)->vertex_begin();
		do 
		{
			Vertex_handle NbVertex=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator hFind=find(this->hExtrusionClosedCurveVertex3d.begin(),
				this->hExtrusionClosedCurveVertex3d.end(),NbVertex);
			if (hFind==this->hExtrusionClosedCurveVertex3d.end())
			{
				vecVertexToSmooth.push_back(NbVertex);
			}
			Havc++;
		} while(Havc!=vecVertexToSmooth.at(i)->vertex_begin());
	}
	return vecVertexToSmooth.size();
}

void CMeshExtrusion::Render(bool bSmoothView,GLenum mode)
{
	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (mode==GL_RENDER)
	{
		RenderRefPlane(bSmoothView,mode);
		RenderCurvePoint2D(modelview,projection,viewport);
		RenderClosedCurve3D();
		RenderSilhCurve3D();
		RenderTestPoints();
	}
	else if (mode==GL_SELECT)
	{
		RenderRefPlane(bSmoothView,mode);
	}
}

void CMeshExtrusion::RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport)
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

void CMeshExtrusion::RenderClosedCurve3D()
{
	if (this->hExtrusionClosedCurveVertex3d.empty())
	{
		return;
	}
	glLineWidth(6.0);
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->hExtrusionClosedCurveVertex3d.size();i++)
	{
		glBegin(GL_LINES);
		glColor3f(0,0,1);
		{
			if (i!=this->hExtrusionClosedCurveVertex3d.size()-1)
			{
				glVertex3d(this->hExtrusionClosedCurveVertex3d.at(i)->point().x(),
					this->hExtrusionClosedCurveVertex3d.at(i)->point().y(),
					this->hExtrusionClosedCurveVertex3d.at(i)->point().z());
				glVertex3d(this->hExtrusionClosedCurveVertex3d.at(i+1)->point().x(),
					this->hExtrusionClosedCurveVertex3d.at(i+1)->point().y(),
					this->hExtrusionClosedCurveVertex3d.at(i+1)->point().z());
			} 
			else
			{
				glVertex3d(this->hExtrusionClosedCurveVertex3d.back()->point().x(),
					this->hExtrusionClosedCurveVertex3d.back()->point().y(),
					this->hExtrusionClosedCurveVertex3d.back()->point().z());
				glVertex3d(this->hExtrusionClosedCurveVertex3d.front()->point().x(),
					this->hExtrusionClosedCurveVertex3d.front()->point().y(),
					this->hExtrusionClosedCurveVertex3d.front()->point().z());
			}
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
	glLineWidth(1.0);		
}

void CMeshExtrusion::RenderSilhCurve3D()
{
	if (this->ExtrusionSilhPoints.empty())
	{
		return;
	}
	//glPointSize(5);
	//glBegin(GL_POINTS);
	//glColor3f(0,1,0);
	//for (unsigned int i=0;i<ExtrusionSilhPoints.size();i++)
	//{
	//	glVertex3d(ExtrusionSilhPoints.at(i).x(),
	//		ExtrusionSilhPoints.at(i).y(),
	//		ExtrusionSilhPoints.at(i).z());
	//}
	//glEnd();
	//glPointSize(1);
	glLineWidth(6.0);
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->ExtrusionSilhPoints.size()-1;i++)
	{
		glBegin(GL_LINES);
		glColor3f(0,1,0);
		glVertex3d(this->ExtrusionSilhPoints.at(i).x(),
			this->ExtrusionSilhPoints.at(i).y(),
			this->ExtrusionSilhPoints.at(i).z());
		glVertex3d(this->ExtrusionSilhPoints.at(i+1).x(),
			this->ExtrusionSilhPoints.at(i+1).y(),
			this->ExtrusionSilhPoints.at(i+1).z());
		glEnd();
	}
	glEnable(GL_LIGHTING);
	glLineWidth(1.0);		
}

void CMeshExtrusion::RenderRefPlane(bool bSmoothView,GLenum mode)
{
	if (this->hExtrusionClosedCurveVertex3d.empty())
	{
		return;
	}
	glLineWidth(2);

	//draw the frame of the plane
	if (mode==GL_RENDER)
	{
		glDisable(GL_LIGHTING);

		if (pDoc->GetRBSelName()==EXTRUSION_REF_PLANE)
		{
			glColor4fv(OPAQUE_SELECTED_COLOR);
		}
		else
		{
			glColor3f(0,0,1);
		}

		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
		glBegin(GL_QUADS);
		glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
		glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
		glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
		glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);
		glEnd();

		glEnable(GL_LIGHTING);
	}

	//draw the transparent face of the plane(2 faces)
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示

	const  GLfloat blue_color[] = {0.0f, 0.0f, 1.0f, 0.2f};
	const GLfloat mat_blue_emission[] = {0.0f, 0.0f, 1.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , blue_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_blue_emission);

	if (pDoc->GetRBSelName()==EXTRUSION_REF_PLANE)
	{
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , TRANSPARENT_SELECTED_COLOR);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  OPAQUE_SELECTED_COLOR);
	}
	if (mode==GL_SELECT)
	{
		glPushName(EXTRUSION_REF_PLANE);
	}

	glClear(GL_DEPTH_BUFFER_BIT);
	//glDisable(GL_DEPTH_TEST);
	glDepthMask(FALSE);

//	glColor4f(0.0f,0.0f,1.0f,0.2f);			// Full Brightness, 50% Alpha

	glBegin(GL_QUADS);
	//				glColor3f(0,1,0);
	glNormal3f(this->BestFittingPlane.orthogonal_vector().x(),
		this->BestFittingPlane.orthogonal_vector().y(),
		this->BestFittingPlane.orthogonal_vector().z());
	glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
	glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
	glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
	glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);

	glNormal3f(this->BestFittingPlane.opposite().orthogonal_vector().x(),
		this->BestFittingPlane.opposite().orthogonal_vector().y(),
		this->BestFittingPlane.opposite().orthogonal_vector().z());
	glVertex3d(this->PlaneBoundaryPoints[3].x,this->PlaneBoundaryPoints[3].y,this->PlaneBoundaryPoints[3].z);
	glVertex3d(this->PlaneBoundaryPoints[2].x,this->PlaneBoundaryPoints[2].y,this->PlaneBoundaryPoints[2].z);
	glVertex3d(this->PlaneBoundaryPoints[1].x,this->PlaneBoundaryPoints[1].y,this->PlaneBoundaryPoints[1].z);
	glVertex3d(this->PlaneBoundaryPoints[0].x,this->PlaneBoundaryPoints[0].y,this->PlaneBoundaryPoints[0].z);
	glEnd();

	glDepthMask(TRUE);
	//glEnable(GL_DEPTH_TEST);

	if (mode==GL_SELECT)
	{
		glPopName();
	}

	if (bSmoothView)
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
	} 
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
	}
	glLineWidth(1);
}

void CMeshExtrusion::RenderTestPoints()
{
	if (this->vecTestPoints.empty())
	{
		return;
	}
	glPointSize(5);
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->vecTestPoints.size();i++)
	{
		glBegin(GL_POINTS);
		glColor3f(0,1,1);
		{
			glVertex3d(vecTestPoints.at(i).x(),vecTestPoints.at(i).y(),vecTestPoints.at(i).z());
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
	glPointSize(1);
}