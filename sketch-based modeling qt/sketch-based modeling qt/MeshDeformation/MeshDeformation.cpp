#include "MeshDeformation.h"
#include "DeformationAlgorithm.h"
#include "EdgeBasedDeform.h"
#include "DualMeshDeform.h"
#include "RSRCellDeform.h"
#include "WedgeEdgeBasedDeform.h"
#include "MatConsDeform.h"
#include "../sketchviewer.h"
#include "../sketchinterface.h"
#include "../PaintingOnMesh.h"
#include "../OBJHandle.h"
#include <QMessageBox>

CMeshDeformation::CMeshDeformation(void)
{
}

CMeshDeformation::~CMeshDeformation(void)
{
}

void CMeshDeformation::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;
	this->CurvePoint2D.clear();
	this->iDrawingCurveType=NONE_SELECTED;
	this->Plane_spin=0.0;
	this->bRenderRefPlane[0]=this->bRenderRefPlane[1]=this->bRenderRefPlane[2]=true;
	this->bRenderSphere=true;
	this->RefSphere=Sphere_3(Point_3(0,0,0),0);
	this->bRenderHandleNb=false;
	this->bRenderROI=false;
	this->bRenderAnchor=false;
	
	this->vecHandlePoint.clear();
	this->vecHandleNbVertex.clear();

	this->vecDeformCurvePoint3d.clear();
	this->vecDeformCurveProjPoint3d.clear();
	this->ROIVertices.clear();
	this->AnchorVertices.clear();
	this->dSquaredDistanceThreshold=0.09;
	this->bHandleStrokeType=true;

	this->dFlexibleDeformLambda=0.5;
	this->iFlexibleDeformIterNum=2;

	this->dMaterial=0.5;

	this->DualMesh.clear();
	this->vecDualHandle.clear();
	this->vecDualROI.clear();
	this->vecDualAnchor.clear();

	//init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPDeformation()!=NULL)
	//{
	//	pCP->GetCPDeformation()->Init();
	//}
}

void CMeshDeformation::InputCurvePoint2D(QPoint Point2D)
{
	if (this->iDrawingCurveType!=NONE_SELECTED)
	{
		this->CurvePoint2D.push_back(Point2D);
	}
}

void CMeshDeformation::SetDrawingCurveType(int iType)
{
	this->iDrawingCurveType=iType;
}

int CMeshDeformation::GetDrawingCurveType()
{
	return this->iDrawingCurveType;
}

void CMeshDeformation::SetSelectedItem()
{
	int iSelectedName=this->pDoc->GetRBSelName();
	if (iSelectedName>=DEFORMATION_PLANE_NAME_BEGIN && iSelectedName<=DEFORMATION_PLANE_NAME_END)
	{
		if (iSelectedName==DEFORMATION_NORM_PLANE)
		{
			this->iDrawingCurveType=DEFORMATION_DEFORM_CURVE;
		}
		else if (iSelectedName=DEFORMATION_TAN_PLANE)
		{
			this->iDrawingCurveType=DEFORMATION_DEFORM_PROJ_CURVE;
		}
	}
	else if (iSelectedName>=DEFORMATION_CURVE_NAME_BEGIN && iSelectedName<=DEFORMATION_CURVE_NAME_END)
	{
		this->iDrawingCurveType=iSelectedName;
	}
	else 
	{
		this->iDrawingCurveType=NONE_SELECTED;
	}

}

void CMeshDeformation::ManipSelItem(int g_iStepX, int g_iStepY)
{
	if (this->pDoc->GetRBSelName()!=this->pDoc->GetLBSelName())
	{
		return;
	}
	int iSelectedName=this->pDoc->GetRBSelName();

	if ((iSelectedName==DEFORMATION_NORM_PLANE ||iSelectedName==DEFORMATION_TAN_PLANE ) && !this->vecHandlePoint.empty())
	{
		this->Plane_spin=g_iStepX;
		if (this->Plane_spin> 180.0 || this->Plane_spin< -180.0)
		{
			this->Plane_spin=0.0;
		}
		RotateDeformationPlane();
	}
	else if (iSelectedName==DEFORMATION_HANDLE_CURVE || iSelectedName==DEFORMATION_DEFORM_CURVE)
	{
		assert(!this->vecHandlePoint.empty());
		TranslateDeformCurvePoint3dOnBFPlane(g_iStepX*0.1,g_iStepY*0.1);
	}
}

void CMeshDeformation::Conver2DCurveTo3D(KW_Mesh& Mesh)
{
	//if (this->CurvePoint2D.empty()&&this->iDrawingCurveType!=5)
	//{
	//	return;
	//}

	SketchViewer* pView=this->pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (this->iDrawingCurveType==DEFORMATION_HANDLE_CURVE)//handle curve
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D);

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

		CPaintingOnMesh Painting;
		int iResult=0;
		if (this->bHandleStrokeType)
		{
			iResult=Painting.PaintingOpenStrokeOnFrontalMesh(Mesh,UserCurvePoint,
				modelview,this->vecHandlePoint,this->vecHandleNbVertex);
		}
		else
		{
//			iResult=Painting.PaintingClosedStrokeOnFrontalAndRearMesh(Mesh,UserCurvePoint,
//				modelview,DeformHandleCurveVertex3d);
		}
		if (iResult)
		{
			GetDeformationPlane(UserCurvePoint,this->BestFittingPlane,this->PlaneBoundaryPoints);
			FindROIVertices(Mesh);
			//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
		}
		this->vecDeformCurvePoint3d.clear();
	}
	else if (this->iDrawingCurveType==DEFORMATION_DEFORM_CURVE)//deformation curve
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D,25.0,this->vecHandlePoint.size());

		vector<Point_3> DeformCurvePoint3d;
		CPaintingOnMesh Painting;
		int iResult=Painting.PaintingOnBFPlane(this->BestFittingPlane,modelview,projection,viewport,
			this->CurvePoint2D,DeformCurvePoint3d);
		if (iResult)
		{
			//compute.ReversePointsOrder3d(this->vecHandleCurveVertex3d.front()->point(),DeformCurvePoint3d);
			this->vecDeformCurvePoint3d=DeformCurvePoint3d;
		}
	}
	else if (this->iDrawingCurveType==DEFORMATION_DEFORM_PROJ_CURVE)//projection of deformation curve
	{
		if (!this->vecDeformCurvePoint3d.empty())
		{
			GeometryAlgorithm compute;
			compute.ProcessCurverPoint(this->CurvePoint2D,25.0,this->vecHandlePoint.size());

			vector<Point_3> DeformCurveProjPoint3d;
			CPaintingOnMesh Painting;
			int iResult=Painting.PaintingOnBFPlane(this->RefTangentialPlane,modelview,projection,viewport,
				this->CurvePoint2D,DeformCurveProjPoint3d);
			if (iResult)
			{
				//compute.ReversePointsOrder3d(this->vecDeformCurvePoint3d.front(),DeformCurveProjPoint3d);
				this->vecDeformCurveProjPoint3d=DeformCurveProjPoint3d;
				CombineDeformCurveAndProj();
			}
		}
	}
	else if (this->iDrawingCurveType==DEFORMATION_GESTURE_CIRCLE_ROI)//curve to circle the ROI
	{
		GeometryAlgorithm compute;
		compute.ProcessCurverPoint(this->CurvePoint2D);

		CircleROIVertices(Mesh,this->CurvePoint2D,modelview,projection,viewport);
	}
	else if (this->iDrawingCurveType==DEFORMATION_GESTURE_PAINT_ROI)//points to paint the ROI
	{
		GetAnchorVertices();
	}
	else if (this->iDrawingCurveType==6)//handle point
	{
		//convert UCP into opengl coordinate(on Znear) 
		GLdouble  winX, winY, winZ; 
		GLdouble posX, posY, posZ; 
		winX =this->CurvePoint2D.front().x();
		winY = viewport[3] - (float)this->CurvePoint2D.front().y();
		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
		Point_3 CurrentPoint(posX,posY,posZ);

		Facet_handle hFacet;
		CPaintingOnMesh Painting;
		bool bResult=Painting.PaintingPointOnFrontalMesh(Mesh,CurrentPoint,hFacet,modelview);

		if (bResult)
		{
			//get nearest vertex to the point
			Halfedge_around_facet_circulator k = hFacet->facet_begin();
			double dMinDist=9999.0;
			Vertex_handle NearestVer;
			do 
			{
				double dCurrentDist=CGAL::squared_distance(CurrentPoint,k->vertex()->point());
				if (dCurrentDist<dMinDist)
				{
					dMinDist=dCurrentDist;
					NearestVer=k->vertex();
				}
			} while(++k != hFacet->facet_begin());

			this->vecHandleNbVertex.clear();
			this->vecHandleNbVertex.push_back(NearestVer);
			this->RefSphere=Sphere_3(NearestVer->point(),DEFAULT_REFSPHERE_RADIUS*DEFAULT_REFSPHERE_RADIUS);
			FindROIVertices(Mesh);
		}
	}
	else if (this->iDrawingCurveType==7)//new position of handle point
	{
		CPaintingOnMesh Painting;
		vector<Point_3> PaintingResult;
		int iResult=Painting.PaintingPointOnSphere(this->RefSphere,modelview,projection,viewport,
			this->CurvePoint2D.front(),PaintingResult);
		if (iResult)
		{
			this->vecDeformCurvePoint3d.clear();
			this->vecDeformCurvePoint3d.push_back(PaintingResult.front());
		}
	}
	else if (this->iDrawingCurveType==8)//circle frontal vertices to specify material
	{
		CPaintingOnMesh Painting;
		vector<Vertex_handle> vecCirVer;
		if (Painting.CircleFrontVertices(Mesh,this->CurvePoint2D,modelview,projection,viewport,vecCirVer))
		{
			CMatConsDeform::SetMatAndColor(vecCirVer,this->dMaterial);
			pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
		}
	}
	else if (this->iDrawingCurveType==9)//circle vertices to specify material
	{
		CPaintingOnMesh Painting;
		vector<Vertex_handle> vecCirVer;
		if (Painting.CircleAllVertices(Mesh,this->CurvePoint2D,modelview,projection,viewport,vecCirVer))
		{
			CMatConsDeform::SetMatAndColor(vecCirVer,this->dMaterial);
			pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
		}
	}
	else if (this->iDrawingCurveType==10)//brush material for vertices
	{
		//convert UCP into opengl coordinate(on Znear) 
		GLdouble  winX, winY, winZ; 
		GLdouble posX, posY, posZ; 

		winX =this->CurvePoint2D.back().x();
		winY = viewport[3] - (float)this->CurvePoint2D.back().y();
		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
		Point_3 CurrentPoint(posX,posY,posZ);

		vector<Vertex_handle> vecBrushVer;

		Facet_handle CurrentFacet;
		CPaintingOnMesh Painting;
		bool bResult=Painting.PaintingPointOnFrontalMesh(Mesh,CurrentPoint,CurrentFacet,modelview);
		if (bResult)
		{
			Halfedge_around_facet_circulator k = CurrentFacet->facet_begin();
			do 
			{
				vecBrushVer.push_back(k->vertex());
			} while(++k != CurrentFacet->facet_begin());
		}
		CMatConsDeform::SetMatAndColor(vecBrushVer,this->dMaterial);
		pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
	}
	else
	{
		//do nothing
	}
	//this->iDrawingCurveType=NONE_SELECTED;
	this->CurvePoint2D.clear();
}

vector<Vertex_handle> CMeshDeformation::GetHandleNbVertex()
{
	return this->vecHandleNbVertex;
}

vector<Point_3> CMeshDeformation::GetDeformCurvePoints()
{
	return this->vecDeformCurvePoint3d;
}

void CMeshDeformation::SetDeformCurvePoints(vector<Point_3> vecDeformCurvePoint3dIn)
{
	this->vecDeformCurvePoint3d=vecDeformCurvePoint3dIn;
}

vector<Vertex_handle> CMeshDeformation::GetROIVertices()
{
	return this->ROIVertices;
}

bool CMeshDeformation::GetHandleStrokeType()
{
	return this->bHandleStrokeType;
}

void CMeshDeformation::SetHandleStrokeType(bool bStrokeType)
{
	this->bHandleStrokeType=bStrokeType;
}

void CMeshDeformation::SetSelectionRange(double iRadius,KW_Mesh& Mesh)
{
	this->dSquaredDistanceThreshold=iRadius*iRadius;

	//calculate the NeighborVertex in real time
	if ((!this->vecHandlePoint.empty()) || (!this->vecHandleNbVertex.empty()))
	{
		FindROIVertices(Mesh);
	}
}

double CMeshDeformation::GetSelectionRange()
{
	return std::sqrt(this->dSquaredDistanceThreshold);
}

void CMeshDeformation::SetFlexibleDeformPara(int iIter,double dLambda)
{
	this->iFlexibleDeformIterNum=iIter;
	this->dFlexibleDeformLambda=dLambda;
}

void CMeshDeformation::GetFlexibleDeformPara(int& iIter,double& dLambda)
{
	iIter=this->iFlexibleDeformIterNum;
	dLambda=this->dFlexibleDeformLambda;
}

void CMeshDeformation::CombineDeformCurveAndProj()
{
	if (!this->bHandleStrokeType)//if handle curve is closed
	{
		return;
	}
	if (this->vecHandlePoint.empty()||this->vecDeformCurvePoint3d.empty()
		||this->vecDeformCurveProjPoint3d.empty())
	{
		return;
	}
	for (unsigned int i=0;i<this->vecDeformCurvePoint3d.size();i++)
	{
		//project the deform curve on the refproj plane
		Point_3 PointProj=this->RefTangentialPlane.projection(this->vecDeformCurvePoint3d.at(i));
		//compute the vector from PointProj to the shadow point
		Vector_3 vec=this->vecDeformCurveProjPoint3d.at(i)-PointProj;
		//add the vector to the deform curve
		this->vecDeformCurvePoint3d.at(i)=this->vecDeformCurvePoint3d.at(i)+vec;
	}

	this->vecDeformCurveProjPoint3d.clear();
}

void CMeshDeformation::GetDeformationPlane(vector<Point_3> RefPoints,Plane_3& BestFittingPlane,
										   Point3D * PlaneBoundaryPoints)
{
	//Find the Best-Fitting Plane first
	list<Point_3> points;
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		points.push_back(vecHandlePoint.at(i).PointPos);
	}
	//keep points fixed,'cos it'll be used to compute plane boundary points
	list<Point_3> Mergedpoints=points;
//	Mergedpoints.insert(Mergedpoints.end(),RefPoints.begin(),RefPoints.end());
	Point_3 c;
	double result=linear_least_squares_fitting_3(Mergedpoints.begin(),Mergedpoints.end(),
							BestFittingPlane,c,CGAL::Dimension_tag<0>());

	//rotate it around the segmant between the projection of 
	//the start and end point
	Point_3 StartPoint=points.front();
	Point_3 StartPointProj=BestFittingPlane.projection(StartPoint);
	Point_3 EndPoint;
	if (this->bHandleStrokeType)
	{
		EndPoint=points.back();
	}
	else
	{
//		EndPoint=vecHandleCurveVertex3d.at((int)vecHandleCurveVertex3d.size()/2)->point();
	}
	Point_3 EndPointProj=BestFittingPlane.projection(EndPoint);

	//get the perpendicular axis to compute the binormal plane
	Point_3 MidPointProj=CGAL::midpoint(StartPointProj,EndPointProj);
	Point_3 MidPointProjExt=MidPointProj+BestFittingPlane.orthogonal_vector();
	Point_3 PerStarPointProj=StartPointProj;
	Point_3 PerEndPointProj=StartPointProj;
	GeometryAlgorithm compute;
	compute.ComputeRotatedPoint(MidPointProj,MidPointProjExt,90,PerStarPointProj);
	compute.ComputeRotatedPoint(MidPointProj,MidPointProjExt,-90,PerEndPointProj);


	//for basic plane
	compute.GetPlaneBoundaryPoints(StartPointProj,EndPointProj,BestFittingPlane,PlaneBoundaryPoints);

	//compute tangential plane
	this->RefTangentialPlane=this->BestFittingPlane;
	compute.ComputeRotatedPlane(StartPointProj,EndPointProj,90,this->RefTangentialPlane);
	compute.GetPlaneBoundaryPoints(StartPointProj,EndPointProj,this->RefTangentialPlane,
		this->RefTangentialPlaneBoundaryPoints);
	//compute binormal plane
	this->RefBiNormalPlane=this->BestFittingPlane;
	compute.ComputeRotatedPlane(PerStarPointProj,PerEndPointProj,90,this->RefBiNormalPlane);
	compute.GetPlaneBoundaryPoints(PerStarPointProj,PerEndPointProj,this->RefBiNormalPlane,
		this->RefBiNormalPlaneBoundaryPoints);
}

void CMeshDeformation::RotateDeformationPlane()
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}
	Point_3 StartPoint=this->vecHandlePoint.front().PointPos;
	Point_3 StartPointProj=this->BestFittingPlane.projection(StartPoint);
	Point_3 EndPoint;
	if (this->bHandleStrokeType)
	{
		EndPoint=this->vecHandlePoint.back().PointPos;
	}
	else
	{
//		EndPoint=this->vecHandleCurveVertex3d.at((int)this->vecHandleCurveVertex3d.size()/2)->point();
	}
	Point_3 EndPointProj=BestFittingPlane.projection(EndPoint);

	GeometryAlgorithm compute;
	compute.ComputeRotatedPlane(StartPointProj,EndPointProj,this->Plane_spin,this->BestFittingPlane);
	if(!this->vecDeformCurvePoint3d.empty())
	{
		compute.ComputeRotatedCurve(StartPointProj,EndPointProj,this->Plane_spin,this->vecDeformCurvePoint3d);
	}
	compute.GetPlaneBoundaryPoints(StartPointProj,EndPointProj,this->BestFittingPlane,this->PlaneBoundaryPoints);

	//compute proj plane
	this->RefTangentialPlane=this->BestFittingPlane;
	compute.ComputeRotatedPlane(StartPointProj,EndPointProj,90,this->RefTangentialPlane);
	compute.GetPlaneBoundaryPoints(StartPointProj,EndPointProj,this->RefTangentialPlane,
		this->RefTangentialPlaneBoundaryPoints);
	//compute projection(if exist)
	if(!this->vecDeformCurveProjPoint3d.empty())
	{
		compute.ComputeRotatedCurve(StartPointProj,EndPointProj,this->Plane_spin,this->vecDeformCurveProjPoint3d);
	}

	//compute rotated binormal plane boundary points only
	Point_3 MidPointProj=CGAL::midpoint(StartPointProj,EndPointProj);
	Point_3 MidPointProjExt=MidPointProj+BestFittingPlane.orthogonal_vector();
	Point_3 PerStarPointProj=StartPointProj;
	Point_3 PerEndPointProj=StartPointProj;
	compute.ComputeRotatedPoint(MidPointProj,MidPointProjExt,90,PerStarPointProj);
	compute.ComputeRotatedPoint(MidPointProj,MidPointProjExt,-90,PerEndPointProj);
	compute.GetPlaneBoundaryPoints(PerStarPointProj,PerEndPointProj,this->RefBiNormalPlane,
		this->RefBiNormalPlaneBoundaryPoints);
//	this->RefBiNormalPlane=this->BestFittingPlane;
//	compute.ComputeRotatedPlane(PerStarPointProj,PerEndPointProj,90,this->RefBiNormalPlane);
//	compute.GetPlaneBoundaryPoints(PerStarPointProj,PerEndPointProj,this->RefBiNormalPlane,
//		this->RefBiNormalPlaneBoundaryPoints);
	for (int i=0;i<4;i++)
	{
		Point_3 temp(this->RefBiNormalPlaneBoundaryPoints[i].x,this->RefBiNormalPlaneBoundaryPoints[i].y,this->RefBiNormalPlaneBoundaryPoints[i].z);
		compute.ComputeRotatedPoint(StartPointProj,EndPointProj,this->Plane_spin,temp);
		this->RefBiNormalPlaneBoundaryPoints[i].x=temp.x();
		this->RefBiNormalPlaneBoundaryPoints[i].y=temp.y();
		this->RefBiNormalPlaneBoundaryPoints[i].z=temp.z();
	}

}

void CMeshDeformation::AdjustPlaneBoundary(int iIncrease)
{
	if (this->vecHandleNbVertex.empty())
	{
		return;
	}
	//only when the planes are selected
	if (this->pDoc->GetRBSelName()<DEFORMATION_PLANE_NAME_BEGIN ||
		this->pDoc->GetRBSelName()>DEFORMATION_PLANE_NAME_END)
	{
		return;
	}
	GeometryAlgorithm compute;
	compute.AdjustPlaneBoundary(iIncrease,this->PlaneBoundaryPoints);
	compute.AdjustPlaneBoundary(iIncrease,this->RefTangentialPlaneBoundaryPoints);
	compute.AdjustPlaneBoundary(iIncrease,this->RefBiNormalPlaneBoundaryPoints);
	//adjust the radius of sphere
	if (iIncrease>=0)
	{
		this->RefSphere=Sphere_3(this->RefSphere.center(),this->RefSphere.squared_radius()*(double)1.2);
	}
	else
	{
		this->RefSphere=Sphere_3(this->RefSphere.center(),this->RefSphere.squared_radius()*(double)0.8);
	}
}

void CMeshDeformation::FindROIVertices(KW_Mesh& Mesh)
{
	this->ROIVertices.clear();
	//get the max distace
	double dMaxX,dMinX,dMaxY,dMinY,dMaxZ,dMinZ;
	dMaxX=dMinX=Mesh.vertices_begin()->point().x();
	dMaxY=dMinY=Mesh.vertices_begin()->point().y();
	dMaxZ=dMinZ=Mesh.vertices_begin()->point().z();
	for ( Vertex_iterator i = Mesh.vertices_begin(); i != Mesh.vertices_end(); i++)
	{
		if (i->point().x()>dMaxX)
		{
			dMaxX=i->point().x();
		}
		else if(i->point().x()<dMinX)
		{
			dMinX=i->point().x();
		}

		if (i->point().y()>dMaxY)
		{
			dMaxY=i->point().y();
		}
		else if(i->point().y()<dMinY)
		{
			dMinY=i->point().y();
		}

		if (i->point().z()>dMaxZ)
		{
			dMaxZ=i->point().z();
		}
		else if(i->point().z()<dMinZ)
		{
			dMinZ=i->point().z();
		}
	}
	double dDistX=dMaxX-dMinX;
	double dDistY=dMaxY-dMinY;
	double dDistZ=dMaxZ-dMinZ;
	double dMaxDist=dDistX>(dDistY>dDistZ?dDistY:dDistZ)?dDistX:(dDistY>dDistZ?dDistY:dDistZ);

	for ( Vertex_iterator i = Mesh.vertices_begin(); i != Mesh.vertices_end(); i++)
	{
		//the handle vertices should be excluded
		vector<Vertex_handle>::iterator pFind=find(this->vecHandleNbVertex.begin(),
													this->vecHandleNbVertex.end(),i);
		if (pFind!=this->vecHandleNbVertex.end())
		{
			continue;
		}

		if (this->vecHandleNbVertex.size()<2)
		{
			Sphere_3 sphere2(this->vecHandleNbVertex.front()->point(),this->dSquaredDistanceThreshold*dMaxDist*dMaxDist);
			if (!sphere2.has_on_unbounded_side(i->point()))
			{
				this->ROIVertices.push_back(i);
			}
		}
		else
		{
			for (unsigned int j=0;j<this->vecHandlePoint.size();j++)
			{
				Sphere_3 sphere(this->vecHandlePoint.at(j).PointPos,this->dSquaredDistanceThreshold*dMaxDist*dMaxDist);
				if (!sphere.has_on_unbounded_side(i->point()))
				{
					this->ROIVertices.push_back(i);
					break;
				}
			}
		}
	}
	GetAnchorVertices();
}

void CMeshDeformation::CircleROIVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve, 
										 GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//compute the 2D bounding polygon first
	Polygon_2 BoundingPolygon;
	for (unsigned int i=0;i<vecBoundingCurve.size();i++)
	{
		double winX = (double)vecBoundingCurve.at(i).x(); 
		double winY = viewport[3] - (double)vecBoundingCurve.at(i).y();
		BoundingPolygon.push_back(Point_2(winX,winY));
		//if (i>1)
		//{
		//	if (!BoundingPolygon.is_simple())
		//	{
		//		Polygon_2::Vertex_iterator j=BoundingPolygon.vertices_end();
		//		j--;
		//		BoundingPolygon.erase(j);
		//		break;
		//	}
		//}
	}
	if ((BoundingPolygon.size()<=2)||(!BoundingPolygon.is_simple()))
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid Drawing!");
		msgBox.exec();
		return;
	}

	//iterate the whole mesh to find the vertices whose projection fall in BoundingPolygon
	vector<Vertex_handle> vecTempROI;
	for ( Vertex_iterator i = Mesh.vertices_begin(); i != Mesh.vertices_end(); i++)
	{
		vector<Vertex_handle>::iterator pFind=find(this->vecHandleNbVertex.begin(),this->vecHandleNbVertex.end(),i);
		if (pFind!=vecHandleNbVertex.end())
		{
			continue;
		}

		double dWinX,dWinY,dWinZ;
		gluProject(i->point().x(),i->point().y(),i->point().z(), modelview, projection, viewport, 
			&dWinX, &dWinY, &dWinZ); 
		Point_2 PtProj(dWinX,dWinY);

		if (!BoundingPolygon.has_on_unbounded_side(PtProj))
		{
			vecTempROI.push_back(i);
		}
	}

	cout<<"Rough ROI Num: "<<vecTempROI.size()<<endl;
	 
	//delete the unconnected part
	vector<Vertex_handle> vecCurrentLayer=this->vecHandleNbVertex;
	vector<Vertex_handle> vecNewLayer,vecConnectedROI;
	do 
	{
		cout<<"CurrentLayer Num: "<<vecCurrentLayer.size()<<endl;
		cout<<"NewLayer Num: "<<vecNewLayer.size()<<endl;
		cout<<"ROI Num: "<<vecConnectedROI.size()<<endl;

		vecNewLayer.clear();
		for (unsigned int i=0;i<vecCurrentLayer.size();i++)
		{
			Vertex_handle CurrentVertex=vecCurrentLayer.at(i);
			Halfedge_around_vertex_circulator Havc=CurrentVertex->vertex_begin();
			do 
			{
				if (vecTempROI.empty())
				{
					vecNewLayer.clear();
					break;
				}
				Vertex_handle NbVertex=Havc->opposite()->vertex();
				vector<Vertex_handle>::iterator pFind=find(vecTempROI.begin(),vecTempROI.end(),NbVertex);
				if (pFind!=vecTempROI.end())
				{
					vecTempROI.erase(pFind);
					vecNewLayer.push_back(NbVertex);
				}
				Havc++;
			} while(Havc!=CurrentVertex->vertex_begin());
		}
		vecCurrentLayer=vecNewLayer;
		if (!vecNewLayer.empty())
		{
			vecConnectedROI.insert(vecConnectedROI.end(),vecNewLayer.begin(),vecNewLayer.end());
		}
	} while(!vecNewLayer.empty());
	
	if (!vecConnectedROI.empty())
	{
		this->ROIVertices=vecConnectedROI;

		GetAnchorVertices();

		//this->vecTestPoint.clear();
		//for (unsigned int i=0;i<this->AnchorVertices.size();i++)
		//{
		//	this->vecTestPoint.push_back(this->AnchorVertices.at(i)->point());
		//}
	}
}

void CMeshDeformation::PaintROIVertices(KW_Mesh& Mesh,GLdouble* modelview,GLdouble* projection,GLint* viewport)
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
		Halfedge_around_facet_circulator k = CurrentFacet->facet_begin();
		do 
		{
			vector<Vertex_handle>::iterator pVer=find(this->ROIVertices.begin(),this->ROIVertices.end(),
										k->vertex());
			vector<Vertex_handle>::iterator pVerHandleNb=find(this->vecHandleNbVertex.begin(),
				this->vecHandleNbVertex.end(),k->vertex());
			if (pVer==this->ROIVertices.end()&&pVerHandleNb==this->vecHandleNbVertex.end())
			{
				this->ROIVertices.push_back(k->vertex());
			}
		} while(++k != CurrentFacet->facet_begin());
	}
	this->CurvePoint2D.clear();
	this->AnchorVertices.clear();
}

//get anchor vertices
int CMeshDeformation::GetAnchorVertices()
{
	this->AnchorVertices.clear();
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),this->vecHandleNbVertex.begin(),this->vecHandleNbVertex.end());
	vecAllVertices.insert(vecAllVertices.end(),this->ROIVertices.begin(),this->ROIVertices.end());

	vector<Vertex_handle> LeftROIVertices=this->ROIVertices;
	vector<Vertex_handle> CurrentLayer=this->vecHandleNbVertex;

	vector<Vertex_handle>::iterator Iter;
	for (Iter=vecAllVertices.begin();Iter!=vecAllVertices.end();Iter++)
	{
		Vertex_handle CurrentVer=*Iter;
		Halfedge_around_vertex_circulator Havc=CurrentVer->vertex_begin();
		do 
		{
			Vertex_handle NbVer=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator IterFind=find(vecAllVertices.begin(),vecAllVertices.end(),NbVer);
			vector<Vertex_handle>::iterator IterFind2=find(AnchorVertices.begin(),AnchorVertices.end(),NbVer);
			if (IterFind==vecAllVertices.end()&&IterFind2==AnchorVertices.end())
			{
				this->AnchorVertices.push_back(NbVer);
			}
			Havc++;
		} while(Havc!=CurrentVer->vertex_begin());
	}

	return (int)this->AnchorVertices.size();
}

int CMeshDeformation::GetCurrentDesiredPointsPos(vector<HPCPoint> & DesiredPointsPosition)
{
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		Point_3 PointOnBestFittingPlane=BestFittingPlane.projection(this->vecHandlePoint.at(i).PointPos);

		Point_2 Pointfor2D=this->BestFittingPlane.to_2d(PointOnBestFittingPlane);
		HPCPoint PointXY;
		PointXY.x=Pointfor2D.x();
		PointXY.y=Pointfor2D.y();
		DesiredPointsPosition.push_back(PointXY);
	}
	return DesiredPointsPosition.size();
}

void CMeshDeformation::SetModifiedPointsPos(KW_Mesh& Mesh,vector<HPCPoint> DesiredPointsNewPosition)
{
	assert(this->vecHandlePoint.size()==DesiredPointsNewPosition.size());

	this->vecDeformCurvePoint3d.clear();

	for (unsigned int i=0;i<DesiredPointsNewPosition.size();i++)
	{
		HPCPoint PointXY=DesiredPointsNewPosition.at(i);
		Point_2 Pointfrom2D(PointXY.x,PointXY.y);
		Point_3 MovedPointOnBestFittingPlane=BestFittingPlane.to_3d(Pointfrom2D);

		//move the mesh point along this direction:
		//from the original point on the best-fitting plane to the moved point on that
		/*
		desired point								mesh point
		--------------------<--<--------------------	3d space
		|											|
		|											|
		|											|
		|											|
		|											|
		|											|
		/--------------------<--<---------------------	original projection of  mesh point
	   /	moved point								  /	                       
	  /                 best-fitting plane		     /
	 /                                              /
	------------------------------------------------

		*/

		Point_3 OriginalPointOnBestFittingPlane=BestFittingPlane.projection(this->vecHandlePoint.at(i).PointPos);
		//if the point keeps fixed
		//if (OriginalPointOnBestFittingPlane==MovedPointOnBestFittingPlane)
		//{
		//	continue;
		//}
		Vector_3 MovedVector=MovedPointOnBestFittingPlane-OriginalPointOnBestFittingPlane;
		Point_3 DesiredPoint=this->vecHandlePoint.at(i).PointPos+MovedVector;
		this->vecDeformCurvePoint3d.push_back(DesiredPoint);
	}

	int iType=1;//1 for uniform,2 for tan weighted,3 for cot weighted
	if (this->AnchorVertices.empty())//the whole mesh involves in the computation
	{
		if (iType==1)
		{
			GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(Mesh);
		} 
		else
		{
			GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(Mesh,iType);
		}
	}
	else
	{
		vector<Vertex_handle> temp=this->vecHandleNbVertex;
		temp.insert(temp.end(),this->ROIVertices.begin(),this->ROIVertices.end());
		temp.insert(temp.end(),this->AnchorVertices.begin(),this->AnchorVertices.end());
		if (iType==1)
		{
			GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
		}
		else
		{
			GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(temp,iType);
		}
	}
	//recaculate the model
	double dLamda=this->dFlexibleDeformLambda;
	int iIterNum=this->iFlexibleDeformIterNum;
	CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
		this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,false);
	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	Mesh.SetRenderInfo(true,true,false,false,false);

	//clear
	this->vecHandlePoint.clear();
	this->vecDeformCurvePoint3d.clear();

}


bool CMeshDeformation::SetModifiedPointsPos(KW_Mesh& Mesh,vector<Point_3>& testpoints)
{
	if(this->vecDeformCurvePoint3d.empty())
		return false;

	//for the one handle vertex case
	if (this->vecHandleNbVertex.size()==1 && this->vecDeformCurvePoint3d.size()==1)
	{
		double dLamda=this->dFlexibleDeformLambda;
		int iIterNum=this->iFlexibleDeformIterNum;
		int iType=1;

		//vector<Vertex_handle> temp=this->vecHandleNbVertex;
		//temp.insert(temp.end(),this->ROIVertices.begin(),this->ROIVertices.end());
		//temp.insert(temp.end(),this->AnchorVertices.begin(),this->AnchorVertices.end());
		//	GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
		//CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandleNbVertex,
		//	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d);

		///////////////////////////////////////////////
		//vector<Point_3> OldPos;
		//CDeformationAlgorithm::BackUpMeshGeometry(Mesh,OldPos);

		CWedgeEdgeBasedDeform::Test(dLamda,iType,iIterNum,Mesh,vecHandleNbVertex,ROIVertices,AnchorVertices,
			vecDeformCurvePoint3d,vecTestPoint);

		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
		Mesh.SetRenderInfo(true,true,false,false,false);

		//std::ofstream out2("2new.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out2,Mesh);		

		//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
		//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

//		CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(dLamda,iType,iIterNum,Mesh,vecHandleNbVertex,ROIVertices,AnchorVertices,
//			vecDeformCurvePoint3d,vecTestPoint);
//		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

		//std::ofstream out("1old.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		this->vecDeformCurvePoint3d.clear();

		return true;
	}




	assert(this->vecDeformCurvePoint3d.size()==this->vecHandlePoint.size());

	if (!this->bHandleStrokeType)
	{
		ReOrderDeformCurve();
	}

	//this->vecDeformCurvePoint3d is actually the projection of the new position of the handle curve
	//so unproject it
	//for (unsigned int i=0;i<this->vecHandleCurveVertex3d.size();i++)
	//{
	//	Point_3 OldProj=BestFittingPlane.projection(this->vecHandleCurveVertex3d.at(i)->point());
	//	Vector_3 MovedVec=this->vecDeformCurvePoint3d.at(i)-OldProj;
	//	this->vecDeformCurvePoint3d.at(i)=this->vecHandleCurveVertex3d.at(i)->point()+MovedVec;
	//}


	//back up for local refine
	KW_Mesh OldMesh=Mesh;
	vector<Point_3> OldHandlePos;
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		OldHandlePos.push_back(this->vecHandlePoint.at(i).PointPos);
	}

	vector<Point_3> NewHandlePos;


	double dLamda=this->dFlexibleDeformLambda;
	int iIterNum=this->iFlexibleDeformIterNum;
	//double dLamda=0;
	//int iIterNum=2;

	//edge based
	//////////////////////////
	vector<Point_3> OldPos;
	CDeformationAlgorithm::BackUpMeshGeometry(Mesh,OldPos);
	//back up anchor
	vector<Vertex_handle> AnchorBack=this->AnchorVertices;

	int iType=1;

	//this->AnchorVertices=AnchorBack;

	//flexible vertex 0
//	iIterNum=3;
//	dLamda=0;

	//vector<Vertex_handle> temp=this->vecHandleNbVertex;
	//temp.insert(temp.end(),this->ROIVertices.begin(),this->ROIVertices.end());
	//temp.insert(temp.end(),this->AnchorVertices.begin(),this->AnchorVertices.end());
	//GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
	//CDeformationAlgorithm::FlexibleImpDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	//	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	//dLamda=0;
	vector<Vertex_handle> temp=this->vecHandleNbVertex;
	temp.insert(temp.end(),this->ROIVertices.begin(),this->ROIVertices.end());
	temp.insert(temp.end(),this->AnchorVertices.begin(),this->AnchorVertices.end());
	GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
	CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
		this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	//std::ofstream outVerLap0("ver0-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outVerLap0,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	////flexible vertex 0.5
	//dLamda=0.5;
	//GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
	//CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	//	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outVerLap05("ver05-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outVerLap05,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	////flexible vertex 1.0
	//dLamda=1.0;
	//GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
	//CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	//	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outVerLap10("ver10-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outVerLap10,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	////////////////////CMatConsDeform::MatConsVerFlexibleDeform(iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	////////////////////	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	////////////////////OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	////////////////////std::ofstream outVerLapMat("verMat-iso.obj",ios_base::out | ios_base::trunc);
	////////////////////print_polyhedron_wavefront(outVerLapMat,Mesh);

	////////////////////CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	////////////////////OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);



	//CMatConsDeform::MatConsEdgeFlexibleDeform(iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	//		this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);




	////flexible edge 0 iso
	//iIterNum=5;
	//dLamda=0;
	//CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outWedgeEdgeLap0iso("edge0-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outWedgeEdgeLap0iso,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);


	////flexible edge 0.5 iso
	//dLamda=0.5;
	//CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outWedgeEdgeLap05iso("edge05-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outWedgeEdgeLap05iso,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	////flexible edge 1.0 iso
	//dLamda=1.0;
	//CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outWedgeEdgeLap10iso("edge10-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outWedgeEdgeLap10iso,Mesh);

	////flexible dual 0 iso
	//dLamda=0;
	//CDualMeshDeform::FlexibleDualMeshDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outDualLap0iso("dual0-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outDualLap0iso,Mesh);
	//
	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);


	////flexible dual 0.5 iso
	//this->AnchorVertices=AnchorBack;
	//dLamda=0.5;
	//CDualMeshDeform::FlexibleDualMeshDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outDualLap05iso("dual05-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outDualLap05iso,Mesh);

	//CDeformationAlgorithm::RestoreMeshGeometry(Mesh,OldPos);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);

	////flexible dual 1.0 iso
	//this->AnchorVertices=AnchorBack;
	//dLamda=1.0;
	//CDualMeshDeform::FlexibleDualMeshDeform(dLamda,iType,iIterNum,Mesh,vecHandlePoint,vecHandleNbVertex,ROIVertices,AnchorVertices,
	//	vecDeformCurvePoint3d,vecTestPoint,true);
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outDualLap10iso("dual10-iso.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outDualLap10iso,Mesh);

	//this->AnchorVertices=AnchorBack;
	//////////////////////////////////////////

	Mesh.SetRenderInfo(true,true,false,false,false);


















	//local refinement related
	//-------------------------------------------------------------------------------------
	//////back up for local refine
	////for (unsigned int i=0;i<this->vecDeformCurvePoint3d.size();i++)
	////{
	////	NewHandlePos.push_back(vecDeformCurvePoint3d.at(i));
	////	//testpoints.push_back(...);
	////}

	////OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//std::ofstream outNoLR("resultNoLR.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outNoLR,Mesh);

	//vector<Point_3> testCentroidPoint;
	//vector<Point_3> testmovedCentroidPoint;
	//vector<Facet_handle> testfhRefineTri;
	//DeformationLocalRefine(OldMesh,Mesh,OldHandlePos,NewHandlePos,this->dSquaredDistanceThreshold,
	//	this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices,
	//	this->vecTestPoint,testmovedCentroidPoint,testfhRefineTri);//testCentroidPoint

	////implement the real deformation
	//temp=this->vecHandleNbVertex;
	//temp.insert(temp.end(),this->ROIVertices.begin(),this->ROIVertices.end());
	//temp.insert(temp.end(),this->AnchorVertices.begin(),this->AnchorVertices.end());
	//GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
	//CDeformationAlgorithm::FlexibleDeform(dLamda,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
	//	this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d,true);
	//////recaculate the model
	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//Mesh.SetRenderInfo(true,true,true,true,true);

	//std::ofstream outLR("resultLR.obj",ios_base::out | ios_base::trunc);
	//print_polyhedron_wavefront(outLR,Mesh);

	//-------------------------------------------------------------------------------------
	//local refinement related

	this->vecHandlePoint.clear();
	this->vecDeformCurvePoint3d.clear();

	//refind the neighbor vertices
	//FindROIVertices(Mesh);

	return true;
}

void CMeshDeformation::DeformInterpolation(KW_Mesh& Mesh)
{
	if(this->vecDeformCurvePoint3d.empty())
		return;
	assert(this->vecDeformCurvePoint3d.size()==this->vecHandlePoint.size());

	if (!this->bHandleStrokeType)
	{
		ReOrderDeformCurve();
	}


	int iType=1;
	int iIterNum=5;

//	CDeformationAlgorithm::FlexibleLinearInterpolation(3,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
//														this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d);
//	CDeformationAlgorithm::FlexibleLaplacianInterpolation(3,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
//		this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d);
	CDeformationAlgorithm::FlexibleLambdaInterpolation(3,iType,iIterNum,Mesh,this->vecHandlePoint,this->vecHandleNbVertex,
		this->ROIVertices,this->AnchorVertices,this->vecDeformCurvePoint3d);

	this->vecHandlePoint.clear();
	this->vecDeformCurvePoint3d.clear();
}

void CMeshDeformation::ReOrderDeformCurve()
{
	Point_3 StartHandlePoint=this->vecHandlePoint.front().PointPos;
	double dMinDistance=9999.0;
	int iMinIndex=0;
	for (unsigned int i=0;i<this->vecDeformCurvePoint3d.size();i++)
	{
		double dCurrentDist=CGAL::squared_distance(this->vecDeformCurvePoint3d.at(i),StartHandlePoint);
		if (dCurrentDist<dMinDistance)
		{
			iMinIndex=i;
			dMinDistance=dCurrentDist;
		}
	}
	rotate(this->vecDeformCurvePoint3d.begin(),this->vecDeformCurvePoint3d.begin()+iMinIndex,
			this->vecDeformCurvePoint3d.end());

	int iReverseNum=0;
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		Point_3 CurrentHandlePoint=this->vecHandlePoint.at(i).PointPos;
		Point_3 CurrentDeformPoint=this->vecDeformCurvePoint3d.at(i);
		Point_3 ReverseDeformPoint=this->vecDeformCurvePoint3d.at(this->vecDeformCurvePoint3d.size()-1-i);
		if (CGAL::squared_distance(CurrentHandlePoint,CurrentDeformPoint)>
			CGAL::squared_distance(CurrentHandlePoint,ReverseDeformPoint))
		{
			iReverseNum++;
		}
	}
	if (iReverseNum>(int)this->vecDeformCurvePoint3d.size()/2)
	{
		reverse(this->vecDeformCurvePoint3d.begin(),this->vecDeformCurvePoint3d.end());
	}
}

void CMeshDeformation::TranslateDeformCurvePoint3dOnBFPlane(double dTransX,double dTransY)
{
	vector<Point_3> NewDeformCurvePoint3d;
	if (this->vecDeformCurvePoint3d.empty())
	{
		for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
		{
			NewDeformCurvePoint3d.push_back(vecHandlePoint.at(i).PointPos);
		}
	}
	else
	{
		NewDeformCurvePoint3d=this->vecDeformCurvePoint3d;
	}

	if (this->bHandleStrokeType)
	{
		Point_3 StartPoint=NewDeformCurvePoint3d.front();
		Point_3 StartPointProj=this->BestFittingPlane.projection(StartPoint);
		Point_3 EndPoint=NewDeformCurvePoint3d.back();
		Point_3 EndPointProj=this->BestFittingPlane.projection(EndPoint);
		Point_3 MidPoint((StartPointProj.x()+EndPointProj.x())/2,(StartPointProj.y()+EndPointProj.y())/2,
			(StartPointProj.z()+EndPointProj.z())/2);
		/*
				RotatedEndPointProj


		StartPointProj   MidPoint    EndPointProj
		*/
		GeometryAlgorithm compute;
		Point_3 RotatedEndPointProj=EndPointProj;
		compute.ComputeRotatedPoint(MidPoint,MidPoint+this->BestFittingPlane.orthogonal_vector(),90,RotatedEndPointProj);

		Vector_3 vTransX=(StartPointProj-MidPoint)*dTransX;
		Vector_3 vTransY=(RotatedEndPointProj-MidPoint)*dTransY;

		for (unsigned int i=0;i<NewDeformCurvePoint3d.size();i++)
		{
			NewDeformCurvePoint3d.at(i)=NewDeformCurvePoint3d.at(i)+vTransX+vTransY;
		}
	}
	else
	{
		vector<Point_3> ProjPoints;
		for (unsigned int i=0;i<NewDeformCurvePoint3d.size();i++)
		{
			ProjPoints.push_back(this->BestFittingPlane.projection(NewDeformCurvePoint3d.at(i)));
		}
		Point_3 ProjCentroid=CGAL::centroid(ProjPoints.begin(),ProjPoints.end());
		for (unsigned int i=0;i<ProjPoints.size();i++)
		{
			Vector_3 CurrentVec=ProjPoints.at(i)-ProjCentroid;
			NewDeformCurvePoint3d.at(i)=NewDeformCurvePoint3d.at(i)+CurrentVec*dTransX;
		}
	}

	this->vecDeformCurvePoint3d=NewDeformCurvePoint3d;
}

void CMeshDeformation::ComputeDualMesh(KW_Mesh& PrimalMesh)
{
	this->DualMesh.clear();
	this->vecDualHandle.clear();
	this->vecDualROI.clear();
	this->vecDualAnchor.clear();
	CDualMeshDeform::BuildDualMesh(PrimalMesh,this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices,this->DualMesh,
		this->vecDualHandle,this->vecDualROI,this->vecDualAnchor);

	//export dual mesh
	OBJHandle::UnitizeCGALPolyhedron(this->DualMesh,false,false);
	std::ofstream out("DualMesh.obj",ios_base::out | ios_base::trunc);
	print_polyhedron_wavefront(out,this->DualMesh);


	//this->DualMeshTest.clear();
	//CDualMeshDeform::GetDualMeshTest(PrimalMesh,this->DualMeshTest);
}

void CMeshDeformation::ComputeEdgeMesh(KW_Mesh& PrimalMesh)
{
	this->EdgeMesh.clear();
//	CWedgeEdgeBasedDeform::GetEdgeDomainMesh(PrimalMesh,this->EdgeMesh);
	KW_Mesh WedgeEdgeMesh;
	CWedgeEdgeBasedDeform::BuildWedgeEdgeMesh(PrimalMesh,this->WedgeEdgeMesh);
	OBJHandle::UnitizeCGALPolyhedron(this->WedgeEdgeMesh,false,false);
	std::ofstream out("WedgeEdgeMesh.obj",ios_base::out | ios_base::trunc);
	print_polyhedron_wavefront(out,this->WedgeEdgeMesh);
}

void CMeshDeformation::SetAutoUniMat()
{
	if (this->vecHandleNbVertex.empty())
	{
		return;
	}
	//compute material
	CMatConsDeform::SetUniformMaterial(this->dMaterial,this->pDoc->GetMesh(),this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices);
	//set color for each vertex
	CMatConsDeform::SetMatColor(this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices);
	pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
}

void CMeshDeformation::SetAutoHarMat()
{
	if (this->vecHandleNbVertex.empty())
	{
		return;
	}

	//compute material
	int iType=1;//uniform laplacian weighting scheme
	CMatConsDeform::SetHarmonicMaterial(iType,this->pDoc->GetMesh(),this->vecHandlePoint,
		this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices);
	//set color for each vertex
	CMatConsDeform::SetMatColor(this->vecHandleNbVertex,this->ROIVertices,this->AnchorVertices);
	pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
}

void CMeshDeformation::LearnMat()
{
	////if not source model exists, return
	//if (pDoc->GetMesh().empty())
	//{
	//	return;
	//}
	////read get stemplate model names
	//char fileBuffer[5010] = {0}, rBuffer[5010] = {0};
	//CFileDialog inDlg(TRUE, NULL, NULL,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT|OFN_ALLOWMULTISELECT, 
	//	_T("3D models(*.obj)|*.obj|All Files(*.*)|*.*||"), NULL);
	//inDlg.m_ofn.lpstrTitle = _T("Open Template Models");
	//inDlg.m_ofn.lpstrFile = fileBuffer;
	//inDlg.m_ofn.nMaxFile = 5000;

	//vector<CString> vecTmpName;
	//CString strFilePath;
	//if(inDlg.DoModal() == IDOK) 
	//{
	//	POSITION posFile=inDlg.GetStartPosition();
	//	while(posFile!=NULL)
	//	{
	//		vecTmpName.push_back(inDlg.GetNextPathName(posFile));
	//	}
	//}
	//if (vecTmpName.empty())
	//{
	//	return;
	//}

	////learn
	//CMatConsDeform::LearnMatFromSrc(1,this->pDoc->GetMesh(),vecTmpName);
	////set color for each vertex
	//CMatConsDeform::SetMatColor(this->pDoc->GetMesh());
	//pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
}

void CMeshDeformation::ExportMat()
{
	////if not source model exists, return
	//if (pDoc->GetMesh().empty())
	//{
	//	return;
	//}
	////get material file name
	//CFileDialog inDlg(FALSE, NULL, _T("Model.mate"),OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT, 
	//	_T("Model material(*.mate)|*.mate|All Files(*.*)|*.*||"), NULL);
	//inDlg.m_ofn.lpstrTitle = _T("Save Material Models");

	//CString strFilePath;
	//if(inDlg.DoModal() == IDOK) 
	//{
	//	strFilePath=inDlg.GetPathName();
	//}
	//else
	//{
	//	return;
	//}
	//CMatConsDeform::ExportMat(this->pDoc->GetMesh(),strFilePath);
}

void CMeshDeformation::ImportMat()
{
	////if not source model exists, return
	//if (pDoc->GetMesh().empty())
	//{
	//	return;
	//}
	////get material file name
	//CFileDialog inDlg(TRUE, NULL, _T("Model.mate"),OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT, 
	//	_T("Model material(*.mate)|*.mate|All Files(*.*)|*.*||"), NULL);
	//inDlg.m_ofn.lpstrTitle = _T("Open Material Models");

	//CString strFilePath;
	//if(inDlg.DoModal() == IDOK) 
	//{
	//	strFilePath=inDlg.GetPathName();
	//}
	//else
	//{
	//	return;
	//}
	//CMatConsDeform::ImportMat(this->pDoc->GetMesh(),strFilePath);
	////set color for each vertex
	//CMatConsDeform::SetMatColor(this->pDoc->GetMesh());
	//pDoc->GetMesh().SetRenderInfo(false,false,false,false,true);
}

void CMeshDeformation::Render(bool bSmoothView,GLenum mode,bool bShowDualMesh)
{
	SketchViewer* pView=this->pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();

	if (mode==GL_RENDER)
	{
		RenderRefSphere(bSmoothView);
		RenderRefPlane(bSmoothView,mode);
		RenderRefTangentialPlane(bSmoothView,mode);
		RenderRefBiNormalPlane(bSmoothView);
		RenderCurvePoint2D(modelview,projection,viewport);
		RenderHandleCurve(mode);
		//	RenderResultHandleCurve();
		RenderDeformCurve(mode);
		RenderDeformCurveProj();
		RenderHandleNb();
		RenderROI();
		RenderAnchor();

		RenderEdgeMesh();

		//	if (bShowDualMesh)
		//	{
		RenderDualMesh();
		//	}

		RenderTestPoint();
	}
	else if (mode==GL_SELECT)
	{
		RenderRefPlane(bSmoothView,mode);
		RenderRefTangentialPlane(bSmoothView,mode);
		RenderHandleCurve(mode);
		RenderDeformCurve(mode);
	}
}

void CMeshDeformation::RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport)
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

void CMeshDeformation::RenderResultHandleCurve()
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}
	glDisable(GL_LIGHTING);
	glPointSize(5);
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		double dNewPos[3];
		dNewPos[0]=dNewPos[1]=dNewPos[2]=0;

		for (unsigned int j=0;j<vecHandlePoint.at(i).vecVertexIndex.size();j++)
		{
			dNewPos[0]=dNewPos[0]+
				vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().x()*vecHandlePoint.at(i).vecPara.at(j);
			dNewPos[1]=dNewPos[1]+
				vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().y()*vecHandlePoint.at(i).vecPara.at(j);
			dNewPos[2]=dNewPos[2]+
				vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().z()*vecHandlePoint.at(i).vecPara.at(j);
		}

		Point_3 NewPoint(dNewPos[0],dNewPos[1],dNewPos[2]);

		glBegin(GL_POINTS);
		glColor3f(0,1,1);
		{
			glVertex3d(NewPoint.x(),NewPoint.y(),NewPoint.z());
		}
		glEnd();
	}
	glPointSize(1);
	glEnable(GL_LIGHTING);
}

void CMeshDeformation::RenderHandleCurve(GLenum mode)
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}

	glDisable(GL_LIGHTING);
	
	if (mode==GL_RENDER && this->pDoc->GetRBSelName()==DEFORMATION_HANDLE_CURVE)
	{
		glColor4fv(TRANSPARENT_SELECTED_COLOR);
	}
	else
	{
		glColor3f(0,0,1);
	}
	if (mode==GL_SELECT)
	{
		glPushName(DEFORMATION_HANDLE_CURVE);
	}

	glLineWidth(6.0);
	for (unsigned int i=0;i<this->vecHandlePoint.size();i++)
	{
		Point_3 CurrentPoint,NextPoint;
		if (i==this->vecHandlePoint.size()-1)
		{
			if (bHandleStrokeType)
			{
				break;
			}
			else
			{
				CurrentPoint=this->vecHandlePoint.back().PointPos;
				NextPoint=this->vecHandlePoint.front().PointPos;
			}
		}
		else
		{
			CurrentPoint=this->vecHandlePoint.at(i).PointPos;
			NextPoint=this->vecHandlePoint.at(i+1).PointPos;
		}
		//glPointSize(5);
		//glBegin(GL_POINTS);
		//glColor3f(0,0,1);
		//glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
		//glEnd();

		glBegin(GL_LINES);
		{
			glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
			glVertex3d(NextPoint.x(),NextPoint.y(),NextPoint.z());
		}
		glEnd();
	}
	glLineWidth(1.0);		

	glPointSize(5);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex3f(this->vecHandlePoint.front().PointPos.x(),
		this->vecHandlePoint.front().PointPos.y(),
		this->vecHandlePoint.front().PointPos.z());
	glEnd();
	glPointSize(1);

	if (mode==GL_SELECT)
	{
		glPopName();
	}

	glEnable(GL_LIGHTING);
}

void CMeshDeformation::RenderDeformCurve(GLenum mode)
{
	//draw user's deform stroke(3d)
	if (this->vecDeformCurvePoint3d.empty())
	{
		return;
	}

	//ignore this case
	if (this->vecDeformCurvePoint3d.size()==1 && this->vecHandleNbVertex.size()==1)//if handle point instead of curve
	{
		glDisable(GL_LIGHTING);

		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(1,0,1);
		glVertex3f(this->vecDeformCurvePoint3d.front().x(),this->vecDeformCurvePoint3d.front().y(),
			this->vecDeformCurvePoint3d.front().z());
		glEnd();
		glPointSize(1);

		glLineWidth(3.0);
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1,0x00FF);
		glBegin(GL_LINES);
		glColor3f(0,1,0);
		{
			glVertex3d(this->vecDeformCurvePoint3d.front().x(),this->vecDeformCurvePoint3d.front().y(),this->vecDeformCurvePoint3d.front().z());
			glVertex3d(this->vecHandleNbVertex.front()->point().x(),this->vecHandleNbVertex.front()->point().y(),this->vecHandleNbVertex.front()->point().z());
		}
		glEnd();
		glDisable(GL_LINE_STIPPLE);
		glLineWidth(1.0);		

		glEnable(GL_LIGHTING);
		return;
	}

	glDisable(GL_LIGHTING);

	if (mode==GL_RENDER && this->pDoc->GetRBSelName()==DEFORMATION_DEFORM_CURVE)
	{
		glColor4fv(TRANSPARENT_SELECTED_COLOR);
	}
	else
	{
		glColor3f(0,1,0);
	}
	if (mode==GL_SELECT)
	{
		glPushName(DEFORMATION_DEFORM_CURVE);
	}

	glLineWidth(3.0);
	for (unsigned int i=0;i<this->vecDeformCurvePoint3d.size();i++)
	{
		Point_3 CurrentPoint,NextPoint;
		if (i==this->vecDeformCurvePoint3d.size()-1)
		{
			if (this->bHandleStrokeType)
			{
				break;
			}
			else
			{
				CurrentPoint=this->vecDeformCurvePoint3d.back();
				NextPoint=this->vecDeformCurvePoint3d.front();
			}
		}
		else
		{
			CurrentPoint=this->vecDeformCurvePoint3d.at(i);
			NextPoint=this->vecDeformCurvePoint3d.at(i+1);
		}
		glBegin(GL_LINES);
		{
			glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
			glVertex3d(NextPoint.x(),NextPoint.y(),NextPoint.z());
		}
		glEnd();
	}
	glLineWidth(1.0);		

	glPointSize(5);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex3f(this->vecDeformCurvePoint3d.front().x(),this->vecDeformCurvePoint3d.front().y(),
		this->vecDeformCurvePoint3d.front().z());
	glEnd();
	glPointSize(1);

	if (mode==GL_SELECT)
	{
		glPopName();
	}

	glEnable(GL_LIGHTING);
}

void CMeshDeformation::RenderDeformCurveProj()
{
	//draw user's deform stroke projection(3d)
	if (this->vecDeformCurveProjPoint3d.empty())
	{
		return;
	}

	glDisable(GL_LIGHTING);

	glLineWidth(3.0);
	for (unsigned int i=0;i<this->vecDeformCurveProjPoint3d.size();i++)
	{
		Point_3 CurrentPoint,NextPoint;
		if (i==this->vecDeformCurveProjPoint3d.size()-1)
		{
			if (this->bHandleStrokeType)
			{
				break;
			}
			else
			{
				CurrentPoint=this->vecDeformCurveProjPoint3d.back();
				NextPoint=this->vecDeformCurveProjPoint3d.front();
			}
		}
		else
		{
			CurrentPoint=this->vecDeformCurveProjPoint3d.at(i);
			NextPoint=this->vecDeformCurveProjPoint3d.at(i+1);
		}
		glBegin(GL_LINES);
		glColor3f(1,1,0);
		{
			glVertex3d(CurrentPoint.x(),CurrentPoint.y(),CurrentPoint.z());
			glVertex3d(NextPoint.x(),NextPoint.y(),NextPoint.z());
		}
		glEnd();
	}
	glLineWidth(1.0);		

	glPointSize(5);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex3f(this->vecDeformCurveProjPoint3d.front().x(),
		this->vecDeformCurveProjPoint3d.front().y(),
		this->vecDeformCurveProjPoint3d.front().z());
	glEnd();
	glPointSize(1);

	glEnable(GL_LIGHTING);
}

void CMeshDeformation::RenderRefPlane(bool bSmoothView,GLenum mode)
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}
	if (!this->bRenderRefPlane[0])
	{
		return;
	}

	glLineWidth(2);

	//draw the frame of the plane
	if (mode==GL_RENDER)
	{
		glDisable(GL_LIGHTING);

		if (pDoc->GetRBSelName()==DEFORMATION_NORM_PLANE)
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
//	glColor4f(0.0f,0.0f,1.0f,0.2f);			// Full Brightness, 50% Alpha
	const  GLfloat blue_color[] = {0.0f, 0.0f, 1.0f, 0.2f};
	const GLfloat mat_blue_emission[] = {0.0f, 0.0f, 1.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , blue_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_blue_emission);
	
	if (pDoc->GetRBSelName()==DEFORMATION_NORM_PLANE)
	{
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , TRANSPARENT_SELECTED_COLOR);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  OPAQUE_SELECTED_COLOR);
	}
	if (mode==GL_SELECT)
	{
		glPushName(DEFORMATION_NORM_PLANE);
	}

	glClear(GL_DEPTH_BUFFER_BIT);

//	glEnable(GL_DEPTH_TEST);
	glDepthMask(FALSE);

	glBegin(GL_QUADS);
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
//	glEnable(GL_DEPTH_TEST);

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

void CMeshDeformation::RenderRefTangentialPlane(bool bSmoothView,GLenum mode)
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}
	if (!this->bRenderRefPlane[1])
	{
		return;
	}
	glLineWidth(2);

	//draw the frame of the plane
	if (mode==GL_RENDER)
	{
		glDisable(GL_LIGHTING);

		if (this->pDoc->GetRBSelName()==DEFORMATION_TAN_PLANE)
		{
			glColor4fv(OPAQUE_SELECTED_COLOR);
		}
		else
		{
			glColor3f(0,1,0);
		}

		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
		glBegin(GL_QUADS);
		glVertex3d(this->RefTangentialPlaneBoundaryPoints[0].x,this->RefTangentialPlaneBoundaryPoints[0].y,this->RefTangentialPlaneBoundaryPoints[0].z);
		glVertex3d(this->RefTangentialPlaneBoundaryPoints[1].x,this->RefTangentialPlaneBoundaryPoints[1].y,this->RefTangentialPlaneBoundaryPoints[1].z);
		glVertex3d(this->RefTangentialPlaneBoundaryPoints[2].x,this->RefTangentialPlaneBoundaryPoints[2].y,this->RefTangentialPlaneBoundaryPoints[2].z);
		glVertex3d(this->RefTangentialPlaneBoundaryPoints[3].x,this->RefTangentialPlaneBoundaryPoints[3].y,this->RefTangentialPlaneBoundaryPoints[3].z);
		glEnd();

		glEnable(GL_LIGHTING);
	}

	//draw the transparent face of the plane(2 faces)
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
//	glColor4f(0.0f,1.0f,0.0f,0.2f);			// Full Brightness, 50% Alpha
	const GLfloat green_color[] = {0.0f, 1.0f, 0.0f, 0.2f};
	const GLfloat mat_green_emission[] = {0.0f, 1.0f, 0.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, green_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_green_emission);

	if (pDoc->GetRBSelName()==DEFORMATION_TAN_PLANE)
	{
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , TRANSPARENT_SELECTED_COLOR);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  OPAQUE_SELECTED_COLOR);
	}
	if (mode==GL_SELECT)
	{
		glPushName(DEFORMATION_TAN_PLANE);
	}

	glClear(GL_DEPTH_BUFFER_BIT);

//	glDisable(GL_DEPTH_TEST);
	glDepthMask(FALSE);


	glBegin(GL_QUADS);
	//				glColor3f(0,1,0);
	glNormal3f(this->RefTangentialPlane.orthogonal_vector().x(),
		this->RefTangentialPlane.orthogonal_vector().y(),
		this->RefTangentialPlane.orthogonal_vector().z());
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[0].x,this->RefTangentialPlaneBoundaryPoints[0].y,this->RefTangentialPlaneBoundaryPoints[0].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[1].x,this->RefTangentialPlaneBoundaryPoints[1].y,this->RefTangentialPlaneBoundaryPoints[1].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[2].x,this->RefTangentialPlaneBoundaryPoints[2].y,this->RefTangentialPlaneBoundaryPoints[2].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[3].x,this->RefTangentialPlaneBoundaryPoints[3].y,this->RefTangentialPlaneBoundaryPoints[3].z);

	glNormal3f(this->RefTangentialPlane.opposite().orthogonal_vector().x(),
		this->RefTangentialPlane.opposite().orthogonal_vector().y(),
		this->RefTangentialPlane.opposite().orthogonal_vector().z());
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[3].x,this->RefTangentialPlaneBoundaryPoints[3].y,this->RefTangentialPlaneBoundaryPoints[3].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[2].x,this->RefTangentialPlaneBoundaryPoints[2].y,this->RefTangentialPlaneBoundaryPoints[2].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[1].x,this->RefTangentialPlaneBoundaryPoints[1].y,this->RefTangentialPlaneBoundaryPoints[1].z);
	glVertex3d(this->RefTangentialPlaneBoundaryPoints[0].x,this->RefTangentialPlaneBoundaryPoints[0].y,this->RefTangentialPlaneBoundaryPoints[0].z);
	glEnd();

	glDepthMask(TRUE);
//	glEnable(GL_DEPTH_TEST);

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

void CMeshDeformation::RenderRefBiNormalPlane(bool bSmoothView)
{
	if (this->vecHandlePoint.empty())
	{
		return;
	}
	if (!this->bRenderRefPlane[2])
	{
		return;
	}
	glLineWidth(2);

	//draw the frame of the plane
	glDisable(GL_LIGHTING);

	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
	glBegin(GL_QUADS);
	glColor3f(1,1,0);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[0].x,this->RefBiNormalPlaneBoundaryPoints[0].y,this->RefBiNormalPlaneBoundaryPoints[0].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[1].x,this->RefBiNormalPlaneBoundaryPoints[1].y,this->RefBiNormalPlaneBoundaryPoints[1].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[2].x,this->RefBiNormalPlaneBoundaryPoints[2].y,this->RefBiNormalPlaneBoundaryPoints[2].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[3].x,this->RefBiNormalPlaneBoundaryPoints[3].y,this->RefBiNormalPlaneBoundaryPoints[3].z);
	glEnd();

	glEnable(GL_LIGHTING);

	//draw the transparent face of the plane(2 faces)
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
//	glColor4f(1.0f,1.0f,0.0f,0.2f);			// Full Brightness, 50% Alpha
	const GLfloat yellow_color[] = {1.0f, 1.0f, 0.0f, 0.2f};
	const GLfloat mat_yellow_emission[] = {1.0f, 1.0f, 0.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, yellow_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_yellow_emission);


	glClear(GL_DEPTH_BUFFER_BIT);
	//glDisable(GL_DEPTH_TEST);
	glDepthMask(FALSE);

	glBegin(GL_QUADS);
	//				glColor3f(0,1,0);
	glNormal3f(this->RefBiNormalPlane.orthogonal_vector().x(),
		this->RefBiNormalPlane.orthogonal_vector().y(),
		this->RefBiNormalPlane.orthogonal_vector().z());
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[0].x,this->RefBiNormalPlaneBoundaryPoints[0].y,this->RefBiNormalPlaneBoundaryPoints[0].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[1].x,this->RefBiNormalPlaneBoundaryPoints[1].y,this->RefBiNormalPlaneBoundaryPoints[1].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[2].x,this->RefBiNormalPlaneBoundaryPoints[2].y,this->RefBiNormalPlaneBoundaryPoints[2].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[3].x,this->RefBiNormalPlaneBoundaryPoints[3].y,this->RefBiNormalPlaneBoundaryPoints[3].z);

	glNormal3f(this->RefBiNormalPlane.opposite().orthogonal_vector().x(),
		this->RefBiNormalPlane.opposite().orthogonal_vector().y(),
		this->RefBiNormalPlane.opposite().orthogonal_vector().z());
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[3].x,this->RefBiNormalPlaneBoundaryPoints[3].y,this->RefBiNormalPlaneBoundaryPoints[3].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[2].x,this->RefBiNormalPlaneBoundaryPoints[2].y,this->RefBiNormalPlaneBoundaryPoints[2].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[1].x,this->RefBiNormalPlaneBoundaryPoints[1].y,this->RefBiNormalPlaneBoundaryPoints[1].z);
	glVertex3d(this->RefBiNormalPlaneBoundaryPoints[0].x,this->RefBiNormalPlaneBoundaryPoints[0].y,this->RefBiNormalPlaneBoundaryPoints[0].z);
	glEnd();

	glDepthMask(TRUE);
	//glEnable(GL_DEPTH_TEST);

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

void CMeshDeformation::RenderRefSphere(bool bSmoothView)
{
	if (this->vecHandleNbVertex.size()!=1)
	{
		return;
	}
	if (!this->bRenderSphere)
	{
		return;
	}

	//draw the transparent face of the plane(2 faces)
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
	const static GLfloat yellow_color[] = {1.0f, 1.0f, 0.0f, 0.3f};
	const GLfloat mat_yellow_emission[] = {1.0f, 1.0f, 0.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, yellow_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_yellow_emission);

	//glDisable(GL_DEPTH_TEST);
	glDepthMask(FALSE);

	GLUquadricObj* SphereToRender=gluNewQuadric();
	glPushMatrix();
	gluQuadricDrawStyle(SphereToRender,GLU_FILL);
	//gluQuadricDrawStyle(SphereToRender,GLU_SILHOUETTE);
	glTranslatef(this->RefSphere.center().x(),this->RefSphere.center().y(),this->RefSphere.center().z());
	gluSphere(SphereToRender,sqrt(this->RefSphere.squared_radius()),40.0,40.0);
	glPopMatrix();
	gluDeleteQuadric(SphereToRender);

	glDepthMask(TRUE);
 	//glEnable(GL_DEPTH_TEST);
	
	if (bSmoothView)
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
	} 
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
	}
}

void CMeshDeformation::RenderHandleNb()
{
	if (this->vecHandleNbVertex.size()==1)//if handle point instead of curve
	{
		glDisable(GL_LIGHTING);
		glPointSize(8);
		glBegin(GL_POINTS);
		glColor3f(0,0,1);
		glVertex3f(this->vecHandleNbVertex.front()->point().x(),this->vecHandleNbVertex.front()->point().y(),
			this->vecHandleNbVertex.front()->point().z());
		glEnd();
		glPointSize(1);
		glEnable(GL_LIGHTING);
		return;
	}
	if ((!this->vecHandleNbVertex.empty())&&bRenderHandleNb)
	{
		glDisable(GL_LIGHTING);
		for (unsigned int i=0;i<this->vecHandleNbVertex.size();i++)
		{
			glPointSize(5);
			glBegin(GL_POINTS);
			glColor3f(1,1,0);
			glVertex3f(this->vecHandleNbVertex.at(i)->point().x(),this->vecHandleNbVertex.at(i)->point().y(),
				this->vecHandleNbVertex.at(i)->point().z());
			glEnd();
			glPointSize(1);
		}
		glEnable(GL_LIGHTING);
	}
}

void CMeshDeformation::RenderROI()
{
	if ((!this->ROIVertices.empty())&&bRenderROI)
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

void CMeshDeformation::RenderAnchor()
{
	if ((!this->AnchorVertices.empty())&&bRenderAnchor)
	{
		glDisable(GL_LIGHTING);
		for (unsigned int i=0;i<this->AnchorVertices.size();i++)
		{
			glPointSize(5);
			glBegin(GL_POINTS);
			glColor3f(0,1,0);
			//glColor3f(1,0,0);
			glVertex3f(this->AnchorVertices.at(i)->point().x(),this->AnchorVertices.at(i)->point().y(),
				this->AnchorVertices.at(i)->point().z());
			glEnd();
			glPointSize(1);
		}
		glEnable(GL_LIGHTING);
	}
}

void CMeshDeformation::RenderTestPoint()
{
	float fStep=1.0/this->vecTestPoint.size();
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->vecTestPoint.size();i++)
	{
		glPointSize(5);
		glBegin(GL_POINTS);
//		glColor3f(0,0,1);
		glColor3f(0,0+(i+1)*fStep,0+(i+1)*fStep);
		glVertex3f(this->vecTestPoint.at(i).x(),this->vecTestPoint.at(i).y(),
			this->vecTestPoint.at(i).z());
		glEnd();
		glPointSize(1);
	}
	glEnable(GL_LIGHTING);
}

void CMeshDeformation::RenderEdgeMesh()
{
	//if (this->EdgeMesh.empty())
	//{
	//	return;
	//}

	//glDisable(GL_LIGHTING);

	//map<Point_3,vector<Point_3>>::iterator Iter;
	//for (Iter=EdgeMesh.begin();Iter!=EdgeMesh.end();Iter++)
	//{
	//	for (unsigned int i=0;i<Iter->second.size();i++)
	//	{
	//		glBegin(GL_LINES);
	//		glLineWidth(3);
	//		glColor3f(1,0,1);
	//		glVertex3f(Iter->first.x(),Iter->first.y(),Iter->first.z());
	//		glVertex3f(Iter->second.at(i).x(),Iter->second.at(i).y(),Iter->second.at(i).z());
	//		glEnd();
	//	}
	//}

	//glLineWidth(1);
	//glEnable(GL_LIGHTING);

	if (this->WedgeEdgeMesh.empty())
	{
		return;
	}
	glDisable(GL_LIGHTING);
	for (Vertex_iterator i=this->WedgeEdgeMesh.vertices_begin();i!=this->WedgeEdgeMesh.vertices_end();i++)
	{
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			glBegin(GL_LINES);
			glLineWidth(3);
//			glColor3f(1,1,0);
			glColor3f(0,0,1);
			glVertex3f(i->point().x(),i->point().y(),i->point().z());
			glVertex3f(Havc->opposite()->vertex()->point().x(),
				Havc->opposite()->vertex()->point().y(),
				Havc->opposite()->vertex()->point().z());
			glEnd();
			Havc++;
		} while(Havc!=i->vertex_begin());
	}
	glEnable(GL_LIGHTING);

}

void CMeshDeformation::RenderDualMesh()
{
	//if (this->DualMeshTest.empty())
	//{
	//	return;
	//}

	//glDisable(GL_LIGHTING);

	//map<Point_3,vector<Point_3>>::iterator Iter;
	//for (Iter=DualMeshTest.begin();Iter!=DualMeshTest.end();Iter++)
	//{
	//	for (unsigned int i=0;i<Iter->second.size();i++)
	//	{
	//		glBegin(GL_LINES);
	//		glLineWidth(3);
	//		glColor3f(1,0,1);
	//		glVertex3f(Iter->first.x(),Iter->first.y(),Iter->first.z());
	//		glVertex3f(Iter->second.at(i).x(),Iter->second.at(i).y(),Iter->second.at(i).z());
	//		glEnd();
	//	}
	//}

	//glLineWidth(1);
	//glEnable(GL_LIGHTING);

	if (this->DualMesh.empty())
	{
		return;
	}
	glDisable(GL_LIGHTING);
	for (Vertex_iterator i=this->DualMesh.vertices_begin();i!=this->DualMesh.vertices_end();i++)
	{
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			glBegin(GL_LINES);
			glLineWidth(3);
			glColor3f(0,0,1);
			glVertex3f(i->point().x(),i->point().y(),i->point().z());
			glVertex3f(Havc->opposite()->vertex()->point().x(),
				Havc->opposite()->vertex()->point().y(),
				Havc->opposite()->vertex()->point().z());
			glEnd();
			Havc++;
		} while(Havc!=i->vertex_begin());
	}
	glEnable(GL_LIGHTING);
}

//double CMeshDeformation::RBFGetMultiQuadricSi(vector<Point_3> OldHandlePos)
//{
//	double dConstant=10;
//	return dConstant;
//	double dMinDist=9999.0;
//	for (unsigned int i=0;i<OldHandlePos.size();i++)
//	{
//		Point_3 CurrentPoint=OldHandlePos.at(i);
//		if (i==OldHandlePos.size()-1)
//		{
//			break;
//		}
//		for (unsigned int j=i+1;j<OldHandlePos.size();j++)
//		{
//			Point_3 NextPoint=OldHandlePos.at(j);
//			double dCurrentDist=sqrt(CGAL::squared_distance(CurrentPoint,NextPoint));
//			if (dCurrentDist<dMinDist)
//			{
//				dMinDist=dCurrentDist;
//			}
//		}
//	}
//	return dMinDist;
//}
//
//void CMeshDeformation::RBFGetCoefficient(vector<Point_3> OldHandlePos,vector<Point_3> NewHandlePos,
//										 vector<double>* Ci,double& Si)
//{
//	double dSi=RBFGetMultiQuadricSi(OldHandlePos);
//	Si=dSi;
//
//	vector<vector<double> > MatrixH;
//	for (unsigned int j=0;j<OldHandlePos.size();j++)
//	{
//		vector<double> H;
//		for (unsigned int k=0;k<OldHandlePos.size();k++)
//		{
//			//double dCurrent=sqrt(CGAL::squared_distance(OldHandlePos.at(j),OldHandlePos.at(k))
//			//	+dSi*dSi);
//			double dCurrent=CGAL::squared_distance(OldHandlePos.at(j),OldHandlePos.at(k));
//			dCurrent=exp(-dCurrent/(dSi*dSi));
//			
//			H.push_back(dCurrent);
//		}
//		MatrixH.push_back(H);
//	}
//
//	for (int i=0;i<3;i++)
//	{
//		vector<double> target;
//		switch(i)
//		{
//		case 0:
//			for (unsigned int j=0;j<NewHandlePos.size();j++)
//			{
////				target.push_back(NewHandlePos.at(j).x());
//				target.push_back(NewHandlePos.at(j).x()-OldHandlePos.at(j).x());
//			}
//			break;
//		case 1:
//			for (unsigned int j=0;j<NewHandlePos.size();j++)
//			{
////				target.push_back(NewHandlePos.at(j).y());
//				target.push_back(NewHandlePos.at(j).y()-OldHandlePos.at(j).y());
//			}
//			break;
//		case 2:
//			for (unsigned int j=0;j<NewHandlePos.size();j++)
//			{
////				target.push_back(NewHandlePos.at(j).z());
//				target.push_back(NewHandlePos.at(j).z()-OldHandlePos.at(j).z());
//			}
//		    break;
//		default:
//		    break;
//		}
//
//		GeometryAlgorithm::SolveLinearEquation(MatrixH,target,Ci[i]);
//	}
//}
//
//void CMeshDeformation::RBFGetNeighborNewPos(KW_Mesh Mesh,map<int,Point_3> NewDesiredPointPos,
//											vector<double>* Ci,double Si,vector<Point_3> OldHandlePos,
//											vector<Point_3> &NeighborNewPos)
//{
//	map<int,vector<int>>::iterator Iter=this->DesiredNeighborVertex.begin();
//	//for every neighbor point
//	while (Iter!=this->DesiredNeighborVertex.end())
//	{
//		Vertex_iterator temp=Mesh.vertices_begin();
//		for (int i=0;i<Iter->first;i++)
//		{
//			temp++;
//		}
//		Point_3 OriginalNeighborPos(temp->point().x(),
//			temp->point().y(),
//			temp->point().z());
//
//		double NewValue[3];
//		for (int i=0;i<3;i++)
//		{
//			NewValue[i]=0.0;
//			for (unsigned int j=0;j<OldHandlePos.size();j++)
//			{
//				//NewValue[i]=NewValue[i]+Ci[i].at(j)*sqrt(CGAL::squared_distance(OriginalNeighborPos,OldHandlePos.at(j))
//				//							+Si*Si);
//				double dDistance=CGAL::squared_distance(OriginalNeighborPos,OldHandlePos.at(j));
//				NewValue[i]=NewValue[i]+Ci[i].at(j)*exp(-dDistance/(Si*Si));
//			}
//		}
//
////		Point_3 NewNeighborPos(NewValue[0],NewValue[1],NewValue[2]);
//		Vector_3 vec(NewValue[0],NewValue[1],NewValue[2]);
//		Point_3 NewNeighborPos=OriginalNeighborPos+vec;
//		NeighborNewPos.push_back(NewNeighborPos);
//
//		Iter++;
//	}
//}
//
//void CMeshDeformation::CheckRBF(KW_Mesh Mesh,vector<double>* Ci,double Si,
//								vector<Point_3> OldHandlePos,vector<Point_3>& NewHandlePos)
//{
//	NewHandlePos.clear();
//	for (unsigned int i=0;i<OldHandlePos.size();i++)
//	{
//		double NewValue[3];
//		for (int k=0;k<3;k++)
//		{
//			NewValue[k]=0.0;
//			for (unsigned int j=0;j<OldHandlePos.size();j++)
//			{
//				double dDistance=CGAL::squared_distance(OldHandlePos.at(i),OldHandlePos.at(j));
//				NewValue[k]=NewValue[k]+Ci[k].at(j)*exp(-dDistance/(Si*Si));
//			}
//		}
////		Point_3 ComputedPoint(NewValue[0],NewValue[1],NewValue[2]);
////		NewHandlePos.push_back(ComputedPoint);
//		Vector_3 vec(NewValue[0],NewValue[1],NewValue[2]);
//		Point_3 ComputedPoint=OldHandlePos.at(i)+vec;
//		NewHandlePos.push_back(ComputedPoint);
//	}
//}