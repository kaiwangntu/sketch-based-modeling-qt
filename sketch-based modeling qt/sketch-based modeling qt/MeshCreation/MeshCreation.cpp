#include "MeshCreation.h"
#include "CrossSectionProc.h"
#include "../sketchviewer.h"
#include "../sketchinterface.h"
#include "../PaintingOnMesh.h"
#include <QMessageBox>
//#include "../OBJHandle.h"
//#include "../ControlPanel/ControlPanel.h"
//#include "ImplicitSurface/ImplicitMesher.h"

CMeshCreation::CMeshCreation(void)
{
//	this->manager=NULL;
//	this->kwcs2surf=NULL;
}

CMeshCreation::~CMeshCreation(void)
{
//	this->manager=NULL;
//	delete this->manager;
//	this->kwcs2surf=NULL;
//	delete this->kwcs2surf;
}

void CMeshCreation::Init(SketchDoc* pDataIn)
{
	this->pDoc=pDataIn;

	RefPlane[0]=Plane_3(0,0,1,0);
	PlaneBoundaryPoints[0][0]=Point_3(-1*CREATION_PLANE_BOUND_SIZE,1*CREATION_PLANE_BOUND_SIZE,0);PlaneBoundaryPoints[0][1]=Point_3(-1*CREATION_PLANE_BOUND_SIZE,-1*CREATION_PLANE_BOUND_SIZE,0);
	PlaneBoundaryPoints[0][2]=Point_3(1*CREATION_PLANE_BOUND_SIZE,-1*CREATION_PLANE_BOUND_SIZE,0);PlaneBoundaryPoints[0][3]=Point_3(1*CREATION_PLANE_BOUND_SIZE,1*CREATION_PLANE_BOUND_SIZE,0);
	RefPlane[1]=Plane_3(0,1,0,0);
	PlaneBoundaryPoints[1][0]=Point_3(-1*CREATION_PLANE_BOUND_SIZE,0,-1*CREATION_PLANE_BOUND_SIZE);PlaneBoundaryPoints[1][1]=Point_3(-1*CREATION_PLANE_BOUND_SIZE,0,1*CREATION_PLANE_BOUND_SIZE);
	PlaneBoundaryPoints[1][2]=Point_3(1*CREATION_PLANE_BOUND_SIZE,0,1*CREATION_PLANE_BOUND_SIZE);PlaneBoundaryPoints[1][3]=Point_3(1*CREATION_PLANE_BOUND_SIZE,0,-1*CREATION_PLANE_BOUND_SIZE);
	RefPlane[2]=Plane_3(1,0,0,0);
	PlaneBoundaryPoints[2][0]=Point_3(0,1*CREATION_PLANE_BOUND_SIZE,1*CREATION_PLANE_BOUND_SIZE);PlaneBoundaryPoints[2][1]=Point_3(0,-1*CREATION_PLANE_BOUND_SIZE,1*CREATION_PLANE_BOUND_SIZE);
	PlaneBoundaryPoints[2][2]=Point_3(0,-1*CREATION_PLANE_BOUND_SIZE,-1*CREATION_PLANE_BOUND_SIZE);PlaneBoundaryPoints[2][3]=Point_3(0,1*CREATION_PLANE_BOUND_SIZE,-1*CREATION_PLANE_BOUND_SIZE);

	this->bRenderRefPlane[0]=this->bRenderRefPlane[1]=this->bRenderRefPlane[2]=true;
	this->bRenderCN=true;
	this->bRenderOnlyUserSketch=true;

	this->bAutoRot=false;

	this->iSelectedCN=NONE_SELECTED;
	this->iSelectedCS=NONE_SELECTED;

	this->iDrawingProfilePlane=NONE_SELECTED;

	this->bPlaneReadyState=CREATION_PLANE_NOT_READY_FOR_DRAWING;

	this->vecCurveNetwork.clear();
	this->vecCurvePlaneIntersectPoint.clear();
	this->vecCurvePlaneIntersectType.clear();

	this->vecComputedCS.clear();

	this->MeshBoundingProfile3D.clear();
	this->bCurvesLeftToFit=false;

	this->iSurfReconstAlgorithm=TaoJuSurfReconstAlgo;//ProgSurfReconstAlgo

	//if (manager!=NULL)
	//{
	//	manager=NULL;
	//	delete manager;
	//}
	//manager= new Ctr2SufManager();

	//if (this->kwcs2surf!=NULL)
	//{
	//	this->kwcs2surf=NULL;
	//	delete this->kwcs2surf;
	//}
	//this->kwcs2surf=new KW_CS2Surf();

	this->vecTestPoint.clear();

	////init control panel
	//CControlPanel* pCP=(CControlPanel*)(pDoc->GetView(RUNTIME_CLASS(CControlPanel)));
	//if (pCP->GetCPCreation()!=NULL)
	//{
	//	pCP->GetCPCreation()->Init();
	//}
}

void CMeshCreation::SetDrawingPlane()
{
	int iSelectedName=this->pDoc->GetRBSelName();
	//plane selected
	if ((iSelectedName>=CREATION_PLANE_NAME_BEGIN)&&(iSelectedName<=CREATION_PLANE_NAME_END))
	{
		this->iDrawingProfilePlane=iSelectedName;
		//check if there exists CN on this plane,if yes, selecting the plane means selecting the CN
		this->iSelectedCN=NONE_SELECTED;
		for (unsigned int iCN=0;iCN<this->vecCurveNetwork.size();iCN++)
		{
			if (this->vecCurveNetwork.at(iCN).ProfilePlaneType==this->iDrawingProfilePlane)
			{
				if (this->vecCurveNetwork.at(iCN).plane==this->RefPlane[this->iDrawingProfilePlane])
				{
					this->iSelectedCN=iCN;
					break;
				}
			}
		}
		this->iSelectedCS=NONE_SELECTED;
		if (this->iSelectedCN==NONE_SELECTED && !this->pDoc->GetMesh().empty())
		{
			//get the intersection between plane and mesh
			this->vecComputedCS.clear();
			//error: remove temporarily, since this function has error
			//GeometryAlgorithm::GetMeshPlaneIntersection(this->pDoc->GetMesh(),this->RefPlane[this->iDrawingProfilePlane],
			//	this->vecComputedCS);
		}
	}
	//sketched curve on plane selected
	else if ((iSelectedName>=CREATION_SKETCH_CURVE_NAME_BEGIN)&&(iSelectedName<=CREATION_SKETCH_CURVE_NAME_END))
	{
		//get the index of curve network that this curve lie on
		CCrossSectionProc::GetCNFromSelectIndex(iSelectedName,this->vecCurveNetwork,this->iSelectedCN,this->iSelectedCS);
		cout<<this->iSelectedCN<<"th CN,"<<this->iSelectedCS<<"th CS"<<endl;
		this->iDrawingProfilePlane=this->vecCurveNetwork.at(this->iSelectedCN).ProfilePlaneType;
		CurveSelSetDrawingPlane();
		//since the plane has been translated
		this->vecComputedCS.clear();
	}
	//computed curve on plane selected,add to total curve network
	else if ((iSelectedName>=CREATION_COMPUTE_CURVE_NAME_BEGIN)&&(iSelectedName<=CREATION_COMPUTE_CURVE_NAME_END))
	{
		CCrossSectionProc::AddComputedCNToTotalCN(this->vecComputedCS,this->vecCurveNetwork,this->RefPlane,
			this->iDrawingProfilePlane,this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
		this->bCurvesLeftToFit=true;
		this->vecComputedCS.clear();
	}
	else
	{
		this->iDrawingProfilePlane=NONE_SELECTED;
		this->iSelectedCN=NONE_SELECTED;
		this->iSelectedCS=NONE_SELECTED;
		//if a plane is selected and computed CS is rendered, clean it
		this->vecComputedCS.clear();
	}

	StartWaitAutoPlaneRot();
}

int CMeshCreation::GetDrawingPlane()
{
	return this->iDrawingProfilePlane;
}

void CMeshCreation::AdjustPlaneBoundary(int iIncrease)
{
	if (!this->bRenderRefPlane[0] && !this->bRenderRefPlane[1] && !this->bRenderRefPlane[2])
	{
		return;
	}
	//only when the planes are selected
	if (this->pDoc->GetRBSelName()<CREATION_PLANE_NAME_BEGIN ||
		this->pDoc->GetRBSelName()>CREATION_PLANE_NAME_END)
	{
		return;
	}

	GeometryAlgorithm compute;
	for (int i=0;i<3;i++)
	{
		compute.AdjustPlaneBoundary(iIncrease,this->PlaneBoundaryPoints[i]);
	}
}

void CMeshCreation::TranslateDrawingPlane(double dOffset)
{
	//name of the selected plane
	int iPlane=this->pDoc->GetRBSelName();
	if ((iPlane<CREATION_PLANE_NAME_BEGIN)||(iPlane>CREATION_PLANE_NAME_END))
	{
		return;
	}

	double dNewD=this->RefPlane[iPlane].d()-dOffset;
	if (abs(dNewD)>=2)
	{
		return;
	}

	this->RefPlane[iPlane]=Plane_3(this->RefPlane[iPlane].a(),this->RefPlane[iPlane].b(),
							this->RefPlane[iPlane].c(),dNewD);

	double dNewCor=0;
	switch(iPlane)
	{
	case 0:
		dNewCor=this->PlaneBoundaryPoints[iPlane][0].z()+dOffset;
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(this->PlaneBoundaryPoints[iPlane][i].x(),
				this->PlaneBoundaryPoints[iPlane][i].y(),dNewCor);
		}
		break;
	case 1:
		dNewCor=this->PlaneBoundaryPoints[iPlane][0].y()+dOffset;
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(this->PlaneBoundaryPoints[iPlane][i].x(),
				dNewCor,this->PlaneBoundaryPoints[iPlane][i].z());
		}
		break;
	case 2:
		dNewCor=this->PlaneBoundaryPoints[iPlane][0].x()+dOffset;
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(dNewCor,this->PlaneBoundaryPoints[iPlane][i].y(),
				this->PlaneBoundaryPoints[iPlane][i].z());
		}
	    break;
	default:
	    break;
	}

	CCrossSectionProc::GetCurvePlaneIntersectPoints(this->vecCurveNetwork,this->RefPlane,
		this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
	//CCrossSectionProc::GetCNIntersectPoints(this->vecCurveNetwork);
}

void CMeshCreation::StopTranslateDrawingPlane()
{
	//name of the selected plane
	int iPlane=this->pDoc->GetRBSelName();
	if ((iPlane<CREATION_PLANE_NAME_BEGIN)||(iPlane>CREATION_PLANE_NAME_END))
	{
		return;
	}

	//check if there exists CN on this plane,if yes, selecting the plane means selecting the CN
	//else,don't forget to set iSelectedCN to NONE_SELECTED !!!
	this->iSelectedCN=NONE_SELECTED;
	for (unsigned int iCN=0;iCN<this->vecCurveNetwork.size();iCN++)
	{
		if (this->vecCurveNetwork.at(iCN).ProfilePlaneType==this->iDrawingProfilePlane)
		{
			if (this->vecCurveNetwork.at(iCN).plane==this->RefPlane[this->iDrawingProfilePlane])
			{
				this->iSelectedCN=iCN;
				break;
			}
		}
	}
	this->iSelectedCS=NONE_SELECTED;

	//get the intersection between plane and mesh
	if (!this->pDoc->GetMesh().empty())
	{
		this->vecComputedCS.clear();
		GeometryAlgorithm::GetMeshPlaneIntersection(this->pDoc->GetMesh(),this->RefPlane[iPlane],
			this->vecComputedCS);
	}

	StartWaitAutoPlaneRot();
}

void CMeshCreation::CurveSelSetDrawingPlane()
{
	int iPlane=this->vecCurveNetwork.at(this->iSelectedCN).ProfilePlaneType;
	if ((iPlane<CREATION_PLANE_NAME_BEGIN)||(iPlane>CREATION_PLANE_NAME_END))
	{
		return;
	}
	
	//set the reference plane
	this->RefPlane[iPlane]=this->vecCurveNetwork.at(this->iSelectedCN).plane;

	switch(iPlane)
	{
	case CREATION_XOY_PLANE:
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(this->PlaneBoundaryPoints[iPlane][i].x(),
				this->PlaneBoundaryPoints[iPlane][i].y(),-this->RefPlane[iPlane].d());
		}
		break;
	case CREATION_XOZ_PLANE:
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(this->PlaneBoundaryPoints[iPlane][i].x(),
				-this->RefPlane[iPlane].d(),this->PlaneBoundaryPoints[iPlane][i].z());
		}
		break;
	case CREATION_YOZ_PLANE:
		for (int i=0;i<4;i++)
		{
			this->PlaneBoundaryPoints[iPlane][i]=Point_3(-this->RefPlane[iPlane].d(),this->PlaneBoundaryPoints[iPlane][i].y(),
				this->PlaneBoundaryPoints[iPlane][i].z());
		}
		break;
	default:
		break;
	}

	CCrossSectionProc::GetCurvePlaneIntersectPoints(this->vecCurveNetwork,this->RefPlane,
		this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
	//CCrossSectionProc::GetCNIntersectPoints(this->vecCurveNetwork);

	StartWaitAutoPlaneRot();
}

bool CMeshCreation::CheckPlaneState()
{
	if (this->iDrawingProfilePlane==NONE_SELECTED)
	{
		QMessageBox msgBox;
		msgBox.setText("Have u selected a plane?");
		msgBox.exec();
		return CREATION_PLANE_NOT_READY_FOR_DRAWING;
	}

	if (this->bAutoRot)
	{
		SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
		GLdouble* modelview=pView->GetModelview();

		//get the camera pos in Local Coordinate
		GLdouble InverseModelviewMatrix[16];
		memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
		GeometryAlgorithm Geo;
		Geo.GetInverseMatrix(InverseModelviewMatrix,4);
		Point3D MovedCameraPostemp;
		MovedCameraPostemp.x=0.0;MovedCameraPostemp.y=0.0;MovedCameraPostemp.z=0.0;
		//should be the center of the bounding points of the plane
		//instead of 0 0 -1
		Point3D MovedCameraPostemp2;
		MovedCameraPostemp2.x=0.0;MovedCameraPostemp2.y=0.0;MovedCameraPostemp2.z=-1.0;
		Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);
		Geo.ComputeTransformedPointPos(&MovedCameraPostemp2,InverseModelviewMatrix);
		Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);
		Point_3 MovedCameraPos2(MovedCameraPostemp2.x,MovedCameraPostemp2.y,MovedCameraPostemp2.z);

		Vector_3 PlaneNormal=this->RefPlane[this->iDrawingProfilePlane].orthogonal_vector();
		Vector_3 CameraPlane(MovedCameraPos2,MovedCameraPos);

		double dAngle=GeometryAlgorithm::GetAngleBetweenTwoVectors3d(PlaneNormal,CameraPlane);

		if (dAngle>=45 && dAngle<=135)
		{
			QMessageBox msgBox;
			msgBox.setText("Make the selected plane face to u pls");
			msgBox.exec();
			return CREATION_PLANE_NOT_READY_FOR_DRAWING;
		}
	}

	this->bPlaneReadyState=CREATION_PLANE_READY_FOR_DRAWING;

	return this->bPlaneReadyState;
}

void CMeshCreation::StartWaitAutoPlaneRot()
{
	if (this->iDrawingProfilePlane==NONE_SELECTED)
		return;
	if (!this->bAutoRot)
	{
		return;
	}

	this->iAutoRotTimeLeft=INTERVAL_BEFORE_ROTATE;
	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	//pView->SetTimer(TIMER_PLANE_WAIT_TO_ROTATE,1000,NULL);
}

void CMeshCreation::WaitAutoPlaneRotate()
{
	if (!this->bAutoRot)
	{
		return;
	}
	if (this->iAutoRotTimeLeft!=0)//continue waiting
	{
		this->iAutoRotTimeLeft=this->iAutoRotTimeLeft-1000;;
	}
	else//don't wait,start auto rotation
	{
		SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
		GLdouble* modelview=pView->GetModelview();

		this->iAutoRotTimeLeft=INTERVAL_BEFORE_ROTATE;
		//pView->KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);

		this->vecTestPoint.clear();

		if (this->PlaneRot.PreCompute(this->RefPlane[this->iDrawingProfilePlane],this->PlaneBoundaryPoints[this->iDrawingProfilePlane],
			modelview,20,pView->GetViewRotMat()))
		{
			//pView->SetTimer(TIMER_PLANE_ROTATE,INTERVAL_BETWEEN_ROTATE,NULL);
		}
	}
}

void CMeshCreation::RotateSelectedPlane()
{
	if (this->iDrawingProfilePlane==NONE_SELECTED)
		return;
	if (!this->bAutoRot)
	{
		return;
	}

	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();

	if (!this->PlaneRot.Rotate(pView->GetViewTransMat(),pView->GetViewRotMat()))
	{
		//pView->KillTimer(TIMER_PLANE_ROTATE);
	}
}

void CMeshCreation::Input2DProfilePoint(QPoint ProfilePoint)
{
	if (this->bPlaneReadyState==CREATION_PLANE_READY_FOR_DRAWING)
	{
		this->UserInput2DProfile.push_back(ProfilePoint);
	}
}

void CMeshCreation::Convert2DProfileTo3D()
{
	if (this->UserInput2DProfile.empty())
	{
		return;
	}

	this->bPlaneReadyState=CREATION_PLANE_NOT_READY_FOR_DRAWING;

	GeometryAlgorithm compute;
//	compute.ProcessCurverPoint(this->UserInput2DProfile,5.0);//20
	compute.ProcessCurverPoint(this->UserInput2DProfile,15.0);//15.0


	SketchViewer* pView=pDoc->GetParent()->GetSketchViewer();
	GLdouble* modelview=pView->GetModelview();
	GLdouble* projection=pView->GetProjection();
	GLint* viewport=pView->GetViewport();


	//0:the curve is open and should be closed 
	//1:the curve is open & is a local modification of a selected complete CS
	//2:the curve is open & is a local modification of an unselected computed CS(the computed CS is put vecCurveNetwork into later)
	//3:the curve is open & is a local modification of a selected partial CS
	//4:the curve is open & is a local modification of a selected CN containing partial CS
	int iCurveType=0;
	//modify a curve, allow the curve to be open (local modify);otherwise,make open curve symmetric
	if (GeometryAlgorithm::JudgeCurveOpen2D(this->UserInput2DProfile))
	{
		if (this->iSelectedCN!=NONE_SELECTED)//CNs exist
		{
			if (this->iSelectedCS!=NONE_SELECTED)//modify a selected curve
			{
				if (this->vecCurveNetwork.at(iSelectedCN).PartProfile3D.empty())
				{
					iCurveType=1;
				}
				else
				{
					iCurveType=3;
				}
			}
			else if (!this->vecCurveNetwork.at(iSelectedCN).PartProfile3D.empty())//plane move to a CN that contains partial CS
			{
				iCurveType=4;
			}
		}
		else if (!this->vecComputedCS.empty())//none selected,computed CN exists,treat as modifying it
		{
			iCurveType=2;
		}
		else
		{
			GeometryAlgorithm::MakeSymmetricCurve2D(UserInput2DProfile);
		}
	}


	vector<Point_3> NewProfile;
	CPaintingOnMesh PaintingOnMesh;
	PaintingOnMesh.PaintingOnBFPlane(this->RefPlane[this->iDrawingProfilePlane],
		modelview,projection,viewport,
		this->UserInput2DProfile,NewProfile);

//	if (NewProfile.size()==SAMPLE_POINT_NUM)//20
	if (!NewProfile.empty())
	{
		//local modify selected complete cs
		vector<Point_3> TempCurve;
		int iLocalModifyCSInd=-1;
		if (iCurveType==1)
		{
			TempCurve=this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iSelectedCS);
		}
		//local modify unselected computed cs
		else if (iCurveType==2)
		{
			//which CS is selected
			int iCSInd=CCrossSectionProc::GetCSToModify(this->vecComputedCS,NewProfile);
			TempCurve=this->vecComputedCS.at(iCSInd);
			iLocalModifyCSInd=iCSInd;
		}
		//local modify selected partial cs
		else if (iCurveType==3)
		{
			//local modify and get which points on the CS are modified
			TempCurve=this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iSelectedCS);
			iLocalModifyCSInd=iSelectedCS;
		}
		//local modify selected cn containing partial cs
		else if (iCurveType==4)
		{
			//which CS is selected
			int iCSInd=CCrossSectionProc::GetCSToModify(this->vecCurveNetwork.at(iSelectedCN).Profile3D,NewProfile);
			TempCurve=this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iCSInd);
			iLocalModifyCSInd=iCSInd;
		}
		std::vector<int> vecModifiedPointInd;
		if (iCurveType!=0)
		{
			GeometryAlgorithm::LocalModifyCurve3D(TempCurve,NewProfile,vecModifiedPointInd,0.8);
			NewProfile=TempCurve;
		}

		Polygon_2 NewProfile2D;
		GeometryAlgorithm::PlanarPolygonToXOY(NewProfile,NewProfile2D,this->iDrawingProfilePlane);
		if (!NewProfile2D.is_simple())
		{
			cout<<"new cs self-intersected"<<endl;
			QMessageBox msgBox;
			msgBox.setText("Invalid Input,violate Rule 3!");
			msgBox.exec();
			this->UserInput2DProfile.clear();
			return;
		}

		//make CCW first to let the intersection check easier
		if (NewProfile2D.orientation()==CGAL::CLOCKWISE)
		{
			NewProfile2D.reverse_orientation();
			reverse(NewProfile.begin(),NewProfile.end());
			//reverse vecModifiedPointInd
			CCrossSectionProc::ReverseModifyPointsID(NewProfile.size(),vecModifiedPointInd);
		}

		if (!this->vecCurveNetwork.empty())//not first curve
		{
			//local modify selected cs
			if (iCurveType==1)
			{
				//exclude the curve to modify for judging intersection
				CurveNetwork TempCN=this->vecCurveNetwork.at(iSelectedCN);
				TempCN.Profile3D.erase(TempCN.Profile3D.begin()+this->iSelectedCS);
				TempCN.Profile2D.erase(TempCN.Profile2D.begin()+this->iSelectedCS);
				if (CCrossSectionProc::CheckIntersectOthers(TempCN,NewProfile2D))//this->vecCurveNetwork.at(iSelectedCN)
				{
					cout<<"modified cs intersected with other cs,curve type:"<<iCurveType<<endl;
					QMessageBox msgBox;
					msgBox.setText("Invalid Input,violate Rule 3!");
					msgBox.exec();
					this->UserInput2DProfile.clear();
					return;
				}
				else
				{
					this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iSelectedCS)=NewProfile;
					this->vecCurveNetwork.at(iSelectedCN).Profile2D.at(iSelectedCS)=NewProfile2D;
				}
				CCrossSectionProc::SetCNOrientation(this->vecCurveNetwork.at(iSelectedCN));
			}
			//local modify unselected computed cs
			else if (iCurveType==2)
			{
				//put the modified curve into computed cs,and put computed cs into vecCurveNetwork
				this->vecComputedCS.at(iLocalModifyCSInd)=NewProfile;
				CCrossSectionProc::AddComputedCNToTotalCN(this->vecComputedCS,this->vecCurveNetwork,this->RefPlane,
					this->iDrawingProfilePlane,this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
				//judge if modified cs intersects with others
				if (CCrossSectionProc::CheckIntersectOthers(this->vecCurveNetwork.back(),NewProfile2D,iLocalModifyCSInd))
				{
					cout<<"modified cs intersected with other cs,curve type:"<<iCurveType<<endl;
					//computed curve on plane selected,add to total curve network
					CCrossSectionProc::DeleteLastCS(this->RefPlane,this->vecCurveNetwork,this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
					QMessageBox msgBox;
					msgBox.setText("Invalid Input,violate Rule 3!");
					msgBox.exec();
					this->UserInput2DProfile.clear();
					this->vecComputedCS.clear();
					return;
				}
				else
				{
					//if the order of points on CS is reversed in CCrossSectionProc::AddComputedCNToTotalCN,vecModifiedPointInd needs to be modified
					if (this->vecCurveNetwork.back().Profile3D.at(iLocalModifyCSInd)!=NewProfile)
					{
						CCrossSectionProc::ReverseModifyPointsID(NewProfile.size(),vecModifiedPointInd);
					}
					//modify the PartProfile3D field
					CCrossSectionProc::SavePartialCSInfo(this->vecCurveNetwork.back(),iLocalModifyCSInd,vecModifiedPointInd);
					this->vecComputedCS.clear();
					//don't forget to update the selected CN
					this->iSelectedCN=this->vecCurveNetwork.size()-1;
				}
			}
			//local modify selected partial cs
			else if (iCurveType==3 || iCurveType==4)
			{
				//exclude the curve to modify for judging intersection
				CurveNetwork TempCN=this->vecCurveNetwork.at(iLocalModifyCSInd);
				TempCN.Profile3D.erase(TempCN.Profile3D.begin()+iLocalModifyCSInd);
				TempCN.Profile2D.erase(TempCN.Profile2D.begin()+iLocalModifyCSInd);
				if (CCrossSectionProc::CheckIntersectOthers(TempCN,NewProfile2D,iLocalModifyCSInd))
				{
					cout<<"modified cs intersected with other cs,curve type:"<<iCurveType<<endl;
					QMessageBox msgBox;
					msgBox.setText("Invalid Input,violate Rule 3!");
					msgBox.exec();
					this->UserInput2DProfile.clear();
					return;
				}
				else
				{
					this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iLocalModifyCSInd)=NewProfile;
					this->vecCurveNetwork.at(iSelectedCN).Profile2D.at(iLocalModifyCSInd)=NewProfile2D;
					CCrossSectionProc::SetCNOrientation(this->vecCurveNetwork.at(iSelectedCN));
					//if the order of points on CS is reversed in CCrossSectionProc::SetCNOrientation,vecModifiedPointInd needs to be modified
					if (this->vecCurveNetwork.at(iSelectedCN).Profile3D.at(iLocalModifyCSInd)!=NewProfile)
					{
						CCrossSectionProc::ReverseModifyPointsID(NewProfile.size(),vecModifiedPointInd);
					}
					//modify the PartProfile3D field
					CCrossSectionProc::SavePartialCSInfo(this->vecCurveNetwork.at(iSelectedCN),iLocalModifyCSInd,vecModifiedPointInd);
				}
			}
			//add CN or CS on a new plane
			else if (this->iSelectedCN==NONE_SELECTED && this->iSelectedCS==NONE_SELECTED)
			{
				if (this->vecCurveNetwork.back().plane==this->RefPlane[this->iDrawingProfilePlane])//same plane
				{
					if (CCrossSectionProc::CheckIntersectOthers(this->vecCurveNetwork.back(),NewProfile2D))
					{
						QMessageBox msgBox;
						msgBox.setText("Invalid Input,violate Rule 3!");
						msgBox.exec();
						this->UserInput2DProfile.clear();
						return;
					}
					else
					{
						this->vecCurveNetwork.back().Profile3D.push_back(NewProfile);
						this->vecCurveNetwork.back().Profile2D.push_back(NewProfile2D);
					}
				}
				else//a totally new plane without CN on
				{
					CurveNetwork NewCN;
					NewCN.Profile3D.push_back(NewProfile);
					NewCN.Profile2D.push_back(NewProfile2D);
					NewCN.plane=this->RefPlane[this->iDrawingProfilePlane];
					NewCN.ProfilePlaneType=this->iDrawingProfilePlane;
					this->vecCurveNetwork.push_back(NewCN);
				}
				CCrossSectionProc::SetCNOrientation(this->vecCurveNetwork.back());
			}
			//add a CS on CN
			else if (this->iSelectedCN!=NONE_SELECTED && this->iSelectedCS==NONE_SELECTED)
			{
				if (CCrossSectionProc::CheckIntersectOthers(this->vecCurveNetwork.at(iSelectedCN),NewProfile2D))
				{
					QMessageBox msgBox;
					msgBox.setText("Invalid Input,violate Rule 3!");
					msgBox.exec();
					this->UserInput2DProfile.clear();
					return;
				}
				else
				{
					this->vecCurveNetwork.at(iSelectedCN).Profile3D.push_back(NewProfile);
					this->vecCurveNetwork.at(iSelectedCN).Profile2D.push_back(NewProfile2D);
				}
				CCrossSectionProc::SetCNOrientation(this->vecCurveNetwork.at(iSelectedCN));
			}
		} 
		else//first curve
		{
			CurveNetwork NewCN;
			NewCN.Profile3D.push_back(NewProfile);
			NewCN.Profile2D.push_back(NewProfile2D);
			NewCN.plane=this->RefPlane[this->iDrawingProfilePlane];
			NewCN.ProfilePlaneType=this->iDrawingProfilePlane;
			this->vecCurveNetwork.push_back(NewCN);
			CCrossSectionProc::SetCNOrientation(this->vecCurveNetwork.back());
		}
		this->bCurvesLeftToFit=true;

		//CCrossSectionProc::GetCNIntersectPoints(this->vecCurveNetwork);
		CCrossSectionProc::GetCurvePlaneIntersectPoints(this->vecCurveNetwork,this->RefPlane,
			this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
	}

	this->UserInput2DProfile.clear();
//	this->iDrawingProfilePlane=NONE_SELECTED;

}

void CMeshCreation::CopyCNFromLastParaPlane(int iPlaneType)
{
	CCrossSectionProc::CopyCNFromLastParaPlane(iPlaneType,this->RefPlane,this->vecCurveNetwork,
		this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
}

void CMeshCreation::CancelLastInput()
{
	if (this->vecCurveNetwork.empty())
	{
		return;
	}
	//if the selected CN and CS is affected,update them
	if (iSelectedCN==this->vecCurveNetwork.size()-1)
	{
		if (this->vecCurveNetwork.back().Profile3D.size()>1 && this->iSelectedCS==vecCurveNetwork.back().Profile3D.size()-1)
		{
			this->iSelectedCS=NONE_SELECTED;
		} 
		else if (vecCurveNetwork.back().Profile3D.size()==1 && this->iSelectedCS==vecCurveNetwork.back().Profile3D.size()-1)
		{
			this->iSelectedCN=NONE_SELECTED;
			this->iSelectedCS=NONE_SELECTED;
		}
		else
		{
			//keep unchanged
		}
	}

	CCrossSectionProc::DeleteLastCS(this->RefPlane,this->vecCurveNetwork,
		this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
}

void CMeshCreation::DeleteSpecifiedCS()
{
	if (this->iSelectedCS==NONE_SELECTED)
	{
		return;
	}

	CCrossSectionProc::DeleteSpecifiedCS(this->RefPlane,this->iSelectedCN,this->iSelectedCS,this->vecCurveNetwork,
		this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
}

bool CMeshCreation::FitLastPlaneCurves()
{
	if (!bCurvesLeftToFit)
	{
		return false;
	}
	//if (this->vecCurveNetwork.size()<2)
	//{
	//	return true;
	//}

	bool bResult=false;

	if (this->iSelectedCN==NONE_SELECTED)
	{
		bResult=CCrossSectionProc::FitSpecifiedCN(this->vecCurveNetwork.size()-1,this->RefPlane,this->vecCurveNetwork,
			this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);
	}
	else
	{
		bResult=CCrossSectionProc::FitSpecifiedCN(this->iSelectedCN,this->RefPlane,this->vecCurveNetwork,
			this->vecCurvePlaneIntersectPoint,this->vecCurvePlaneIntersectType);

	}

	if (!bResult)
	{
		this->iSelectedCN=NONE_SELECTED;
		this->iSelectedCS=NONE_SELECTED;
	}
	else
	{
		this->iSelectedCS=NONE_SELECTED;
		//since the curve has been sketched
		this->vecComputedCS.clear();
	}


	return bResult;
}

void CMeshCreation::AdjustContourView()
{
//	if (this->vecCurveNetwork.empty())
//	{
//		return;
//	}
//	this->MeshBoundingProfile3D.clear();
//	//////special process
//	vector<CurveNetwork> vecTempCurveNetwork=this->vecCurveNetwork;
//	//get bounding box,multiply DIM,change the plane para
//	float fPreSetBBox[ 6 ];
//	bool bboxset=false;
//	for (unsigned int iPlane=0;iPlane<vecTempCurveNetwork.size();iPlane++)
//	{
//		for (unsigned int iCurve=0;iCurve<vecTempCurveNetwork.at(iPlane).Profile3D.size();iCurve++)
//		{
//			for (unsigned int iPoint=0;iPoint<vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).size();iPoint++)
//			{
//				if (bboxset==false)
//				{
//					fPreSetBBox[0]=fPreSetBBox[3]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x();
//					fPreSetBBox[1]=fPreSetBBox[4]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y();
//					fPreSetBBox[2]=fPreSetBBox[5]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z();
//					bboxset=true;
//				}
//				else
//				{
//					if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x()<fPreSetBBox[0])
//					{
//						fPreSetBBox[0]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x();
//					}
//					else if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x()>fPreSetBBox[3])
//					{
//						fPreSetBBox[3]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x();
//					}
//					if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y()<fPreSetBBox[1])
//					{
//						fPreSetBBox[1]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y();
//					}
//					else if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y()>fPreSetBBox[4])
//					{
//						fPreSetBBox[4]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y();
//					}
//					if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z()<fPreSetBBox[2])
//					{
//						fPreSetBBox[2]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z();
//					}
//					else if (vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z()>fPreSetBBox[5])
//					{
//						fPreSetBBox[5]=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z();
//					}
//				}
//				double dNewX=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).x()/DIM;
//				double dNewY=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).y()/DIM;
//				double dNewZ=vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint).z()/DIM;
//				vecTempCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(iPoint)=Point_3(dNewX,dNewY,dNewZ);
//			}
//		}
//		//will be wrong if the plane is not of the three basic types
//		double dNewPlaneD=vecTempCurveNetwork.at(iPlane).plane.d()/DIM;
//		vecTempCurveNetwork.at(iPlane).plane=Plane_3(vecTempCurveNetwork.at(iPlane).plane.a(),
//			vecTempCurveNetwork.at(iPlane).plane.b(),
//			vecTempCurveNetwork.at(iPlane).plane.c(),dNewPlaneD);
//	}
//
//	//get center
//	//this->OldCenter=Point_3((bbox[0]+bbox[3])/2.0,(bbox[1]+bbox[4])/2.0,(bbox[2]+bbox[5])/2.0);
//	this->OldCenter=Point_3(0,0,0);
//
////	manager->readContourFromVec(this->vecCurveNetwork,this->MeshBoundingProfile3D);
//
//	//kw test
//	fPreSetBBox[0]=fPreSetBBox[1]=fPreSetBBox[2]=-CREATION_PLANE_BOUND_SIZE/DIM;
//	fPreSetBBox[3]=fPreSetBBox[4]=fPreSetBBox[5]=CREATION_PLANE_BOUND_SIZE/DIM;
//
//	if (this->iSurfReconstAlgorithm==TaoJuSurfReconstAlgo)
//	{
//		this->manager->readContourFromVec(vecTempCurveNetwork,fPreSetBBox,this->MeshBoundingProfile3D);
//		this->manager->SetOldCenter(this->OldCenter);
//	}
//	else if (this->iSurfReconstAlgorithm=ProgSurfReconstAlgo)
//	{
//		this->kwcs2surf->readContourFromVec(vecTempCurveNetwork,fPreSetBBox,this->MeshBoundingProfile3D);
//		this->kwcs2surf->SetOldCenter(this->OldCenter);
//	}
//
////	this->vecCurveNetwork.clear();
////	this->vecCurvePlaneIntersectPoint.clear();
////	this->vecCurvePlaneIntersectType.clear();
}

void CMeshCreation::GenerateMesh(KW_Mesh& Mesh,vector<double> vecMeshColor)
{
	//Mesh.clear();


	//if (this->iSurfReconstAlgorithm==TaoJuSurfReconstAlgo)
	//{
	//	if (manager!=NULL)
	//	{
	//		manager=NULL;
	//		delete manager; 
	//	}
	//	manager= new Ctr2SufManager();
	//}
	//else if (this->iSurfReconstAlgorithm=ProgSurfReconstAlgo)
	//{
	//	if (this->kwcs2surf!=NULL)
	//	{
	//		this->kwcs2surf=NULL;
	//		delete this->kwcs2surf; 
	//	}
	//	this->kwcs2surf= new KW_CS2Surf();
	//}

	//AdjustContourView();

	//this->MeshBoundingProfile3D.clear();

	//if (this->iSurfReconstAlgorithm==TaoJuSurfReconstAlgo)
	//{
	//	this->manager->ctr2sufProc(this->MeshBoundingProfile3D,this->vecTestPoint);
	//	this->manager->mesh->splitsmooth2(0, 1.414, 10, 50, 0.5);
	//	this->manager->CheckCCW();
	//	Convert_Mesh_To_CGALPoly<HalfedgeDS> triangle(this->manager->mesh);
	//	Mesh.delegate(triangle);
	//}
	//else if (this->iSurfReconstAlgorithm=ProgSurfReconstAlgo)
	//{
	//	if (!this->kwcs2surf->ctr2sufProc(this->MeshBoundingProfile3D,this->vecTestPoint))
	//	{
	//		this->MeshBoundingProfile3D.clear();
	//		return;
	//	}
	//	//this->kwcs2surf->mesh->splitsmooth2(0, 2.5, 10, 50, 0.5);
	//	this->kwcs2surf->mesh->splitsmooth2(0, 1.414, 10, 50, 0.5);
	//	//this->kwcs2surf->mesh->splitsmooth(0, 2.5, 10, 50);
	//	//this->kwcs2surf->mesh->splitsmooth(0, 1.414, 10, 50);
	//	this->kwcs2surf->CheckCCW();
	//	Convert_Mesh_To_CGALPoly<HalfedgeDS> triangle(this->kwcs2surf->mesh);
	//	Mesh.delegate(triangle);
	//}


	//this->MeshBoundingProfile3D.clear();

	////move back acoording to the old center
	//for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)
	//{
	//	double dNewX=i->point().x()+this->OldCenter.x();
	//	double dNewY=i->point().y()+this->OldCenter.y();
	//	double dNewZ=i->point().z()+this->OldCenter.z();
	//	i->point()=Point_3(dNewX,dNewY,dNewZ);
	//}

	//OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	//GeometryAlgorithm::SetUniformMeshColor(Mesh,vecMeshColor);

	//Mesh.SetRenderInfo(true,true,true,true,true);
}

void CMeshCreation::ReadContourFromFile(char* pFileName)
{	
	//manager= new Ctr2SufManager();
	//manager->readContourFromFile(pFileName,this->MeshBoundingProfile3D);
	if (!CCrossSectionProc::ImportCrossSections(pFileName,this->vecCurveNetwork))
	{
		return;
	}
	this->bCurvesLeftToFit=true;
}

void CMeshCreation::WriteContourToFile(char* pFileName)
{
	CCrossSectionProc::ExportCrossSections(pFileName,this->vecCurveNetwork);
}

void CMeshCreation::Render(bool bSmoothView,GLenum mode,GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//if (mode==GL_RENDER)
	//{
	//	RenderRefPlanes(bSmoothView,mode);
	//	Render2DProfile(modelview,projection,viewport);
	//	RenderProfile3D(mode);
	//	RenderMeshBoundingProfile3D(mode);
	//	RenderComputedCS(mode);
	//	//RenderCNIntersectPoint();
	//	RenderCurvePlaneIntersectPoints();
	//	RenderTestPoint();
	//	if (this->manager!=NULL)
	//	{
	//		this->manager->Render();
	//	}
	//	if (this->kwcs2surf!=NULL)
	//	{
	//		this->kwcs2surf->Render();
	//	}
	//}
	//else
	//{
	//	RenderRefPlanes(bSmoothView,mode);
	//	RenderProfile3D(mode);
	//	RenderMeshBoundingProfile3D(mode);
	//	RenderComputedCS(mode);
	//}

	////if (manager->mesh!=NULL)
	////{
	////	testrender();
	////}
}

void CMeshCreation::RenderRefPlanes(bool bSmoothView,GLenum mode)
{
	glLineWidth(2);

	//draw the frame of the plane
	if (mode==GL_RENDER)
	{
		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
		for (int i=0;i<3;i++)
		{
			if (!this->bRenderRefPlane[i])
			{
				continue;
			}
			switch(i)
			{
			case 0:
				glColor3f(0,0,1);
				break;
			case 1:
				glColor3f(0,1,0);
				break;
			case 2:
				glColor3f(1,0,0);
				break;
			default:
				break;
			}

			//one reference plane is selected
			if (pDoc->GetRBSelName()==i)  
				//|| (this->iSelectedCN!=NONE_SELECTED && this->vecCurveNetwork.at(this->iSelectedCN).ProfilePlaneType==i))
			{
				glColor4fv(OPAQUE_SELECTED_COLOR);
			}

			glBegin(GL_QUADS);
			glVertex3d(PlaneBoundaryPoints[i][0].x(),PlaneBoundaryPoints[i][0].y(),PlaneBoundaryPoints[i][0].z());
			glVertex3d(PlaneBoundaryPoints[i][1].x(),PlaneBoundaryPoints[i][1].y(),PlaneBoundaryPoints[i][1].z());
			glVertex3d(PlaneBoundaryPoints[i][2].x(),PlaneBoundaryPoints[i][2].y(),PlaneBoundaryPoints[i][2].z());
			glVertex3d(PlaneBoundaryPoints[i][3].x(),PlaneBoundaryPoints[i][3].y(),PlaneBoundaryPoints[i][3].z());
			glEnd();
		}

		//draw the transparent face of the plane(2 faces)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
		glEnable(GL_LIGHTING);
	}

	glDepthMask(FALSE);
//	glDisable(GL_DEPTH_TEST);

	const GLfloat red_color[] = {1.0f, 0.0f, 0.0f, 0.2f};
	const GLfloat mat_red_emission[] = {1.0f, 0.0f, 0.0f, 1.0f};
	const GLfloat green_color[] = {0.0f, 1.0f, 0.0f, 0.2f};
	const GLfloat mat_green_emission[] = {0.0f, 1.0f, 0.0f, 1.0f};
	const GLfloat blue_color[] = {0.0f, 0.0f, 0.6f, 0.2f};
	const GLfloat mat_blue_emission[] = {0.0f, 0.0f, 1.0f, 1.0f};

	for (int i=0;i<3;i++)
	{
		if (!this->bRenderRefPlane[i])
		{
			continue;
		}

		switch(i)
		{
		case 0:
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , blue_color);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_blue_emission);
			//glColor4f(0.0f,0.0f,1.0f,0.2f);			// Full Brightness, 50% Alpha
			break;
		case 1:
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, green_color);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_green_emission);
			//glColor4f(0.0f,1.0f,0.0f,0.2f);			// Full Brightness, 50% Alpha
			break;
		case 2:
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red_color);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_red_emission);
			//glColor4f(1.0f,0.0f,0.0f,0.2f);			// Full Brightness, 50% Alpha
		    break;
		default:
		    break;
		}

		//one reference plane is selected or the CN on the plane is selected
		if (pDoc->GetRBSelName()==i) 
			//||(this->iSelectedCN!=NONE_SELECTED && this->vecCurveNetwork.at(this->iSelectedCN).ProfilePlaneType==i))
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE , TRANSPARENT_SELECTED_COLOR);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  OPAQUE_SELECTED_COLOR);
		}

		if ((mode==GL_SELECT)&&(i==0))
		{
			glPushName(CREATION_XOY_PLANE);
		}
		else if ((mode==GL_SELECT)&&(i==1))
		{
			glPushName(CREATION_XOZ_PLANE);
		}
		else if ((mode==GL_SELECT)&&(i==2))
		{
			glPushName(CREATION_YOZ_PLANE);
		}

		glBegin(GL_QUADS);
		glNormal3f(RefPlane[i].orthogonal_vector().x(),
			RefPlane[i].orthogonal_vector().y(),
			RefPlane[i].orthogonal_vector().z());
		glVertex3d(PlaneBoundaryPoints[i][0].x(),PlaneBoundaryPoints[i][0].y(),PlaneBoundaryPoints[i][0].z());
		glVertex3d(PlaneBoundaryPoints[i][1].x(),PlaneBoundaryPoints[i][1].y(),PlaneBoundaryPoints[i][1].z());
		glVertex3d(PlaneBoundaryPoints[i][2].x(),PlaneBoundaryPoints[i][2].y(),PlaneBoundaryPoints[i][2].z());
		glVertex3d(PlaneBoundaryPoints[i][3].x(),PlaneBoundaryPoints[i][3].y(),PlaneBoundaryPoints[i][3].z());

		glNormal3f(RefPlane[i].opposite().orthogonal_vector().x(),
			RefPlane[i].opposite().orthogonal_vector().y(),
			RefPlane[i].opposite().orthogonal_vector().z());
		glVertex3d(PlaneBoundaryPoints[i][3].x(),PlaneBoundaryPoints[i][3].y(),PlaneBoundaryPoints[i][3].z());
		glVertex3d(PlaneBoundaryPoints[i][2].x(),PlaneBoundaryPoints[i][2].y(),PlaneBoundaryPoints[i][2].z());
		glVertex3d(PlaneBoundaryPoints[i][1].x(),PlaneBoundaryPoints[i][1].y(),PlaneBoundaryPoints[i][1].z());
		glVertex3d(PlaneBoundaryPoints[i][0].x(),PlaneBoundaryPoints[i][0].y(),PlaneBoundaryPoints[i][0].z());
		glEnd();

		if (mode==GL_SELECT)
		{
			glPopName();
		}
	}

//	glEnable(GL_DEPTH_TEST);
	glDepthMask(TRUE);

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

void CMeshCreation::Render2DProfile(GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
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

void CMeshCreation::RenderComputedCS(GLenum mode)
{
	if (pDoc->GetRenderPreMesh()!=MESH_PREVIEW)
	{
		return;
	}
	int iSelName=CREATION_COMPUTE_CURVE_NAME_BEGIN;
	glDisable(GL_LIGHTING);
	glLineWidth(6.0);
	glColor4fv(TRANSPARENT_CROSS_SECTION_COLOR);
	for (unsigned int i=0;i<this->vecComputedCS.size();i++)
	{
		int iLength=this->vecComputedCS.at(i).size();

		if (mode==GL_SELECT)
		{
			glPushName(iSelName);
			iSelName++;
		}

		glBegin(GL_LINES);
		for (int j=0;j<iLength;j++)
		{
			glVertex3d(this->vecComputedCS.at(i).at(j).x(),this->vecComputedCS.at(i).at(j).y(),
				this->vecComputedCS.at(i).at(j).z());
			glVertex3d(this->vecComputedCS.at(i).at((j+1)%iLength).x(),this->vecComputedCS.at(i).at((j+1)%iLength).y(),
				this->vecComputedCS.at(i).at((j+1)%iLength).z());
		}
		glEnd();

		if (mode==GL_SELECT)
		{
			glPopName();
		}
	}
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}

void CMeshCreation::RenderProfile3D(GLenum mode)
{
	if (!this->bRenderCN)
	{
		return;
	}
	int iSelName=CREATION_SKETCH_CURVE_NAME_BEGIN;
	glDisable(GL_LIGHTING);
	for (unsigned int iIndex=0;iIndex<this->vecCurveNetwork.size();iIndex++)
	{
		map<int,vector<vector<int>>>::iterator MapIter;
		if (!this->vecCurveNetwork.at(iIndex).PartProfile3D.empty())
		{
			MapIter=this->vecCurveNetwork.at(iIndex).PartProfile3D.begin();
		}
		for (unsigned int i=0;i<this->vecCurveNetwork.at(iIndex).Profile3D.size();i++)
		{
			//if (mode==GL_RENDER)
			//{
			//	glPointSize(10);
			//	glBegin(GL_POINTS);
			//	glColor3f(1,0,0);
			//	glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).back().x(),
			//		this->vecCurveNetwork.at(iIndex).Profile3D.at(i).back().y(),
			//		this->vecCurveNetwork.at(iIndex).Profile3D.at(i).back().z());
			//	glColor3f(0,1,0);
			//	glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).front().x(),
			//		this->vecCurveNetwork.at(iIndex).Profile3D.at(i).front().y(),
			//		this->vecCurveNetwork.at(iIndex).Profile3D.at(i).front().z());
			//	glEnd();
			//}
			vector<vector<int>> vecPartialCS;
			if (!this->vecCurveNetwork.at(iIndex).PartProfile3D.empty())
			{
				if (MapIter!=this->vecCurveNetwork.at(iIndex).PartProfile3D.end())
				{
					if (i==MapIter->first)
					{
						vecPartialCS=MapIter->second;
						MapIter++;
					}
				}
			}

			if (mode==GL_SELECT)
			{
				glPushName(iSelName);
				iSelName++;
			}
			glLineWidth(6.0);
			//if contains partial cross section,render the partials in opaque color
			if (!vecPartialCS.empty())
			{
				if (mode==GL_RENDER && iIndex==this->iSelectedCN && i==this->iSelectedCS)
				{
					glColor4fv(OPAQUE_SELECTED_COLOR);
					//glColor3f(0,0,1);
				}
				else
				{
					glColor4fv(OPAQUE_CROSS_SECTION_COLOR);
					//glColor3f(1,0,0);
				}
				for (unsigned int j=0;j<vecPartialCS.size();j++)
				{
					vector<int> PartialCS=vecPartialCS.at(j);
					glBegin(GL_LINES);
					for (unsigned int k=0;k<PartialCS.size()-1;k++)
					{
						glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).x(),
							this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).y(),
							this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).z());
						glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).x(),
							this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).y(),
							this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).z());
					}
					glEnd();
				}
			}

			if (mode==GL_RENDER && iIndex==this->iSelectedCN && i==this->iSelectedCS && this->vecCurveNetwork.at(iIndex).PartProfile3D.empty())
			{
				//glColor4fv(TRANSPARENT_SELECTED_COLOR);
				glColor4fv(OPAQUE_SELECTED_COLOR);
			}
			else if (mode==GL_RENDER && iIndex==this->iSelectedCN && i==this->iSelectedCS && !this->vecCurveNetwork.at(iIndex).PartProfile3D.empty())
			{
				glColor4fv(TRANSPARENT_SELECTED_COLOR);
			}
			else if (!this->vecCurveNetwork.at(iIndex).PartProfile3D.empty())//all cs of the cn is rendered transparently
			{
				glColor4fv(TRANSPARENT_CROSS_SECTION_COLOR);
			}
			else
			{
				glColor4fv(OPAQUE_CROSS_SECTION_COLOR);
			}
			//if (mode==GL_SELECT)
			//{
			//	glPushName(iSelName);
			//	iSelName++;
			//}
			glLineWidth(6.0);

			if ((!this->vecCurveNetwork.at(iIndex).PartProfile3D.empty() && !this->bRenderOnlyUserSketch)
				|| (this->vecCurveNetwork.at(iIndex).PartProfile3D.empty()))
			{
				glBegin(GL_LINE_LOOP);
				for (unsigned int j=0;j<this->vecCurveNetwork.at(iIndex).Profile3D.at(i).size();j++)
				{
					glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(j).x(),
						this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(j).y(),
						this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(j).z());
				}
				glEnd();
			}

			////if contains partial cross section,render the partials in opaque color
			//if (!vecPartialCS.empty())
			//{
			//	if (mode==GL_RENDER && iIndex==this->iSelectedCN && i==this->iSelectedCS)
			//	{
			//		glColor4fv(OPAQUE_SELECTED_COLOR);
			//		//glColor3f(0,0,1);
			//	}
			//	else
			//	{
			//		glColor4fv(OPAQUE_CROSS_SECTION_COLOR);
			//		//glColor3f(1,0,0);
			//	}
			//	for (unsigned int j=0;j<vecPartialCS.size();j++)
			//	{
			//		vector<int> PartialCS=vecPartialCS.at(j);
			//		//glBegin(GL_LINES);
			//		for (unsigned int k=0;k<PartialCS.size()-1;k++)
			//		{
			//			glBegin(GL_LINES);
			//			glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).x(),
			//				this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).y(),
			//				this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k)).z());
			//			glVertex3d(this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).x(),
			//				this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).y(),
			//				this->vecCurveNetwork.at(iIndex).Profile3D.at(i).at(PartialCS.at(k+1)).z());
			//			glEnd();
			//		}
			//		//glEnd();
			//	}
			//}

			glLineWidth(1.0);		
			if (mode==GL_SELECT)
			{
				glPopName();
			}
		}
	}
	glEnable(GL_LIGHTING);
}

void CMeshCreation::RenderMeshBoundingProfile3D(GLenum mode)
{
	if (mode==GL_SELECT)
	{
		return;
	}
	glDisable(GL_LIGHTING);
	for (unsigned int iIndex=0;iIndex<this->MeshBoundingProfile3D.size();iIndex++)
	{
		if (!this->MeshBoundingProfile3D.at(iIndex).empty())
		{
			glLineWidth(6.0);
			for (unsigned int i=0;i<this->MeshBoundingProfile3D.at(iIndex).size();i++)
			{
				glBegin(GL_LINES);
				glColor3f(0,1,0);
				{
					if (i!=this->MeshBoundingProfile3D.at(iIndex).size()-1)
					{
						glVertex3d(this->MeshBoundingProfile3D.at(iIndex).at(i).x(),
							this->MeshBoundingProfile3D.at(iIndex).at(i).y(),
							this->MeshBoundingProfile3D.at(iIndex).at(i).z());
						glVertex3d(this->MeshBoundingProfile3D.at(iIndex).at(i+1).x(),
							this->MeshBoundingProfile3D.at(iIndex).at(i+1).y(),
							this->MeshBoundingProfile3D.at(iIndex).at(i+1).z());
					} 
					else
					{
						glVertex3d(this->MeshBoundingProfile3D.at(iIndex).back().x(),
							this->MeshBoundingProfile3D.at(iIndex).back().y(),
							this->MeshBoundingProfile3D.at(iIndex).back().z());
						glVertex3d(this->MeshBoundingProfile3D.at(iIndex).front().x(),
							this->MeshBoundingProfile3D.at(iIndex).front().y(),
							this->MeshBoundingProfile3D.at(iIndex).front().z());
					}
				}
				glEnd();
			}
			glLineWidth(1.0);		
		}
	}
	glEnable(GL_LIGHTING);
}

//void CMeshCreation::RenderCNIntersectPoint()
//{
//	glDisable(GL_LIGHTING);
//	glPointSize(12.0);
//	glBegin(GL_POINTS);
//	for (unsigned int iPlane=0;iPlane<this->vecCurveNetwork.size();iPlane++)
//	{
//		for (unsigned int iInterPoint=0;iInterPoint<this->vecCurveNetwork.at(iPlane).CurvePlaneIntersect.size();iInterPoint++)
//		{
//			switch(this->vecCurveNetwork.at(this->vecCurveNetwork.at(iPlane).IntersectCNInd.at(iInterPoint)).ProfilePlaneType)
//			{
//			case 0:
//				glColor3f(0,0,1);
//				break;
//			case 1:
//				glColor3f(0,1,0);
//				break;
//			case 2:
//				glColor3f(1,0,0);
//			    break;
//			default:
//			    break;
//			}
//			glVertex3d(this->vecCurveNetwork.at(iPlane).CurvePlaneIntersect.at(iInterPoint).x(),
//				this->vecCurveNetwork.at(iPlane).CurvePlaneIntersect.at(iInterPoint).y(),
//				this->vecCurveNetwork.at(iPlane).CurvePlaneIntersect.at(iInterPoint).z());
//		}
//	}
//	glEnd();
//	glPointSize(1.0);
//	glEnable(GL_LIGHTING);
//}

void CMeshCreation::RenderCurvePlaneIntersectPoints()
{
	glDisable(GL_LIGHTING);
	glPointSize(12.0);
	glBegin(GL_POINTS);
	for (unsigned int i=0;i<this->vecCurvePlaneIntersectPoint.size();i++)
	{
		switch(this->vecCurvePlaneIntersectType.at(i))
		{
		case 0:
			glColor3f(0,0,1);
			break;
		case 1:
			glColor3f(0,1,0);
			break;
		case 2:
			glColor3f(1,0,0);
			break;
		default:
			break;
		}
		//only when the reference plane is shown, these hint points are shown
		if (bRenderRefPlane[this->vecCurvePlaneIntersectType.at(i)])
		{
			glVertex3d(this->vecCurvePlaneIntersectPoint.at(i).x(),
				this->vecCurvePlaneIntersectPoint.at(i).y(),
				this->vecCurvePlaneIntersectPoint.at(i).z());
		}
	}
	glEnd();
	glPointSize(1.0);
	glEnable(GL_LIGHTING);
}

void CMeshCreation::RenderTestPoint()
{
	if (this->vecTestPoint.empty())
	{
		return;
	}
	float fStep=1.0/this->vecTestPoint.size();
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->vecTestPoint.size();i++)
	{
		glPointSize(10);
		glBegin(GL_POINTS);
		//glColor3f(0,0,1);
		glColor3f(0,0+(i+1)*fStep,0+(i+1)*fStep);
		glVertex3f(this->vecTestPoint.at(i).x(),this->vecTestPoint.at(i).y(),
			this->vecTestPoint.at(i).z());
		glEnd();
		glPointSize(1);

		//glLineWidth(5);
		//glBegin(GL_LINES);
		//int iNextInd=(i+1)%this->vecTestPoint.size();
		//glVertex3f(this->vecTestPoint.at(i).x(),this->vecTestPoint.at(i).y(),
		//	this->vecTestPoint.at(i).z());
		//glVertex3f(this->vecTestPoint.at(iNextInd).x(),this->vecTestPoint.at(iNextInd).y(),
		//	this->vecTestPoint.at(iNextInd).z());
		//glEnd();
	}
	//for (int i=1;i<4;i++)
	//{
	//	if (i==1)
	//	{
	//		glColor3f(1,0,0);
	//	}
	//	else if (i==2)
	//	{
	//		glColor3f(0,1,0);
	//	}
	//	else if (i==3)
	//	{
	//		glColor3f(0,0,1);
	//	}
	//	glLineWidth(5);
	//	glBegin(GL_LINES);
	//	glVertex3f(this->vecTestPoint.front().x(),this->vecTestPoint.front().y(),
	//		this->vecTestPoint.front().z());
	//	glVertex3f(this->vecTestPoint.at(i).x(),this->vecTestPoint.at(i).y(),
	//		this->vecTestPoint.at(i).z());
	//	glEnd();
	//}
	glEnable(GL_LIGHTING);
}

void CMeshCreation::testrender()
{
	//glBegin(GL_TRIANGLES);
	//for( int i = 0; i < manager->mesh->suffacenum; i ++)
	//{
	//	//if (i==155)
	//	//{
	//	//	break;
	//	//}
	//	glColor3f(1.0,0.0,0.0);
	//	glNormal3fv( &manager->mesh->suffacenorm[ i*3 ]);
	//	for( int k = 0; k < 3; k ++)
	//	{
	//		//			glVertex3fv( &manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3]);
	//		int itest=manager->mesh->sufface[ i * 3 + k];
	//		glVertex3f( manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+0],
	//			manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+1],
	//			manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+2]);
	//	}
	//}
	//glEnd();

	//glPointSize(10.0);
	//for( int i = 0; i < manager->mesh->suffacenum; i ++)
	//{
	//	if (i!=113)
	//	{
	//		continue;
	//	}
	//	for( int k = 0; k < 3; k ++)
	//	{
	//		//			glVertex3fv( &manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3]);
	//		switch(k)
	//		{
	//		case 0:
	//			glColor3f(1.0,0.0,0.0);
	//			break;
	//		case 1:
	//			glColor3f(0.0,1.0,0.0);
	//			break;
	//		case 2:
	//			glColor3f(0.0,0.0,1.0);
	//			break;
	//		default:
	//			break;
	//		}
	//		glBegin(GL_POINTS);
	//		glVertex3f( manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+0],
	//			manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+1],
	//			manager->mesh->sufver[manager->mesh->sufface[ i * 3 + k] * 3+2]);
	//		glEnd();
	//	}
	//}
	//glPointSize(1.0);

}

