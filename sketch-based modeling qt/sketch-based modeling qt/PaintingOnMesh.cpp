#include "PaintingOnMesh.h"
#include "Math/Math.h"
#include <QMessageBox>

CPaintingOnMesh::CPaintingOnMesh(void)
{
	this->hhPrevs.clear();
}

CPaintingOnMesh::~CPaintingOnMesh(void)
{
}

int CPaintingOnMesh::CircleFrontVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve, 
										   GLdouble* modelview,GLdouble* projection,GLint* viewport, 
										   vector<Vertex_handle>& vecCirVer)
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
		return 0;
	}

	//iterate the whole mesh to find the vertices whose projection fall in BoundingPolygon
	//meanwhile, to ensure only the front part is selected,check the angle between the vertex
	//normal and the vector pointing from vertex to camera, delete those whose angles are 
	//larger than 90

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);
	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	vector<Vertex_handle> vecTempROI;
	for ( Vertex_iterator i = Mesh.vertices_begin(); i != Mesh.vertices_end(); i++)
	{
		double dWinX,dWinY,dWinZ;
		gluProject(i->point().x(),i->point().y(),i->point().z(), modelview, projection, viewport, 
			&dWinX, &dWinY, &dWinZ); 
		Point_2 PtProj(dWinX,dWinY);

		if (!BoundingPolygon.has_on_unbounded_side(PtProj))
		{
			//judge the angle
			Vector_3 VerCam(i->point(),MovedCameraPos);
			double dAngle=GeometryAlgorithm::GetAngleBetweenTwoVectors3d(VerCam,i->normal());
			if (dAngle<=90)
			{
				vecTempROI.push_back(i);
			}
		}
	}
	cout<<"selected ROI Num: "<<vecTempROI.size()<<endl;

	vecCirVer=vecTempROI;

	return vecCirVer.size();
}

int CPaintingOnMesh::CircleAllVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve, GLdouble* modelview,GLdouble* projection,GLint* viewport,
									   vector<Vertex_handle>& vecCirVer)
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
		return 0;
	}

	//iterate the whole mesh to find the vertices whose projection fall in BoundingPolygon
	//meanwhile
	vector<Vertex_handle> vecTempROI;
	for ( Vertex_iterator i = Mesh.vertices_begin(); i != Mesh.vertices_end(); i++)
	{
		double dWinX,dWinY,dWinZ;
		gluProject(i->point().x(),i->point().y(),i->point().z(), modelview, projection, viewport, 
			&dWinX, &dWinY, &dWinZ); 
		Point_2 PtProj(dWinX,dWinY);

		if (!BoundingPolygon.has_on_unbounded_side(PtProj))
		{
			vecTempROI.push_back(i);
		}
	}
	cout<<"selected ROI Num: "<<vecTempROI.size()<<endl;

	vecCirVer=vecTempROI;

	return vecCirVer.size();
}

int CPaintingOnMesh::PaintingOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
													 GLdouble* modelview,vector<HandlePointStruct>& vecHandlePoint,
													 vector<Vertex_handle>& vecHandleNbVertex)
{
	vecHandlePoint.clear();
	vecHandleNbVertex.clear();

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//find the IP point and remesh the current triangle
	vector<Point_3> HandleCurvePoint3d;

	//the following two variables have corresponding indices
	vector<Point_3> RoughHandleCurvePoint3d;
	vector<Facet_handle> fhInterSecTri;

	//first calculate and record 
	for (unsigned int i=0;i<UserCurvePoint.size();i++)
	{
		Ray_3 RayCameraUCP(MovedCameraPos,UserCurvePoint.at(i));
		Point_3 IP;
		Facet_handle fTri;
		double fMinDistance=9999.0;

		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			Halfedge_around_facet_circulator k = j->facet_begin();
			Point_3 TriVertex[3];
			int index=0;
			do 
			{
				TriVertex[index]=k->vertex()->point();
				index++;
			} while(++k != j->facet_begin());

			Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
			// note that since has_on method in CGAL is not accurate,so forbiden strictly!
			if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
			{
				Plane_3 TriPlane=CurrentTri.supporting_plane();
				CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
				Point_3 CurrentIP;
				if (CGAL::assign(CurrentIP, result)) 
				{
					std::list<Point_3> points;
					points.push_back(TriVertex[0]);
					points.push_back(TriVertex[1]);
					points.push_back(TriVertex[2]);
					Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
					double fCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
					if (fCurrentDistance<fMinDistance)
					{
						fMinDistance=fCurrentDistance;
						IP=CurrentIP;
						fTri=j;
					}
				}
			}
		}
		if (fMinDistance==9999.0)
		{
			if (!RoughHandleCurvePoint3d.empty())
			{
				break;
			}
		}
		else
		{
			RoughHandleCurvePoint3d.push_back(IP);
			fhInterSecTri.push_back(fTri);
		}
	}

	if (RoughHandleCurvePoint3d.empty())
	{
		return 0;
	}
	//ensure one facet has only one point(excluding the mid point added on the edge)
	vector<Point_3> TempRoughHandleCurvePoint3d;
	vector<Facet_handle> TempfhInterSecTri;
	TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	TempfhInterSecTri.push_back(fhInterSecTri.front());
	for (unsigned int i=1;i<RoughHandleCurvePoint3d.size();i++)
	{
		if (fhInterSecTri.at(i)!=fhInterSecTri.at(i-1))
		{
			TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.at(i));
			TempfhInterSecTri.push_back(fhInterSecTri.at(i));
		}
	}
	RoughHandleCurvePoint3d=TempRoughHandleCurvePoint3d;
	fhInterSecTri=TempfhInterSecTri;


	//second,calculate the middle points
	HandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	for (unsigned int i=0;i<RoughHandleCurvePoint3d.size()-1;i++)
	{
		Point_3 StartPoint=RoughHandleCurvePoint3d.at(i);
		Point_3 EndPoint=RoughHandleCurvePoint3d.at(i+1);
		Facet_handle fhStartTri=fhInterSecTri.at(i);
		Facet_handle fhEndTri=fhInterSecTri.at(i+1);
		Halfedge_handle hhPrev;
		vector<Point_3> MidPoints;
		while (true)
		{
			if (fhStartTri==fhEndTri)
			{
				break;
			}
			Triangle_3 CurrentTri(MovedCameraPos,StartPoint,EndPoint);
			Segment_3 Edge[3];
			Halfedge_handle hhNext;
			int FinalIndex=3;
			Point_3 MidPoint;

			if (StartPoint==RoughHandleCurvePoint3d.at(i))
			{
				hhNext=fhStartTri->halfedge();
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				double dMinDistance=9999.0;
				for (int j=0;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							double dDistance=CGAL::squared_distance(CurrentIP,EndPoint);
							if (dDistance<dMinDistance)
							{
								dMinDistance=dDistance;
								FinalIndex=j;
								MidPoint=CurrentIP;
							}
						}
						else
						{
							//							MessageBox("Error0");
						}	
					}
				}
				if (dMinDistance==9999.0)
				{
					QMessageBox msgBox;
					msgBox.setText("Error1!");
					msgBox.exec();
				}
			}
			else
			{
				hhNext=hhPrev;
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				for (int j=1;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							FinalIndex=j;
							MidPoint=CurrentIP;
						}
						else
						{
							//							MessageBox("Error2");
						}	
					}
				}
			}

			QMessageBox msgBox;
			switch(FinalIndex)
			{
			case 0:
				hhPrev=hhNext->opposite();
				break;
			case 1:
				hhPrev=hhNext->next()->opposite();
				break;
			case 2:
				hhPrev=hhNext->prev()->opposite();
				break;
			case 3:
				msgBox.setText("Error3!");
				msgBox.exec();
				break;
			default:
				break;
			}

			StartPoint=MidPoint;
			fhStartTri=hhPrev->facet();
			this->hhPrevs.push_back(hhPrev);

			MidPoints.push_back(MidPoint);
		}

		if (!MidPoints.empty())
		{
			HandleCurvePoint3d.push_back(MidPoints.front());
			for (unsigned int j=1;j<MidPoints.size();j++)
			{
				Point_3 PointPrev=MidPoints.at(j-1);
				Point_3 PointNext=MidPoints.at(j);
				Point_3 NewMidPoint((PointPrev.x()+PointNext.x())/2,
					(PointPrev.y()+PointNext.y())/2,
					(PointPrev.z()+PointNext.z())/2);
				HandleCurvePoint3d.push_back(NewMidPoint);
				HandleCurvePoint3d.push_back(PointNext);
			}
		}
		HandleCurvePoint3d.push_back(EndPoint);
	}

	if (HandleCurvePoint3d.size()<2)//num of invalid curvepoint <2
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	assert(HandleCurvePoint3d.size()==2*this->hhPrevs.size()+1);

	if (DeleteExtraPointsOnFacet(HandleCurvePoint3d,false)<3)
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	SmoothHandleCurvePoint3d(HandleCurvePoint3d);

	//to make the reconstruction from the handle curve to the triangle vertices over-determined,
	//insert one more handle point between each two
	vector<Point_3> NewHandleCurvePoint3d;
	for (unsigned int i=0;i<HandleCurvePoint3d.size();i++)
	{
		if (i==0)
		{
			NewHandleCurvePoint3d.push_back(HandleCurvePoint3d.at(i));
			continue;
		}

		Point_3 NewPoint=CGAL::midpoint(HandleCurvePoint3d.at(i-1),HandleCurvePoint3d.at(i));
		NewHandleCurvePoint3d.push_back(NewPoint);
		NewHandleCurvePoint3d.push_back(HandleCurvePoint3d.at(i));
	}
	HandleCurvePoint3d=NewHandleCurvePoint3d;

	for (unsigned int i=0;i<HandleCurvePoint3d.size();i++)
	{
		HandlePointStruct CurrentHandle;
		CurrentHandle.PointPos=HandleCurvePoint3d.at(i);
		vecHandlePoint.push_back(CurrentHandle);
	}

	GetLinearCombineInfo(vecHandlePoint,vecHandleNbVertex);
//	CheckLinearCombineInfo(vecHandlePoint,vecHandleNbVertex);

	return vecHandlePoint.size();
}

int CPaintingOnMesh::PaintingOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint, 
										   GLdouble* modelview,vector<Vertex_handle>& hCurveVertex3d)
{
	hCurveVertex3d.clear();

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//find the IP point and remesh the current triangle
	vector<Point_3> HandleCurvePoint3d;

	//the following two variables have corresponding indices
	vector<Point_3> RoughHandleCurvePoint3d;
	vector<Facet_handle> fhInterSecTri;

	//first calculate and record 
	for (unsigned int i=0;i<UserCurvePoint.size();i++)
	{
		Ray_3 RayCameraUCP(MovedCameraPos,UserCurvePoint.at(i));
		Point_3 IP;
		Facet_handle fTri;
		double fMinDistance=9999.0;

		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			Halfedge_around_facet_circulator k = j->facet_begin();
			Point_3 TriVertex[3];
			int index=0;
			do 
			{
				TriVertex[index]=k->vertex()->point();
				index++;
			} while(++k != j->facet_begin());

			Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
			// note that since has_on method in CGAL is not accurate,so forbiden strictly!
			if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
			{
				Plane_3 TriPlane=CurrentTri.supporting_plane();
				CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
				Point_3 CurrentIP;
				if (CGAL::assign(CurrentIP, result)) 
				{
					std::list<Point_3> points;
					points.push_back(TriVertex[0]);
					points.push_back(TriVertex[1]);
					points.push_back(TriVertex[2]);
					Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
					double fCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
					if (fCurrentDistance<fMinDistance)
					{
						fMinDistance=fCurrentDistance;
						IP=CurrentIP;
						fTri=j;
					}
				}
			}
		}
		if (fMinDistance==9999.0)
		{
			if (!RoughHandleCurvePoint3d.empty())
			{
				break;
			}
		}
		else
		{
			RoughHandleCurvePoint3d.push_back(IP);
			fhInterSecTri.push_back(fTri);
		}
	}

	if (RoughHandleCurvePoint3d.empty())
	{
		return 0;
	}
	//ensure one facet has only one point(excluding the mid point added on the edge)
	vector<Point_3> TempRoughHandleCurvePoint3d;
	vector<Facet_handle> TempfhInterSecTri;
	TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	TempfhInterSecTri.push_back(fhInterSecTri.front());
	for (unsigned int i=1;i<RoughHandleCurvePoint3d.size();i++)
	{
		if (fhInterSecTri.at(i)!=fhInterSecTri.at(i-1))
		{
			TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.at(i));
			TempfhInterSecTri.push_back(fhInterSecTri.at(i));
		}
	}
	RoughHandleCurvePoint3d=TempRoughHandleCurvePoint3d;
	fhInterSecTri=TempfhInterSecTri;
	

	//second,calculate the middle points
	HandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	for (unsigned int i=0;i<RoughHandleCurvePoint3d.size()-1;i++)
	{
		Point_3 StartPoint=RoughHandleCurvePoint3d.at(i);
		Point_3 EndPoint=RoughHandleCurvePoint3d.at(i+1);
		Facet_handle fhStartTri=fhInterSecTri.at(i);
		Facet_handle fhEndTri=fhInterSecTri.at(i+1);
		Halfedge_handle hhPrev;
		vector<Point_3> MidPoints;
		while (true)
		{
			if (fhStartTri==fhEndTri)
			{
				break;
			}
			Triangle_3 CurrentTri(MovedCameraPos,StartPoint,EndPoint);
			Segment_3 Edge[3];
			Halfedge_handle hhNext;
			int FinalIndex=3;
			Point_3 MidPoint;

			if (StartPoint==RoughHandleCurvePoint3d.at(i))
			{
				hhNext=fhStartTri->halfedge();
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				double dMinDistance=9999.0;
				for (int j=0;j<3;j++)
				{
//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							double dDistance=CGAL::squared_distance(CurrentIP,EndPoint);
							if (dDistance<dMinDistance)
							{
								dMinDistance=dDistance;
								FinalIndex=j;
								MidPoint=CurrentIP;
							}
						}
						else
						{
//							MessageBox("Error0");
						}	
					}
				}
				if (dMinDistance==9999.0)
				{
					QMessageBox msgBox;
					msgBox.setText("Error1");
					msgBox.exec();
				}
			}
			else
			{
				hhNext=hhPrev;
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				for (int j=1;j<3;j++)
				{
//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							FinalIndex=j;
							MidPoint=CurrentIP;
						}
						else
						{
//							MessageBox("Error2");
						}	
					}
				}
			}

			QMessageBox msgBox;
			switch(FinalIndex)
			{
			case 0:
				hhPrev=hhNext->opposite();
				break;
			case 1:
				hhPrev=hhNext->next()->opposite();
				break;
			case 2:
				hhPrev=hhNext->prev()->opposite();
				break;
			case 3:
				msgBox.setText("Error3");
				msgBox.exec();
				break;
			default:
				break;
			}

			StartPoint=MidPoint;
			fhStartTri=hhPrev->facet();
			this->hhPrevs.push_back(hhPrev);

			MidPoints.push_back(MidPoint);
		}

		if (!MidPoints.empty())
		{
			HandleCurvePoint3d.push_back(MidPoints.front());
			for (unsigned int j=1;j<MidPoints.size();j++)
			{
				Point_3 PointPrev=MidPoints.at(j-1);
				Point_3 PointNext=MidPoints.at(j);
				Point_3 NewMidPoint((PointPrev.x()+PointNext.x())/2,
					(PointPrev.y()+PointNext.y())/2,
					(PointPrev.z()+PointNext.z())/2);
				HandleCurvePoint3d.push_back(NewMidPoint);
				HandleCurvePoint3d.push_back(PointNext);
			}
		}
		HandleCurvePoint3d.push_back(EndPoint);
	}

	if (HandleCurvePoint3d.size()<2)//num of invalid curvepoint <2
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	assert(HandleCurvePoint3d.size()==2*this->hhPrevs.size()+1);

	if (DeleteExtraPointsOnFacet(HandleCurvePoint3d,false)<3)
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	SmoothHandleCurvePoint3d(HandleCurvePoint3d);

//	hCurveVertex3d.clear();
	RemeshOpenStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hCurveVertex3d);

	CheckMesh(Mesh);

	return HandleCurvePoint3d.size();
}

int CPaintingOnMesh::SmoothHandleCurvePoint3d(vector<Point_3>& HandleCurvePoint3d,bool bAverage)
{
	if (HandleCurvePoint3d.size()<4)
	{
		return HandleCurvePoint3d.size();
	}
	for (unsigned int i=1;i<HandleCurvePoint3d.size()-3;i=i+2)
	{
		Point_3 StartPointOnEdge=HandleCurvePoint3d.at(i);
		Point_3 EndPointOnEdge=HandleCurvePoint3d.at(i+2);
		Point_3 PointOnFace=HandleCurvePoint3d.at(i+1);
		Point_3 NewPointOnFace;
		if (bAverage)
		{
			NewPointOnFace=Point_3((StartPointOnEdge.x()+EndPointOnEdge.x())/2,
				(StartPointOnEdge.y()+EndPointOnEdge.y())/2,
				(StartPointOnEdge.z()+EndPointOnEdge.z())/2);
		}
		else
		{
			NewPointOnFace=Point_3((StartPointOnEdge.x()+EndPointOnEdge.x()+4*PointOnFace.x())/6,
											(StartPointOnEdge.y()+EndPointOnEdge.y()+4*PointOnFace.y())/6,
											(StartPointOnEdge.z()+EndPointOnEdge.z()+4*PointOnFace.z())/6);
		}
		HandleCurvePoint3d.at(i+1)=NewPointOnFace;
	}
	return HandleCurvePoint3d.size();
}

/*
this->hhPrevs should be like this
     ---------------------
    /\ hh      /\        /
   /  \prev   /  \      /
  / *  *  *  * *  * *  /
 /      \   /      \  /
/        \ /        \/
---------------------
*/
int CPaintingOnMesh::RemeshOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3>& HandleCurvePoint3d, 
										 vector<Vertex_handle>& hCurveVertex3d,
										 bool bFacetVertexInclude)
{
	for (unsigned int i=0;i<this->hhPrevs.size();i++)
	{
		Halfedge_handle hhPrevOpp=this->hhPrevs.at(i)->opposite();
		Halfedge_handle hhPrevOpp00=Mesh.split_edge(hhPrevOpp);
		Point_3 NewFacetPoint=HandleCurvePoint3d.at(2*i);
		Point_3 NewEdgePoint=HandleCurvePoint3d.at(2*i+1);
		hhPrevOpp00->vertex()->point()=NewEdgePoint;
		Halfedge_handle hhNewFacet=Mesh.create_center_vertex(hhPrevOpp);
		hhNewFacet->vertex()->point()=NewFacetPoint;
		hCurveVertex3d.push_back(hhNewFacet->vertex());
		hCurveVertex3d.push_back(hhPrevOpp00->vertex());
	}
	//for the last facet
	Halfedge_handle hhLastFacet=Mesh.create_center_vertex(this->hhPrevs.back());
	hhLastFacet->vertex()->point()=HandleCurvePoint3d.back();
	hCurveVertex3d.push_back(hhLastFacet->vertex());

	assert(hCurveVertex3d.size()==HandleCurvePoint3d.size());

	return hCurveVertex3d.size();
}

int CPaintingOnMesh::GetLinearCombineInfo(vector<HandlePointStruct>& vecHandlePoint,
										  vector<Vertex_handle>& vecHandleNbVertex)
{
	int iHandlePointIndex=0;
	for (unsigned int i=0;i<this->hhPrevs.size();i++)
	{
		Vertex_handle hVertex[3];
		hVertex[0]=this->hhPrevs.at(i)->opposite()->vertex();
		hVertex[1]=this->hhPrevs.at(i)->vertex();
		hVertex[2]=this->hhPrevs.at(i)->opposite()->next()->vertex();

		//get index of the three vertices
		int iVerIndex[3];
		for (int j=0;j<3;j++)
		{
			vector<Vertex_handle>::iterator iIter=find(vecHandleNbVertex.begin(),vecHandleNbVertex.end(),hVertex[j]);
			if (iIter!=vecHandleNbVertex.end())
			{
				iVerIndex[j]=iIter-vecHandleNbVertex.begin();
			}
			else
			{
				vecHandleNbVertex.push_back(hVertex[j]);
				iVerIndex[j]=vecHandleNbVertex.size()-1;
			}
		}

		//for handle point on facet
		int iFacetPointIter;
		if (i==0)
		{
			iFacetPointIter=2;
		}
		else
		{
			iFacetPointIter=3;
		}
		//for handle point on facet
		//compute parameters
		vector<vector<double>> LeftMatrix;
		vector<double> Row0,Row1,Row2;
		Row0.push_back(hVertex[0]->point().x()-hVertex[2]->point().x());
		Row0.push_back(hVertex[1]->point().x()-hVertex[2]->point().x());
		LeftMatrix.push_back(Row0);
		Row1.push_back(hVertex[0]->point().y()-hVertex[2]->point().y());
		Row1.push_back(hVertex[1]->point().y()-hVertex[2]->point().y());
		LeftMatrix.push_back(Row1);
		Row2.push_back(hVertex[0]->point().z()-hVertex[2]->point().z());
		Row2.push_back(hVertex[1]->point().z()-hVertex[2]->point().z());
		LeftMatrix.push_back(Row2);

		vector<vector<double>> RightMatrix;
		vector<double> RightCol;
		vector<vector<double>> ResultMatrix;
		for (int iIter=0;iIter<iFacetPointIter;iIter++)
		{
			RightMatrix.clear();
			RightCol.clear();
			RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.x()-hVertex[2]->point().x());
			RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.y()-hVertex[2]->point().y());
			RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.z()-hVertex[2]->point().z());
			RightMatrix.push_back(RightCol);

			ResultMatrix.clear();
			CMath::ComputeLSESmallSize(LeftMatrix,RightMatrix,ResultMatrix);

			for (int j=0;j<3;j++)
			{
				//set index
				vecHandlePoint.at(iHandlePointIndex).vecVertexIndex.push_back(iVerIndex[j]);
				if (j<2)
				{
					vecHandlePoint.at(iHandlePointIndex).vecPara.push_back(ResultMatrix.front().at(j));
				}
				else
				{
					vecHandlePoint.at(iHandlePointIndex).vecPara.push_back((double)(1-ResultMatrix.front().at(0)
						-ResultMatrix.front().at(1)));
				}
			}
			iHandlePointIndex++;
		}

		//for handle point on edge
		//compute parameters
		LeftMatrix.clear();
		Row0.clear();
		Row0.push_back(hVertex[0]->point().x()-hVertex[1]->point().x());
		LeftMatrix.push_back(Row0);
		Row1.clear();
		Row1.push_back(hVertex[0]->point().y()-hVertex[1]->point().y());
		LeftMatrix.push_back(Row1);
		Row2.clear();
		Row2.push_back(hVertex[0]->point().z()-hVertex[1]->point().z());
		LeftMatrix.push_back(Row2);

		RightMatrix.clear();
		RightCol.clear();
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.x()-hVertex[1]->point().x());
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.y()-hVertex[1]->point().y());
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.z()-hVertex[1]->point().z());
		RightMatrix.push_back(RightCol);

		ResultMatrix.clear();
		CMath::ComputeLSESmallSize(LeftMatrix,RightMatrix,ResultMatrix);

		for (int j=0;j<2;j++)
		{
			//set index
			vecHandlePoint.at(iHandlePointIndex).vecVertexIndex.push_back(iVerIndex[j]);
			if (j==0)
			{
				vecHandlePoint.at(iHandlePointIndex).vecPara.push_back(ResultMatrix.front().at(0));
			}
			else
			{
				vecHandlePoint.at(iHandlePointIndex).vecPara.push_back((double)(1-ResultMatrix.front().at(0)));
			}
		}
		iHandlePointIndex++;
	}

	//for the last two points on facet
	Vertex_handle hVertex[3];
	hVertex[0]=this->hhPrevs.back()->vertex();
	hVertex[1]=this->hhPrevs.back()->next()->vertex();
	hVertex[2]=this->hhPrevs.back()->prev()->vertex();

	//get index of the three vertices
	int iVerIndex[3];
	for (int j=0;j<3;j++)
	{
		vector<Vertex_handle>::iterator iIter=find(vecHandleNbVertex.begin(),vecHandleNbVertex.end(),hVertex[j]);
		if (iIter!=vecHandleNbVertex.end())
		{
			iVerIndex[j]=iIter-vecHandleNbVertex.begin();
		}
		else
		{
			vecHandleNbVertex.push_back(hVertex[j]);
			iVerIndex[j]=vecHandleNbVertex.size()-1;
		}
	}

	//for handle point on facet
	//compute parameters
	vector<vector<double>> LeftMatrix;
	vector<double> Row0,Row1,Row2;
	Row0.push_back(hVertex[0]->point().x()-hVertex[2]->point().x());
	Row0.push_back(hVertex[1]->point().x()-hVertex[2]->point().x());
	LeftMatrix.push_back(Row0);
	Row1.push_back(hVertex[0]->point().y()-hVertex[2]->point().y());
	Row1.push_back(hVertex[1]->point().y()-hVertex[2]->point().y());
	LeftMatrix.push_back(Row1);
	Row2.push_back(hVertex[0]->point().z()-hVertex[2]->point().z());
	Row2.push_back(hVertex[1]->point().z()-hVertex[2]->point().z());
	LeftMatrix.push_back(Row2);

	for (int iIter=0;iIter<2;iIter++)
	{
		vector<vector<double>> RightMatrix;
		vector<double> RightCol;
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.x()-hVertex[2]->point().x());
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.y()-hVertex[2]->point().y());
		RightCol.push_back(vecHandlePoint.at(iHandlePointIndex).PointPos.z()-hVertex[2]->point().z());
		RightMatrix.push_back(RightCol);

		vector<vector<double>> ResultMatrix;
		CMath::ComputeLSESmallSize(LeftMatrix,RightMatrix,ResultMatrix);

		for (int j=0;j<3;j++)
		{
			//set index
			vecHandlePoint.at(iHandlePointIndex).vecVertexIndex.push_back(iVerIndex[j]);
			if (j<2)
			{
				vecHandlePoint.at(iHandlePointIndex).vecPara.push_back(ResultMatrix.front().at(j));
			}
			else
			{
				vecHandlePoint.at(iHandlePointIndex).vecPara.push_back((double)(1-ResultMatrix.front().at(0)
					-ResultMatrix.front().at(1)));
			}
		}
		iHandlePointIndex++;
	}



	return vecHandleNbVertex.size();
}

int CPaintingOnMesh::PaintingClosedStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint, 
													   GLdouble* modelview,vector<Vertex_handle>& hCurveVertex3d,
													   bool bRemesh)
{
	hCurveVertex3d.clear();

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//find the IP point and remesh the current triangle
	vector<Point_3> HandleCurvePoint3d;

	//the following two variables have corresponding indices
	vector<Point_3> RoughHandleCurvePoint3d;
	vector<Facet_handle> fhInterSecTri;

	//first calculate and record 
	for (unsigned int i=0;i<UserCurvePoint.size();i++)
	{
		Ray_3 RayCameraUCP(MovedCameraPos,UserCurvePoint.at(i));
		Point_3 IP;
		Facet_handle fTri;
		double fMinDistance=9999.0;

		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			int test=j->facet_degree();
			if (test!=3)
			{
				QMessageBox msgBox;
				msgBox.setText("!=3");
				msgBox.exec();
			}

			Halfedge_around_facet_circulator k = j->facet_begin();
			Point_3 TriVertex[3];
			int index=0;
			do 
			{
				TriVertex[index]=k->vertex()->point();
				index++;
				k++;
			} while( k!= j->facet_begin());//++k

			Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
			// note that since has_on method in CGAL is not accurate,so forbiden strictly!
			if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
			{
				Plane_3 TriPlane=CurrentTri.supporting_plane();
				CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
				Point_3 CurrentIP;
				if (CGAL::assign(CurrentIP, result)) 
				{
					std::list<Point_3> points;
					points.push_back(TriVertex[0]);
					points.push_back(TriVertex[1]);
					points.push_back(TriVertex[2]);
					Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
					double fCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
					if (fCurrentDistance<fMinDistance)
					{
						fMinDistance=fCurrentDistance;
						IP=CurrentIP;
						fTri=j;
					}
				}
			}
		}
		if (fMinDistance==9999.0)
		{
			if (!RoughHandleCurvePoint3d.empty())
			{
				break;
			}
		}
		else
		{
			RoughHandleCurvePoint3d.push_back(IP);
			fhInterSecTri.push_back(fTri);
		}
	}

	if ((RoughHandleCurvePoint3d.size()<2)||(fhInterSecTri.size()<2))
	{
		return 0;
	}

	//ensure one facet has only one point(excluding the mid point added on the edge)
	vector<Point_3> TempRoughHandleCurvePoint3d;
	vector<Facet_handle> TempfhInterSecTri;
	TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	TempfhInterSecTri.push_back(fhInterSecTri.front());
	for (unsigned int i=1;i<RoughHandleCurvePoint3d.size();i++)
	{
		//vector<Facet_handle>::iterator Iter=find(TempfhInterSecTri.begin(),TempfhInterSecTri.end(),
		//										fhInterSecTri.at(i));
		//if (Iter==TempfhInterSecTri.end())
		if (fhInterSecTri.at(i)!=fhInterSecTri.at(i-1))
		{
			TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.at(i));
			TempfhInterSecTri.push_back(fhInterSecTri.at(i));
		}
	}
	//ensure the start and end point do not lie on the same triangle
	while (TempfhInterSecTri.front()==TempfhInterSecTri.back())
	{
		TempRoughHandleCurvePoint3d.pop_back();
		TempfhInterSecTri.pop_back();
	}
	RoughHandleCurvePoint3d=TempRoughHandleCurvePoint3d;
	fhInterSecTri=TempfhInterSecTri;

	RoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	fhInterSecTri.push_back(fhInterSecTri.front());
	//second,calculate the middle points
	HandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	for (unsigned int i=0;i<RoughHandleCurvePoint3d.size()-1;i++)
	{
		Point_3 StartPoint=RoughHandleCurvePoint3d.at(i);
		Point_3 EndPoint=RoughHandleCurvePoint3d.at(i+1);
		Facet_handle fhStartTri=fhInterSecTri.at(i);
		Facet_handle fhEndTri=fhInterSecTri.at(i+1);
		Halfedge_handle hhPrev;
		vector<Point_3> MidPoints;
		while (true)
		{
			if (fhStartTri==fhEndTri)
			{
				break;
			}
			Triangle_3 CurrentTri(MovedCameraPos,StartPoint,EndPoint);
			Segment_3 Edge[3];
			Halfedge_handle hhNext;
			int FinalIndex=3;
			Point_3 MidPoint;

			if (StartPoint==RoughHandleCurvePoint3d.at(i))
			{
				hhNext=fhStartTri->halfedge();
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				double dMinDistance=9999.0;
				for (int j=0;j<3;j++)
				{
//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							double dDistance=CGAL::squared_distance(CurrentIP,EndPoint);
							if (dDistance<dMinDistance)
							{
								dMinDistance=dDistance;
								FinalIndex=j;
								MidPoint=CurrentIP;
							}
						}
						else
						{
//							AfxMessageBox("Error0");
						}	
					}
				}
				if (dMinDistance==9999.0)
				{
					QMessageBox msgBox;
					msgBox.setText("Error1");
					msgBox.exec();
				}
			}
			else
			{
				hhNext=hhPrev;
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				for (int j=1;j<3;j++)
				{
//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							FinalIndex=j;
							MidPoint=CurrentIP;
						}
						else
						{
//							AfxMessageBox("Error2");
						}	
					}
				}
			}

			QMessageBox msgBox;
			switch(FinalIndex)
			{
			case 0:
				hhPrev=hhNext->opposite();
				break;
			case 1:
				hhPrev=hhNext->next()->opposite();
				break;
			case 2:
				hhPrev=hhNext->prev()->opposite();
				break;
			case 3:
				msgBox.setText("Error3");
				msgBox.exec();
				break;
			default:
				break;
			}

			StartPoint=MidPoint;
			fhStartTri=hhPrev->facet();
			this->hhPrevs.push_back(hhPrev);

			MidPoints.push_back(MidPoint);
		}

		if (!MidPoints.empty())
		{
			HandleCurvePoint3d.push_back(MidPoints.front());
			for (unsigned int j=1;j<MidPoints.size();j++)
			{
				Point_3 PointPrev=MidPoints.at(j-1);
				Point_3 PointNext=MidPoints.at(j);
				Point_3 NewMidPoint((PointPrev.x()+PointNext.x())/2,
					(PointPrev.y()+PointNext.y())/2,
					(PointPrev.z()+PointNext.z())/2);
				HandleCurvePoint3d.push_back(NewMidPoint);
				HandleCurvePoint3d.push_back(PointNext);
			}
		}
		if (EndPoint!=RoughHandleCurvePoint3d.front())
		{
			HandleCurvePoint3d.push_back(EndPoint);
		}
	}

	if (HandleCurvePoint3d.size()<2)//num of invalid curvepoint <2
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	assert(HandleCurvePoint3d.size()==2*this->hhPrevs.size());

	if (DeleteExtraPointsOnFacet(HandleCurvePoint3d,true)<4)
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	SmoothHandleCurvePoint3d(HandleCurvePoint3d);

//	hCurveVertex3d.clear();
	if (bRemesh)//remesh to make the HandleCurvePoint3d vertices of the mesh
	{
//		RemeshClosedStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hCurveVertex3d);
		RemeshClosedStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hCurveVertex3d,false);
		SmoothHandleCurvePoint3d(HandleCurvePoint3d,true);
		CheckMesh(Mesh);
	}
	else//get the vertices inside and nearest to HandleCurvePoint3d
	{
		for (unsigned int i=0;i<this->hhPrevs.size();i++)
		{
			Vertex_handle CurrentVertex=this->hhPrevs.at(i)->opposite()->vertex();
			vector<Vertex_handle>::iterator vecIter=find(hCurveVertex3d.begin(),hCurveVertex3d.end(),CurrentVertex);
			if (vecIter==hCurveVertex3d.end())
			{
				hCurveVertex3d.push_back(CurrentVertex);
			}
		}
	}

	return HandleCurvePoint3d.size();
}

int CPaintingOnMesh::PaintingFrontalCurveOnRearMesh(KW_Mesh& Mesh,vector<Vertex_handle> hFrontalCurveVertex3d,Vector_3 GivenDirection, 
													vector<Vertex_handle>& hRearCurveVertex3d,vector<Point_3>& testpoint,bool bRemesh/* =true */)
{
	hRearCurveVertex3d.clear();

	//find the IP point and remesh the current triangle
	vector<Point_3> HandleCurvePoint3d;

	//the following two variables have corresponding indices
	vector<Point_3> RoughHandleCurvePoint3d;
	vector<Facet_handle> fhInterSecTri;

	//move the vertices on frontal curve a little bit
	vector<Point_3> MovedFrontalCurvePoint3d;

	//first calculate and record 
	for (unsigned int i=0;i<hFrontalCurveVertex3d.size();i++)
	{
		//move the frontal curve point a little bit,else,result would be they themselves
		Point_3 StartPoint=hFrontalCurveVertex3d.at(i)->point()+0.01*GivenDirection;

		MovedFrontalCurvePoint3d.push_back(StartPoint);

		Ray_3 RayCameraUCP(StartPoint,GivenDirection);
		Point_3 IP;
		Facet_handle fTri;
		double fMinDistance=9999.0;

		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			int test=j->facet_degree();
			if (test!=3)
			{
				QMessageBox msgBox;
				msgBox.setText("!=3");
				msgBox.exec();
			}

			Halfedge_around_facet_circulator k = j->facet_begin();
			Point_3 TriVertex[3];
			int index=0;
			do 
			{
				TriVertex[index]=k->vertex()->point();
				index++;
				k++;
			} while( k!= j->facet_begin());//++k

			Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
			// note that since has_on method in CGAL is not accurate,so forbiden strictly!
			if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
			{
				Plane_3 TriPlane=CurrentTri.supporting_plane();
				CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
				Point_3 CurrentIP;
				if (CGAL::assign(CurrentIP, result)) 
				{
					std::list<Point_3> points;
					points.push_back(TriVertex[0]);
					points.push_back(TriVertex[1]);
					points.push_back(TriVertex[2]);
					Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
					double fCurrentDistance=CGAL::squared_distance(MovedFrontalCurvePoint3d.at(i),CentroidPoint);
					if (fCurrentDistance<fMinDistance)
					{
						fMinDistance=fCurrentDistance;
						IP=CurrentIP;
						fTri=j;
					}
				}
			}
		}
		if (fMinDistance==9999.0)
		{
			if (!RoughHandleCurvePoint3d.empty())
			{
				break;
			}
		}
		else
		{
			RoughHandleCurvePoint3d.push_back(IP);
			fhInterSecTri.push_back(fTri);
		}
	}

	if ((RoughHandleCurvePoint3d.size()<2)||(fhInterSecTri.size()<2))
	{
		return 0;
	}

	//ensure one facet has only one point(excluding the mid point added on the edge)
	vector<Point_3> TempRoughHandleCurvePoint3d;
	vector<Facet_handle> TempfhInterSecTri;
	vector<Point_3> TempMovedFrontalCurvePoint3d;
	TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	TempfhInterSecTri.push_back(fhInterSecTri.front());
	TempMovedFrontalCurvePoint3d.push_back(MovedFrontalCurvePoint3d.front());
	for (unsigned int i=1;i<RoughHandleCurvePoint3d.size();i++)
	{
		//vector<Facet_handle>::iterator Iter=find(TempfhInterSecTri.begin(),TempfhInterSecTri.end(),
		//										fhInterSecTri.at(i));
		//if (Iter==TempfhInterSecTri.end())
		if (fhInterSecTri.at(i)!=fhInterSecTri.at(i-1))
		{
			TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.at(i));
			TempfhInterSecTri.push_back(fhInterSecTri.at(i));
			TempMovedFrontalCurvePoint3d.push_back(MovedFrontalCurvePoint3d.at(i));
		}
	}
	//ensure the start and end point do not lie on the same triangle
	while (TempfhInterSecTri.front()==TempfhInterSecTri.back())
	{
		TempRoughHandleCurvePoint3d.pop_back();
		TempfhInterSecTri.pop_back();
		TempMovedFrontalCurvePoint3d.pop_back();
	}
	RoughHandleCurvePoint3d=TempRoughHandleCurvePoint3d;
	fhInterSecTri=TempfhInterSecTri;
	MovedFrontalCurvePoint3d=TempMovedFrontalCurvePoint3d;
	
	RoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	fhInterSecTri.push_back(fhInterSecTri.front());
	MovedFrontalCurvePoint3d.push_back(MovedFrontalCurvePoint3d.front());
	//second,calculate the middle points
	HandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	for (unsigned int i=0;i<RoughHandleCurvePoint3d.size()-1;i++)
	{
		Point_3 StartPoint=RoughHandleCurvePoint3d.at(i);
		Point_3 EndPoint=RoughHandleCurvePoint3d.at(i+1);
		Facet_handle fhStartTri=fhInterSecTri.at(i);
		Facet_handle fhEndTri=fhInterSecTri.at(i+1);
		Halfedge_handle hhPrev;
		vector<Point_3> MidPoints;
		while (true)
		{
			if (fhStartTri==fhEndTri)
			{
				break;
			}
			Triangle_3 CurrentTri(MovedFrontalCurvePoint3d.at(i),StartPoint,EndPoint);
			Segment_3 Edge[3];
			Halfedge_handle hhNext;
			int FinalIndex=3;
			Point_3 MidPoint;

			if (StartPoint==RoughHandleCurvePoint3d.at(i))
			{
				hhNext=fhStartTri->halfedge();
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				double dMinDistance=9999.0;
				for (int j=0;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							double dDistance=CGAL::squared_distance(CurrentIP,EndPoint);
							if (dDistance<dMinDistance)
							{
								dMinDistance=dDistance;
								FinalIndex=j;
								MidPoint=CurrentIP;
							}
						}
						else
						{
							//							AfxMessageBox("Error0");
						}	
					}
				}
				if (dMinDistance==9999.0)
				{
					QMessageBox msgBox;
					msgBox.setText("Error1");
					msgBox.exec();
				}
			}
			else
			{
				hhNext=hhPrev;
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				for (int j=1;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							FinalIndex=j;
							MidPoint=CurrentIP;
						}
						else
						{
							//							AfxMessageBox("Error2");
						}	
					}
				}
			}

			QMessageBox msgBox;
			switch(FinalIndex)
			{
			case 0:
				hhPrev=hhNext->opposite();
				break;
			case 1:
				hhPrev=hhNext->next()->opposite();
				break;
			case 2:
				hhPrev=hhNext->prev()->opposite();
				break;
			case 3:
				msgBox.setText("!=3");
				msgBox.exec();
				break;
			default:
				break;
			}

			StartPoint=MidPoint;
			fhStartTri=hhPrev->facet();
			this->hhPrevs.push_back(hhPrev);

			MidPoints.push_back(MidPoint);
		}

		if (!MidPoints.empty())
		{
			HandleCurvePoint3d.push_back(MidPoints.front());
			for (unsigned int j=1;j<MidPoints.size();j++)
			{
				Point_3 PointPrev=MidPoints.at(j-1);
				Point_3 PointNext=MidPoints.at(j);
				Point_3 NewMidPoint((PointPrev.x()+PointNext.x())/2,
					(PointPrev.y()+PointNext.y())/2,
					(PointPrev.z()+PointNext.z())/2);
				HandleCurvePoint3d.push_back(NewMidPoint);
				HandleCurvePoint3d.push_back(PointNext);
			}
		}
		if (EndPoint!=RoughHandleCurvePoint3d.front())
		{
			HandleCurvePoint3d.push_back(EndPoint);
		}
	}

	if (HandleCurvePoint3d.size()<2)//num of invalid curvepoint <2
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	assert(HandleCurvePoint3d.size()==2*this->hhPrevs.size());

	if (DeleteExtraPointsOnFacet(HandleCurvePoint3d,true)<4)
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	SmoothHandleCurvePoint3d(HandleCurvePoint3d);

	//	hCurveVertex3d.clear();
	if (bRemesh)//remesh to make the HandleCurvePoint3d vertices of the mesh
	{
		//		RemeshClosedStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hRearCurveVertex3d);
		RemeshClosedStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hRearCurveVertex3d,false);
		SmoothHandleCurvePoint3d(HandleCurvePoint3d,true);
		CheckMesh(Mesh);
	}
	else//get the vertices inside and nearest to HandleCurvePoint3d
	{
		for (unsigned int i=0;i<this->hhPrevs.size();i++)
		{
			Vertex_handle CurrentVertex=this->hhPrevs.at(i)->opposite()->vertex();
			vector<Vertex_handle>::iterator vecIter=find(hRearCurveVertex3d.begin(),hRearCurveVertex3d.end(),CurrentVertex);
			if (vecIter==hRearCurveVertex3d.end())
			{
				hRearCurveVertex3d.push_back(CurrentVertex);
			}
		}
	}

	return HandleCurvePoint3d.size();
}

int CPaintingOnMesh::PaintingClosedStrokeOnFrontalAndRearMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
															  GLdouble* modelview,
															  vector<Vertex_handle>& hCurveVertex3d)
{
	hCurveVertex3d.clear();

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//find the IP point and remesh the current triangle
	vector<Point_3> HandleCurvePoint3d;

	//the following two variables have corresponding indices
	vector<Point_3> FrontalRoughHandleCurvePoint3d;
	vector<Point_3> RearRoughHandleCurvePoint3d;
	vector<Facet_handle> fhFrontalInterSecTri;
	vector<Facet_handle> fhRearInterSecTri;
	vector<Point_3> RoughHandleCurvePoint3d;
	vector<Facet_handle> fhInterSecTri;

	//first calculate and record 
	for (unsigned int i=0;i<UserCurvePoint.size();i++)
	{
		Ray_3 RayCameraUCP(MovedCameraPos,UserCurvePoint.at(i));
		vector<double> vecDistance;
		vector<Point_3> vecIP;
		vector<Facet_handle> vecfTri;
		

		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			int test=j->facet_degree();
			if (test!=3)
			{
				QMessageBox msgBox;
				msgBox.setText("!=3");
				msgBox.exec();
			}

			Halfedge_around_facet_circulator k = j->facet_begin();
			Point_3 TriVertex[3];
			int index=0;
			do 
			{
				TriVertex[index]=k->vertex()->point();
				index++;
				k++;
			} while( k!= j->facet_begin());//++k

			Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
			// note that since has_on method in CGAL is not accurate,so forbiden strictly!
			if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
			{
				Plane_3 TriPlane=CurrentTri.supporting_plane();
				CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
				Point_3 CurrentIP;
				if (CGAL::assign(CurrentIP, result)) 
				{
					std::list<Point_3> points;
					points.push_back(TriVertex[0]);
					points.push_back(TriVertex[1]);
					points.push_back(TriVertex[2]);
					Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
					double dCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
					vecDistance.push_back(dCurrentDistance);
					vecIP.push_back(CurrentIP);
					vecfTri.push_back(j);
				}
			}
		}
		//if (vecDistance.empty())
		if (vecDistance.size()<2)
		{
			if ((!FrontalRoughHandleCurvePoint3d.empty())&&(!RearRoughHandleCurvePoint3d.empty()))
			{
				break;
			}
		}
		else
		{
			vector<double>::iterator IterMin=min_element(vecDistance.begin(),vecDistance.end());
			int iMinIndex=IterMin-vecDistance.begin();
			vector<double>::iterator IterMax=max_element(vecDistance.begin(),vecDistance.end());
			int iMaxIndex=IterMax-vecDistance.begin();
			FrontalRoughHandleCurvePoint3d.push_back(vecIP.at(iMinIndex));
			RearRoughHandleCurvePoint3d.push_back(vecIP.at(iMaxIndex));
			fhFrontalInterSecTri.push_back(vecfTri.at(iMinIndex));
			fhRearInterSecTri.push_back(vecfTri.at(iMaxIndex));
		}
	}

	if ((FrontalRoughHandleCurvePoint3d.size()<2)||(fhFrontalInterSecTri.size()<2))
	{
		return 0;
	}

	reverse(RearRoughHandleCurvePoint3d.begin(),RearRoughHandleCurvePoint3d.end());
	reverse(fhRearInterSecTri.begin(),fhRearInterSecTri.end());
	//sine the last point of frontal,the first point of rear,the moved camera lie on the same line
	//so delete the last point of frontal
	//similarly,delete the last point of back
	RoughHandleCurvePoint3d.insert(RoughHandleCurvePoint3d.begin(),
									FrontalRoughHandleCurvePoint3d.begin(),FrontalRoughHandleCurvePoint3d.end()-1);
	RoughHandleCurvePoint3d.insert(RoughHandleCurvePoint3d.end(),RearRoughHandleCurvePoint3d.begin(),
									RearRoughHandleCurvePoint3d.end()-1);
	fhInterSecTri.insert(fhInterSecTri.begin(),fhFrontalInterSecTri.begin(),fhFrontalInterSecTri.end()-1);
	fhInterSecTri.insert(fhInterSecTri.end(),fhRearInterSecTri.begin(),fhRearInterSecTri.end()-1);

	//ensure one facet has only one point(excluding the mid point added on the edge)
	vector<Point_3> TempRoughHandleCurvePoint3d;
	vector<Facet_handle> TempfhInterSecTri;
	TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	TempfhInterSecTri.push_back(fhInterSecTri.front());
	for (unsigned int i=1;i<RoughHandleCurvePoint3d.size();i++)
	{
		//vector<Facet_handle>::iterator Iter=find(TempfhInterSecTri.begin(),TempfhInterSecTri.end(),
		//										fhInterSecTri.at(i));
		//if (Iter==TempfhInterSecTri.end())
		if (fhInterSecTri.at(i)!=fhInterSecTri.at(i-1))
		{
			TempRoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.at(i));
			TempfhInterSecTri.push_back(fhInterSecTri.at(i));
		}
	}
	//ensure the start and end point do not lie on the same triangle
	while (TempfhInterSecTri.front()==TempfhInterSecTri.back())
	{
		TempRoughHandleCurvePoint3d.pop_back();
		TempfhInterSecTri.pop_back();
	}
	RoughHandleCurvePoint3d=TempRoughHandleCurvePoint3d;
	fhInterSecTri=TempfhInterSecTri;

	RoughHandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	fhInterSecTri.push_back(fhInterSecTri.front());
	//second,calculate the middle points
	HandleCurvePoint3d.push_back(RoughHandleCurvePoint3d.front());
	for (unsigned int i=0;i<RoughHandleCurvePoint3d.size()-1;i++)
	{
		Point_3 StartPoint=RoughHandleCurvePoint3d.at(i);
		Point_3 EndPoint=RoughHandleCurvePoint3d.at(i+1);
		Facet_handle fhStartTri=fhInterSecTri.at(i);
		Facet_handle fhEndTri=fhInterSecTri.at(i+1);
		Halfedge_handle hhPrev;
		vector<Point_3> MidPoints;
		while (true)
		{
			if (fhStartTri==fhEndTri)
			{
				break;
			}
			Triangle_3 CurrentTri(MovedCameraPos,StartPoint,EndPoint);
			Segment_3 Edge[3];
			Halfedge_handle hhNext;
			int FinalIndex=3;
			Point_3 MidPoint;

			if (StartPoint==RoughHandleCurvePoint3d.at(i))
			{
				hhNext=fhStartTri->halfedge();
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				double dMinDistance=9999.0;
				for (int j=0;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							double dDistance=CGAL::squared_distance(CurrentIP,EndPoint);
							if (dDistance<dMinDistance)
							{
								dMinDistance=dDistance;
								FinalIndex=j;
								MidPoint=CurrentIP;
							}
						}
						else
						{
							//							AfxMessageBox("Error0");
						}	
					}
				}
				if (dMinDistance==9999.0)
				{
					QMessageBox msgBox;
					msgBox.setText("Error1");
					msgBox.exec();
				}
			}
			else
			{
				hhNext=hhPrev;
				Edge[0]=Segment_3(hhNext->prev()->vertex()->point(),
					hhNext->vertex()->point());
				Edge[1]=Segment_3(hhNext->vertex()->point(),
					hhNext->next()->vertex()->point());
				Edge[2]=Segment_3(hhNext->next()->vertex()->point(),
					hhNext->prev()->vertex()->point());
				for (int j=1;j<3;j++)
				{
					//					if (CGAL::do_intersect(CurrentTri,Edge[j]))
					{
						Plane_3 TriPlane=CurrentTri.supporting_plane();
						CGAL::Object result = CGAL::intersection(TriPlane,Edge[j]);
						Point_3 CurrentIP;
						if (CGAL::assign(CurrentIP, result)) 
						{
							FinalIndex=j;
							MidPoint=CurrentIP;
						}
						else
						{
							//							AfxMessageBox("Error2");
						}	
					}
				}
			}

			QMessageBox msgBox;
			switch(FinalIndex)
			{
			case 0:
				hhPrev=hhNext->opposite();
				break;
			case 1:
				hhPrev=hhNext->next()->opposite();
				break;
			case 2:
				hhPrev=hhNext->prev()->opposite();
				break;
			case 3:
				msgBox.setText("Error3");
				msgBox.exec();
				break;
			default:
				break;
			}

			StartPoint=MidPoint;
			fhStartTri=hhPrev->facet();
			this->hhPrevs.push_back(hhPrev);

			MidPoints.push_back(MidPoint);
		}

		if (!MidPoints.empty())
		{
			HandleCurvePoint3d.push_back(MidPoints.front());
			for (unsigned int j=1;j<MidPoints.size();j++)
			{
				Point_3 PointPrev=MidPoints.at(j-1);
				Point_3 PointNext=MidPoints.at(j);
				Point_3 NewMidPoint((PointPrev.x()+PointNext.x())/2,
					(PointPrev.y()+PointNext.y())/2,
					(PointPrev.z()+PointNext.z())/2);
				HandleCurvePoint3d.push_back(NewMidPoint);
				HandleCurvePoint3d.push_back(PointNext);
			}
		}
		if (EndPoint!=RoughHandleCurvePoint3d.front())
		{
			HandleCurvePoint3d.push_back(EndPoint);
		}
	}

	if (HandleCurvePoint3d.size()<2)//num of invalid curvepoint <2
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

	assert(HandleCurvePoint3d.size()==2*this->hhPrevs.size());

	if (DeleteExtraPointsOnFacet(HandleCurvePoint3d,true)<4)
	{
		QMessageBox msgBox;
		msgBox.setText("Invalied Curve! Draw again please");
		msgBox.exec();
		return 0;
	}

//	SmoothHandleCurvePoint3d(HandleCurvePoint3d);

	RemeshClosedStrokeOnFrontalMesh(Mesh,HandleCurvePoint3d,hCurveVertex3d,false);

	CheckMesh(Mesh);

	return HandleCurvePoint3d.size();
}

/*In case this occurs,ensure each triangles facet has only one vertex
   ---------------------
   \        /\         /
    \  *   /  \       /
	 \    *    \     /
	  \  / *    * * /
	   \/      * \ /
        ---*--*---/
        \        /
         \  *   /
          \    /
		   \  /
		    \/
*/
int CPaintingOnMesh::DeleteExtraPointsOnFacet(vector<Point_3>& HandleCurvePoint3d,bool bStrokeClosed)
{
	vector<Point_3> NewHandleCurvePoint3d;
	vector<Halfedge_handle> NewhhPrevs;

	vector<Facet_handle> fhFacets;
	//if open stroke,count on the facet which the first point lies on in advance
	if (!bStrokeClosed)
	{
		fhFacets.push_back(hhPrevs.front()->opposite()->facet());
	}

	for (unsigned int i=0;i<this->hhPrevs.size();i++)
	{
		vector<Facet_handle>::iterator Iter=find(fhFacets.begin(),fhFacets.end(),hhPrevs.at(i)->facet());
		if (Iter!=fhFacets.end())
		{
			int iStart=Iter-fhFacets.begin();
			int iEnd=fhFacets.size();
			int iPruneNum=iEnd-iStart;
			NewhhPrevs.erase(NewhhPrevs.end()-(iPruneNum-1),NewhhPrevs.end());
			NewHandleCurvePoint3d.erase(NewHandleCurvePoint3d.end()-2*(iPruneNum-1),NewHandleCurvePoint3d.end());
			fhFacets.erase(Iter+1,fhFacets.end());
		}
		else
		{
			NewhhPrevs.push_back(this->hhPrevs.at(i));
			NewHandleCurvePoint3d.push_back(HandleCurvePoint3d.at(2*i));
			NewHandleCurvePoint3d.push_back(HandleCurvePoint3d.at(2*i+1));
			fhFacets.push_back(hhPrevs.at(i)->facet());
		}
	}

	if(!bStrokeClosed)
	{
		NewHandleCurvePoint3d.push_back(HandleCurvePoint3d.back());
	}

	if (bStrokeClosed)
	{
		assert(NewhhPrevs.size()==fhFacets.size());
		assert(NewHandleCurvePoint3d.size()==2*NewhhPrevs.size());
	} 
	else
	{
		assert(NewhhPrevs.size()+1==fhFacets.size());
		assert(NewHandleCurvePoint3d.size()==2*NewhhPrevs.size()+1);
	}

	//check if two facets of NewhhPrevs.at(i) and NewhhPrevs.at(i+1)
	//are neighbor facets
	for (unsigned int i=0;i<NewhhPrevs.size()-1;i++)
	{
		if (NewhhPrevs.at(i)->next()->opposite()==NewhhPrevs.at(i+1))
		{
		}
		else if (NewhhPrevs.at(i)->prev()->opposite()==NewhhPrevs.at(i+1))
		{
		}
		else
		{
			NewHandleCurvePoint3d.clear();
			NewhhPrevs.clear();
			break;
		}
	}

	//for closed stroke,check the last facet
	if ((bStrokeClosed)&&(!NewhhPrevs.empty()))
	{
		if (NewhhPrevs.back()->next()->opposite()==NewhhPrevs.front())
		{
		}
		else if (NewhhPrevs.back()->prev()->opposite()==NewhhPrevs.front())
		{
		}
		else
		{
			NewHandleCurvePoint3d.clear();
			NewhhPrevs.clear();
		}
	}

	HandleCurvePoint3d=NewHandleCurvePoint3d;
	this->hhPrevs=NewhhPrevs;

	return HandleCurvePoint3d.size();
}


/*
this->hhPrevs should be like this
  
              /|\
             / | \
			/  |  \
           /   *   \
          / *  | *  \
         /     |     \
        |---*-----*---|
        | hh   /\     |
        | *   /  \  * |
		|    /    *   |
		|   *   *  \  |
		|  /        \ |
		| /          \|
		---------------
*/
int CPaintingOnMesh::RemeshClosedStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3>& HandleCurvePoint3d, 
													 vector<Vertex_handle>& hCurveVertex3d,
													 bool bFacetVertexInclude)
{
	if (!bFacetVertexInclude)
	{
		//delete the points on facets
		vector<Point_3> RefinedCurvePoint3d;
		for (unsigned int i=0;i<HandleCurvePoint3d.size()/2;i++)
		{
			RefinedCurvePoint3d.push_back(HandleCurvePoint3d.at(2*i+1));
		}
		HandleCurvePoint3d=RefinedCurvePoint3d;
		assert(HandleCurvePoint3d.size()==this->hhPrevs.size());
		//split each edge
		for (unsigned int i=0;i<this->hhPrevs.size();i++)
		{
			Halfedge_handle hhNew=Mesh.split_edge(this->hhPrevs.at(i));
			hhNew->vertex()->point()=HandleCurvePoint3d.at(i);
			hCurveVertex3d.push_back(hhNew->vertex());
		}
		//connect points on edge
		for (unsigned int i=0;i<this->hhPrevs.size()-1;i++)
		{
			Mesh.split_facet(this->hhPrevs.at(i)->prev(),this->hhPrevs.at(i+1)->opposite());
		}
		Halfedge_handle hhNew=Mesh.split_facet(this->hhPrevs.back()->prev(),this->hhPrevs.front()->opposite());
		//split facets with four vertices
		for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
		{
			if (j->facet_degree()==4)
			{
				Mesh.split_facet(j->halfedge(),j->halfedge()->next()->next());
			}
		}
	}
	else
	{
		Halfedge_handle hhLastNewEdge;
		for (unsigned int i=0;i<this->hhPrevs.size();i++)
		{
			Halfedge_handle hhPrevOpp=this->hhPrevs.at(i)->opposite();
			Halfedge_handle hhPrevOpp00=Mesh.split_edge(hhPrevOpp);
			Point_3 NewFacetPoint=HandleCurvePoint3d.at(2*i);
			Point_3 NewEdgePoint=HandleCurvePoint3d.at(2*i+1);
			hhPrevOpp00->vertex()->point()=NewEdgePoint;
			Halfedge_handle hhNewFacet=Mesh.create_center_vertex(hhPrevOpp);
			hhNewFacet->vertex()->point()=NewFacetPoint;
			hCurveVertex3d.push_back(hhNewFacet->vertex());
			hCurveVertex3d.push_back(hhPrevOpp00->vertex());
			if (i==this->hhPrevs.size()-1)
			{
				hhLastNewEdge=hhPrevOpp->opposite();
			}
		}
		//for the last facet
		Mesh.split_facet(hhLastNewEdge,hhLastNewEdge->next()->next());
	}

	assert(hCurveVertex3d.size()==HandleCurvePoint3d.size());

	return hCurveVertex3d.size();
}

void CPaintingOnMesh::CheckMesh(KW_Mesh Mesh)
{
	for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
	{
		int test=j->facet_degree();
		if (test!=3)
		{
			QMessageBox msgBox;
			msgBox.setText("CheckMesh!=3");
			msgBox.exec();
		}
	}
}

int CPaintingOnMesh::PaintingOnBFPlane(Plane_3 BestFittingPlane,GLdouble* modelview,GLdouble* projection,
									   GLint* viewport, const vector<QPoint> DeformCurvePoint,
									   vector<Point_3>& DeformCurvePoint3d)
{
	//first convert UCP into opengl coordinate(on Znear) 
	vector<Point_3> UserDeformCurvePoint;
	for (unsigned int i=0;i<DeformCurvePoint.size();i++)
	{
		HPCPoint pt;
		pt.x =DeformCurvePoint.at(i).x();
		pt.y =DeformCurvePoint.at(i).y();
		GLdouble  winX, winY, winZ; 
		GLdouble posX, posY, posZ; 
		winX = (float)pt.x; 
		winY = viewport[3] - (float)pt.y;
		glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
		gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
		Point_3 CurrentPoint(posX,posY,posZ);
		UserDeformCurvePoint.push_back(CurrentPoint);
	}
	assert(UserDeformCurvePoint.size()==DeformCurvePoint.size());

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	DeformCurvePoint3d.clear();
	for (unsigned int i=0;i<UserDeformCurvePoint.size();i++)
	{
		Ray_3 RayCameraUCP(MovedCameraPos,UserDeformCurvePoint.at(i));
		CGAL::Object result = CGAL::intersection(BestFittingPlane,RayCameraUCP);
		Point_3 CurrentIP;
		if (CGAL::assign(CurrentIP, result)) 
		{
			DeformCurvePoint3d.push_back(CurrentIP);
		}
	}
	assert(DeformCurvePoint.size()==DeformCurvePoint3d.size());

	return DeformCurvePoint3d.size();
}

int CPaintingOnMesh::PaintingPointOnSphere(Sphere_3 sphere,GLdouble* modelview,GLdouble* projection,GLint* viewport, 
									  QPoint InputPoint,vector<Point_3>& IntersectPoints)
{
	//first convert UCP into opengl coordinate(on Znear) 
	HPCPoint pt;
	pt.x =InputPoint.x();
	pt.y =InputPoint.y();
	GLdouble  winX, winY, winZ; 
	GLdouble posX, posY, posZ; 
	winX = (float)pt.x; 
	winY = viewport[3] - (float)pt.y;
	glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	Point_3 CurrentPoint(posX,posY,posZ);

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	IntersectPoints.clear();

	Line_3 line(MovedCameraPos,CurrentPoint);

	int iIntersctNum=GeometryAlgorithm::GetLineSphereIntersection(line,sphere,IntersectPoints);
	
	if (iIntersctNum==2)
	{
		if (CGAL::has_larger_distance_to_point(MovedCameraPos,IntersectPoints.front(),IntersectPoints.back()))
		{
			reverse(IntersectPoints.begin(),IntersectPoints.end());
		} 
	}
	return iIntersctNum;
}

bool CPaintingOnMesh::PaintingPointOnFrontalMesh(KW_Mesh& Mesh,Point_3& UserPoint,Facet_handle& UserPointFacet,
												 GLdouble* modelview)
{
	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//the following two variables have corresponding indices
	Point_3 IntersectionPoint=UserPoint;

	//first calculate and record 
	Ray_3 RayCameraUCP(MovedCameraPos,UserPoint);
	Point_3 IP;
	double fMinDistance=9999.0;

	for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
	{
		Halfedge_around_facet_circulator k = j->facet_begin();
		Point_3 TriVertex[3];
		int index=0;
		do 
		{
			TriVertex[index]=k->vertex()->point();
			index++;
			if (index==3)
			{
				break;
			}
		} while(++k != j->facet_begin());

		Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
		// note that since has_on method in CGAL is not accurate,so forbiden strictly!
		if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
		{
			Plane_3 TriPlane=CurrentTri.supporting_plane();
			CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
			Point_3 CurrentIP;
			if (CGAL::assign(CurrentIP, result)) 
			{
				std::list<Point_3> points;
				points.push_back(TriVertex[0]);
				points.push_back(TriVertex[1]);
				points.push_back(TriVertex[2]);
				Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
				double fCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
				if (fCurrentDistance<fMinDistance)
				{
					fMinDistance=fCurrentDistance;
					IP=CurrentIP;
					UserPointFacet=j;
				}
			}
		}
	}
	if (fMinDistance<9999.0)
	{
		IntersectionPoint=IP;
	}

	if (IntersectionPoint!=UserPoint)
	{
		UserPoint=IntersectionPoint;
		return true;
	}
	return false;
}

bool CPaintingOnMesh::PaintingScrPointOnFrontalMesh(KW_Mesh& Mesh,QPoint UserScrPoint,Point_3& UserPoint,Facet_handle& UserPointFacet, 
													GLdouble* modelview,GLdouble* projection,GLint* viewport)
{
	//convert UserScrPoint from 2D windows coordinates to 3D opengl coordinates
	GLdouble  winX, winY, winZ; 
	GLdouble posX, posY, posZ; 
	winX =UserScrPoint.x();
	winY = viewport[3] - UserScrPoint.y();
	glReadPixels((int)winX, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ); 
	gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	UserPoint=Point_3(posX,posY,posZ);

	//get the camera pos in Local Coordinate
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;
	MovedCameraPostemp.y=0.0;
	MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);

	Point_3 MovedCameraPos(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);

	//the following two variables have corresponding indices
	Point_3 IntersectionPoint=UserPoint;

	//first calculate and record 
	Ray_3 RayCameraUCP(MovedCameraPos,UserPoint);
	Point_3 IP;
	double fMinDistance=9999.0;

	for ( Facet_iterator j=Mesh.facets_begin(); j!=Mesh.facets_end(); j++)
	{
		Halfedge_around_facet_circulator k = j->facet_begin();
		Point_3 TriVertex[3];
		int index=0;
		do 
		{
			TriVertex[index]=k->vertex()->point();
			index++;
			if (index==3)
			{
				break;
			}
		} while(++k != j->facet_begin());

		Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
		// note that since has_on method in CGAL is not accurate,so forbiden strictly!
		if (CGAL::do_intersect(CurrentTri,RayCameraUCP))
		{
			Plane_3 TriPlane=CurrentTri.supporting_plane();
			CGAL::Object result = CGAL::intersection(TriPlane,RayCameraUCP);
			Point_3 CurrentIP;
			if (CGAL::assign(CurrentIP, result)) 
			{
				std::list<Point_3> points;
				points.push_back(TriVertex[0]);
				points.push_back(TriVertex[1]);
				points.push_back(TriVertex[2]);
				Point_3 CentroidPoint=CGAL::centroid(points.begin(),points.end());
				double fCurrentDistance=CGAL::squared_distance(MovedCameraPos,CentroidPoint);
				if (fCurrentDistance<fMinDistance)
				{
					fMinDistance=fCurrentDistance;
					IP=CurrentIP;
					UserPointFacet=j;
				}
			}
		}
	}
	if (fMinDistance<9999.0)
	{
		IntersectionPoint=IP;
	}

	if (IntersectionPoint!=UserPoint)
	{
		UserPoint=IntersectionPoint;
		return true;
	}
	return false;
}

void CPaintingOnMesh::CheckLinearCombineInfo(vector<HandlePointStruct> vecHandlePoint,
											 vector<Vertex_handle> vecHandleNbVertex)
{
#ifdef DEBUG
	for (unsigned int i=0;i<vecHandlePoint.size();i++)
	{
		double Pointx=vecHandlePoint.at(i).PointPos.x();
		double Pointy=vecHandlePoint.at(i).PointPos.y();
		double Pointz=vecHandlePoint.at(i).PointPos.z();

		double dDifferenceX,dDifferenceY,dDifferenceZ;
		dDifferenceX=dDifferenceY=dDifferenceZ=0;
		for (unsigned int j=0;j<vecHandlePoint.at(i).vecVertexIndex.size();j++)
		{
			dDifferenceX=dDifferenceX+vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().x()
				*vecHandlePoint.at(i).vecPara.at(j);
			dDifferenceY=dDifferenceY+vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().y()
				*vecHandlePoint.at(i).vecPara.at(j);
			dDifferenceZ=dDifferenceZ+vecHandleNbVertex.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().z()
				*vecHandlePoint.at(i).vecPara.at(j);
		}
		dDifferenceX=dDifferenceX-Pointx;
		dDifferenceY=dDifferenceY-Pointy;
		dDifferenceZ=dDifferenceZ-Pointz;
		DBWindowWrite("%f\t%f\t%f\n",dDifferenceX,dDifferenceY,dDifferenceZ);
	}
#endif
}