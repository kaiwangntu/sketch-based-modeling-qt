#include "GeometryAlgorithm.h"
//#include "PreDef.h"
#include "OBJHandle.h"

void KW_Mesh::SetRenderInfo(bool bSetVerInfo,bool bSetNormInfo,bool bSetVerInd,bool bSetFaceInd,bool bSetColorInfo)
{
	SetRenderVerInfo(bSetVerInfo,bSetNormInfo,bSetColorInfo);
	if (bSetVerInd)
	{
		SetVerIndices();
	}
	if (bSetFaceInd)
	{
		buildFacesIndices();
	}
}
void KW_Mesh::SetRenderVerInfo(bool bSetVerInfo,bool bSetNormInfo,bool bSetColorInfo)
{
	if (bSetVerInfo)
	{
		this->vecRenderVerPos.clear();
	}
	if (bSetNormInfo)
	{
		this->vecRenderNorm.clear();
	}
	if (bSetColorInfo)
	{
		this->vecRenderVerColor.clear();
	}
	for (Vertex_iterator i=this->vertices_begin(); i!=this->vertices_end(); i++)
	{
		if (bSetVerInfo)
		{
			this->vecRenderVerPos.push_back(i->point().x());
			this->vecRenderVerPos.push_back(i->point().y());
			this->vecRenderVerPos.push_back(i->point().z());
		}
		if (bSetNormInfo)
		{
			this->vecRenderNorm.push_back(i->normal().x());
			this->vecRenderNorm.push_back(i->normal().y());
			this->vecRenderNorm.push_back(i->normal().z());
		}
		if (bSetColorInfo)
		{
			for (int j=0;j<4;j++)
			{
				this->vecRenderVerColor.push_back(i->GetColor()[j]);
			}
		}
	}
}

void KW_Mesh::SetVerIndices()
{
	int iIndex=0;
	for (Vertex_iterator i=this->vertices_begin(); i!=this->vertices_end(); i++)
	{
		i->SetVertexIndex(iIndex);
		iIndex++;
	}
}

void KW_Mesh::buildFacesIndices()
{
	vecRenderFaceType.clear();
	vecvecRenderFaceID.clear();

	for (Facet_iterator i=this->facets_begin();i!= this->facets_end(); i++)
	{
		int iDegree=i->facet_degree();
		vector<int>::iterator iIter=find(vecRenderFaceType.begin(),vecRenderFaceType.end(),iDegree);
		int iGroup=0;
		if (iIter==vecRenderFaceType.end())
		{
			vecRenderFaceType.push_back(iDegree);
			iGroup=vecRenderFaceType.size()-1;
			//create space for the new group
			vector<int> vecTemp;vecTemp.push_back(0);
			vecvecRenderFaceID.push_back(vecTemp);vecvecRenderFaceID.back().clear();
		}
		else
		{
			iGroup=iIter-vecRenderFaceType.begin();	
		}
		Halfedge_around_facet_circulator j = i->facet_begin();
		do 
		{
			int iIndex=j->vertex()->GetVertexIndex();
			vecvecRenderFaceID.at(iGroup).push_back(iIndex);
		} while(++j != i->facet_begin());
	}
}

void KW_Mesh::clear()
{
	cout<<"clearing CGAL Polyhedron\n";
	Polyhedron::clear();
	cout<<"clearing render info\n";
	this->vecRenderVerPos.clear();
	this->vecRenderNorm.clear();
	this->vecRenderFaceType.clear();
	this->vecvecRenderFaceID.clear();
	this->vecRenderVerColor.clear();
}

GeometryAlgorithm::GeometryAlgorithm(void)
{
}

GeometryAlgorithm::~GeometryAlgorithm(void)
{
}

void GeometryAlgorithm::SampleCircle(Point_3 CircleCenter,double dRadius,int iSamplePointNum,std::vector<Point_3>& SamplePoints)
{
	double dUnitAngle=2*CGAL_PI/iSamplePointNum;
	for (int i=0;i<iSamplePointNum;i++)
	{
		double dAngle=dUnitAngle*i;
		double dX=dRadius*cos(dAngle)+CircleCenter.x();
		double dY=dRadius*sin(dAngle)+CircleCenter.y();
		SamplePoints.push_back(Point_3(dX,dY,CircleCenter.z()));
	}
}

bool GeometryAlgorithm::GetArbiPointInPolygon(Polygon_2 PolygonIn,Point_2& ResultPoint)
{
	Point_2 V,A,B,R;
	int iIndex=0;
	V=PolygonIn.vertex(0);
	for (unsigned int i=1;i<PolygonIn.size();i++)//寻找一个凸顶点，实际上最低点肯定是凸顶点
	{
		if (PolygonIn.vertex(i).y()<V.y())
		{
			V=PolygonIn.vertex(i);
			iIndex=i;
		}
	}
	A=PolygonIn.vertex((iIndex-1+PolygonIn.size())%PolygonIn.size());//得到v的前一个顶点
	B=PolygonIn.vertex((iIndex+1)%PolygonIn.size());//得到v的后一个顶点

	Triangle_2 Tri(V,A,B);
	bool bIn=false;
	double dMinDist=9999999999999;
	Point_2 Q;
	for (unsigned int i=0;i<PolygonIn.size();i++)//寻找在三角形avb内且离顶点v最近的顶点q
	{
		if (i==iIndex||i==(iIndex-1+PolygonIn.size())%PolygonIn.size()||i==(iIndex+1)%PolygonIn.size())
		{
			continue;
		}
		if (Tri.has_on_unbounded_side(PolygonIn.vertex(i)))
		{
			continue;
		}
		bIn=true;
		if (CGAL::squared_distance(V,PolygonIn.vertex(i))<dMinDist)
		{
			Q=PolygonIn.vertex(i);
			dMinDist=CGAL::squared_distance(V,PolygonIn.vertex(i));
		}
	}
	if (!bIn)//没有顶点在三角形avb内，返回线段ab中点
	{
		ResultPoint=CGAL::midpoint(A,B);
	}
	else//返回线段vq的中点
	{
		ResultPoint=CGAL::midpoint(V,Q);
	}

	if (PolygonIn.has_on_bounded_side(ResultPoint))
	{
		return true;
	}

	return false;
}

//Judge if the point is in triangle
bool GeometryAlgorithm::IfPointInTriangle2d(HPCPoint* Triangle, QPoint point)
{
	for(int i=0;i<3;i++)   
	{
		float x0=(point.x()-Triangle[(i+1)%3].x)*(Triangle[i].y-Triangle[(i+1)%3].y)-(Triangle[i].x-Triangle[(i+1)%3].x)*(point.y()-Triangle[(i+1)%3].y);
		float x1=(point.x()-Triangle[(i+1)%3].x)*(Triangle[(i+2)%3].y-Triangle[(i+1)%3].y)-(Triangle[(i+2)%3].x-Triangle[(i+1)%3].x)*(point.y()-Triangle[(i+1)%3].y);
		if   (x0*x1>0)   
			return   0;  
	}
	return 1;
}

//this function is still imperfect
bool GeometryAlgorithm::GetCircleAndSegIntersection(Circle_2 circle,Segment_2 seg,Point_2& IntersectionPoint)
{
	bool bIntersected=false;
	Circular_Circle_2 Ccircle(Circular_Point_2(circle.center().x(),circle.center().y()),circle.squared_radius());
	Circular_Line_arc_2 Cline(Circular_Point_2(seg.source().x(),seg.source().y()),
								Circular_Point_2(seg.target().x(),seg.target().y()));

	std::vector<CGAL::Object> intersections_objects;
	Circular_k::Intersect_2 intersection;
	intersection(Ccircle,Cline,std::back_inserter(intersections_objects));
	for (unsigned int i=0;i<intersections_objects.size();i++)
	{
		CGAL::Object result=intersections_objects.at(i);
		std::pair<Circular_arc_point_2, unsigned> pointpair; 
		if (CGAL::assign(pointpair,result)) 
		{
			Point_2 point(pointpair.first.x(),pointpair.first.y());
			if (seg.collinear_has_on(point))
			{
				bIntersected=true;
				IntersectionPoint=point;
				break;
			}
		}

	}
	return bIntersected;
}

double GeometryAlgorithm::GetDerivation(vector<double> vecInputNumber)
{
	double dSum=0;
	for (unsigned int i=0;i<vecInputNumber.size();i++)
	{
		dSum=dSum+vecInputNumber.at(i);
	}
	double dAverage=dSum/(double)vecInputNumber.size();
	double dSumSquare=0;
	for (unsigned int i=0;i<vecInputNumber.size();i++)
	{
		dSumSquare=dSumSquare+pow((dAverage-vecInputNumber.at(i)),2);
	}
	double dResult=sqrt(dSumSquare/(double)vecInputNumber.size());
	return dResult;
}

void GeometryAlgorithm::PlanarPolygonToXOY(vector<Point_3> InputPolygon,Polygon_2& OutputPolygon, int iPlaneType)
{
	if (iPlaneType==0)//XOY
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(InputPolygon.at(i).x(),InputPolygon.at(i).y()));
		}
		//assert(OutputPolygon.is_simple());
	}
	else if (iPlaneType==1)//XOZ
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(InputPolygon.at(i).x(),-InputPolygon.at(i).z()));
		}
		//assert(OutputPolygon.is_simple());
	}
	else if (iPlaneType==2)//YOZ
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(-InputPolygon.at(i).z(),InputPolygon.at(i).y()));
		}
		//assert(OutputPolygon.is_simple());
	}
}

void GeometryAlgorithm::PlanarPolygonToXOYCCW(vector<Point_3>& InputPolygon,Polygon_2& OutputPolygon, int iPlaneType)
{
	if (iPlaneType==0)//XOY
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(InputPolygon.at(i).x(),InputPolygon.at(i).y()));
		}
	}
	else if (iPlaneType==1)//XOZ
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(InputPolygon.at(i).x(),-InputPolygon.at(i).z()));
		}
	}
	else if (iPlaneType==2)//YOZ
	{
		for (unsigned int i=0;i<InputPolygon.size();i++)
		{
			OutputPolygon.push_back(Point_2(-InputPolygon.at(i).z(),InputPolygon.at(i).y()));
		}
	}
	if (OutputPolygon.is_clockwise_oriented())
	{
		OutputPolygon.reverse_orientation();
		reverse(InputPolygon.begin()+1,InputPolygon.end());
	}
}

void GeometryAlgorithm::XOYPolygonTo3DPlanar(Polygon_2 InputPolygon,std::vector<Point_3>& OutputPolygon,int iPlaneType,Point_3 SamplePoint)
{
	for (Vertex_iterator_2 VerIter=InputPolygon.vertices_begin();VerIter!=InputPolygon.vertices_end();VerIter++)
	{
		if (iPlaneType==0)//XOY
		{
			OutputPolygon.push_back(Point_3((*VerIter).x(),(*VerIter).y(),SamplePoint.z()));
		}
		else if (iPlaneType==1)//XOZ
		{
			OutputPolygon.push_back(Point_3((*VerIter).x(),SamplePoint.y(),-(*VerIter).y()));
		}
		else if (iPlaneType==2)//YOZ
		{
			OutputPolygon.push_back(Point_3(SamplePoint.x(),(*VerIter).y(),-(*VerIter).x()));
		}
	}
}

bool GeometryAlgorithm::IsPlanarPolygonCCW(vector<Point_3> InputPolygon, int iPlaneType)
{
	Polygon_2 XOYPolygon;
	PlanarPolygonToXOY(InputPolygon,XOYPolygon,iPlaneType);
	bool bIsCCW=XOYPolygon.is_counterclockwise_oriented();
	return bIsCCW;
}

bool GeometryAlgorithm::IsPlanarPolygonsIntersect(vector<Point_3> InputPolygon0,vector<Point_3> InputPolygon1, int iPlaneType)
{
	Polygon_2 XOYPolygon0,XOYPolygon1;
	PlanarPolygonToXOY(InputPolygon0,XOYPolygon0,iPlaneType);
	PlanarPolygonToXOY(InputPolygon1,XOYPolygon1,iPlaneType);
	assert(XOYPolygon0.is_simple());
	assert(XOYPolygon1.is_simple());
	if (XOYPolygon0.orientation()==CGAL::CLOCKWISE)
	{
		XOYPolygon0.reverse_orientation();
	}
	if (XOYPolygon1.orientation()==CGAL::CLOCKWISE)
	{
		XOYPolygon1.reverse_orientation();
	}

	bool bIntersect=CGAL::do_intersect(XOYPolygon0,XOYPolygon1);
	return bIntersect;
}

bool GeometryAlgorithm::IsPlanarPolygonsIntersect2D(Polygon_2 InputPolygon0,Polygon_2 InputPolygon1)
{
	assert(InputPolygon0.is_simple());
	assert(InputPolygon1.is_simple());
	if (InputPolygon0.orientation()==CGAL::CLOCKWISE)
	{
		InputPolygon0.reverse_orientation();
	}
	if (InputPolygon1.orientation()==CGAL::CLOCKWISE)
	{
		InputPolygon1.reverse_orientation();
	}
	bool bIntersect=CGAL::do_intersect(InputPolygon0,InputPolygon1);
	return bIntersect;
}

int GeometryAlgorithm::PlanarPolygonsContained(vector<Point_3> InputPolygon0,vector<Point_3> InputPolygon1, int iPlaneType)
{
	Polygon_2 XOYPolygon0,XOYPolygon1;
	PlanarPolygonToXOY(InputPolygon0,XOYPolygon0,iPlaneType);
	PlanarPolygonToXOY(InputPolygon1,XOYPolygon1,iPlaneType);
	int iResult=3;

	for (Vertex_iterator_2 Vi=XOYPolygon0.vertices_begin();Vi!=XOYPolygon0.vertices_end();Vi++)
	{
		if (!XOYPolygon1.has_on_bounded_side(*Vi))
		{
			iResult=3;
			break;
		}
		else
		{
			iResult=0;
		}
	}
	if (iResult!=3)
	{
		return iResult;
	}

	for (Vertex_iterator_2 Vi=XOYPolygon1.vertices_begin();Vi!=XOYPolygon1.vertices_end();Vi++)
	{
		if (!XOYPolygon0.has_on_bounded_side(*Vi))
		{
			iResult=3;
			break;
		}
		else
		{
			iResult=1;
		}
	}
	if (iResult!=3)
	{
		return iResult;
	}

	bool bIntersect=IsPlanarPolygonsIntersect2D(XOYPolygon0,XOYPolygon1);
	if (bIntersect)
	{
		iResult=2;
	}
	else
	{
		iResult=3;
	}

	return iResult;
}

int GeometryAlgorithm::PlanarPolygonsContained2D(Polygon_2 InputPolygon0,Polygon_2 InputPolygon1)
{
	int iResult=3;

	assert(InputPolygon0.is_simple());
	assert(InputPolygon1.is_simple());

	for (Vertex_iterator_2 Vi=InputPolygon0.vertices_begin();Vi!=InputPolygon0.vertices_end();Vi++)
	{
		if (!InputPolygon1.has_on_bounded_side(*Vi))
		{
			iResult=3;
			break;
		}
		else
		{
			iResult=0;
		}
	}
	if (iResult!=3)
	{
		return iResult;
	}

	for (Vertex_iterator_2 Vi=InputPolygon1.vertices_begin();Vi!=InputPolygon1.vertices_end();Vi++)
	{
		if (!InputPolygon0.has_on_bounded_side(*Vi))
		{
			iResult=3;
			break;
		}
		else
		{
			iResult=1;
		}
	}
	if (iResult!=3)
	{
		return iResult;
	}

	bool bIntersect=IsPlanarPolygonsIntersect2D(InputPolygon0,InputPolygon1);
	if (bIntersect)
	{
		iResult=2;
	}
	else
	{
		iResult=3;
	}

	return iResult;
}

/*
判断两条线段是否相交(有交点)
*/
bool GeometryAlgorithm::IsLineSegmentCross(HPCPoint pFirst1, HPCPoint pFirst2, HPCPoint pSecond1, HPCPoint pSecond2)
{
	//每个线段的两点都在另一个线段的左右不同侧，则能断定线段相交
	//公式对于向量(x1,y1)->(x2,y2),判断点(x3,y3)在向量的左边,右边,还是线上.
	//p=x1(y3-y2)+x2(y1-y3)+x3(y2-y1).p<0 左侧,	p=0 线上, p>0 右侧
	double Linep1,Linep2;
	//判断pSecond1和pSecond2是否在pFirst1->pFirst2两侧
	Linep1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
		pFirst2.x * (pFirst1.y - pSecond1.y) +
		pSecond1.x * (pFirst2.y - pFirst1.y);
	Linep2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
		pFirst2.x * (pFirst1.y - pSecond2.y) +
		pSecond2.x * (pFirst2.y - pFirst1.y);
	//if (Linep1!=0 && Linep2!=0)//符号位异或为0:pSecond1和pSecond2在pFirst1->pFirst2同侧
	//{
	//	if (((Linep1>0)&&(Linep2<0))||((Linep1<0)&&(Linep2>0)))
	//	{
	//		return false;
	//	}
	//}
	if(!(Linep1==0 && Linep2==0)) //符号位异或为0:pSecond1和pSecond2在pFirst1->pFirst2同侧
	{
		if(((long)Linep1 ^ (long)Linep2) >= 0 ) 
		{
			return false;
		}
	}
	//判断pFirst1和pFirst2是否在pSecond1->pSecond2两侧
	Linep1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
		pSecond2.x * (pSecond1.y - pFirst1.y) +
		pFirst1.x * (pSecond2.y - pSecond1.y);
	Linep2 = pSecond1.x * (pFirst2.y - pSecond2.y) +
		pSecond2.x * (pSecond1.y - pFirst2.y) +
		pFirst2.x * (pSecond2.y - pSecond1.y);
	//if (Linep1!=0 && Linep2!=0)//符号位异或为0:pFirst1和pFirst2在pSecond1->pSecond2同侧
	//{
	//	if (((Linep1>0)&&(Linep2<0))||((Linep1<0)&&(Linep2>0)))
	//	{
	//		return false;
	//	}
	//}
	if(!(Linep1==0 && Linep2==0)) //符号位异或为0:pSecond1和pSecond2在pFirst1->pFirst2同侧
	{
		if (((long)Linep1 ^ (long)Linep2) >= 0)
		{
			return false;
		}
	}
	//否则判为相交
	return true;
}

int GeometryAlgorithm::GetLineSphereIntersection(Line_3 line,Sphere_3 sphere,std::vector<Point_3>& IntersectPoints)
{
	// x1,y1,z1  P1 coordinates (point of line)
	// x2,y2,z2  P2 coordinates (point of line)
	// x3,y3,z3, r  P3 coordinates and radius (sphere)
	// x,y,z   intersection coordinates
	//
	// This function returns a pointer array which first index indicates
	// the number of intersection point, followed by coordinate pairs.

	double a, b, c, mu, i ;

	Point_3 linepoint1=line.point(1);
	Point_3 linepoint2=line.point(2);
	double x1=linepoint1.x();double y1=linepoint1.y();double z1=linepoint1.z();
	double x2=linepoint2.x();double y2=linepoint2.y();double z2=linepoint2.z();
	double x3=sphere.center().x();double y3=sphere.center().y();double z3=sphere.center().z();
	a =  (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1);
	b =  2* ( (x2 - x1)*(x1 - x3)
		+ (y2 - y1)*(y1 - y3)
		+ (z2 - z1)*(z1 - z3) ) ;
	c = x3*x3+y3*y3+z3*z3+x1*x1+y1*y1 + z1*z1-2* ( x3*x1 + y3*y1 + z3*z1 ) - sphere.squared_radius() ;
	i =   b * b - 4 * a * c ;

	if ( i < 0.0 )
	{
		// no intersection
		return 0;
	}
	else if ( i == 0.0 )
	{
		// one intersection
		mu = -b/(2*a) ;
		Point_3 result(x1 + mu*(x2-x1),y1 + mu*(y2-y1),z1 + mu*(z2-z1));
		IntersectPoints.push_back(result);
		return 1;
	}
	else if ( i > 0.0 )
	{
		// first intersection
		mu = (-b + sqrt( b*b - 4*a*c )) / (2*a);
		Point_3 result0(x1 + mu*(x2-x1),y1 + mu*(y2-y1),z1 + mu*(z2-z1));
		IntersectPoints.push_back(result0);
		// second intersection
		mu = (-b - sqrt(b*b - 4*a*c )) / (2*a);
		Point_3 result1(x1 + mu*(x2-x1),y1 + mu*(y2-y1),z1 + mu*(z2-z1));
		IntersectPoints.push_back(result1);
		return 2;
	}
	return 0;
}

double GeometryAlgorithm::GetAngleBetweenTwoVectors2d(Vector_2 vecFrom,Vector_2 vecTo)
{
	vecFrom=vecFrom/sqrt(vecFrom.x()*vecFrom.x()+vecFrom.y()*vecFrom.y());
	vecTo=vecTo/sqrt(vecTo.x()*vecTo.x()+vecTo.y()*vecTo.y());
	double angle=acos(vecFrom.x()*vecTo.x()+vecFrom.y()*vecTo.y());
	angle=angle*180/CGAL_PI;
	return angle;
}

double GeometryAlgorithm::GetAngleBetweenTwoVectors2d2PI(Vector_2 vecFrom,Vector_2 vecTo)
{
	vecFrom=vecFrom/sqrt(vecFrom.x()*vecFrom.x()+vecFrom.y()*vecFrom.y());
	vecTo=vecTo/sqrt(vecTo.x()*vecTo.x()+vecTo.y()*vecTo.y());
	double angle=acos(vecFrom.x()*vecTo.x()+vecFrom.y()*vecTo.y());//radian

	angle=angle*180/CGAL_PI;//convert to angle

	if (vecFrom.x()*vecTo.y()-vecFrom.y()*vecTo.x()<0)
	{
		//		printf("cw");
		angle=360-angle;
	}
	else
	{
		//		printf("ccw");
	}
	return angle;
}

double GeometryAlgorithm::GetAngleBetweenTwoVectors3d(Vector_3 vecFrom,Vector_3 vecTo,Vector_3& vecCrossResult)
{
	vecFrom=vecFrom/sqrt(vecFrom.x()*vecFrom.x()+vecFrom.y()*vecFrom.y()+vecFrom.z()*vecFrom.z());
	vecTo=vecTo/sqrt(vecTo.x()*vecTo.x()+vecTo.y()*vecTo.y()+vecTo.z()*vecTo.z());
	double angle=acos(vecFrom.x()*vecTo.x()+vecFrom.y()*vecTo.y()+vecFrom.z()*vecTo.z());//radian

	angle=angle*180/CGAL_PI;//convert to angle

	vecCrossResult=CGAL::cross_product(vecFrom,vecTo);
	vecCrossResult=vecCrossResult/sqrt(vecCrossResult.x()*vecCrossResult.x()+
										vecCrossResult.y()*vecCrossResult.y()+
										vecCrossResult.z()*vecCrossResult.z());

	if (vecCrossResult.x()+vecCrossResult.y()+vecCrossResult.z()<0)
	{
//		printf("cw");
		angle=360-angle;
	}
	else
	{
//		printf("ccw");
	}
	return angle;
}

double GeometryAlgorithm::GetAngleBetweenTwoVectors3d(Vector_3 vecFrom,Vector_3 vecTo,bool bReturnRadius)
{
	vecFrom=vecFrom/sqrt(vecFrom.x()*vecFrom.x()+vecFrom.y()*vecFrom.y()+vecFrom.z()*vecFrom.z());
	vecTo=vecTo/sqrt(vecTo.x()*vecTo.x()+vecTo.y()*vecTo.y()+vecTo.z()*vecTo.z());
	double angle=acos(vecFrom.x()*vecTo.x()+vecFrom.y()*vecTo.y()+vecFrom.z()*vecTo.z());//radian

	if (bReturnRadius)
	{
		return angle;
	}

	angle=angle*180/CGAL_PI;//convert to angle

	return angle;
}

Point_3 GeometryAlgorithm::GetFacetCenter(Facet_handle facet)
{
	Halfedge_around_facet_circulator Hafc = facet->facet_begin();
	double dSumX,dSumY,dSumZ;
	dSumX=dSumY=dSumZ=0;
	do 
	{
		dSumX=dSumX+Hafc->vertex()->point().x();
		dSumY=dSumY+Hafc->vertex()->point().y();
		dSumZ=dSumZ+Hafc->vertex()->point().z();
	} while(++Hafc != facet->facet_begin());
	double dNum=facet->facet_degree();
	return Point_3(dSumX/dNum,dSumY/dNum,dSumZ/dNum);
}

//get the weight of weighted laplacian for the current edge(between vertex0 and vertex1)
double GeometryAlgorithm::GetWeightForWeightedLaplacian(Vertex_handle Vertex0,Vertex_handle Vertex1,int iWeightType)
{
	Halfedge_around_vertex_circulator Havc=Vertex0->vertex_begin();
	do 
	{
		if (Havc->opposite()->vertex()==Vertex1)
		{
			break;
		}
		Havc++;
	} while(Havc!=Vertex0->vertex_begin());

	Vertex_handle VertexNb0=Havc->next()->vertex();
	Vertex_handle VertexNb1=Havc->opposite()->next()->vertex();

	if (iWeightType==2)
	{
		Vector_3 EdgeVector=Vertex0->point()-Vertex1->point();
		Vector_3 NbEdgeVector0=Vertex0->point()-VertexNb0->point();
		Vector_3 NbEdgeVector1=Vertex0->point()-VertexNb1->point();

		double dAngle0=GetAngleBetweenTwoVectors3d(EdgeVector,NbEdgeVector0);
		double dAngle1=GetAngleBetweenTwoVectors3d(EdgeVector,NbEdgeVector1);

		double dSqrtDistance=sqrt(CGAL::squared_distance(Vertex0->point(),Vertex1->point()));

		double dRadius0=tan((dAngle0/2)*CGAL_PI/180);
		double dRadius1=tan((dAngle1/2)*CGAL_PI/180);
		return (dRadius0+dRadius1)/dSqrtDistance;
	}
	else if (iWeightType==3)
	{
		Vector_3 EdgeVector00=VertexNb0->point()-Vertex0->point();
		Vector_3 EdgeVector01=VertexNb0->point()-Vertex1->point();

		Vector_3 EdgeVector10=VertexNb1->point()-Vertex0->point();
		Vector_3 EdgeVector11=VertexNb1->point()-Vertex1->point();

		double dAngle0=GetAngleBetweenTwoVectors3d(EdgeVector00,EdgeVector01);
		double dAngle1=GetAngleBetweenTwoVectors3d(EdgeVector10,EdgeVector11);

		double dRadius0=1/tan((dAngle0)*CGAL_PI/180);
		double dRadius1=1/tan((dAngle1)*CGAL_PI/180);
		return (dRadius0+dRadius1)/0.5;
	}

	return 1.0;
}

double GeometryAlgorithm::GetWeightForWeightedLaplacian(Point_3 Vertex0,Point_3 Vertex1,Point_3 Vertex1Prev,Point_3 Vertex1Next,int iWeightType)
{
	if (iWeightType==3)
	{
		Vector_3 EdgeVector00=Vertex1Prev-Vertex0;
		Vector_3 EdgeVector01=Vertex1Prev-Vertex1;

		Vector_3 EdgeVector10=Vertex1Next-Vertex0;
		Vector_3 EdgeVector11=Vertex1Next-Vertex1;

		double dAngle0=GetAngleBetweenTwoVectors3d(EdgeVector00,EdgeVector01);
		double dAngle1=GetAngleBetweenTwoVectors3d(EdgeVector10,EdgeVector11);

		double dRadius0=1/tan((dAngle0)*CGAL_PI/180);
		double dRadius1=1/tan((dAngle1)*CGAL_PI/180);
		return (dRadius0+dRadius1)/0.5;
	}

	return 1.0;
}

//void GeometryAlgorithm::ComputeRotatedPlane(Point_3 StartPointOnPlane,Point_3 EndPointOnPlane,double Plane_spin,Plane_3 & plane)
//{
//	/*the rotation equation is as follows:
//	[NewNormalx    x*x*(1-cos(Plane_spin))+cos(Plane_spin),  x*y*(1-cos(Plane_spin))-z*sin(Plane_spin),x*z*(1-cos(Plane_spin))+y*sin(Plane_spin)  [OldNormalx
//	NewNormaly  =[ x*y*(1-cos(Plane_spin))+z*sin(Plane_spin),y*y*(1-cos(Plane_spin))+cos(Plane_spin),  y*z*(1-cos(Plane_spin))-x*sin(Plane_spin)] OldNormaly
//	NewNormalz]    x*z*(1-cos(Plane_spin))-y*sin(Plane_spin),y*z*(1-cos(Plane_spin))+x*sin(Plane_spin),z*z*(1-cos(Plane_spin))+cos(Plane_spin)    OldNormalz]
//
//	Normal means the normal of the plane
//	x,y,z means the unit vector of the segment
//	*/	
//
//	Plane_spin=Plane_spin*CGAL_PI/180;
//
//	double x=EndPointOnPlane.x()-StartPointOnPlane.x();
//	double y=EndPointOnPlane.y()-StartPointOnPlane.y();
//	double z=EndPointOnPlane.z()-StartPointOnPlane.z();
//	double tempx=x/sqrt(x*x+y*y+z*z);
//	double tempy=y/sqrt(x*x+y*y+z*z);
//	double tempz=z/sqrt(x*x+y*y+z*z);
//	x=tempx;y=tempy;z=tempz;
//
//	double RotationMatrix[9];
//	RotationMatrix[0]=x*x*(1-cos(Plane_spin))+cos(Plane_spin);
//	RotationMatrix[1]=x*y*(1-cos(Plane_spin))-z*sin(Plane_spin);
//	RotationMatrix[2]=x*z*(1-cos(Plane_spin))+y*sin(Plane_spin);
//	RotationMatrix[3]=x*y*(1-cos(Plane_spin))+z*sin(Plane_spin);
//	RotationMatrix[4]=y*y*(1-cos(Plane_spin))+cos(Plane_spin);
//	RotationMatrix[5]=y*z*(1-cos(Plane_spin))-x*sin(Plane_spin);
//	RotationMatrix[6]=x*z*(1-cos(Plane_spin))-y*sin(Plane_spin);
//	RotationMatrix[7]=y*z*(1-cos(Plane_spin))+x*sin(Plane_spin);
//	RotationMatrix[8]=z*z*(1-cos(Plane_spin))+cos(Plane_spin);
//
//	double dNewNormalx=RotationMatrix[0]*(plane.orthogonal_vector().x())
//						+RotationMatrix[1]*(plane.orthogonal_vector().y())
//						+RotationMatrix[2]*(plane.orthogonal_vector().z());
//	double dNewNormaly=RotationMatrix[3]*(plane.orthogonal_vector().x())
//						+RotationMatrix[4]*(plane.orthogonal_vector().y())
//						+RotationMatrix[5]*(plane.orthogonal_vector().z());
//	double dNewNormalz=RotationMatrix[6]*(plane.orthogonal_vector().x())
//		+RotationMatrix[7]*(plane.orthogonal_vector().y())
//		+RotationMatrix[8]*(plane.orthogonal_vector().z());
//
//	Vector_3 NewNormal(dNewNormalx,dNewNormaly,dNewNormalz);
//	NewNormal=NewNormal/sqrt(NewNormal.squared_length());
//	Plane_3 NewPlane(StartPointOnPlane,NewNormal);
//	plane=NewPlane;
//}

void GeometryAlgorithm::ComputeRotatedPlane(Point_3 StartPointOnPlane,Point_3 EndPointOnPlane,double Plane_spin,Plane_3 & plane)
{
	/*the rotation equation is as follows:
	cos(theta)+(1-cos(theta))*rx*rx		(1-cos(theta))*rx*ry+rz*sin(theta) (1-cos(theta))*rx*rz-ry*sin(theta)
	(1-cos(theta))*rx*ry-rz*sin(theta)  cos(theta)+(1-cos(theta))*ry*ry    (1-cos(theta))*ry*rz+rx*sin(theta)
	(1-cos(theta))*rx*rz+ry*sin(theta)  (1-cos(theta))*ry*rz-rx*sin(theta)  cos(theta)+(1-cos(theta))*rz*rz
	*/
	Plane_spin=Plane_spin*CGAL_PI/180;

	double x=EndPointOnPlane.x()-StartPointOnPlane.x();
	double y=EndPointOnPlane.y()-StartPointOnPlane.y();
	double z=EndPointOnPlane.z()-StartPointOnPlane.z();
	double tempx=x/sqrt(x*x+y*y+z*z);
	double tempy=y/sqrt(x*x+y*y+z*z);
	double tempz=z/sqrt(x*x+y*y+z*z);
	x=tempx;y=tempy;z=tempz;

	double dCosPlane_spin=cos(Plane_spin);
	double dSinPlane_spin=sin(Plane_spin);
	double RotationMatrix[9];
	RotationMatrix[0]=x*x*(1-dCosPlane_spin)+dCosPlane_spin;
	RotationMatrix[1]=x*y*(1-dCosPlane_spin)+z*dSinPlane_spin;
	RotationMatrix[2]=x*z*(1-dCosPlane_spin)-y*dSinPlane_spin;
	RotationMatrix[3]=x*y*(1-dCosPlane_spin)-z*dSinPlane_spin;
	RotationMatrix[4]=y*y*(1-dCosPlane_spin)+dCosPlane_spin;
	RotationMatrix[5]=y*z*(1-dCosPlane_spin)+x*dSinPlane_spin;
	RotationMatrix[6]=x*z*(1-dCosPlane_spin)+y*dSinPlane_spin;
	RotationMatrix[7]=y*z*(1-dCosPlane_spin)-x*dSinPlane_spin;
	RotationMatrix[8]=z*z*(1-dCosPlane_spin)+dCosPlane_spin;

	Vector_3 PlaneNormal=plane.orthogonal_vector();
	double dNewNormalx=RotationMatrix[0]*(PlaneNormal.x())
		+RotationMatrix[3]*(PlaneNormal.y())
		+RotationMatrix[6]*(PlaneNormal.z());
	double dNewNormaly=RotationMatrix[1]*(PlaneNormal.x())
		+RotationMatrix[4]*(PlaneNormal.y())
		+RotationMatrix[7]*(PlaneNormal.z());
	double dNewNormalz=RotationMatrix[2]*(PlaneNormal.x())
		+RotationMatrix[5]*(PlaneNormal.y())
		+RotationMatrix[8]*(PlaneNormal.z());

	Vector_3 NewNormal(dNewNormalx,dNewNormaly,dNewNormalz);
	Plane_3 NewPlane(StartPointOnPlane,NewNormal);
	plane=NewPlane;
}

void GeometryAlgorithm::ComputeRotatedCurve(Point_3 RotateAxisStartPoint,Point_3 RotateAxisEndPoint,
											double Point_spin,std::vector<Point_3>& Curve3d)
{
	/*translate the point to coordinate with origin at(0 0 0)
	1                                     0                          0             0
	0                                     1                          0             0
	0                                     0                          1             0
	-RotateAxisStartPoint.x()  -RotateAxisStartPoint.y()  -RotateAxisStartPoint.y() 1
	*/
	double TransMatrix[16]=
	{1,0,0,0,0,1,0,0,0,0,1,0,-RotateAxisStartPoint.x(),-RotateAxisStartPoint.y(),
	-RotateAxisStartPoint.z(),1};
	for (unsigned int i=0;i<Curve3d.size();i++)
	{
		double dNewx=TransMatrix[0]*(Curve3d.at(i).x())
			+TransMatrix[4]*(Curve3d.at(i).y())
			+TransMatrix[8]*(Curve3d.at(i).z())
			+TransMatrix[12];
		double dNewy=TransMatrix[1]*(Curve3d.at(i).x())
			+TransMatrix[5]*(Curve3d.at(i).y())
			+TransMatrix[9]*(Curve3d.at(i).z())
			+TransMatrix[13];
		double dNewz=TransMatrix[2]*(Curve3d.at(i).x())
			+TransMatrix[6]*(Curve3d.at(i).y())
			+TransMatrix[10]*(Curve3d.at(i).z())
			+TransMatrix[14];
		Curve3d.at(i)=Point_3(dNewx,dNewy,dNewz);
	}

	/*the rotation equation is as follows:
	cos(theta)+(1-cos(theta))*rx*rx		(1-cos(theta))*rx*ry+rz*sin(theta) (1-cos(theta))*rx*rz-ry*sin(theta)
	(1-cos(theta))*rx*ry-rz*sin(theta)  cos(theta)+(1-cos(theta))*ry*ry    (1-cos(theta))*ry*rz+rx*sin(theta)
	(1-cos(theta))*rx*rz+ry*sin(theta)  (1-cos(theta))*ry*rz-rx*sin(theta)  cos(theta)+(1-cos(theta))*rz*rz
	*/
	Point_spin=Point_spin*CGAL_PI/180;

	double x=RotateAxisEndPoint.x()-RotateAxisStartPoint.x();
	double y=RotateAxisEndPoint.y()-RotateAxisStartPoint.y();
	double z=RotateAxisEndPoint.z()-RotateAxisStartPoint.z();
	double tempx=x/sqrt(x*x+y*y+z*z);
	double tempy=y/sqrt(x*x+y*y+z*z);
	double tempz=z/sqrt(x*x+y*y+z*z);
	x=tempx;y=tempy;z=tempz;

	double dCosPoint_spin=cos(Point_spin);
	double dSinPoint_spin=sin(Point_spin);
	double RotationMatrix[9];
	RotationMatrix[0]=x*x*(1-dCosPoint_spin)+dCosPoint_spin;
	RotationMatrix[1]=x*y*(1-dCosPoint_spin)+z*dSinPoint_spin;
	RotationMatrix[2]=x*z*(1-dCosPoint_spin)-y*dSinPoint_spin;
	RotationMatrix[3]=x*y*(1-dCosPoint_spin)-z*dSinPoint_spin;
	RotationMatrix[4]=y*y*(1-dCosPoint_spin)+dCosPoint_spin;
	RotationMatrix[5]=y*z*(1-dCosPoint_spin)+x*dSinPoint_spin;
	RotationMatrix[6]=x*z*(1-dCosPoint_spin)+y*dSinPoint_spin;
	RotationMatrix[7]=y*z*(1-dCosPoint_spin)-x*dSinPoint_spin;
	RotationMatrix[8]=z*z*(1-dCosPoint_spin)+dCosPoint_spin;

	for (unsigned int i=0;i<Curve3d.size();i++)
	{
		double dNewx=RotationMatrix[0]*(Curve3d.at(i).x())
			+RotationMatrix[3]*(Curve3d.at(i).y())
			+RotationMatrix[6]*(Curve3d.at(i).z());
		double dNewy=RotationMatrix[1]*(Curve3d.at(i).x())
			+RotationMatrix[4]*(Curve3d.at(i).y())
			+RotationMatrix[7]*(Curve3d.at(i).z());
		double dNewz=RotationMatrix[2]*(Curve3d.at(i).x())
			+RotationMatrix[5]*(Curve3d.at(i).y())
			+RotationMatrix[8]*(Curve3d.at(i).z());
		Curve3d.at(i)=Point_3(dNewx,dNewy,dNewz);
	}

	/*translate the point back to coordinate with origin at RotateAxisStartPoint
	1                                     0                          0             0
	0                                     1                          0             0
	0                                     0                          1             0
	RotateAxisStartPoint.x()  RotateAxisStartPoint.y()  RotateAxisStartPoint.y()   1
	*/
	double TransBackMatrix[16]=
	{1,0,0,0,0,1,0,0,0,0,1,0,RotateAxisStartPoint.x(),RotateAxisStartPoint.y(),
	RotateAxisStartPoint.z(),1};
	for (unsigned int i=0;i<Curve3d.size();i++)
	{
		double dNewx=TransBackMatrix[0]*(Curve3d.at(i).x())
			+TransBackMatrix[4]*(Curve3d.at(i).y())
			+TransBackMatrix[8]*(Curve3d.at(i).z())
			+TransBackMatrix[12];
		double dNewy=TransBackMatrix[1]*(Curve3d.at(i).x())
			+TransBackMatrix[5]*(Curve3d.at(i).y())
			+TransBackMatrix[9]*(Curve3d.at(i).z())
			+TransBackMatrix[13];
		double dNewz=TransBackMatrix[2]*(Curve3d.at(i).x())
			+TransBackMatrix[6]*(Curve3d.at(i).y())
			+TransBackMatrix[10]*(Curve3d.at(i).z())
			+TransBackMatrix[14];
		Curve3d.at(i)=Point_3(dNewx,dNewy,dNewz);
	}
}

void GeometryAlgorithm::ComputeRotatedPoint(Point_3 RotateAxisStartPoint,Point_3 RotateAxisEndPoint, 
											double Point_spin,Point_3& Point3d)
{
	/*translate the point to coordinate with origin at(0 0 0)
	1                                     0                          0             0
	0                                     1                          0             0
	0                                     0                          1             0
	-RotateAxisStartPoint.x()  -RotateAxisStartPoint.y()  -RotateAxisStartPoint.y() 1
	*/
	double TransMatrix[16]=
	{1,0,0,0,0,1,0,0,0,0,1,0,-RotateAxisStartPoint.x(),-RotateAxisStartPoint.y(),
	-RotateAxisStartPoint.z(),1};

	double dNewx=TransMatrix[0]*(Point3d.x())
			+TransMatrix[4]*(Point3d.y())
			+TransMatrix[8]*(Point3d.z())
			+TransMatrix[12];
	double dNewy=TransMatrix[1]*(Point3d.x())
			+TransMatrix[5]*(Point3d.y())
			+TransMatrix[9]*(Point3d.z())
			+TransMatrix[13];
	double dNewz=TransMatrix[2]*(Point3d.x())
			+TransMatrix[6]*(Point3d.y())
			+TransMatrix[10]*(Point3d.z())
			+TransMatrix[14];
	Point3d=Point_3(dNewx,dNewy,dNewz);
	
	/*the rotation equation is as follows:
	cos(theta)+(1-cos(theta))*rx*rx		(1-cos(theta))*rx*ry+rz*sin(theta) (1-cos(theta))*rx*rz-ry*sin(theta)
	(1-cos(theta))*rx*ry-rz*sin(theta)  cos(theta)+(1-cos(theta))*ry*ry    (1-cos(theta))*ry*rz+rx*sin(theta)
	(1-cos(theta))*rx*rz+ry*sin(theta)  (1-cos(theta))*ry*rz-rx*sin(theta)  cos(theta)+(1-cos(theta))*rz*rz
	*/
	Point_spin=Point_spin*CGAL_PI/180;

	double x=RotateAxisEndPoint.x()-RotateAxisStartPoint.x();
	double y=RotateAxisEndPoint.y()-RotateAxisStartPoint.y();
	double z=RotateAxisEndPoint.z()-RotateAxisStartPoint.z();
	double tempx=x/sqrt(x*x+y*y+z*z);
	double tempy=y/sqrt(x*x+y*y+z*z);
	double tempz=z/sqrt(x*x+y*y+z*z);
	x=tempx;y=tempy;z=tempz;

	double dCosPoint_spin=cos(Point_spin);
	double dSinPoint_spin=sin(Point_spin);
	double RotationMatrix[9];
	RotationMatrix[0]=x*x*(1-dCosPoint_spin)+dCosPoint_spin;
	RotationMatrix[1]=x*y*(1-dCosPoint_spin)+z*dSinPoint_spin;
	RotationMatrix[2]=x*z*(1-dCosPoint_spin)-y*dSinPoint_spin;
	RotationMatrix[3]=x*y*(1-dCosPoint_spin)-z*dSinPoint_spin;
	RotationMatrix[4]=y*y*(1-dCosPoint_spin)+dCosPoint_spin;
	RotationMatrix[5]=y*z*(1-dCosPoint_spin)+x*dSinPoint_spin;
	RotationMatrix[6]=x*z*(1-dCosPoint_spin)+y*dSinPoint_spin;
	RotationMatrix[7]=y*z*(1-dCosPoint_spin)-x*dSinPoint_spin;
	RotationMatrix[8]=z*z*(1-dCosPoint_spin)+dCosPoint_spin;

	dNewx=RotationMatrix[0]*(Point3d.x())
		+RotationMatrix[3]*(Point3d.y())
		+RotationMatrix[6]*(Point3d.z());
	dNewy=RotationMatrix[1]*(Point3d.x())
		+RotationMatrix[4]*(Point3d.y())
		+RotationMatrix[7]*(Point3d.z());
	dNewz=RotationMatrix[2]*(Point3d.x())
		+RotationMatrix[5]*(Point3d.y())
		+RotationMatrix[8]*(Point3d.z());
	Point3d=Point_3(dNewx,dNewy,dNewz);

	/*translate the point back to coordinate with origin at RotateAxisStartPoint
	1                                     0                          0             0
	0                                     1                          0             0
	0                                     0                          1             0
	RotateAxisStartPoint.x()  RotateAxisStartPoint.y()  RotateAxisStartPoint.y()   1
	*/
	double TransBackMatrix[16]=
	{1,0,0,0,0,1,0,0,0,0,1,0,RotateAxisStartPoint.x(),RotateAxisStartPoint.y(),
	RotateAxisStartPoint.z(),1};

	dNewx=TransBackMatrix[0]*(Point3d.x())
			+TransBackMatrix[4]*(Point3d.y())
			+TransBackMatrix[8]*(Point3d.z())
			+TransBackMatrix[12];
	dNewy=TransBackMatrix[1]*(Point3d.x())
			+TransBackMatrix[5]*(Point3d.y())
			+TransBackMatrix[9]*(Point3d.z())
			+TransBackMatrix[13];
	dNewz=TransBackMatrix[2]*(Point3d.x())
			+TransBackMatrix[6]*(Point3d.y())
			+TransBackMatrix[10]*(Point3d.z())
			+TransBackMatrix[14];
	Point3d=Point_3(dNewx,dNewy,dNewz);

}

void GeometryAlgorithm::ComputeTransformedPointPos(Point3D* point,double * ModelViewMatrix)
{
	float dOldCoordinate[4]={0.0,0.0,0.0,1.0};
	float dNewCoordinate[4]={0.0,0.0,0.0,1.0};

	dOldCoordinate[0]=point->x;
	dOldCoordinate[1]=point->y;
	dOldCoordinate[2]=point->z;

	dNewCoordinate[0]=dOldCoordinate[0]*ModelViewMatrix[0]+dOldCoordinate[1]*ModelViewMatrix[4]
	+dOldCoordinate[2]*ModelViewMatrix[8]+dOldCoordinate[3]*ModelViewMatrix[12];
	dNewCoordinate[1]=dOldCoordinate[0]*ModelViewMatrix[1]+dOldCoordinate[1]*ModelViewMatrix[5]
	+dOldCoordinate[2]*ModelViewMatrix[9]+dOldCoordinate[3]*ModelViewMatrix[13];
	dNewCoordinate[2]=dOldCoordinate[0]*ModelViewMatrix[2]+dOldCoordinate[1]*ModelViewMatrix[6]
	+dOldCoordinate[2]*ModelViewMatrix[10]+dOldCoordinate[3]*ModelViewMatrix[14];

	point->x=dNewCoordinate[0];
	point->y=dNewCoordinate[1];
	point->z=dNewCoordinate[2];
}

bool GeometryAlgorithm::MultiplyMatrixAandMatrixB(std::vector<std::vector<double>> MatrixA,std::vector<std::vector<double>> MatrixB, 
												  std::vector<std::vector<double>>& Result)
{
	if (MatrixA.front().size()!=MatrixB.size())
	{
		return false;
	}
	for (unsigned int i=0;i<MatrixA.size();i++)
	{
//		DBWindowWrite("matrix row %d\t",i);
		std::vector<double> vecCurrentRow;
		for (unsigned int j=0;j<MatrixB.front().size();j++)
		{
			double dResult=0;
			for (unsigned int k=0;k<MatrixB.size();k++)
			{
				dResult=dResult+MatrixA.at(i).at(k)*MatrixB.at(k).at(j);
			}
			vecCurrentRow.push_back(dResult);
		}
		Result.push_back(vecCurrentRow);
	}
	return true;
}

void GeometryAlgorithm::GetInverseMatrix(double  *a,int  n)
{
		int   *is,*js,i,j,k,l,u,v;   
		double   d,p;   

		is   =   new   int[n*sizeof(int)];   
		js   =   new   int[n*sizeof(int)];   

		for(k=0;k<n;k++)   
		{   
			d=0.0;   
			for(i=k;i<n;i++)   
				for(j=k;j<n;j++)   
				{   
					l=i*n+j; p=fabs(a[l]);   
					if(p>d) {   d=p;   is[k]=i;   js[k]=j;   }   
				}   

				if(d+1.0==1.0)   
				{   
					delete   is;   delete   js;   
					cout<<"error   **not   inverse"<<endl;
					//return(0);   
				}   

				if(is[k]!=k)   
					for(j=0;j<n;j++)   
					{   
						u=k*n+j;   v=is[k]*n+j;   
						p=a[u];   a[u]=a[v];   a[v]=p;   
					}   

					if(js[k]!=k)   
						for(i=0;i<n;i++)   
						{   
							u=i*n+k;   v=i*n+js[k];   
							p=a[u];   a[u]=a[v];   a[v]=p;   
						}   

						l=k*n+k;   
						a[l]=1.0/a[l];   

						for(j=0;j<n;j++)   
							if(j!=k)   
							{   
								u=k*n+j;   a[u]=a[u]*a[l];   
							}   

							for(i=0;i<n;i++)   
								if(i!=k)   
									for(j=0;j<n;j++)   
										if(j!=k)   
										{   
											u=i*n+j;   
											a[u]=a[u]-a[i*n+k]*a[k*n+j];   
										}   

										for(i=0;i<n;i++)   
											if(i!=k)   
											{   
												u=i*n+k;   a[u]=-a[u]*a[l];   
											}   
		}   

		for(k=n-1;k>=0;k--)   
		{   
			if(js[k]!=k)   
				for(j=0;j<n;j++)   
				{   
					u=k*n+j;   v=js[k]*n+j;   
					p=a[u];   a[u]=a[v];   a[v]=p;   
				}   
				if(is[k]!=k)   
					for(i=0;i<n;i++)   
					{   
						u=i*n+k;   v=i*n+is[k];   
						p=a[u];   a[u]=a[v];   a[v]=p;   
					}   
		}   

		delete   is;   delete   js;   
}

int GeometryAlgorithm::FilterCurvePoint(std::vector<QPoint>& CurvePoint,double dSpaceThreshold)
{
	vector<QPoint> ResampledCurvePoint;
	ResampledCurvePoint.push_back(CurvePoint.at(0));
	for (unsigned int i=1;i<CurvePoint.size()-1;i++)
	{
		if (sqrt(pow((double)(CurvePoint.at(i).x()-ResampledCurvePoint.back().x()),2)+
			pow((double)(CurvePoint.at(i).y()-ResampledCurvePoint.back().y()),2))>dSpaceThreshold)
		{
			ResampledCurvePoint.push_back(CurvePoint.at(i));
		}
	}
	ResampledCurvePoint.push_back(CurvePoint.back());
	CurvePoint=ResampledCurvePoint;

	return CurvePoint.size();
}

int GeometryAlgorithm::ResampleCurvePoint(std::vector<QPoint>& CurvePoint,int iDesiredPointNum)
{
	//uncomment if want to show curve preprocessing process
//	iDesiredPointNum=iDesiredPointNum-1;
	double dTotalLen=0;
	for (unsigned int i=1;i<CurvePoint.size();i++)
	{
		Point_2 StartPoint(CurvePoint.at(i-1).x(),CurvePoint.at(i-1).y());
		Point_2 EndPoint(CurvePoint.at(i).x(),CurvePoint.at(i).y());
		Vector_2 vector=EndPoint-StartPoint;
		dTotalLen+=sqrt(vector.squared_length());
	}

	////for test
	//vector<double> testlen0;
	//for (unsigned int i=0;i<CurvePoint.size()-1;i++)
	//{
	//	Point_2 point0(CurvePoint.at(i).x,CurvePoint.at(i).y);
	//	Point_2 point1(CurvePoint.at(i+1).x,CurvePoint.at(i+1).y);
	//	Vector_2 vect=point1-point0;
	//	double len=sqrt(vect.squared_length());
	//	testlen0.push_back(len);
	//}


	double dNewUnitLen=dTotalLen/iDesiredPointNum;
	vector<QPoint> NewCurvePoint;
	NewCurvePoint.push_back(CurvePoint.at(0));

	Point_2 CurrentStartPoint(CurvePoint.at(0).x(),CurvePoint.at(0).y());
	int iNextCurvePointIndex=1;
	//remove -1 if want to show curve preprocessing process
	for (int i=1;i<iDesiredPointNum-1;i++)//-1
	{
		double dLeftLen=dNewUnitLen;
		for (unsigned int j=iNextCurvePointIndex;j<CurvePoint.size();j++)
		{
			Point_2 CurrentEndPoint(CurvePoint.at(j).x(),CurvePoint.at(j).y());
			Vector_2 CurrentVect=CurrentEndPoint-CurrentStartPoint;
			double dCurrentVectLen=sqrt(CurrentVect.squared_length());
			if (dCurrentVectLen>dLeftLen)
			{
				Vector_2 AddVect=CurrentVect/dCurrentVectLen*dLeftLen;
				Point_2 NewPoint=CurrentStartPoint+AddVect;
				NewCurvePoint.push_back(QPoint(NewPoint.x(),NewPoint.y()));
				CurrentStartPoint=NewPoint;
				break;
			}
			else
			{
				CurrentStartPoint=CurrentEndPoint;
				iNextCurvePointIndex++;
				dLeftLen=dLeftLen-dCurrentVectLen;
			}
		}
	}

	NewCurvePoint.push_back(CurvePoint.back());

	////for test
	//vector<double> testlen;
	//for (unsigned int i=0;i<NewCurvePoint.size()-1;i++)
	//{
	//	Point_2 point0(NewCurvePoint.at(i).x,NewCurvePoint.at(i).y);
	//	Point_2 point1(NewCurvePoint.at(i+1).x,NewCurvePoint.at(i+1).y);
	//	Vector_2 vect=point1-point0;
	//	double len=sqrt(vect.squared_length());
	//	testlen.push_back(len);
	//}

	CurvePoint=NewCurvePoint;
	return CurvePoint.size();
}

int GeometryAlgorithm::SmoothCurvePoint(std::vector<QPoint>& CurvePoint)
{
	vector<QPoint> ResampledCurvePoint=CurvePoint;
	for (unsigned int i=1;i<CurvePoint.size()-1;i++)
	{
		//CurvePoint.at(i).x=(ResampledCurvePoint.at(i-1).x+4*ResampledCurvePoint.at(i).x
		//	+ResampledCurvePoint.at(i+1).x)/6;
		CurvePoint.at(i).setX((ResampledCurvePoint.at(i-1).x()+4*ResampledCurvePoint.at(i).x()
			+ResampledCurvePoint.at(i+1).x())/6);
		//CurvePoint.at(i).y=(ResampledCurvePoint.at(i-1).y+4*ResampledCurvePoint.at(i).y
		//	+ResampledCurvePoint.at(i+1).y)/6;
		CurvePoint.at(i).setY((ResampledCurvePoint.at(i-1).y()+4*ResampledCurvePoint.at(i).y()
			+ResampledCurvePoint.at(i+1).y())/6);
	}
	//for (unsigned int i=1;i<CurvePoint.size()-1;i++)
	//{
	//	CurvePoint.at(i).x=(ResampledCurvePoint.at(i-1).x
	//						+ResampledCurvePoint.at(i+1).x)/2;
	//	CurvePoint.at(i).y=(ResampledCurvePoint.at(i-1).y
	//						+ResampledCurvePoint.at(i+1).y)/2;
	//}
	return CurvePoint.size();
}

bool GeometryAlgorithm::JudgeCurveOpen2D(std::vector<QPoint> CurvePoint)
{
	//judge the type of curve: open/closed according to the distance of the first and last point
	//get the average length
	double dTotalLen=0;
	for (unsigned int i=0;i<CurvePoint.size()-1;i++)
	{
		Point_2 CurrentPoint(CurvePoint.at(i).x(),CurvePoint.at(i).y());
		Point_2 NextPoint(CurvePoint.at(i+1).x(),CurvePoint.at(i+1).y());
		dTotalLen=dTotalLen+sqrt(CGAL::squared_distance(CurrentPoint,NextPoint));
	}
	double dAverageLen=dTotalLen/(double)(CurvePoint.size()-1);
	//get length between the first and last point
	Point_2 FirstPoint(CurvePoint.front().x(),CurvePoint.front().y());
	Point_2 FinalPoint(CurvePoint.back().x(),CurvePoint.back().y());
	double dFinalLen=sqrt(CGAL::squared_distance(FirstPoint,FinalPoint));

	//judge
	if (dFinalLen>dAverageLen*3)//open curve
	{
		return true;
	}

	return false;
}

bool GeometryAlgorithm::MakeSymmetricCurve2D(vector<QPoint>& CurvePoint)
{
	vector<Point_2> InputCurve;
	for (unsigned int i=0;i<CurvePoint.size();i++)
	{
		InputCurve.push_back(Point_2(CurvePoint.at(i).x(),CurvePoint.at(i).y()));
	}
	//take the line connecting the first and last points as the axis
	//check if all the points lie on the same side of the axis
	Line_2 Axis(InputCurve.front(),InputCurve.back());
	CGAL::Oriented_side LastSide=CGAL::ON_ORIENTED_BOUNDARY;
	vector<Point_2> MirrorCurve;
	for (unsigned int i=1;i<InputCurve.size()-1;i++)
	{
		CGAL::Oriented_side CurrentSide=Axis.oriented_side(InputCurve.at(i));
		if (LastSide!=CGAL::ON_ORIENTED_BOUNDARY && CurrentSide!=CGAL::ON_ORIENTED_BOUNDARY && LastSide!=CurrentSide)
		{
			return false;
		}
		else
		{
			LastSide=CurrentSide;
			Point_2 ProjPoint=Axis.projection(InputCurve.at(i));
			Vector_2 SymVec(InputCurve.at(i),ProjPoint);
			Point_2 MirrorPoint=ProjPoint+SymVec;
			MirrorCurve.push_back(MirrorPoint);
		}
	}
	reverse(MirrorCurve.begin(),MirrorCurve.end());
	InputCurve.insert(InputCurve.end(),MirrorCurve.begin(),MirrorCurve.end());
	CurvePoint.clear();
	for (unsigned int i=0;i<InputCurve.size();i++)
	{
		CurvePoint.push_back(QPoint(InputCurve.at(i).x(),InputCurve.at(i).y()));
	}
	return true;
}

int GeometryAlgorithm::ResampleCurvePoint3D(std::vector<Point_3>& CurvePoint,int iDesiredPointNum)
{
	//uncomment if want to show curve preprocessing process
	//	iDesiredPointNum=iDesiredPointNum-1;
	double dTotalLen=0;
	for (unsigned int i=1;i<CurvePoint.size();i++)
	{
		Point_3 StartPoint(CurvePoint.at(i-1).x(),CurvePoint.at(i-1).y(),CurvePoint.at(i-1).z());
		Point_3 EndPoint(CurvePoint.at(i).x(),CurvePoint.at(i).y(),CurvePoint.at(i).z());
		Vector_3 vector=EndPoint-StartPoint;
		dTotalLen+=sqrt(vector.squared_length());
	}

	double dNewUnitLen=dTotalLen/(iDesiredPointNum-1);
	vector<Point_3> NewCurvePoint;
	NewCurvePoint.push_back(CurvePoint.at(0));

	Point_3 CurrentStartPoint(CurvePoint.at(0).x(),CurvePoint.at(0).y(),CurvePoint.at(0).z());
	int iNextCurvePointIndex=1;
	//remove -1 if want to show curve preprocessing process
	for (int i=1;i<iDesiredPointNum-1;i++)//-1
	{
		double dLeftLen=dNewUnitLen;
		for (unsigned int j=iNextCurvePointIndex;j<CurvePoint.size();j++)
		{
			Point_3 CurrentEndPoint(CurvePoint.at(j).x(),CurvePoint.at(j).y(),CurvePoint.at(j).z());
			Vector_3 CurrentVect=CurrentEndPoint-CurrentStartPoint;
			double dCurrentVectLen=sqrt(CurrentVect.squared_length());
			if (dCurrentVectLen>dLeftLen)
			{
				Vector_3 AddVect=CurrentVect/dCurrentVectLen*dLeftLen;
				Point_3 NewPoint=CurrentStartPoint+AddVect;
				NewCurvePoint.push_back(NewPoint);
				CurrentStartPoint=NewPoint;
				break;
			}
			else
			{
				CurrentStartPoint=CurrentEndPoint;
				iNextCurvePointIndex++;
				dLeftLen=dLeftLen-dCurrentVectLen;
			}
		}
	}

	NewCurvePoint.push_back(CurvePoint.back());

	CurvePoint=NewCurvePoint;
	return CurvePoint.size();
}

int GeometryAlgorithm::LocalModifyCurve3D(std::vector<Point_3>& OriginalCurve,std::vector<Point_3> ModifyCurve,std::vector<int>& ModifiedPointInd,double dBlendPara)
{
	//find two ends points on original curve corresponds to the modify curve
	double dDistBegin,dDistEnd;
	dDistBegin=dDistEnd=9999;
	int iBeginInd,iEndInd;
	iBeginInd=iEndInd=0;
	for (unsigned int i=0;i<OriginalCurve.size();i++)
	{
		double dCurrentDistBegin=CGAL::squared_distance(ModifyCurve.front(),OriginalCurve.at(i));
		if (dCurrentDistBegin<dDistBegin)
		{
			iBeginInd=i;
			dDistBegin=dCurrentDistBegin;
		}
		double dCurrentDistEnd=CGAL::squared_distance(ModifyCurve.back(),OriginalCurve.at(i));
		if (dCurrentDistEnd<dDistEnd)
		{
			iEndInd=i;
			dDistEnd=dCurrentDistEnd;
		}
	}
	if (iBeginInd==iEndInd)
	{
		return 0;
	}
	//reverse the modify curve if iBeginInd>iEndInd
	if (iBeginInd>iEndInd)
	{
		reverse(ModifyCurve.begin(),ModifyCurve.end());
		int iTemp=iBeginInd;
		iBeginInd=iEndInd;
		iEndInd=iTemp;
	}
	//compute and store the intermediate indices on the corresponding part of original curve
	//according to the distance between middle points on this part and modify curve
	int iInterInd=iBeginInd;
	vector<int> CorreInd1;
	while (iInterInd<=iEndInd)
	{
		CorreInd1.push_back(iInterInd);
		iInterInd++;
	}

	iInterInd=iBeginInd;
	vector<int> CorreInd2;
	while (iInterInd!=iEndInd)
	{
		CorreInd2.push_back(iInterInd);
		iInterInd=(iInterInd-1+OriginalCurve.size())%OriginalCurve.size();
	}
	CorreInd2.push_back(iInterInd);
	//compare and store
	vector<int> CorreInd;
	Point_3 ModifyMidPoint=ModifyCurve.at(ModifyCurve.size()/2);
	Point_3 Part1MidPoint=OriginalCurve.at(CorreInd1.at(CorreInd1.size()/2));
	Point_3 Part2MidPoint=OriginalCurve.at(CorreInd2.at(CorreInd2.size()/2));
	if (CGAL::has_smaller_distance_to_point(ModifyMidPoint,Part1MidPoint,Part2MidPoint))
	{
		CorreInd=CorreInd1;
	}
	else
	{
		CorreInd=CorreInd2;
	}

	//resample the modify curve to make the points number same with the part on original curve
	ResampleCurvePoint3D(ModifyCurve,CorreInd.size());
	assert(CorreInd.size()==ModifyCurve.size());
	//blend the mofify curve and original curve
	Vector_3 StartMoveVector,EndMoveVector;
	for (unsigned int i=0;i<ModifyCurve.size();i++)
	{
		if (i==0)
		{
			StartMoveVector=Vector_3(OriginalCurve.at(CorreInd.front()),ModifyCurve.at(i));
		}
		if (i==ModifyCurve.size()-1)
		{
			EndMoveVector=Vector_3(OriginalCurve.at(CorreInd.back()),ModifyCurve.at(i));
		}
		double dNewX,dNewY,dNewZ;
		dNewX=ModifyCurve.at(i).x()*dBlendPara+OriginalCurve.at(CorreInd.at(i)).x()*(1-dBlendPara);
		dNewY=ModifyCurve.at(i).y()*dBlendPara+OriginalCurve.at(CorreInd.at(i)).y()*(1-dBlendPara);
		dNewZ=ModifyCurve.at(i).z()*dBlendPara+OriginalCurve.at(CorreInd.at(i)).z()*(1-dBlendPara);
		OriginalCurve.at(CorreInd.at(i))=Point_3(dNewX,dNewY,dNewZ);
	}
	//compute transition of 3 neighbor points at both sides
	for (int i=1;i<4;i++)
	{
		int iStartNBIndex,iEndNBIndex;
		if (CorreInd.at(1)>CorreInd.front())//or if CorreInd==CorreInd1
		{
			iStartNBIndex=(CorreInd.front()-i+OriginalCurve.size())%OriginalCurve.size();
			iEndNBIndex=(CorreInd.back()+i)%OriginalCurve.size();
		}
		else
		{
			iStartNBIndex=(CorreInd.front()+i)%OriginalCurve.size();
			iEndNBIndex=(CorreInd.back()-i+OriginalCurve.size())%OriginalCurve.size();
		}
		if (iStartNBIndex==iEndNBIndex)
		{
			continue;
		}
		//double dTransFactor=0.5*sin(i+0.5*CGAL_PI)+0.5;
		double dTransFactor=1-0.3*i;//kw self-defined,no theoretical base
		OriginalCurve.at(iStartNBIndex)=OriginalCurve.at(iStartNBIndex)+StartMoveVector*dTransFactor;
		OriginalCurve.at(iEndNBIndex)=OriginalCurve.at(iEndNBIndex)+EndMoveVector*dTransFactor;
	}
	//currently do not include the neighbor points into the modified points
	ModifiedPointInd=CorreInd;
	//the direction of ModifiedPointInd should be the same as OriginalCurve (cw or ccw)
	if (ModifiedPointInd.at(1)!=0 && ModifiedPointInd.at(0)!=0)
	{
		if (ModifiedPointInd.at(1)<ModifiedPointInd.at(0))
		{
			reverse(ModifiedPointInd.begin(),ModifiedPointInd.end());
		}
	}
	else if (ModifiedPointInd.at(0)==0)
	{
		if (ModifiedPointInd.at(2)<ModifiedPointInd.at(1))
		{
			reverse(ModifiedPointInd.begin(),ModifiedPointInd.end());
		}
	}
	else if (ModifiedPointInd.at(1)==0)
	{
		if (ModifiedPointInd.at(2)>ModifiedPointInd.at(0))
		{
			reverse(ModifiedPointInd.begin(),ModifiedPointInd.end());
		}
	}
	return OriginalCurve.size();
}

int GeometryAlgorithm::ProcessCurverPoint(std::vector<QPoint>& CurvePoint,double dSpaceThreshold,int iDesiredPointNum)
{
	int num=FilterCurvePoint(CurvePoint,dSpaceThreshold);

	if (iDesiredPointNum==0)
	{
		ResampleCurvePoint(CurvePoint,num);
	}
	else
	{
		ResampleCurvePoint(CurvePoint,iDesiredPointNum);
	}

	SmoothCurvePoint(CurvePoint);
	return CurvePoint.size();
}

int GeometryAlgorithm::MakeClosedStroke2d(std::vector<QPoint>& CurvePoint)
{
	std::vector<QPoint> ClosedStrokePoint;
	Polygon_2 BoundingPolygon;
	BoundingPolygon.push_back(Point_2(CurvePoint.front().x(),CurvePoint.front().y()));
	for (unsigned int i=0;i<CurvePoint.size()-1;i++)
	{
		BoundingPolygon.push_back(Point_2(CurvePoint.at(i+1).x(),CurvePoint.at(i+1).y()));
		if (!BoundingPolygon.is_simple())
		{
			break;
		}
		ClosedStrokePoint.push_back(CurvePoint.at(i));
		if (i==CurvePoint.size()-2)
		{
			ClosedStrokePoint.push_back(CurvePoint.at(i+1));
		}
	}
	CurvePoint=ClosedStrokePoint;
	//judge if the polygon is valid
	if (BoundingPolygon.size()<3)
	{
		CurvePoint.clear();
	}
	else
	{
		if (!BoundingPolygon.is_simple())
		{
			Polygon_2::Vertex_iterator Iter=BoundingPolygon.vertices_end();
			Iter--;
			BoundingPolygon.erase(Iter);
		}
		//note that the windows coordinate is inverse to CGAL
		if (BoundingPolygon.is_counterclockwise_oriented())
		{
			std::reverse(CurvePoint.begin(),CurvePoint.end());
		}
	}

	return CurvePoint.size();
}

bool GeometryAlgorithm::ReversePointsOrder3d(Point_3 OriginCurveStartPoint,vector<Point_3>& DeformCurve)
{
	if (CGAL::squared_distance(OriginCurveStartPoint,DeformCurve.at(0))<
		CGAL::squared_distance(OriginCurveStartPoint,DeformCurve.back()))
	{
		return false;
	}
	reverse(DeformCurve.begin(),DeformCurve.end());
	return true;
}

bool GeometryAlgorithm::ReversePointsOrder2d(HPCPoint OriginCurveStartPoint,vector<HPCPoint>& DeformCurve)
{
	if (CGAL::squared_distance(Point_2(OriginCurveStartPoint.x,OriginCurveStartPoint.y),
								Point_2(DeformCurve.at(0).x,DeformCurve.at(0).y))<
		CGAL::squared_distance(Point_2(OriginCurveStartPoint.x,OriginCurveStartPoint.y),
								Point_2(DeformCurve.back().x,DeformCurve.back().y)))
	{
		return false;
	}
	reverse(DeformCurve.begin(),DeformCurve.end());
	return true;
}

void GeometryAlgorithm::ExtrusionLocalRefine(KW_Mesh& NewMesh,std::vector<Facet_handle>& fhRefineTri)
{
	vector<Vertex_handle> vecNewEdgeVertex,vecOriVertex;
	DivideFacets(NewMesh,fhRefineTri,vecNewEdgeVertex,vecOriVertex);
}


void GeometryAlgorithm::DivideFacets(KW_Mesh& NewMesh,std::vector<Facet_handle>& fhRefineTri,
									 vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	//first divide the fhRefineTri into 4 parts
	for (unsigned int i=0;i<fhRefineTri.size();i++)
	{
		if (fhRefineTri.at(i)->facet_degree()==3)
		{
			DivideFacet3Eto4Tri(NewMesh,fhRefineTri.at(i),vecNewEdgeVertex,vecOriVertex);
		}
		else if(fhRefineTri.at(i)->facet_degree()==4)
		{
			DivideFacet4Eto4Tri(NewMesh,fhRefineTri.at(i),vecNewEdgeVertex,vecOriVertex);
		}
		else if(fhRefineTri.at(i)->facet_degree()==5)
		{
			DivideFacet5Eto4Tri(NewMesh,fhRefineTri.at(i),vecNewEdgeVertex,vecOriVertex);
		}
		else if(fhRefineTri.at(i)->facet_degree()==6)
		{
			DivideFacet6Eto4Tri(NewMesh,fhRefineTri.at(i),vecNewEdgeVertex,vecOriVertex);
		}
	}

	//second,divide the facet with more than 4 edges into 4 triangles 
	//until all the facets have no more than 4 edges
	while (true)
	{
		if (DivideFacetsto4Tri(NewMesh,vecNewEdgeVertex,vecOriVertex))
		{
			break;
		}
	}

	//then,divide the facet with 4 edges into 2 triangles
	for ( Facet_iterator i = NewMesh.facets_begin(); i != NewMesh.facets_end(); i++)
	{
		assert(i->facet_degree()<=4);
		if (i->facet_degree()==4)
		{
			DivideFacet4Eto2Tri(NewMesh,i);
		}
	}

	//last,check if there're facets with other than 3 edges
	for ( Facet_iterator i = NewMesh.facets_begin(); i != NewMesh.facets_end(); i++)
	{
		assert(i->facet_degree()==3);
	}
}

/*
			/ \
		 02/   \1
		  / 001 \
		 /------ \
	    / \     / \01
     2 /   \   /   \
      / 002 \ /000  \
     ------------------       
		00       0
*/
bool GeometryAlgorithm::DivideFacetsto4Tri(KW_Mesh& NewMesh,vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	bool bAllDivided=true;
	for ( Facet_iterator i = NewMesh.facets_begin(); i != NewMesh.facets_end(); i++)
	{
		assert(i->facet_degree()<=6);
		if (i->facet_degree()==6)
		{
			bAllDivided=false;
			DivideFacet6Eto4Tri(NewMesh,i,vecNewEdgeVertex,vecOriVertex);
		}
		else if (i->facet_degree()==5)
		{
			bAllDivided=false;
			DivideFacet5Eto4Tri(NewMesh,i,vecNewEdgeVertex,vecOriVertex);
		}
	}
	return bAllDivided;
}

void GeometryAlgorithm::DivideFacet6Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	Halfedge_handle hh0,hh1,hh2,hh00,hh01,hh02,hh000,hh001,hh002;
	Halfedge_around_facet_circulator i = fhRefineTri->facet_begin();
	do 
	{
		Point_3 midpoint((i->next()->vertex()->point().x()+i->prev()->vertex()->point().x())/2,
			(i->next()->vertex()->point().y()+i->prev()->vertex()->point().y())/2,
			(i->next()->vertex()->point().z()+i->prev()->vertex()->point().z())/2);
		if (i->vertex()->point()==midpoint)
		{
			hh00=i;
			hh0=hh00->next();hh01=hh0->next();hh1=hh01->next();hh02=hh1->next();hh2=hh02->next();
			assert(hh00==hh2->next());
			break;
		}
	} while(++i != fhRefineTri->facet_begin());

	//no need to store new pos for new edge vertices, since this has been done in neighbor triangles
	//no need to collect new edge and ori vertices, since this has been done in neighbor triangles

	hh000=NewMesh.split_facet(hh01,hh00);
	hh001=NewMesh.split_facet(hh02,hh000->opposite());
	hh002=NewMesh.split_facet(hh00,hh001->opposite());
	//NewMesh.erase_facet(hh00);
	//hh000=NewMesh.add_facet_to_border(hh00,hh01);
	//hh001=NewMesh.add_facet_to_border(hh000->opposite(),hh02);
	//hh002=NewMesh.add_facet_to_border(hh001->opposite(),hh00);
	//NewMesh.fill_hole(hh000->opposite());

}

void GeometryAlgorithm::DivideFacet5Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	Halfedge_handle hh0,hh1,hh2,hh00,hh01,hh02,hh000,hh001,hh002;
	Halfedge_around_facet_circulator i = fhRefineTri->facet_begin();
	do 
	{
		Point_3 midpoint((i->next()->vertex()->point().x()+i->prev()->vertex()->point().x())/2,
			(i->next()->vertex()->point().y()+i->prev()->vertex()->point().y())/2,
			(i->next()->vertex()->point().z()+i->prev()->vertex()->point().z())/2);
		Point_3 midpoint2((i->next()->next()->next()->vertex()->point().x()+i->next()->vertex()->point().x())/2,
			(i->next()->next()->next()->vertex()->point().y()+i->next()->vertex()->point().y())/2,
			(i->next()->next()->next()->vertex()->point().z()+i->next()->vertex()->point().z())/2);
		if ((i->vertex()->point()==midpoint)&&(i->next()->next()->vertex()->point()==midpoint2))
		{
			hh00=i;
			hh0=hh00->next();hh01=hh0->next();hh1=hh01->next();hh2=hh1->next();
			assert(hh00==hh2->next());
			break;
		}
	} while(++i != fhRefineTri->facet_begin());

	hh02=NewMesh.split_edge(hh2);
	hh02->vertex()->point()=Point_3((hh1->vertex()->point().x()+hh2->vertex()->point().x())/2,
									(hh1->vertex()->point().y()+hh2->vertex()->point().y())/2,
									(hh1->vertex()->point().z()+hh2->vertex()->point().z())/2);

	//store new position for newedge vertices
	hh02->vertex()->SetLRNewPos(hh2->GetNewMidPoint());
	//mark the new vertex, for later updating the roi and static vertices for deformation 
	hh02->vertex()->SetReserved(-1);

	//collect newedge and ori vertices
	vector<Vertex_handle>::iterator Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh02->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh02->vertex());
	}
	vector<Vertex_handle>::iterator IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh2->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh2->vertex());
	}


	hh000=NewMesh.split_facet(hh01,hh00);
	hh001=NewMesh.split_facet(hh02,hh000->opposite());
	hh002=NewMesh.split_facet(hh00,hh001->opposite());
	//NewMesh.erase_facet(hh00);
	//hh000=NewMesh.add_facet_to_border(hh00,hh01);
	//hh001=NewMesh.add_facet_to_border(hh000->opposite(),hh02);
	//hh002=NewMesh.add_facet_to_border(hh001->opposite(),hh00);
	//NewMesh.fill_hole(hh000->opposite());
}

void GeometryAlgorithm::DivideFacet4Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	Halfedge_handle hh0,hh1,hh2,hh00,hh01,hh02,hh000,hh001,hh002;
	Halfedge_around_facet_circulator i = fhRefineTri->facet_begin();
	do 
	{
		Point_3 midpoint((i->next()->vertex()->point().x()+i->prev()->vertex()->point().x())/2,
			(i->next()->vertex()->point().y()+i->prev()->vertex()->point().y())/2,
			(i->next()->vertex()->point().z()+i->prev()->vertex()->point().z())/2);
		if (i->vertex()->point()==midpoint)
		{
			hh00=i;
			hh0=hh00->next();hh1=hh0->next();hh2=hh1->next();
			assert(hh00==hh2->next());
			break;
		}
	} while(++i != fhRefineTri->facet_begin());

	hh01=NewMesh.split_edge(hh1);
	hh01->vertex()->point()=Point_3((hh0->vertex()->point().x()+hh1->vertex()->point().x())/2,
								(hh0->vertex()->point().y()+hh1->vertex()->point().y())/2,
								(hh0->vertex()->point().z()+hh1->vertex()->point().z())/2);

	hh02=NewMesh.split_edge(hh2);
	hh02->vertex()->point()=Point_3((hh1->vertex()->point().x()+hh2->vertex()->point().x())/2,
		(hh1->vertex()->point().y()+hh2->vertex()->point().y())/2,
		(hh1->vertex()->point().z()+hh2->vertex()->point().z())/2);


	//store new position for newedge vertices
	hh01->vertex()->SetLRNewPos(hh1->GetNewMidPoint());
	hh02->vertex()->SetLRNewPos(hh2->GetNewMidPoint());
	//mark the new vertex, for later updating the roi and static vertices for deformation 
	hh01->vertex()->SetReserved(-1);
	hh02->vertex()->SetReserved(-1);

	//collect newedge and ori vertices
	vector<Vertex_handle>::iterator Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh01->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh01->vertex());
	}
	Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh02->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh02->vertex());
	}
	vector<Vertex_handle>::iterator IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh1->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh1->vertex());
	}
	IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh2->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh2->vertex());
	}


	hh000=NewMesh.split_facet(hh01,hh00);
	hh001=NewMesh.split_facet(hh02,hh000->opposite());
	hh002=NewMesh.split_facet(hh00,hh001->opposite());
	//NewMesh.erase_facet(hh00);
	//hh000=NewMesh.add_facet_to_border(hh00,hh01);
	//hh001=NewMesh.add_facet_to_border(hh000->opposite(),hh02);
	//hh002=NewMesh.add_facet_to_border(hh001->opposite(),hh00);
	//NewMesh.fill_hole(hh000->opposite());

}

void GeometryAlgorithm::DivideFacet3Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,vector<Vertex_handle>& vecNewEdgeVertex, vector<Vertex_handle>& vecOriVertex)
{
	Halfedge_handle hh0,hh1,hh2,hh00,hh01,hh02,hh000,hh001,hh002;
	hh0=fhRefineTri->halfedge();
	hh1=hh0->next();
	hh2=hh1->next();
	assert(hh0==hh2->next());

	hh00=NewMesh.split_edge(hh0);
	hh00->vertex()->point()=Point_3((hh0->vertex()->point().x()+hh2->vertex()->point().x())/2,
		(hh0->vertex()->point().y()+hh2->vertex()->point().y())/2,
		(hh0->vertex()->point().z()+hh2->vertex()->point().z())/2);
	hh01=NewMesh.split_edge(hh1);
	hh01->vertex()->point()=Point_3((hh0->vertex()->point().x()+hh1->vertex()->point().x())/2,
		(hh0->vertex()->point().y()+hh1->vertex()->point().y())/2,
		(hh0->vertex()->point().z()+hh1->vertex()->point().z())/2);
	hh02=NewMesh.split_edge(hh2);
	hh02->vertex()->point()=Point_3((hh1->vertex()->point().x()+hh2->vertex()->point().x())/2,
		(hh1->vertex()->point().y()+hh2->vertex()->point().y())/2,
		(hh1->vertex()->point().z()+hh2->vertex()->point().z())/2);

	//store new position for newedge vertices
	hh00->vertex()->SetLRNewPos(hh0->GetNewMidPoint());
	hh01->vertex()->SetLRNewPos(hh1->GetNewMidPoint());
	hh02->vertex()->SetLRNewPos(hh2->GetNewMidPoint());
	//mark the new vertex, for later updating the roi and static vertices for deformation 
	hh00->vertex()->SetReserved(-1);
	hh01->vertex()->SetReserved(-1);
	hh02->vertex()->SetReserved(-1);

	//collect newedge and ori vertices
	vector<Vertex_handle>::iterator Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh00->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh00->vertex());
	}
	Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh01->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh01->vertex());
	}
	Iter=find(vecNewEdgeVertex.begin(),vecNewEdgeVertex.end(),hh02->vertex());
	if (Iter==vecNewEdgeVertex.end())
	{
		vecNewEdgeVertex.push_back(hh02->vertex());
	}
	vector<Vertex_handle>::iterator IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh0->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh0->vertex());
	}
	IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh1->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh1->vertex());
	}
	IterOri=find(vecOriVertex.begin(),vecOriVertex.end(),hh2->vertex());
	if (IterOri==vecOriVertex.end())
	{
		vecOriVertex.push_back(hh2->vertex());
	}

		
	hh000=NewMesh.split_facet(hh01,hh00);
	hh001=NewMesh.split_facet(hh02,hh000->opposite());
	hh002=NewMesh.split_facet(hh00,hh001->opposite());
	//NewMesh.erase_facet(hh00);
	//hh000=NewMesh.add_facet_to_border(hh00,hh01);
	//hh001=NewMesh.add_facet_to_border(hh000->opposite(),hh02);
	//hh002=NewMesh.add_facet_to_border(hh001->opposite(),hh00);
	//NewMesh.fill_hole(hh000->opposite());
}

/*
			/\
		   /| \
          / |  \
        2/  |   \1
        /   |    \
       /    |000  \
      /     |      \
      --------------
        00      0
*/

void GeometryAlgorithm::DivideFacet4Eto2Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri)
{
	Halfedge_handle hh0,hh1,hh2,hh00,hh000;
	Halfedge_around_facet_circulator i = fhRefineTri->facet_begin();
	do 
	{
		Point_3 midpoint((i->next()->vertex()->point().x()+i->prev()->vertex()->point().x())/2,
			(i->next()->vertex()->point().y()+i->prev()->vertex()->point().y())/2,
			(i->next()->vertex()->point().z()+i->prev()->vertex()->point().z())/2);
		if (i->vertex()->point()==midpoint)
		{
			hh00=i;
			hh0=hh00->next();hh1=hh0->next();hh2=hh1->next();
			assert(hh00==hh2->next());
			break;
		}
	} while(++i != fhRefineTri->facet_begin());

	//hh00=NewMesh.split_edge(hh0);
	//hh00->vertex()->point()=Point_3((hh0->vertex()->point().x()+hh2->vertex()->point().x())/2,
	//	(hh0->vertex()->point().y()+hh2->vertex()->point().y())/2,
	//	(hh0->vertex()->point().z()+hh2->vertex()->point().z())/2);


	hh000=NewMesh.split_facet(hh1,hh00);
}



bool GeometryAlgorithm::GetPlaneBoundaryPoints(Point_3 StartPointProj,Point_3 EndPointProj,
												 Plane_3 plane,Point3D* PlaneBoundaryPoints)
{
	/*
	Bpoint0												        		Bpoint3



	EndPoint0          StartPointProj   MidPoint    EndPointProj        EndPoint1                  



	Bpoint1														        Bpoint2
	*/
	Vector_3 vector0=StartPointProj-EndPointProj;
	Point_3 EndPoint0=StartPointProj+2*vector0;
	Vector_3 vector1=-vector0;
	Point_3 EndPoint1=EndPointProj+2*vector1;

	Point_3 MidPoint((EndPoint0.x()+EndPoint1.x())/2,(EndPoint0.y()+EndPoint1.y())/2,
		(EndPoint0.z()+EndPoint1.z())/2);
	Point_3 Bpoint03d=MidPoint;
	ComputeRotatedPoint(EndPoint0,EndPoint0+plane.orthogonal_vector(),90,Bpoint03d);
	Point_3 Bpoint13d=MidPoint;
	ComputeRotatedPoint(EndPoint0,EndPoint0+plane.orthogonal_vector(),-90,Bpoint13d);
	Point_3 Bpoint23d=MidPoint;
	ComputeRotatedPoint(EndPoint1,EndPoint1+plane.orthogonal_vector(),90,Bpoint23d);
	Point_3 Bpoint33d=MidPoint;
	ComputeRotatedPoint(EndPoint1,EndPoint1+plane.orthogonal_vector(),-90,Bpoint33d);

	PlaneBoundaryPoints[0].x=Bpoint03d.x();
	PlaneBoundaryPoints[0].y=Bpoint03d.y();
	PlaneBoundaryPoints[0].z=Bpoint03d.z();
	PlaneBoundaryPoints[1].x=Bpoint13d.x();
	PlaneBoundaryPoints[1].y=Bpoint13d.y();
	PlaneBoundaryPoints[1].z=Bpoint13d.z();
	PlaneBoundaryPoints[2].x=Bpoint23d.x();
	PlaneBoundaryPoints[2].y=Bpoint23d.y();
	PlaneBoundaryPoints[2].z=Bpoint23d.z();
	PlaneBoundaryPoints[3].x=Bpoint33d.x();
	PlaneBoundaryPoints[3].y=Bpoint33d.y();
	PlaneBoundaryPoints[3].z=Bpoint33d.z();
	//	bool test0=plane.has_on(Bpoint03d);
	//	bool test1=plane.has_on(Bpoint13d);
	//	bool test2=plane.has_on(Bpoint23d);
	//	bool test3=plane.has_on(Bpoint33d);

	return true;
}

void GeometryAlgorithm::AdjustPlaneBoundary(int iIncrease,Point3D* PlaneBoundaryPoints)
{
	Point_3 BoundaryPoints[4];
	for (int i=0;i<4;i++)
	{
		BoundaryPoints[i]=Point_3(PlaneBoundaryPoints[i].x,PlaneBoundaryPoints[i].y,PlaneBoundaryPoints[i].z);
	}
	Point_3 Centroid=CGAL::centroid(BoundaryPoints[0],BoundaryPoints[1],BoundaryPoints[2],BoundaryPoints[3]);
	for (int i=0;i<4;i++)
	{
		Vector_3 vector=BoundaryPoints[i]-Centroid;
		Point_3 NewEnd;
		if (iIncrease>0)
		{
			NewEnd=Centroid+vector*(double)1.2;//1.2;
		}
		else
		{
			NewEnd=Centroid+vector*(double)0.8;//(0.8);
		}
		PlaneBoundaryPoints[i].x=NewEnd.x();
		PlaneBoundaryPoints[i].y=NewEnd.y();
		PlaneBoundaryPoints[i].z=NewEnd.z();
	}
}

void GeometryAlgorithm::AdjustPlaneBoundary(int iIncrease,Point_3* PlaneBoundaryPoints)
{
	Point_3 Centroid=CGAL::centroid(PlaneBoundaryPoints[0],PlaneBoundaryPoints[1],PlaneBoundaryPoints[2],PlaneBoundaryPoints[3]);
	for (int i=0;i<4;i++)
	{
		Vector_3 vector=PlaneBoundaryPoints[i]-Centroid;
		Point_3 NewEnd;
		if (iIncrease>0)
		{
			NewEnd=Centroid+vector*(double)1.05;//1.2;
		}
		else
		{
			NewEnd=Centroid+vector*(double)0.95;//(0.8);
		}
		PlaneBoundaryPoints[i]=NewEnd;
	}
}

void GeometryAlgorithm::GetNearestPointOnCurve(Point_3 RefPoint,std::vector<Point_3> Curve,
													  bool bRefPointOnCurve,int iRefPointInd, 
													  double& dMinSquaredDist,int& iNearestInd)
{
	double dMinDist=9999.0;
	for (unsigned int i=0;i<Curve.size();i++)
	{
		if (bRefPointOnCurve&&i==iRefPointInd)
		{
			continue;
		}
		double dDist=CGAL::squared_distance(RefPoint,Curve.at(i));
		if (dDist<dMinDist)
		{
			dMinDist=dDist;
			iNearestInd=i;
		}
	}
	dMinSquaredDist=dMinDist;
}

void GeometryAlgorithm::GetFurthestPointOnCurve(Point_3 RefPoint,std::vector<Point_3> Curve,
													   bool bRefPointOnCurve,int iRefPointInd, 
													   double& dMaxSquaredDist,int& iFurthestInd)
{
	double dMaxDist=0.0;
	for (unsigned int i=0;i<Curve.size();i++)
	{
		if (bRefPointOnCurve&&i==iRefPointInd)
		{
			continue;
		}
		double dDist=CGAL::squared_distance(RefPoint,Curve.at(i));
		if (dDist>dMaxDist)
		{
			dMaxDist=dDist;
			iFurthestInd=i;
		}
	}
	dMaxSquaredDist=dMaxDist;
}

int GeometryAlgorithm::GetClosedCurvePlaneIntersection(std::vector<Point_3> Curve,Plane_3 plane,
													   std::vector<std::vector<int> >& SegPoinsInd,
														std::vector<Point_3>& InterSectPoints)
{
	for (unsigned int i=0;i<Curve.size();i++)
	{
		Point_3 StartPoint,EndPoint;
		int SegPointsId[2];
		if (i==Curve.size()-1)
		{
			SegPointsId[0]=i;
			SegPointsId[1]=0;
		}
		else
		{
			SegPointsId[0]=i;
			SegPointsId[1]=i+1;
		}

		StartPoint=Curve.at(SegPointsId[0]);
		EndPoint=Curve.at(SegPointsId[1]);

		CGAL::Object result;
		Segment_3 seg(StartPoint,EndPoint);
		Point_3 IntersectPoint;
		result = CGAL::intersection(seg, plane);
		if (CGAL::assign(IntersectPoint, result)) 
		{
			vector<Point_3>::iterator Iter=find(InterSectPoints.begin(),InterSectPoints.end(),IntersectPoint);
			//in case the intersectpoint is a vertex on the curve,adopt just one segment
			if (Iter==InterSectPoints.end())
			{
				vector<int> temp;
				temp.push_back(SegPointsId[0]);
				temp.push_back(SegPointsId[1]);
				SegPoinsInd.push_back(temp);
				InterSectPoints.push_back(IntersectPoint);
			}
		}
	}
	assert(SegPoinsInd.size()==InterSectPoints.size());
	return InterSectPoints.size();
}

void GeometryAlgorithm::GroupNearestPoints(std::vector<Point_3> vecPoint0,std::vector<Point_3> vecPoint1, 
										   std::vector<Int_Int_Pair>& GroupResult,std::vector<Point_3>& vecMidPoint)//,std::vector<Point_3>& vecMidPoint
{
	assert(vecPoint0.size()==vecPoint1.size());
	vector<int> CheckedList;
	for (unsigned int i=0;i<vecPoint0.size();i++)
	{
		vector<double> vecDistance;
		for (unsigned int j=0;j<vecPoint1.size();j++)
		{
			vector<int>::iterator pFind=find(CheckedList.begin(),CheckedList.end(),j);
			if (pFind==CheckedList.end())
			{
				vecDistance.push_back(CGAL::squared_distance(vecPoint0.at(i),vecPoint1.at(j)));
			}
			else
			{
				vecDistance.push_back(99999999);
			}
		}
		vector<double>::iterator pMin=min_element(vecDistance.begin(),vecDistance.end());
		int iIndex=pMin-vecDistance.begin();
		CheckedList.push_back(iIndex);
		//vector<int> CurrentGroup;
		//CurrentGroup.push_back(i);CurrentGroup.push_back(iIndex);
		//GroupResult.push_back(CurrentGroup);
		GroupResult.push_back(make_pair(i,iIndex));
	}
	assert(GroupResult.size()==vecPoint0.size());
	for (unsigned int i=0;i<GroupResult.size();i++)
	{
		Point_3 MidPoint=CGAL::midpoint(vecPoint0.at(GroupResult.at(i).first),
			vecPoint1.at(GroupResult.at(i).second));
		vecMidPoint.push_back(MidPoint);
	}
}

int GeometryAlgorithm::GetMeshPlaneIntersection(KW_Mesh mesh,Plane_3 plane,vector<vector<Point_3>>& IntersectCurves)
{
	//collect the triangles which intersect with the plane
	vector<Facet_iterator> vecIntersectFacet;
	for ( Facet_iterator Fi=mesh.facets_begin(); Fi!=mesh.facets_end(); Fi++)
	{
		Halfedge_around_facet_circulator Hafc = Fi->facet_begin();
		Point_3 TriVertex[3];
		int index=0;
		do 
		{
			TriVertex[index]=Hafc->vertex()->point();
			index++;
			Hafc++;
		} while( Hafc!= Fi->facet_begin());
		Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);

		if (CGAL::do_intersect(CurrentTri,plane))
		{
			vecIntersectFacet.push_back(Fi);
		}
	}

	//DBWindowWrite("intersect facet num: %d\n",vecIntersectFacet.size());

	while (!vecIntersectFacet.empty())
	{
		Facet_handle FhFirst=vecIntersectFacet.front();
		//vecIntersectFacet.erase(vecIntersectFacet.begin());
		Facet_handle FhCurrent,FhLast;
		FhCurrent=FhFirst;FhLast=FhFirst;
		vector<Point_3> CurrentIntersectCurve;

		//int iMaxTime=999999;
		//int iTime=0;
		//if (iTime==iMaxTime)
		//{
		//	DBWindowWrite("compute plane mesh intersection error\n");
		//	break;
		//}
		//iTime++;

		do
		{
			Halfedge_around_facet_circulator Hafc = FhCurrent->facet_begin();
			do 
			{
				Segment_3 CurrentEdge(Hafc->vertex()->point(),Hafc->opposite()->vertex()->point());
				Segment_3 ResultSeg;
				Point_3 ResultPoint;
				CGAL::Object result = CGAL::intersection(CurrentEdge, plane);
				if (CGAL::assign(ResultPoint,result)) 
				{
					// handle the point intersection case.
					if (Hafc->opposite()->facet()!=FhLast)
					{
						//intersection point is a vertex
						if (ResultPoint==Hafc->vertex()->point())
						{
							Halfedge_around_vertex_circulator Havc=Hafc->vertex()->vertex_begin();
							do 
							{
								Segment_3 OppSeg(Havc->prev()->vertex()->point(),Havc->next()->vertex()->point());
								if ((CGAL::do_intersect(OppSeg,plane))&&(Havc->facet()!=FhCurrent))
								{
									break;
								}
								Havc++;
							} while(Havc!=Hafc->vertex()->vertex_begin());

							FhLast=FhCurrent;
							FhCurrent=Havc->facet();
							assert(FhLast!=FhCurrent);
							CurrentIntersectCurve.push_back(ResultPoint);
							vector<Facet_iterator>::iterator Iter=find(vecIntersectFacet.begin(),vecIntersectFacet.end(), FhLast);
							if (Iter!=vecIntersectFacet.end())
							{
								vecIntersectFacet.erase(Iter);
							}
						}
						//intersection point is a vertex
						else if (ResultPoint==Hafc->opposite()->vertex()->point())
						{
							Halfedge_around_vertex_circulator Havc=Hafc->opposite()->vertex()->vertex_begin();
							do 
							{
								Segment_3 OppSeg(Havc->prev()->vertex()->point(),Havc->next()->vertex()->point());
								if ((CGAL::do_intersect(OppSeg,plane))&&(Havc->facet()!=FhCurrent))
								{
									break;
								}
								Havc++;
							} while(Havc!=Hafc->opposite()->vertex()->vertex_begin());

							FhLast=FhCurrent;
							FhCurrent=Havc->facet();
							assert(FhLast!=FhCurrent);
							CurrentIntersectCurve.push_back(ResultPoint);
							vector<Facet_iterator>::iterator Iter=find(vecIntersectFacet.begin(),vecIntersectFacet.end(), FhLast);
							if (Iter!=vecIntersectFacet.end())
							{
								vecIntersectFacet.erase(Iter);
							}
						}
						//intersection point is not a vertex
						else
						{
							FhLast=FhCurrent;
							FhCurrent=Hafc->opposite()->facet();
							assert(FhLast!=FhCurrent);
							CurrentIntersectCurve.push_back(ResultPoint);
							vector<Facet_iterator>::iterator Iter=find(vecIntersectFacet.begin(),vecIntersectFacet.end(), FhLast);
							if (Iter!=vecIntersectFacet.end())
							{
								vecIntersectFacet.erase(Iter);
							}
						}
						break;
					}
				}
				else if (CGAL::assign(ResultSeg,result)) 
				{
					// handle the segment intersection case.
					cout<<"segment lie on plane\n";
				} 
				else 
				{
					// handle the no intersection case.
				}
				Hafc++;
			} while( Hafc!= FhCurrent->facet_begin());
			//DBWindowWrite("intersect facet num: %d\n",vecIntersectFacet.size());
		}while (FhCurrent!=FhFirst);

		if (!CurrentIntersectCurve.empty())
		{
			IntersectCurves.push_back(CurrentIntersectCurve);
		}
	}

	return IntersectCurves.size();
}

bool GeometryAlgorithm::JudgeMeshOpenCurveIntersec(KW_Mesh mesh,vector<Point_3> OpenCurve)
{
	vector<Triangle_3> vecFacets;
	for ( Facet_iterator Fi=mesh.facets_begin(); Fi!=mesh.facets_end(); Fi++)
	{
		Halfedge_around_facet_circulator Hafc = Fi->facet_begin();
		Point_3 TriVertex[3];
		int index=0;
		do 
		{
			TriVertex[index]=Hafc->vertex()->point();
			index++;
			Hafc++;
		} while( Hafc!= Fi->facet_begin());
		Triangle_3 CurrentTri(TriVertex[0],TriVertex[1],TriVertex[2]);
		vecFacets.push_back(CurrentTri);
	}

	for (unsigned int i=0;i<OpenCurve.size()-1;i++)
	{
		Segment_3 CurrentSeg(OpenCurve.at(i),OpenCurve.at(i+1));
		
		for (unsigned int j=0;j<vecFacets.size();j++)
		{
			if (CGAL::do_intersect(vecFacets.at(j),CurrentSeg))
			{
				return true;
			}
		}
	}

	return false;
}

void GeometryAlgorithm::SolveLinearEquation(std::vector<std::vector<double>> A,std::vector<double>B,
											std::vector<double>& X)
{

}

void GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(KW_Mesh& mesh)
{
	mesh.normalize_border();
	for ( Vertex_iterator i = mesh.vertices_begin(); i != mesh.vertices_end(); i++)
	{
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		double dSumaArea=0;
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
			dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
			dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();

			Facet_handle CurrentFacet=Havc->facet();
			if (true)//CurrentFacet->facet_degree()==3
			{
				Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
				dSumaArea=dSumaArea+std::sqrt(TriFace.squared_area());
			}
			Havc++;
		} while(Havc!=i->vertex_begin());

		double dNeighborNum=i->vertex_degree();
		//dLaplacian[0]=i->point().x()-dLaplacian[0]/dNeighborNum;
		//dLaplacian[1]=i->point().y()-dLaplacian[1]/dNeighborNum;
		//dLaplacian[2]=i->point().z()-dLaplacian[2]/dNeighborNum;
		dLaplacian[0]=dNeighborNum*(i->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dNeighborNum*(i->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dNeighborNum*(i->point().z())-dLaplacian[2]; 

		i->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		i->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(KW_Mesh& mesh,int iWeightType)
{
	for ( Vertex_iterator i = mesh.vertices_begin(); i != mesh.vertices_end(); i++)
	{
		double dLaplacian[3],dSumWeight;
		vector<double> EdgeWeights;
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=dSumWeight=0;
		double dSumaArea=0;
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			double dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(i,Havc->opposite()->vertex(),
				iWeightType);
			EdgeWeights.push_back(dCurrentWeight);
			dSumWeight=dSumWeight+dCurrentWeight;
			dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Havc->opposite()->vertex()->point().x();
			dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Havc->opposite()->vertex()->point().y();
			dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Havc->opposite()->vertex()->point().z();

			Facet_handle CurrentFacet=Havc->facet();
			if (CurrentFacet->is_triangle())
			{
				Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
				dSumaArea=dSumaArea+std::sqrt(TriFace.squared_area());
			}

			Havc++;
		} while(Havc!=i->vertex_begin());

		//double dNeighborNum=i->vertex_degree();
		//dLaplacian[0]=i->point().x()-dLaplacian[0]/dNeighborNum;
		//dLaplacian[1]=i->point().y()-dLaplacian[1]/dNeighborNum;
		//dLaplacian[2]=i->point().z()-dLaplacian[2]/dNeighborNum;
		dLaplacian[0]=dSumWeight*(i->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dSumWeight*(i->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dSumWeight*(i->point().z())-dLaplacian[2]; 

		i->SetWeightedLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		i->SetEdgeWeights(EdgeWeights);
		i->SetWeightedLaplacianSumWeight(dSumWeight);
		i->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(std::vector<Vertex_handle>& Vertices)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		double dSumaArea=0;
		Halfedge_around_vertex_circulator Havc=Vertices.at(i)->vertex_begin();
		do 
		{
			dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
			dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
			dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();

			//Facet_handle CurrentFacet=Havc->facet();
			//if (CurrentFacet->is_triangle())
			//{
			//	Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
			//	dSumaArea=dSumaArea+std::sqrt(TriFace.squared_area());
			//}

			Havc++;
		} while(Havc!=Vertices.at(i)->vertex_begin());

		double dNeighborNum=Vertices.at(i)->vertex_degree();
		
		//dLaplacian[0]=i->point().x()-dLaplacian[0]/dNeighborNum;
		//dLaplacian[1]=i->point().y()-dLaplacian[1]/dNeighborNum;
		//dLaplacian[2]=i->point().z()-dLaplacian[2]/dNeighborNum;
		dLaplacian[0]=dNeighborNum*(Vertices.at(i)->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dNeighborNum*(Vertices.at(i)->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dNeighborNum*(Vertices.at(i)->point().z())-dLaplacian[2]; 

		Vertices.at(i)->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		Vertices.at(i)->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(std::vector<Vertex_handle>& Vertices,int iWeightType)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		double dLaplacian[3],dSumWeight;
		vector<double> EdgeWeights;
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=dSumWeight=0;
		double dSumaArea=0;
		Halfedge_around_vertex_circulator Havc=Vertices.at(i)->vertex_begin();
		do 
		{
			double dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(Vertices.at(i),
				Havc->opposite()->vertex(),iWeightType);
			EdgeWeights.push_back(dCurrentWeight);
			dSumWeight=dSumWeight+dCurrentWeight;
			dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Havc->opposite()->vertex()->point().x();
			dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Havc->opposite()->vertex()->point().y();
			dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Havc->opposite()->vertex()->point().z();

			Facet_handle CurrentFacet=Havc->facet();
			if (CurrentFacet->is_triangle())
			{
				Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
				dSumaArea=dSumaArea+std::sqrt(TriFace.squared_area());
			}

			Havc++;
		} while(Havc!=Vertices.at(i)->vertex_begin());

		dLaplacian[0]=dSumWeight*(Vertices.at(i)->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dSumWeight*(Vertices.at(i)->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dSumWeight*(Vertices.at(i)->point().z())-dLaplacian[2]; 

		Vertices.at(i)->SetWeightedLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		Vertices.at(i)->SetEdgeWeights(EdgeWeights);
		Vertices.at(i)->SetWeightedLaplacianSumWeight(dSumWeight);
		Vertices.at(i)->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeCGALDualMeshUniformLaplacian(std::vector<Vertex_handle>& Vertices)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		vector<Point_3> NbPoint;
		Halfedge_around_vertex_circulator Havc=Vertices.at(i)->vertex_begin();
		do 
		{
			dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
			dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
			dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();

			NbPoint.push_back(Havc->opposite()->vertex()->point());
			Havc++;
		} while(Havc!=Vertices.at(i)->vertex_begin());

		double dNeighborNum=Vertices.at(i)->vertex_degree();
		dLaplacian[0]=dNeighborNum*(Vertices.at(i)->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dNeighborNum*(Vertices.at(i)->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dNeighborNum*(Vertices.at(i)->point().z())-dLaplacian[2]; 

		Triangle_3 BaseTri(NbPoint.at(0),NbPoint.at(1),NbPoint.at(2));
		double dSumaArea=std::sqrt(BaseTri.squared_area());

		Vertices.at(i)->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		Vertices.at(i)->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeCGALDualMeshWeightedLaplacian(std::vector<Vertex_handle>& Vertices,int iWeightType)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		double dLaplacian[3],dSumWeight;
		vector<double> EdgeWeights;
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=dSumWeight=0;
		vector<Point_3> NbPoint;
		Halfedge_around_vertex_circulator Havc=Vertices.at(i)->vertex_begin();
		do 
		{
			NbPoint.push_back(Havc->opposite()->vertex()->point());
			Havc++;
		} while(Havc!=Vertices.at(i)->vertex_begin());
		
		double dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(Vertices.at(i)->point(),NbPoint.at(0),
			NbPoint.at(1),NbPoint.at(2),iWeightType);
		EdgeWeights.push_back(dCurrentWeight);
		dSumWeight=dSumWeight+dCurrentWeight;
		dLaplacian[0]=dLaplacian[0]+dCurrentWeight*NbPoint.at(0).x();
		dLaplacian[1]=dLaplacian[1]+dCurrentWeight*NbPoint.at(0).y();
		dLaplacian[2]=dLaplacian[2]+dCurrentWeight*NbPoint.at(0).z();

		dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(Vertices.at(i)->point(),NbPoint.at(1),
			NbPoint.at(0),NbPoint.at(2),iWeightType);
		EdgeWeights.push_back(dCurrentWeight);
		dSumWeight=dSumWeight+dCurrentWeight;
		dLaplacian[0]=dLaplacian[0]+dCurrentWeight*NbPoint.at(1).x();
		dLaplacian[1]=dLaplacian[1]+dCurrentWeight*NbPoint.at(1).y();
		dLaplacian[2]=dLaplacian[2]+dCurrentWeight*NbPoint.at(1).z();

		dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(Vertices.at(i)->point(),NbPoint.at(2),
			NbPoint.at(0),NbPoint.at(1),iWeightType);
		EdgeWeights.push_back(dCurrentWeight);
		dSumWeight=dSumWeight+dCurrentWeight;
		dLaplacian[0]=dLaplacian[0]+dCurrentWeight*NbPoint.at(2).x();
		dLaplacian[1]=dLaplacian[1]+dCurrentWeight*NbPoint.at(2).y();
		dLaplacian[2]=dLaplacian[2]+dCurrentWeight*NbPoint.at(2).z();


		dLaplacian[0]=dSumWeight*(Vertices.at(i)->point().x())-dLaplacian[0]; 
		dLaplacian[1]=dSumWeight*(Vertices.at(i)->point().y())-dLaplacian[1]; 
		dLaplacian[2]=dSumWeight*(Vertices.at(i)->point().z())-dLaplacian[2]; 

		Triangle_3 BaseTri(NbPoint.at(0),NbPoint.at(1),NbPoint.at(2));
		double dSumaArea=std::sqrt(BaseTri.squared_area());

		Vertices.at(i)->SetWeightedLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		Vertices.at(i)->SetEdgeWeights(EdgeWeights);
		Vertices.at(i)->SetWeightedLaplacianSumWeight(dSumWeight);
		Vertices.at(i)->SetSumArea(dSumaArea);
	}
}

void GeometryAlgorithm::ComputeOldEdgeVectors(KW_Mesh& mesh)
{
	for ( Vertex_iterator i = mesh.vertices_begin(); i != mesh.vertices_end(); i++)
	{
		vector<Vector_3> OldEdges;
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			Vector_3 CurrentEdge=i->point()-Havc->opposite()->vertex()->point();
			OldEdges.push_back(CurrentEdge);
			Havc++;
		} while(Havc!=i->vertex_begin());
		i->SetOldEdgeVectors(OldEdges);
	}
}

void GeometryAlgorithm::ComputeOldEdgeVectors(std::vector<Vertex_handle>& Vertices)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		vector<Vector_3> OldEdges;
		Halfedge_around_vertex_circulator Havc=Vertices.at(i)->vertex_begin();
		do 
		{
			Vector_3 CurrentEdge=Vertices.at(i)->point()-Havc->opposite()->vertex()->point();
			OldEdges.push_back(CurrentEdge);
			Havc++;
		} while(Havc!=Vertices.at(i)->vertex_begin());
		Vertices.at(i)->SetOldEdgeVectors(OldEdges);
	}
}

//judge if vertex i and j are neighbors,return the index of j among all the neighbors of i(start from 1)
int GeometryAlgorithm::JudgeIfNeighbors(Vertex_handle i,Vertex_handle j)
{
	int iIndex=1;
	Halfedge_around_vertex_circulator Havc=i->vertex_begin();
	do 
	{
		if (j==Havc->opposite()->vertex())
		{
			return iIndex;
		}
		iIndex++;
		Havc++;
	} while(Havc!=i->vertex_begin());

	return 0;
}

//judge if Dual vertex i and j are neighbors
bool GeometryAlgorithm::JudgeIfNeighbors(DualMeshVertexStruct i,int j)
{
	vector<int>::iterator pFind=find(i.vecDualNeighborIndex.begin(),i.vecDualNeighborIndex.end(),j);
	if (pFind!=i.vecDualNeighborIndex.end())
	{
		return true;
	}

	return false;
}

void GeometryAlgorithm::SetOrderForVer(std::vector<Vertex_handle>& Vertices)
{
	for (unsigned int i=0;i<Vertices.size();i++)
	{
		Vertices.at(i)->SetReserved(i);
	}
}

void GeometryAlgorithm::LaplacianSmooth(int iIterNum,double dLambda,std::vector<Vertex_handle>& vecVertexToSmooth)
{
	for (int i=0;i<iIterNum;i++)
	{
		vector<Point_3> tempPointPos;
		for (unsigned int j=0;j<vecVertexToSmooth.size();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=vecVertexToSmooth.at(j)->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=vecVertexToSmooth.at(j)->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=vecVertexToSmooth.at(j)->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}
			
			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-vecVertexToSmooth.at(j)->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dLambda;
			dVecY=CurrenVector.y()*dLambda;
			dVecZ=CurrenVector.z()*dLambda;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);
			
			tempPointPos.push_back(vecVertexToSmooth.at(j)->point()+VecToMove);
		}

		for (unsigned int j=0;j<tempPointPos.size();j++)
		{
			vecVertexToSmooth.at(j)->point()=tempPointPos.at(j);
		}
	}
}

void GeometryAlgorithm::LaplacianSmooth(int iIterNum,double dLambda,KW_Mesh& Mesh)
{
	for (int i=0;i<iIterNum;i++)
	{
		vector<Point_3> tempPointPos;
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=j->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=j->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=j->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}

			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-j->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dLambda;
			dVecY=CurrenVector.y()*dLambda;
			dVecZ=CurrenVector.z()*dLambda;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);

			tempPointPos.push_back(j->point()+VecToMove);
		}

		int iIndex=0;
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			j->point()=tempPointPos.at(iIndex);
			iIndex++;
		}
	}
	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	Mesh.SetRenderInfo(true,true,false,false,false);
}

void GeometryAlgorithm::TaubinLambdaMuSmooth(int iIterNum,double dLambda,double dMu,KW_Mesh& Mesh)
{
	for (int i=0;i<iIterNum;i++)
	{
		//first step
		vector<Point_3> tempPointPos;
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=j->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=j->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=j->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}

			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-j->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dLambda;
			dVecY=CurrenVector.y()*dLambda;
			dVecZ=CurrenVector.z()*dLambda;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);

			tempPointPos.push_back(j->point()+VecToMove);
		}

		int iIndex=0;
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			j->point()=tempPointPos.at(iIndex);
			iIndex++;
		}

		//second step
		tempPointPos.clear();
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=j->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=j->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=j->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}

			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-j->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dMu;
			dVecY=CurrenVector.y()*dMu;
			dVecZ=CurrenVector.z()*dMu;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);

			tempPointPos.push_back(j->point()+VecToMove);
		}

		iIndex=0;
		for (Vertex_iterator j=Mesh.vertices_begin();j!=Mesh.vertices_end();j++)
		{
			j->point()=tempPointPos.at(iIndex);
			iIndex++;
		}
	}
	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	Mesh.SetRenderInfo(true,true,false,false,false);
}

void GeometryAlgorithm::TaubinLambdaMuSmooth(int iIterNum,double dLambda,double dMu,std::vector<Vertex_handle>& vecVertexToSmooth)
{
	for (int i=0;i<iIterNum;i++)
	{
		//first step
		vector<Point_3> tempPointPos;
		for (unsigned int j=0;j<vecVertexToSmooth.size();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=vecVertexToSmooth.at(j)->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=vecVertexToSmooth.at(j)->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=vecVertexToSmooth.at(j)->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}

			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-vecVertexToSmooth.at(j)->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dLambda;
			dVecY=CurrenVector.y()*dLambda;
			dVecZ=CurrenVector.z()*dLambda;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);

			tempPointPos.push_back(vecVertexToSmooth.at(j)->point()+VecToMove);
		}

		for (unsigned int j=0;j<tempPointPos.size();j++)
		{
			vecVertexToSmooth.at(j)->point()=tempPointPos.at(j);
		}

		//second step
		tempPointPos.clear();
		for (unsigned int j=0;j<vecVertexToSmooth.size();j++)
		{
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Halfedge_around_vertex_circulator Havc=vecVertexToSmooth.at(j)->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();
				Havc++;
			} while(Havc!=vecVertexToSmooth.at(j)->vertex_begin());

			for (unsigned int k=0;k<3;k++)
			{
				double dNeighborNum=vecVertexToSmooth.at(j)->vertex_degree();
				dLaplacian[k]=dLaplacian[k]/dNeighborNum;
			}

			Point_3 CentroidPoint(dLaplacian[0],dLaplacian[1],dLaplacian[2]);
			Vector_3 CurrenVector=CentroidPoint-vecVertexToSmooth.at(j)->point();

			double dVecX,dVecY,dVecZ;
			dVecX=CurrenVector.x()*dMu;
			dVecY=CurrenVector.y()*dMu;
			dVecZ=CurrenVector.z()*dMu;
			Vector_3 VecToMove(dVecX,dVecY,dVecZ);

			tempPointPos.push_back(vecVertexToSmooth.at(j)->point()+VecToMove);
		}

		for (unsigned int j=0;j<tempPointPos.size();j++)
		{
			vecVertexToSmooth.at(j)->point()=tempPointPos.at(j);
		}
	}
}

void GeometryAlgorithm::ComputeMeshMeanCurvature(KW_Mesh& mesh)
{
	system("cls");
	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
		//get Mixed Area
		double dMixedArea=0;
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			Vertex_handle VerNb1=Havc->opposite()->vertex();
			Vertex_handle VerNb2=Havc->opposite()->next()->vertex();
			double dAngle0=GetAngleBetweenTwoVectors3d(Vector_3(i->point(),VerNb1->point()),
				Vector_3(i->point(),VerNb2->point()));
			double dAngle1=GetAngleBetweenTwoVectors3d(Vector_3(VerNb1->point(),i->point()),
				Vector_3(VerNb1->point(),VerNb2->point()));
//			double dAngle2=GetAngleBetweenTwoVectors3d(Vector_3(VerNb2->point(),VerNb1->point()),
//				Vector_3(VerNb2->point(),i->point()));
			double dAngle2=180-dAngle0-dAngle1;
			double dCurrentArea=0;
			if ((dAngle0<=90)&&(dAngle1<=90)&&(dAngle2<=90))
			{
				double dVec1Dist=CGAL::squared_distance(i->point(),VerNb1->point());
				double dVec2Dist=CGAL::squared_distance(i->point(),VerNb2->point());
				dCurrentArea=(dVec1Dist/tan(dAngle2*CGAL_PI/180)+
					dVec2Dist/tan(dAngle1*CGAL_PI/180))/8;
			}
			else
			{
				Triangle_3 CurrentTri(i->point(),VerNb1->point(),VerNb2->point());
				if (dAngle0>90)
				{
					dCurrentArea=sqrt(CurrentTri.squared_area())/2;
				} 
				else
				{
					dCurrentArea=sqrt(CurrentTri.squared_area())/4;
				}
			}
			dMixedArea=dMixedArea+dCurrentArea;
			Havc++;
		} while(Havc!=i->vertex_begin());

		Vector_3 SumVec(0,0,0);
		Havc=i->vertex_begin();
		do 
		{
			Vertex_handle Vertex1=Havc->opposite()->vertex();

			Vertex_handle VertexNb0=Havc->next()->vertex();
			Vertex_handle VertexNb1=Havc->opposite()->next()->vertex();

			Vector_3 EdgeVector00=VertexNb0->point()-i->point();
			Vector_3 EdgeVector01=VertexNb0->point()-Vertex1->point();

			Vector_3 EdgeVector10=VertexNb1->point()-i->point();
			Vector_3 EdgeVector11=VertexNb1->point()-Vertex1->point();

			double dAngle0=GetAngleBetweenTwoVectors3d(EdgeVector00,EdgeVector01);
			double dAngle1=GetAngleBetweenTwoVectors3d(EdgeVector10,EdgeVector11);
			double dRadius0=1/tan(dAngle0*CGAL_PI/180);
			double dRadius1=1/tan(dAngle1*CGAL_PI/180);

			Vector_3 CurrentVec=Vector_3(i->point(),Vertex1->point())*(dRadius0+dRadius1);
			SumVec=SumVec+CurrentVec;
			Havc++;
		} while(Havc!=i->vertex_begin());

		double dMeanCur=sqrt(SumVec.squared_length())/dMixedArea/4;

		cout<<"%f\t"<<dMeanCur;

		i->SetMeanCurvature(dMeanCur);
	}
//	NormalizeMeshMeanCurvature(mesh);
	SetCurvatureColor(mesh,COLOR_MEAN_CURVATURE);
}

void GeometryAlgorithm::NormalizeMeshMeanCurvature(KW_Mesh& mesh)
{
	double dXmin,dXmax,dYmin,dYmax,dZmin,dZmax;
	dXmin=dXmax=mesh.vertices_begin()->point().x();
	dYmin=dYmax=mesh.vertices_begin()->point().y();
	dZmin=dZmax=mesh.vertices_begin()->point().z();
	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
		if (i->point().x()>dXmax)
		{
			dXmax=i->point().x();
		}
		else if (i->point().x()<dXmin)
		{
			dXmin=i->point().x();
		}

		if (i->point().y()>dYmax)
		{
			dYmax=i->point().y();
		}
		else if (i->point().y()<dYmin)
		{
			dYmin=i->point().y();
		}

		if (i->point().z()>dZmax)
		{
			dZmax=i->point().z();
		}
		else if (i->point().z()<dZmin)
		{
			dZmin=i->point().z();
		}
	}
	double dXScale=dXmax-dXmin;
	double dYScale=dYmax-dYmin;
	double dZScale=dZmax-dZmin;
	double dmeshScale = sqrt(dXScale*dXScale+dYScale*dYScale+dZScale*dZScale);
	cout<<"dmeshScale %f\n"<<dmeshScale;

	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
		cout<<"dMeanCur %f\t"<<i->GetMeanCurvature();
		i->SetMeanCurvature(i->GetMeanCurvature()*dmeshScale);
		cout<<"%f\n"<<i->GetMeanCurvature();
	}
}

void GeometryAlgorithm::ComputeMeshGaussianCurvature(KW_Mesh& mesh)
{
	system("cls");
	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
		//get Mixed Area
		double dMixedArea=0;
		Halfedge_around_vertex_circulator Havc=i->vertex_begin();
		do 
		{
			Vertex_handle VerNb1=Havc->opposite()->vertex();
			Vertex_handle VerNb2=Havc->opposite()->next()->vertex();
			double dAngle0=GetAngleBetweenTwoVectors3d(Vector_3(i->point(),VerNb1->point()),
				Vector_3(i->point(),VerNb2->point()));
			double dAngle1=GetAngleBetweenTwoVectors3d(Vector_3(VerNb1->point(),i->point()),
				Vector_3(VerNb1->point(),VerNb2->point()));
			//			double dAngle2=GetAngleBetweenTwoVectors3d(Vector_3(VerNb2->point(),VerNb1->point()),
			//				Vector_3(VerNb2->point(),i->point()));
			double dAngle2=180-dAngle0-dAngle1;
			double dCurrentArea=0;
			if ((dAngle0<=90)&&(dAngle1<=90)&&(dAngle2<=90))
			{
				double dVec1Dist=CGAL::squared_distance(i->point(),VerNb1->point());
				double dVec2Dist=CGAL::squared_distance(i->point(),VerNb2->point());
				dCurrentArea=(dVec1Dist/tan(dAngle2*CGAL_PI/180)+
					dVec2Dist/tan(dAngle1*CGAL_PI/180))/8;
			}
			else
			{
				Triangle_3 CurrentTri(i->point(),VerNb1->point(),VerNb2->point());
				if (dAngle0>90)
				{
					dCurrentArea=sqrt(CurrentTri.squared_area())/2;
				} 
				else
				{
					dCurrentArea=sqrt(CurrentTri.squared_area())/4;
				}
			}
			dMixedArea=dMixedArea+dCurrentArea;
			Havc++;
		} while(Havc!=i->vertex_begin());

		double dSumRadius=0.0;
		Havc=i->vertex_begin();
		do 
		{
			Vertex_handle Vertex1=Havc->opposite()->vertex();

			Vertex_handle VertexNb=Havc->opposite()->next()->vertex();

			Vector_3 CurrentEdgeVector=i->point()-Vertex1->point();
			Vector_3 NextEdgeVector=i->point()-VertexNb->point();

			double dCurrentRadius=GetAngleBetweenTwoVectors3d(CurrentEdgeVector,NextEdgeVector,true);

			dSumRadius=dSumRadius+dCurrentRadius;
			Havc++;
		} while(Havc!=i->vertex_begin());

		cout<<"%f\t"<<dSumRadius;

		double dGauCur=(2*CGAL_PI-dSumRadius)/dMixedArea;

//		DBWindowWrite("%f\t",dGauCur);

		i->SetGaussianCurvature(dGauCur);
	}
	SetCurvatureColor(mesh,COLOR_GAUSSIAN_CURVATURE);
}

void GeometryAlgorithm::SetCurvatureColor(KW_Mesh& mesh,int iCurType)
{
	if (iCurType==COLOR_MEAN_CURVATURE)
	{
		for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
		{
			int iLevel=0;
			if (i->GetMeanCurvature()<0.2)
			{
				iLevel=0;
			} 
			else if (i->GetMeanCurvature()>=0.2&&i->GetMeanCurvature()<0.5)
			{
				iLevel=1;
			}
			else if (i->GetMeanCurvature()>=0.5&&i->GetMeanCurvature()<1.0)
			{
				iLevel=2;
			}
			else if (i->GetMeanCurvature()>=1.0&&i->GetMeanCurvature()<2.0)
			{
				iLevel=3;
			}
			else if (i->GetMeanCurvature()>=2.0&&i->GetMeanCurvature()<3.5)
			{
				iLevel=4;
			}
			else if (i->GetMeanCurvature()>=3.5&&i->GetMeanCurvature()<6.0)
			{
				iLevel=5;
			}
			else if (i->GetMeanCurvature()>=6.0&&i->GetMeanCurvature()<10.0)
			{
				iLevel=6;
			}
			else if (i->GetMeanCurvature()>=10.0&&i->GetMeanCurvature()<14.5)
			{
				iLevel=7;
			}
			else if (i->GetMeanCurvature()>=14.5&&i->GetMeanCurvature()<20.0)
			{
				iLevel=8;
			}
			else if (i->GetMeanCurvature()>=20.0&&i->GetMeanCurvature()<30.0)
			{
				iLevel=9;
			}
			else if (i->GetMeanCurvature()>=30.0&&i->GetMeanCurvature()<40.0)
			{
				iLevel=10;
			}
			else if (i->GetMeanCurvature()>=40.0&&i->GetMeanCurvature()<50.0)
			{
				iLevel=11;
			}
			else if (i->GetMeanCurvature()>=50.0&&i->GetMeanCurvature()<60.0)
			{
				iLevel=12;
			}
			else if (i->GetMeanCurvature()>=60.0&&i->GetMeanCurvature()<70.0)
			{
				iLevel=13;
			}
			else if (i->GetMeanCurvature()>=70.0)
			{
				iLevel=14;
			}
			vector<double> dColorMap;
			dColorMap.push_back(0.23*(14-iLevel)/14.0+iLevel/14.0);
			dColorMap.push_back(0.81*(14-iLevel)/14.0);
			dColorMap.push_back(0.92*(14-iLevel)/14.0);
			dColorMap.push_back(1.0);
			i->SetColor(dColorMap);
		}
	} 
	else if (iCurType==COLOR_GAUSSIAN_CURVATURE)
	{
		for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
		{
			int iLevel=0;
			if (i->GetGaussianCurvature()<0.2)
			{
				iLevel=0;
			} 
			else if (i->GetGaussianCurvature()>=0.2&&i->GetGaussianCurvature()<0.5)
			{
				iLevel=1;
			}
			else if (i->GetGaussianCurvature()>=0.5&&i->GetGaussianCurvature()<1.0)
			{
				iLevel=2;
			}
			else if (i->GetGaussianCurvature()>=1.0&&i->GetGaussianCurvature()<2.0)
			{
				iLevel=3;
			}
			else if (i->GetGaussianCurvature()>=2.0&&i->GetGaussianCurvature()<3.5)
			{
				iLevel=4;
			}
			else if (i->GetGaussianCurvature()>=3.5&&i->GetGaussianCurvature()<6.0)
			{
				iLevel=5;
			}
			else if (i->GetGaussianCurvature()>=6.0&&i->GetGaussianCurvature()<10.0)
			{
				iLevel=6;
			}
			else if (i->GetGaussianCurvature()>=10.0&&i->GetGaussianCurvature()<14.5)
			{
				iLevel=7;
			}
			else if (i->GetGaussianCurvature()>=14.5&&i->GetGaussianCurvature()<20.0)
			{
				iLevel=8;
			}
			else if (i->GetGaussianCurvature()>=20.0&&i->GetGaussianCurvature()<30.0)
			{
				iLevel=9;
			}
			else if (i->GetGaussianCurvature()>=30.0&&i->GetGaussianCurvature()<40.0)
			{
				iLevel=10;
			}
			else if (i->GetGaussianCurvature()>=40.0&&i->GetGaussianCurvature()<50.0)
			{
				iLevel=11;
			}
			else if (i->GetGaussianCurvature()>=50.0&&i->GetGaussianCurvature()<60.0)
			{
				iLevel=12;
			}
			else if (i->GetGaussianCurvature()>=60.0&&i->GetGaussianCurvature()<70.0)
			{
				iLevel=13;
			}
			else if (i->GetGaussianCurvature()>=70.0)
			{
				iLevel=14;
			}
			vector<double> dColorMap;
			dColorMap.push_back(0.23*(14-iLevel)/14.0+iLevel/14.0);
			dColorMap.push_back(0.81*(14-iLevel)/14.0);
			dColorMap.push_back(0.92*(14-iLevel)/14.0);
			dColorMap.push_back(1.0);
			i->SetColor(dColorMap);
		}
	}

	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
//		DBWindowWrite("Color: %f,%f,%f\n",i->GetColor()[0],i->GetColor()[1],i->GetColor()[2]);
	}

}

void GeometryAlgorithm::SetUniformMeshColor(KW_Mesh& mesh,vector<double> vecColor)
{
	assert(vecColor.size()==4);
	for (Vertex_iterator i=mesh.vertices_begin();i!=mesh.vertices_end();i++)
	{
		i->SetColor(vecColor);
	}
}







