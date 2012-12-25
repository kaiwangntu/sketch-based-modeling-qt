#include "Tunnelling.h"

CTunnelling::CTunnelling(void)
{
}

CTunnelling::~CTunnelling(void)
{
}

void CTunnelling::clear()
{
	this->vecvecTunVertex.clear();
	this->TunnelDirectCurve.clear();
}

vector<vector<Vertex_handle>> CTunnelling::GetTunVer()
{
	return this->vecvecTunVertex;
}

int CTunnelling::ExtractSkelFromSilhCurve(vector<Point_3> ExtrusionSilhPoints,int iSampleNum,vector<Point_3>& SkeletonCurve)
{
	int iGroupNum=(int)(ExtrusionSilhPoints.size()/2);
	vector<Point_3> TempResult;
	for (int i=0;i<iGroupNum;i++)
	{
		Point_3 NewAxisStartPoint=ExtrusionSilhPoints.at(i);
		Point_3 NewAxisEndPoint=ExtrusionSilhPoints.at(ExtrusionSilhPoints.size()-1-i);
		Point_3 NewMiddlePoint=CGAL::midpoint(NewAxisStartPoint,NewAxisEndPoint);
		TempResult.push_back(NewMiddlePoint);
	}
	if ((int)TempResult.size()>iSampleNum)
	{
		for (int i=0;i<iSampleNum;i++)
		{
			SkeletonCurve.push_back(TempResult.at(i));
		}
	}
	else
	{
		SkeletonCurve=TempResult;
	}
	return SkeletonCurve.size();
}

vector<Point_3> CTunnelling::GetTunnelDirectCurve()
{
	return this->TunnelDirectCurve;
}

void CTunnelling::SetTunnelDirectCurve(vector<Point_3> vecInput)
{
	//judge the order of points according to 
	//the distances between the first point and CenterA and CenterB
	if (CGAL::has_larger_distance_to_point(vecInput.front(),CenterA,CenterB))
	{
		reverse(vecInput.begin(),vecInput.end());
	}
	this->TunnelDirectCurve=vecInput;
}

void CTunnelling::InputTunVer(vector<Vertex_handle> vecTunVertex)
{
	this->vecvecTunVertex.push_back(vecTunVertex);
}

void CTunnelling::SetRefPlane(Plane_3& BestFittingPlane,Point3D* PlaneBoundaryPoints,Point_3& RotateXAxisStartPoint, 
							  Point_3& RotateXAxisEndPoint)
{
	Point_3 CenterAB((CenterA.x()+CenterB.x())/2.0,(CenterA.y()+CenterB.y())/2.0,(CenterA.z()+CenterB.z())/2.0);
	Point_3 ShortCenterA=CenterAB+(CenterA-CenterAB)/4.0;
	Point_3 ShortCenterB=CenterAB+(CenterB-CenterAB)/4.0;

	Point_3 c;
	list<Point_3> points;
	points.push_back(ShortCenterA);points.push_back(ShortCenterB);
	double result=linear_least_squares_fitting_3(points.begin(),points.end(),BestFittingPlane,c,CGAL::Dimension_tag<0>());

	RotateXAxisStartPoint=BestFittingPlane.projection(ShortCenterA);
	RotateXAxisEndPoint=BestFittingPlane.projection(ShortCenterB);
	
	GeometryAlgorithm compute;
	compute.GetPlaneBoundaryPoints(RotateXAxisStartPoint,RotateXAxisEndPoint,BestFittingPlane,PlaneBoundaryPoints);
}

void CTunnelling::Tunnel(KW_Mesh& Mesh,vector<Point_3>& vecTestPoint)
{
	//CorrespondTunCircles();

	//the agent curve
	vector<Point_3> vecAgentPoint;
	Point_3 CenterC;
	vector<double> arclengthA,arclengthB,arclengthC;
	ComputeAgentCurveShape(vecAgentPoint,CenterC,arclengthA,arclengthB,arclengthC);
	vecTestPoint=vecAgentPoint;

	vector<vector<Point_3>> vecvecAgentCurveA,vecvecAgentCurveB;
	vector<Point_3> vecMiddleAgentCurve;
	ComputeAgentCurves(vecAgentPoint,CenterC,TunnelDirectCurve,vecvecAgentCurveA,vecvecAgentCurveB,vecMiddleAgentCurve);
	for (unsigned int i=0;i<vecvecAgentCurveA.size();i++)
	{
		vecTestPoint.insert(vecTestPoint.end(),vecvecAgentCurveA.at(i).begin(),vecvecAgentCurveA.at(i).end());
	}
	for (unsigned int i=0;i<vecvecAgentCurveB.size();i++)
	{
		vecTestPoint.insert(vecTestPoint.end(),vecvecAgentCurveB.at(i).begin(),vecvecAgentCurveB.at(i).end());
	}
	vecTestPoint.insert(vecTestPoint.end(),vecMiddleAgentCurve.begin(),vecMiddleAgentCurve.end());


	//dig the hole and remesh
	//get the two set of halfedges along two tunneling curves
	//hhClosedCurveA is ccw,hhClosedCurveB is cw
	//such that the order of their vertices correspond to the agent curve
	vector<Halfedge_handle> hhClosedCurveA,hhClosedCurveB;
	GetClosedStrokeHH(hhClosedCurveA,hhClosedCurveB);
	//dig hole for tunneling curve A
	DeleteFacetsInClosedCurve(Mesh,hhClosedCurveA);
	//fill the hole of A temporarily 
	TempFillHole(Mesh,hhClosedCurveA,CenterA);
	//dig hole for tunneling curve B
	DeleteFacetsInClosedCurve(Mesh,hhClosedCurveB,true);
	//fill the hole of B temporarily 
	TempFillHole(Mesh,hhClosedCurveB,CenterB,true);
	//connect cirve A and agent curve C
	Halfedge_handle hFirstCSegA;
//	ConnectTunAAgent(Mesh,hhClosedCurveA,vecAgentPoint,arclengthA,arclengthC,hFirstCSegA,vecTestPoint);
	ConnectTunAAgent(Mesh,hhClosedCurveA,vecvecAgentCurveA.front(),arclengthA,arclengthC,hFirstCSegA,vecTestPoint);
	//connect cirve B and agent curve C
	Halfedge_handle hFirstCSegB;
//	ConnectTunBAgent(Mesh,hhClosedCurveB,vecAgentPoint,arclengthB,arclengthC,hFirstCSegB,vecTestPoint);
	ConnectTunBAgent(Mesh,hhClosedCurveB,vecvecAgentCurveB.front(),arclengthB,arclengthC,hFirstCSegB,vecTestPoint);

	//expand half tunnel A
	ExpandHalfTunA(Mesh,vecvecAgentCurveA,hFirstCSegA,vecTestPoint);
	//expand half tunnel B
	ExpandHalfTunB(Mesh,vecvecAgentCurveB,hFirstCSegB,vecTestPoint);

	//join them
	assert(hFirstCSegA->is_border());
	assert(hFirstCSegB->opposite()->is_border());
	assert(hFirstCSegA->vertex()->point()==hFirstCSegB->vertex()->point());
	Mesh.normalize_border();
	assert(Mesh.size_of_border_halfedges()!=0);

	Mesh.join_loop(hFirstCSegA,hFirstCSegB->opposite());

	Mesh.normalize_border();
	assert(Mesh.size_of_border_halfedges()==0);
}

void CTunnelling::CorrespondTunCircles()
{
	//get the centers of the two circles
	double dSumX,dSumY,dSumZ;
	dSumX=dSumY=dSumZ=0;
	int iCurveLengthA=this->vecvecTunVertex.at(0).size();
	for (int i=0;i<iCurveLengthA;i++)
	{
		dSumX=dSumX+this->vecvecTunVertex.at(0).at(i)->point().x();
		dSumY=dSumY+this->vecvecTunVertex.at(0).at(i)->point().y();
		dSumZ=dSumZ+this->vecvecTunVertex.at(0).at(i)->point().z();
	}
	CenterA=Point_3(dSumX/(double)iCurveLengthA,dSumY/(double)iCurveLengthA,dSumZ/(double)iCurveLengthA);

	dSumX=dSumY=dSumZ=0;
	int iCurveLengthB=this->vecvecTunVertex.at(1).size();
	for (int i=0;i<iCurveLengthB;i++)
	{
		dSumX=dSumX+this->vecvecTunVertex.at(1).at(i)->point().x();
		dSumY=dSumY+this->vecvecTunVertex.at(1).at(i)->point().y();
		dSumZ=dSumZ+this->vecvecTunVertex.at(1).at(i)->point().z();
	}
	CenterB=Point_3(dSumX/(double)iCurveLengthB,dSumY/(double)iCurveLengthB,dSumZ/(double)iCurveLengthB);

	//get the line passing through the first point of cirve A
	//and has the direction of CenterA to CenterB
	Vector_3 VectA2B(CenterA,CenterB);
	Line_3 line(this->vecvecTunVertex.at(0).at(0)->point(),VectA2B);
	int iClosestVerIndex = 0;
	double dist = 10000000000.0;
	for (int i=0;i<iCurveLengthB;i++)
	{
		double dCurrentDist=CGAL::squared_distance(this->vecvecTunVertex.at(1).at(i)->point(),line);
		if (dCurrentDist<dist)
		{
			dist=dCurrentDist;
			iClosestVerIndex=i;
		}
	}
	//sort vertex order of cirve B
	rotate(this->vecvecTunVertex.at(1).begin(),this->vecvecTunVertex.at(1).begin()+iClosestVerIndex,
		this->vecvecTunVertex.at(1).end());
}

void CTunnelling::ComputeAgentCurveShape(vector<Point_3>& vecAgentPoint,Point_3& CenterC,vector<double>& arclengthA,
										 vector<double>& arclengthB, vector<double>& arclengthC)
{
	//get summed piece-wise arclength of cirve A and B
	int iCurveLengthA=this->vecvecTunVertex.at(0).size();
	int iCurveLengthB=this->vecvecTunVertex.at(1).size();
	arclengthA.push_back(0.0);
	for (int i=0;i<iCurveLengthA;i++)
	{
		Point_3 CurrentPoint=this->vecvecTunVertex.at(0).at((i+1)%iCurveLengthA)->point();
		Point_3 PrevPoint=this->vecvecTunVertex.at(0).at(i)->point();
		double dCurrentDist=std::sqrt(squared_distance(CurrentPoint,PrevPoint));
		arclengthA.push_back(arclengthA.back()+dCurrentDist);
	}
	//normalize 
	for (unsigned int i=0;i<arclengthA.size();i++)
	{
		arclengthA.at(i)=arclengthA.at(i)/arclengthA.back();
	}

	arclengthB.push_back(0.0);
	for (int i=0;i<iCurveLengthB;i++)
	{
		Point_3 CurrentPoint=this->vecvecTunVertex.at(1).at((i+1)%iCurveLengthB)->point();
		Point_3 PrevPoint=this->vecvecTunVertex.at(1).at(i)->point();
		double dCurrentDist=std::sqrt(squared_distance(CurrentPoint,PrevPoint));
		arclengthB.push_back(arclengthB.back()+dCurrentDist);
	}
	//normalize 
	for (unsigned int i=0;i<arclengthB.size();i++)
	{
		arclengthB.at(i)=arclengthB.at(i)/arclengthB.back();
	}

	//define the agent curve C
	int iCurveLengthC=(iCurveLengthA+iCurveLengthB)/2;
	for (int i=0;i<=iCurveLengthC;i++)
	{
		arclengthC.push_back((double)i/(double)iCurveLengthC);
	}

	//compute the vertex positions of curve C and its center
	double dSumX,dSumY,dSumZ;
	dSumX=dSumY=dSumZ=0;
	for(int i = 0; i < iCurveLengthC; i++)
	{
		double x,y,z;
		x=y=z=0.0;
		//kw compute a vertex for circle C using the vertices in A
		//[((j + cA) - 1) % cA] is the previous one of [j]
		//ensure that vertex C[i] fall between A[j] and A[j-1]
		int j = 0;
		while (arclengthA.at(j) < arclengthC.at(i)) j++;
		double w = (arclengthA.at(j) - arclengthC.at(i)) / (arclengthA.at(j) - arclengthA.at(((j+iCurveLengthA)-1)%iCurveLengthA));
		Point_3 PrevPointA=this->vecvecTunVertex.at(0).at(((j+iCurveLengthA)-1)%iCurveLengthA)->point();
		Point_3 CurrentPointA=this->vecvecTunVertex.at(0).at(j%iCurveLengthA)->point();
		x += 0.5 * (w * PrevPointA.x() + (1.0 - w) * CurrentPointA.x());
		y += 0.5 * (w * PrevPointA.y() + (1.0 - w) * CurrentPointA.y());
		z += 0.5 * (w * PrevPointA.z() + (1.0 - w) * CurrentPointA.z());
		//kw compute a vertex for circle C using the vertices in B
		j = 0;
		while (arclengthB.at(j) < arclengthC.at(i)) j++;
		w = (arclengthB.at(j) - arclengthC.at(i)) / (arclengthB.at(j) - arclengthB.at(((j+iCurveLengthB)-1)%iCurveLengthB));
		Point_3 PrevPointB=this->vecvecTunVertex.at(1).at(((j+iCurveLengthB)-1)%iCurveLengthB)->point();
		Point_3 CurrentPointB=this->vecvecTunVertex.at(1).at(j%iCurveLengthB)->point();
		x += 0.5 * (w * PrevPointB.x() + (1.0 - w) * CurrentPointB.x());
		y += 0.5 * (w * PrevPointB.y() + (1.0 - w) * CurrentPointB.y());
		z += 0.5 * (w * PrevPointB.z() + (1.0 - w) * CurrentPointB.z());
		//kw the final vertex in C blends the two results with weight 0.5

		Point_3 PointC(x,y,z);
		vecAgentPoint.push_back(PointC);
		dSumX=dSumX+x;dSumY=dSumY+y;dSumZ=dSumZ+z;
	}
	dSumX=dSumX/(double)iCurveLengthC;dSumY=dSumY/(double)iCurveLengthC;dSumZ=dSumZ/(double)iCurveLengthC;
	CenterC=Point_3(dSumX,dSumY,dSumZ);
}

void CTunnelling::ComputeAgentCurves(vector<Point_3> vecAgentPoint,Point_3 CenterC,vector<Point_3> TunnelDirectCurve, 
									 vector<vector<Point_3>>& vecvecAgentCurveA,vector<vector<Point_3>>& vecvecAgentCurveB, 
									 vector<Point_3>& vecMiddleAgentCurve)
{
	int iAgentNum=TunnelDirectCurve.size();
	//iAgentNum=9;
	int iHalfAgentNum=iAgentNum/2;
	//compute the agent curves for tunnel curve A
	for (int i=0;i<iHalfAgentNum;i++)
	{
		Vector_3 VecToMove=TunnelDirectCurve.at(i)-CenterC;
		vector<Point_3> CurrentAgentCurve;
		for (unsigned int j=0;j<vecAgentPoint.size();j++)
		{
			Point_3 NewPoint=vecAgentPoint.at(j)+VecToMove;
			CurrentAgentCurve.push_back(NewPoint);
		}
		vecvecAgentCurveA.push_back(CurrentAgentCurve);
	}
	//compute the agent curves for tunnel curve B
	for (int i=0;i<iHalfAgentNum;i++)
	{
		Vector_3 VecToMove=TunnelDirectCurve.at(iAgentNum-1-i)-CenterC;
		vector<Point_3> CurrentAgentCurve;
		for (unsigned int j=0;j<vecAgentPoint.size();j++)
		{
			Point_3 NewPoint=vecAgentPoint.at(j)+VecToMove;
			CurrentAgentCurve.push_back(NewPoint);
		}
		vecvecAgentCurveB.push_back(CurrentAgentCurve);
	}
	
	//compute the middle agent curve, put it in tunnel curve A and B
	Vector_3 CenterVecToMove;
	if (iAgentNum%2==0)//even numbers,push the last ring of A to the end of B
	{
		vecvecAgentCurveB.push_back(vecvecAgentCurveA.back());
	}
	else//odd number, put the computed middle ring to the end of A andB
	{
		Vector_3 CenterVecToMove=Vector_3(TunnelDirectCurve.at(iHalfAgentNum)-CenterC);
		for (unsigned int j=0;j<vecAgentPoint.size();j++)
		{
			Point_3 NewPoint=vecAgentPoint.at(j)+CenterVecToMove;
			vecMiddleAgentCurve.push_back(NewPoint);
		}
		vecvecAgentCurveA.push_back(vecMiddleAgentCurve);
		vecvecAgentCurveB.push_back(vecMiddleAgentCurve);
	}
	assert(vecvecAgentCurveA.back()==vecvecAgentCurveB.back());
}

void CTunnelling::GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurveA,vector<Halfedge_handle>& hhClosedCurveB)
{
	int iCurveLen=vecvecTunVertex.at(0).size();
	for (int i=0;i<iCurveLen;i++)
	{
		Vertex_handle vhCurrent,vhNext;
		vhCurrent=this->vecvecTunVertex.at(0).at(i);
		//(i+1)%iCurveLen is the next one of i
		vhNext=this->vecvecTunVertex.at(0).at((i+1)%iCurveLen);
		Halfedge_around_vertex_circulator Havc=vhCurrent->vertex_begin();
		do 
		{
			if (Havc->opposite()->vertex()==vhNext)
			{
				hhClosedCurveA.push_back(Havc->opposite());
				break;
			}
			else
			{
				Havc++;
			}
		} while(Havc!=vhCurrent->vertex_begin());
	}
	assert(hhClosedCurveA.size()==iCurveLen);

	iCurveLen=vecvecTunVertex.at(1).size();
	for (int i=0;i<iCurveLen;i++)
	{
		Vertex_handle vhCurrent,vhNext;
		vhCurrent=this->vecvecTunVertex.at(1).at(i);
		//(i+1)%iCurveLen is the next one of i
		vhNext=this->vecvecTunVertex.at(1).at((i+1)%iCurveLen);
		Halfedge_around_vertex_circulator Havc=vhCurrent->vertex_begin();
		do 
		{
			if (Havc->opposite()->vertex()==vhNext)
			{
				hhClosedCurveB.push_back(Havc->opposite());
				break;
			}
			else
			{
				Havc++;
			}
		} while(Havc!=vhCurrent->vertex_begin());
	}
	assert(hhClosedCurveB.size()==iCurveLen);
}

void CTunnelling::DeleteFacetsInClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve,bool bFlip/* =false */)
{
	//triangle incident to hhClosedCurve->opposite(outside the closed curve)
	vector<Facet_handle> fhCurveOutTri;
	//triangle incident to hhClosedCurve(inside the closed curve)
	vector<Facet_handle> fhCurveInTri;
	for (unsigned int i=0;i<hhClosedCurve.size();i++)
	{
		Halfedge_handle hhCurrent=hhClosedCurve.at(i);
		Halfedge_handle hhOpp=hhCurrent->opposite();
		if (bFlip)
		{
			fhCurveInTri.push_back(hhOpp->facet());
			fhCurveOutTri.push_back(hhCurrent->facet());
		} 
		else
		{
			fhCurveInTri.push_back(hhCurrent->facet());
			fhCurveOutTri.push_back(hhOpp->facet());
		}
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
}

int CTunnelling::GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,
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

void CTunnelling::TempFillHole(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve,Point_3 Center,bool bFlip/* =false */)
{
	//fill the hole of hhClosedCurve 
	if (bFlip)
	{
		Mesh.fill_hole(hhClosedCurve.front()->opposite());
	} 
	else
	{
		Mesh.fill_hole(hhClosedCurve.front());
	}
	Mesh.normalize_border();
	int test00=Mesh.size_of_border_edges();
	assert(test00==0);

	Halfedge_handle hhToCenterPoint;
	if (bFlip)
	{
		hhToCenterPoint=Mesh.create_center_vertex(hhClosedCurve.front()->opposite());
	} 
	else
	{
		hhToCenterPoint=Mesh.create_center_vertex(hhClosedCurve.front());
	}
	hhToCenterPoint->vertex()->point()=Center;

	Mesh.normalize_border();
	assert(Mesh.size_of_border_edges()==0);
}

void CTunnelling::ConnectTunAAgent(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurveA,vector<Point_3> vecAgentPoint, 
								   vector<double>& arclengthA,vector<double>& arclengthC,Halfedge_handle& hFirstCSegA,
								   vector<Point_3>& vecTestPoint)
{
	Mesh.erase_center_vertex(hhClosedCurveA.at(0)->next());
	Mesh.erase_facet(hhClosedCurveA.at(0));

	int iIndexA,iIndexC;
	iIndexA=iIndexC=0;
	int iLengthA=hhClosedCurveA.size();//this->vecvecTunVertex.at(0).size();
	int iLengthC=vecAgentPoint.size();

	Halfedge_handle hASeg,hLastBE,hFirstCSeg;
	hLastBE=hhClosedCurveA.at(iIndexA);
	do 
	{
		if (arclengthA.at(iIndexA+1)<arclengthC.at(iIndexC+1))
		{
			if (iIndexA==0)//iIndexC must also be 0, or else cannot come here
			{
				Halfedge_handle hNew=Mesh.split_edge(hLastBE->opposite());
				hNew->vertex()->point()=vecAgentPoint.at(iIndexC);
				Mesh.split_facet(hNew->prev(),hLastBE->opposite());
				iIndexA++;
				hASeg=hhClosedCurveA.at(iIndexA);
			}
			//if C has reached end,just continue processing A till the last vertex on A
			//however, when A also reached end, the length of A will never be shorter than C
			//so don't need to handle the case of the last one here
			else
			{
				Mesh.add_facet_to_border(hLastBE,hASeg);
				iIndexA++;
				hASeg=hhClosedCurveA.at(iIndexA);
			}
		} 
		else if (arclengthA.at(iIndexA+1)>=arclengthC.at(iIndexC+1))
		{
			if ((iIndexC==0)&&(iIndexA==0))
			{
				vector<Halfedge_handle> vecHPrev;
				vecHPrev.push_back(Mesh.split_edge(hLastBE->opposite()));
				vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
				iIndexC++;
				while (arclengthA.at(iIndexA+1)>=arclengthC.at(iIndexC+1))
				{
					vecHPrev.push_back(Mesh.split_edge(vecHPrev.back()));
					vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
					if (iIndexC==1)
					{
						hFirstCSeg=vecHPrev.back()->next()->opposite();
					}
					iIndexC++;
				}
				Halfedge_handle hLast=Mesh.split_edge(vecHPrev.back());
				if (iIndexC==1)
				{
					hFirstCSeg=hLast->next()->opposite();
				}
				hLast->vertex()->point()=vecAgentPoint.at(iIndexC);
				Mesh.split_facet(hLast->prev(),hLastBE->opposite());
				for (int i=(int)vecHPrev.size()-1;i>=0;i--)
				{
					Mesh.split_facet(vecHPrev.at(i)->prev(),hLastBE->opposite());
				}

				hLastBE=vecHPrev.back()->opposite();
				iIndexA++;
				hASeg=hhClosedCurveA.at(iIndexA);

				assert(hLastBE->vertex()->point()==vecAgentPoint.at(iIndexC));
			}
			else if ((iIndexC==iLengthC-1)&&(iIndexA==iLengthA-1))//both A and C reached the end
			{
				Halfedge_handle hNew=Mesh.add_facet_to_border(hLastBE,hASeg->next());
				assert(hNew->opposite()==hLastBE->next());
				Mesh.split_facet(hNew,hNew->next()->next());
				iIndexC++;
				hFirstCSegA=hFirstCSeg;
				//the process should finish from here
			}
			else
			{
				Halfedge_handle hNew=Mesh.add_vertex_and_facet_to_border(hLastBE,hASeg->prev());
				if (iIndexC==0)
				{
					hFirstCSeg=hNew->next()->opposite();
				}
				iIndexC++;
				hNew->vertex()->point()=vecAgentPoint.at(iIndexC);
				hLastBE=hNew->next()->opposite();
			}
		}
	} while((iIndexA<iLengthA-1)||(iIndexC<=iLengthC-1));
}

void CTunnelling::ConnectTunBAgent(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurveB,vector<Point_3> vecAgentPoint, 
								   vector<double>& arclengthB,vector<double>& arclengthC,Halfedge_handle& hFirstCSegB,
								   vector<Point_3>& vecTestPoint)
{
	Mesh.erase_center_vertex(hhClosedCurveB.at(0)->opposite()->next());
	Mesh.erase_facet(hhClosedCurveB.at(0)->opposite());

	int iIndexB,iIndexC;
	iIndexB=iIndexC=0;
	int iLengthB=hhClosedCurveB.size();//this->vecvecTunVertex.at(1).size();
	int iLengthC=vecAgentPoint.size();

//	arclengthB[1]=arclengthC[10]+0.01;

	Halfedge_handle hBSeg,hLastBE,hFirstCSeg;
	hLastBE=hhClosedCurveB.at(iIndexB);
	do 
	{
		if (arclengthB.at(iIndexB+1)<arclengthC.at(iIndexC+1))
		{
			if (iIndexB==0)//iIndexC must also be 0, or else cannot come here
			{
				Halfedge_handle hNew=Mesh.split_edge(hLastBE);
				hNew->vertex()->point()=vecAgentPoint.at(iIndexC);
				Mesh.split_facet(hNew->prev(),hLastBE);
				iIndexB++;
				hBSeg=hhClosedCurveB.at(iIndexB);
			}
	//		//if C has reached end,just continue processing A till the last vertex on A
	//		//however, when A also reached end, the length of A will never be shorter than C
	//		//so don't need to handle the case of the last one here
			else
			{
				iIndexB++;
				hBSeg=hhClosedCurveB.at(iIndexB);
				Halfedge_handle hNew=Mesh.add_facet_to_border(hBSeg->opposite(),hLastBE->opposite());
				hLastBE=hNew;
			}
		} 
		else if (arclengthB.at(iIndexB+1)>=arclengthC.at(iIndexC+1))
		{
			if ((iIndexC==0)&&(iIndexB==0))
			{
				vector<Halfedge_handle> vecHPrev;
				vecHPrev.push_back(Mesh.split_edge(hLastBE));
				vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
				iIndexC++;
				while (arclengthB.at(iIndexB+1)>=arclengthC.at(iIndexC+1))
				{
					vecHPrev.push_back(Mesh.split_edge(hLastBE));
					if (iIndexC==1)
					{
						hFirstCSeg=hLastBE->prev();
					}
					vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
					iIndexC++;
				}
				Halfedge_handle hLast=Mesh.split_edge(hLastBE);
				if (iIndexC==1)
				{
					hFirstCSeg=hLastBE->prev();
				}
				vecHPrev.push_back(hLast);
				hLast->vertex()->point()=vecAgentPoint.at(iIndexC);
				Mesh.split_facet(hLastBE,vecHPrev.front()->prev());
				for (int i=(int)vecHPrev.size()-1;i>0;i--)
				{
					Mesh.split_facet(vecHPrev.at(i),vecHPrev.front()->prev());
				}

				iIndexB++;
				hBSeg=hhClosedCurveB.at(iIndexB);
				//assert(hLastBE->opposite()->vertex()->point()==vecAgentPoint.at(iIndexC));
			}
			else if ((iIndexC==iLengthC-1)&&(iIndexB==iLengthB-1))//both B and C reached the end
			{
				Halfedge_handle hNew=Mesh.add_facet_to_border(hFirstCSeg->opposite(),hLastBE->opposite());
				assert(hNew->opposite()==hFirstCSeg->opposite()->next());
				Mesh.split_facet(hNew,hNew->next()->next());
				iIndexC++;
				hFirstCSegB=hFirstCSeg;
				//the process should finish from here
			}
			else
			{
				Halfedge_handle hNew=Mesh.add_vertex_and_facet_to_border(hBSeg->opposite(),hLastBE->opposite());
				if (iIndexC==0)
				{
					hFirstCSeg=hNew;
				}
				iIndexC++;
				hNew->vertex()->point()=vecAgentPoint.at(iIndexC);
				hLastBE=hNew->next();
			}
		}
	} while((iIndexB<iLengthB-1)||(iIndexC<=iLengthC-1));
}

void CTunnelling::ExpandHalfTunA(KW_Mesh& Mesh,vector<vector<Point_3>>& vecvecAgentCurveA,Halfedge_handle& hFirstCSegA, 
								 vector<Point_3>& vecTestPoint)
{
	for (unsigned int i=1;i<vecvecAgentCurveA.size();i++)
	{
		Halfedge_handle hASeg,hLastBE,hFirstCSeg;
		hLastBE=hFirstCSegA;
		for (int iCount=0;iCount<(int)vecvecAgentCurveA.at(i).size()*2;iCount++)
		{
			if (iCount%2==0)
			{
				if (iCount==0)
				{
					Halfedge_handle hNew=Mesh.split_edge(hLastBE->opposite());
					hNew->vertex()->point()=vecvecAgentCurveA.at(i).at(iCount/2);
					Mesh.split_facet(hNew->prev(),hLastBE->opposite());
					hASeg=hLastBE->next()->next();
				}
				else
				{
					Mesh.add_facet_to_border(hLastBE,hASeg);
					hASeg=hLastBE->next()->next();
				}
			}
			else
			{
				if (iCount==vecvecAgentCurveA.at(i).size()*2-1)//the last facet
				{
					Halfedge_handle hNew=Mesh.add_facet_to_border(hLastBE,hASeg);
					hFirstCSegA=hFirstCSeg;
				}
				else
				{
					Halfedge_handle hNew=Mesh.add_vertex_and_facet_to_border(hLastBE,hASeg->prev());
					if (iCount==1)
					{
						hFirstCSeg=hNew->next()->opposite();
					}
					hNew->vertex()->point()=vecvecAgentCurveA.at(i).at((iCount+1)/2);
					hLastBE=hNew->next()->opposite();
				}
			}
		}
	}
}

void CTunnelling::ExpandHalfTunB(KW_Mesh& Mesh,vector<vector<Point_3>>& vecvecAgentCurveB,Halfedge_handle& hFirstCSegB, 
								 vector<Point_3>& vecTestPoint)
{
	for (unsigned int i=1;i<vecvecAgentCurveB.size();i++)
	{
		Halfedge_handle hBSeg,hLastBE,hFirstCSeg;
		hLastBE=hFirstCSegB;
		for (int iCount=0;iCount<(int)vecvecAgentCurveB.at(i).size()*2;iCount++)
		{
			if (iCount%2==0)
			{
				if (iCount==0)
				{
					Halfedge_handle hNew=Mesh.split_edge(hLastBE);
					hNew->vertex()->point()=vecvecAgentCurveB.at(i).at(iCount/2);
					Mesh.split_facet(hNew->prev(),hLastBE);
					hBSeg=hLastBE->opposite()->prev()->opposite();
				}
				else
				{
					hBSeg=hBSeg->opposite()->prev()->opposite();
					Halfedge_handle hNew=Mesh.add_facet_to_border(hBSeg->opposite(),hLastBE->opposite());
					hLastBE=hNew;
				}

			}
			else
			{
				if (iCount==vecvecAgentCurveB.at(i).size()*2-1)//the last facet
				{
					Halfedge_handle hNew=Mesh.add_facet_to_border(hFirstCSeg->opposite(),hLastBE->opposite());
					assert(hNew->opposite()==hFirstCSeg->opposite()->next());
					hFirstCSegB=hFirstCSeg;
				}
				else
				{
					Halfedge_handle hNew=Mesh.add_vertex_and_facet_to_border(hBSeg->opposite(),hLastBE->opposite());
					if (iCount==1)
					{
						hFirstCSeg=hNew;
					}
					hNew->vertex()->point()=vecvecAgentCurveB.at(i).at((iCount+1)/2);
					hLastBE=hNew->next();
				}

			}
		}
	}
}

//Mesh.erase_center_vertex(hhClosedCurveA.at(0)->next());
//Mesh.erase_facet(hhClosedCurveA.at(0));
//
////for operation convenient, get the opposite of each halfedge of hhClosedCurveA 
//vector<Halfedge_handle> hhClosedCurveAOpp;
//for (unsigned int i=0;i<hhClosedCurveA.size();i++)
//{
//	hhClosedCurveAOpp.push_back(hhClosedCurveA.at(i)->opposite());
//}
//
//int iIndexA,iIndexC;
//iIndexA=iIndexC=0;
//int iLengthA=this->vecvecTunVertex.at(0).size();
//int iLengthC=vecAgentPoint.size();
//
//Halfedge_handle hBegin,hCLast;
//hBegin=hhClosedCurveAOpp.at(iIndexA);
//do 
//{
//	if (arclengthA.at(iIndexA+1)<arclengthC.at(iIndexC+1))//or maybe C has reached the end
//	{
//		Halfedge_handle hPrev=Mesh.split_edge(hBegin);
//		hPrev->vertex()->point()=vecAgentPoint.at(iIndexC);
//		Mesh.split_facet(hPrev->prev(),hBegin);
//	} 
//	else if (arclengthA.at(iIndexA+1)>=arclengthC.at(iIndexC+1))//or maybe A has reached the end
//	{
//		if ((iIndexA==iLengthA-1)&&(iIndexC==iLengthC-1))
//		{
//			Mesh.add_facet_to_border(hhClosedCurveAOpp.at(0)->prev(),hCLast);
//			Mesh.split_facet(hhClosedCurveA.at(0),hCLast);
//		}
//		else if ((iIndexA==iLengthA-1)&&(iIndexC==iLengthC-))
//		{
//		}
//		else 
//		{
//			vector<Halfedge_handle> vecHPrev;
//			vecHPrev.push_back(Mesh.split_edge(hBegin));
//			vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
//			iIndexC++;
//
//			while (arclengthA.at(iIndexA+1)>=arclengthC.at(iIndexC+1))
//			{
//				vecHPrev.push_back(Mesh.split_edge(vecHPrev.back()));
//				vecHPrev.back()->vertex()->point()=vecAgentPoint.at(iIndexC);
//				iIndexC++;
//			}
//			Halfedge_handle hLast=Mesh.split_edge(vecHPrev.back());
//			hLast->vertex()->point()=vecAgentPoint.at(iIndexC);
//
//			hCLast=hLast;
//
//			Mesh.split_facet(hLast->prev(),hBegin);
//			for (int i=(int)vecHPrev.size()-1;i>=0;i--)
//			{
//				Mesh.split_facet(vecHPrev.at(i)->prev(),hBegin);
//			}
//		}
//	}
//	if (iIndexA<iLengthA-1)
//	{
//		iIndexA++;
//		hBegin=hhClosedCurveAOpp.at(iIndexA);
//	}
//} while((iIndexA<iLengthA-1)||(iIndexC<=iLengthC-1));
//
////assert number of border edges == length of cirve B + cirve C