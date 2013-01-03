#include "CrossSectionProc.h"
#include "../GeometryAlgorithm.h"
#include "../CurveDeform.h"
#include <QMessageBox>

CCrossSectionProc::CCrossSectionProc(void)
{
}

CCrossSectionProc::~CCrossSectionProc(void)
{
}

//void CCrossSectionProc::GetCNIntersectPoints(vector<CurveNetwork>& vecCurveNetwork)
//{
//	for (unsigned int iPlane=0;iPlane<vecCurveNetwork.size();iPlane++)
//	{
//		vecCurveNetwork.at(iPlane).CurvePlaneIntersect.clear();
//		vecCurveNetwork.at(iPlane).NeighborInd.clear();
//		vecCurveNetwork.at(iPlane).IntersectCurveIndex.clear();
//		vecCurveNetwork.at(iPlane).IntersectCNInd.clear();
//
//		for (unsigned int iCheckPlane=0;iCheckPlane<vecCurveNetwork.size();iCheckPlane++)
//		{
//			if (vecCurveNetwork.at(iPlane).ProfilePlaneType==vecCurveNetwork.at(iCheckPlane).ProfilePlaneType)
//			{
//				continue;
//			}
//			for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iPlane).Profile3D.size();iCurve++)
//			{
//				vector<vector<int> > SegPoinsInd;
//				vector<Point_3> InterSectPoints;
//				int iIntersectNum=GeometryAlgorithm::GetClosedCurvePlaneIntersection(vecCurveNetwork.at(iPlane).Profile3D.at(iCurve),
//					vecCurveNetwork.at(iCheckPlane).plane,
//					SegPoinsInd,InterSectPoints);
//				if (iIntersectNum)
//				{
//					vecCurveNetwork.at(iPlane).CurvePlaneIntersect.insert(vecCurveNetwork.at(iPlane).CurvePlaneIntersect.end(),
//						InterSectPoints.begin(),InterSectPoints.end());
//					vecCurveNetwork.at(iPlane).NeighborInd.insert(vecCurveNetwork.at(iPlane).NeighborInd.end(),
//						SegPoinsInd.begin(),SegPoinsInd.end());
//					for (int iInterPoint=0;iInterPoint<iIntersectNum;iInterPoint++)
//					{
//						vecCurveNetwork.at(iPlane).IntersectCurveIndex.push_back(iCurve);
//						vecCurveNetwork.at(iPlane).IntersectCNInd.push_back(iCheckPlane);
//					}
//				}
//
//			}
//		}
//		assert(vecCurveNetwork.at(iPlane).CurvePlaneIntersect.size()==vecCurveNetwork.at(iPlane).NeighborInd.size());
//		assert(vecCurveNetwork.at(iPlane).CurvePlaneIntersect.size()==vecCurveNetwork.at(iPlane).IntersectCurveIndex.size());
//		assert(vecCurveNetwork.at(iPlane).CurvePlaneIntersect.size()==vecCurveNetwork.at(iPlane).IntersectCNInd.size());
//	}
//}

void CCrossSectionProc::GetCurvePlaneIntersectPoints(vector<CurveNetwork> vecCurveNetwork,Plane_3 RefPlane[3], 
													 vector<Point_3>& vecCurvePlaneIntersectPoint,
													 vector<int>& vecCurvePlaneIntersectType)
{
	vecCurvePlaneIntersectPoint.clear();
	vecCurvePlaneIntersectType.clear();

	for (unsigned int iCN=0;iCN<vecCurveNetwork.size();iCN++)
	{
		for (int iPlaneType=0;iPlaneType<3;iPlaneType++)
		{
			if (vecCurveNetwork.at(iCN).ProfilePlaneType==iPlaneType)
			{
				continue;
			}
			for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iCN).Profile3D.size();iCurve++)
			{
				vector<vector<int> > SegPoinsInd;
				vector<Point_3> InterSectPoints;
				int iIntersectNum=GeometryAlgorithm::GetClosedCurvePlaneIntersection(vecCurveNetwork.at(iCN).Profile3D.at(iCurve),
					RefPlane[iPlaneType],SegPoinsInd,InterSectPoints);
				if (iIntersectNum)
				{
					for (int i=0;i<iIntersectNum;i++)
					{
						vecCurvePlaneIntersectPoint.push_back(InterSectPoints.at(i));
						vecCurvePlaneIntersectType.push_back(iPlaneType);
					}
				}

			}
		}
	}
	assert(vecCurvePlaneIntersectPoint.size()==vecCurvePlaneIntersectType.size());
}

bool CCrossSectionProc::CheckIntersectOthers(CurveNetwork CnToCheck,Polygon_2 NewProfile2D,int iExclProfileInd/*=-1*/)
{
	//assume the profile to check lie on the same plane with CnToCheck
	for (unsigned int i=0;i<CnToCheck.Profile2D.size();i++)
	{
		if (i==iExclProfileInd)
		{
			continue;
		}
		int iResult=GeometryAlgorithm::PlanarPolygonsContained2D(CnToCheck.Profile2D.at(i),NewProfile2D);
		if (iResult==2)
		{
			cout<<"Error: The input cross section intersects with other cross sections..."<<endl;
			return true;
		}
	}
	return false;
}


void CCrossSectionProc::SetCNOrientation(CurveNetwork& CnToSet)
{
	tree<int> TotalTree;
	tree<int>::iterator TotalParent=TotalTree.set_head(-1);

	//iterate all curves on the plane
	for (unsigned int iCurve=0;iCurve<CnToSet.Profile2D.size();iCurve++)
	{
		//for each curve,judge from the super parent of the tree
		tree<int>::iterator ParentNode=TotalParent;

		cout<<"in tree, "<<iCurve<<" th curve"<<endl;

		while (true)
		{
			if (TotalTree.number_of_children(ParentNode)==0)
			{
				//the parent has no children, add a child, turn to the next curve
				TotalTree.append_child(ParentNode,iCurve);
				break;
			}
			else
			{
				int iResult=4;
				//iterate all the children of the parent
				tree<int>::sibling_iterator sibIter=TotalTree.begin(ParentNode);

				cout<<"compare with "<<*sibIter<<" th curve"<<endl;

				vector<tree<int>::iterator> vecChildrenToContain;

				while(sibIter!=TotalTree.end(ParentNode)) 
				{
					iResult=GeometryAlgorithm::PlanarPolygonsContained2D(CnToSet.Profile2D.at(iCurve),CnToSet.Profile2D.at(*sibIter));
					//if inside the child,parent node=this child,continue;
					if (iResult==0)
					{
						ParentNode=sibIter;
						break;
					}
					//else if contain some children,collect all the children
					else if (iResult==1)
					{
						vecChildrenToContain.push_back(sibIter);
					}
					++sibIter;
				}

				if (iResult==1)
				{
					//reconstruct the tree
					tree<int>::iterator NewNode=TotalTree.append_child(ParentNode,iCurve);
					for (unsigned int iChild=0;iChild<vecChildrenToContain.size();iChild++)
					{
						tree<int>::iterator TempNode=TotalTree.append_child(NewNode,999);
						TotalTree.replace(TempNode,vecChildrenToContain.at(iChild));
						TotalTree.erase(vecChildrenToContain.at(iChild));
					}
					//turn to the next curve
					break;
				}
				else if (iResult==2)
				{
					//report an error
					break;
				}
				else if (iResult==3)
				{
					//independent, add a child to the parent
					//turn to the next curve
					TotalTree.append_child(ParentNode,iCurve);
					break;
				}
			}
		}
	}

	//finally, set the orientation acoording to the distance to the parent
	//meanwhile, record the out/in property of each curve (see the definition of CurveNetwork)
	CnToSet.CurveInOut.clear();
	CnToSet.CurveInOut.resize(CnToSet.Profile3D.size(),-1);
	tree<int>::iterator IterTotal=TotalTree.begin();
	while(IterTotal!=TotalTree.end())
	{
		int iDepth=TotalTree.depth(IterTotal);
		if (iDepth!=0&&iDepth%2==0)
		{
			if (GeometryAlgorithm::IsPlanarPolygonCCW(CnToSet.Profile3D.at(*IterTotal),CnToSet.ProfilePlaneType))
			{
				reverse(CnToSet.Profile3D.at(*IterTotal).begin(),CnToSet.Profile3D.at(*IterTotal).end());
				CnToSet.Profile2D.at(*IterTotal).reverse_orientation();
			}
			CnToSet.CurveInOut.at(*IterTotal)=*(TotalTree.parent(IterTotal));
		}
		else if (iDepth%2==1)//out side of a ring/circle
		{
			if (!GeometryAlgorithm::IsPlanarPolygonCCW(CnToSet.Profile3D.at(*IterTotal),CnToSet.ProfilePlaneType))
			{
				reverse(CnToSet.Profile3D.at(*IterTotal).begin(),CnToSet.Profile3D.at(*IterTotal).end());
				CnToSet.Profile2D.at(*IterTotal).reverse_orientation();
			}
		}
		++IterTotal;
	}

	assert(CnToSet.Profile3D.size()==CnToSet.CurveInOut.size());

	tree<int>::pre_order_iterator it=TotalTree.begin();
	tree<int>::pre_order_iterator end=TotalTree.end();
	int rootdepth=TotalTree.depth(it);
	cout<<"-----"<<endl;
	//std::cout << "-----" << std::endl;
	while(it!=end) {
		for(int i=0; i<TotalTree.depth(it)-rootdepth; ++i)
			cout<<"  ";
		//std::cout << "  ";
		//std::cout << (*it) << std::endl << std::flush;
		cout<<(*it)<<endl;
		++it;
	}
	//std::cout << "-----" << std::endl;
	cout<<"-----"<<endl;
}

bool CCrossSectionProc::GetCNFromSelectIndex(int iSelInd,vector<CurveNetwork> vecCurveNetwork,int& iWhichCN,int& iWhichCS)
{
	if (iSelInd<CREATION_CURVE_NAME_BEGIN || iSelInd>CREATION_CURVE_NAME_END)
	{
		return false;
	}

	int iBase=CREATION_CURVE_NAME_BEGIN;
	for (unsigned int i=0;i<vecCurveNetwork.size();i++)
	{
		int iNumCurve=vecCurveNetwork.at(i).Profile3D.size();
		if (iSelInd<iBase+iNumCurve)
		{
			iWhichCN=i;
			iWhichCS=iSelInd-iBase;
			break;
		}
		else
		{
			iBase=iBase+iNumCurve;
		}
	}
	return true;
}

void CCrossSectionProc::CopyCNFromLastParaPlane(int iPlaneType,Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
												vector<Point_3>& vecCurvePlaneIntersectPoint,
												vector<int>& vecCurvePlaneIntersectType)
{
	CurveNetwork LastCN;
	if (vecCurveNetwork.empty())
	{
		return;
	}
	for (int iCN=vecCurveNetwork.size()-1;iCN>=0;iCN--)
	{
		if ((vecCurveNetwork.at(iCN).ProfilePlaneType==iPlaneType)&&(vecCurveNetwork.at(iCN).plane!=RefPlane[iPlaneType]))
		{
			LastCN=vecCurveNetwork.at(iCN);
			break;
		}
	}
	if (LastCN.Profile3D.empty())
	{
		return;
	}
	//start copy
	CurveNetwork NewCN;
	for (unsigned int iProfile=0;iProfile<LastCN.Profile3D.size();iProfile++)
	{
		vector<Point_3> NewCurve;
		for (unsigned int iPoint=0;iPoint<LastCN.Profile3D.at(iProfile).size();iPoint++)
		{
			switch(iPlaneType)
			{
			case 0:
				NewCurve.push_back(Point_3(LastCN.Profile3D.at(iProfile).at(iPoint).x(),LastCN.Profile3D.at(iProfile).at(iPoint).y(),
					-RefPlane[iPlaneType].d()));
				break;
			case 1:
				NewCurve.push_back(Point_3(LastCN.Profile3D.at(iProfile).at(iPoint).x(),-RefPlane[iPlaneType].d(),
					LastCN.Profile3D.at(iProfile).at(iPoint).z()));
				break;
			case 2:
				NewCurve.push_back(Point_3(-RefPlane[iPlaneType].d(),LastCN.Profile3D.at(iProfile).at(iPoint).y(),
					LastCN.Profile3D.at(iProfile).at(iPoint).z()));
				break;
			default:
				break;
			}
		}
		NewCN.Profile3D.push_back(NewCurve);
	}
	NewCN.Profile2D=LastCN.Profile2D;
	NewCN.plane=RefPlane[iPlaneType];
	NewCN.ProfilePlaneType=iPlaneType;
	vecCurveNetwork.push_back(NewCN);
	SetCNOrientation(vecCurveNetwork.back());


	//GetCNIntersectPoints(vecCurveNetwork);
	GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,
		vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);
}

void CCrossSectionProc::DeleteLastCS(Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
										vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType)
{
	if (vecCurveNetwork.empty())
	{
		return;
	}
	if (vecCurveNetwork.back().Profile3D.size()==1)
	{
		vecCurveNetwork.pop_back();
	} 
	else
	{
		vecCurveNetwork.back().Profile3D.pop_back();
		vecCurveNetwork.back().Profile2D.pop_back();
		SetCNOrientation(vecCurveNetwork.back());
	}
	//GetCNIntersectPoints(vecCurveNetwork);
	GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,
		vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);
}

void CCrossSectionProc::DeleteSpecifiedCS(Plane_3 RefPlane[3],int& iWhichCN,int& iWhichCS,vector<CurveNetwork>& vecCurveNetwork, 
										  vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType)
{
	if (vecCurveNetwork.empty())
	{
		return;
	}
	if (vecCurveNetwork.at(iWhichCN).Profile3D.size()==1)
	{
		vecCurveNetwork.erase(vecCurveNetwork.begin()+iWhichCN);
		iWhichCN=NONE_SELECTED;
		iWhichCS=NONE_SELECTED;
	}
	else
	{
		vecCurveNetwork.at(iWhichCN).Profile3D.erase(vecCurveNetwork.at(iWhichCN).Profile3D.begin()+iWhichCS);
		vecCurveNetwork.at(iWhichCN).Profile2D.erase(vecCurveNetwork.at(iWhichCN).Profile2D.begin()+iWhichCS);
		//don't forget to handle the partial info(if exist)
		map<int,vector<vector<int>>>::iterator MapIter=vecCurveNetwork.at(iWhichCN).PartProfile3D.find(iWhichCS);
		if (MapIter!=vecCurveNetwork.at(iWhichCN).PartProfile3D.end())
		{
			vecCurveNetwork.at(iWhichCN).PartProfile3D.erase(MapIter);
		}
		SetCNOrientation(vecCurveNetwork.at(iWhichCN));
		iWhichCS=NONE_SELECTED;
	}
	//GetCNIntersectPoints(vecCurveNetwork);
	GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,
		vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);

}

//before fitting a CN,GetCNIntersectPoints & GetCurvePlaneIntersectPoints must be done
bool CCrossSectionProc::FitSpecifiedCN(int iSpecifiedCN,Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
									   vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType)
{
	bool bResult=true;
	for (unsigned int iPlane=0;iPlane<vecCurveNetwork.size();iPlane++)
	{
		if (iPlane==iSpecifiedCN)//itself,don't check
		{
			continue;
		}
		if (vecCurveNetwork.at(iPlane).ProfilePlaneType==vecCurveNetwork.at(iSpecifiedCN).ProfilePlaneType)//parallel,don't check
		{
			continue;
		}


		//AB:intersection with existing CN(iPlane) and Specified CN plane (iSpecifiedCN)
		vector<Point_3> InterSectPointsAB;
		vector<int> vecCurveAB;//curve id of each InterSectPointsAB in existing CN
		vector<vector<int>> SegPoinsIndAB;
		int iIntersectNumAB=0;
		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iPlane).Profile3D.size();iCurve++)
		{
			vector<vector<int>> SegPoinsInd;
			vector<Point_3> InterSectPoints;
			int iNum=GeometryAlgorithm::GetClosedCurvePlaneIntersection(vecCurveNetwork.at(iPlane).Profile3D.at(iCurve),
				vecCurveNetwork.at(iSpecifiedCN).plane,	SegPoinsInd,InterSectPoints);
			if (iNum)
			{
				iIntersectNumAB=iIntersectNumAB+iNum;
				InterSectPointsAB.insert(InterSectPointsAB.end(),InterSectPoints.begin(),InterSectPoints.end());
				SegPoinsIndAB.insert(SegPoinsIndAB.end(),SegPoinsInd.begin(),SegPoinsInd.end());
				for (unsigned int j=0;j<InterSectPoints.size();j++)
				{
					vecCurveAB.push_back(iCurve);
				}
			}
		}

		//BA:intersection with existing CN plane(iPlane) and Specified CN(iSpecifiedCN)
		vector<Point_3> InterSectPointsBA;
		vector<int> vecCurveBA;//curve id of each InterSectPointsBA in specified CN
		vector<vector<int>> SegPoinsIndBA;
		int iIntersectNumBA=0;
		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iSpecifiedCN).Profile3D.size();iCurve++)
		{
			vector<vector<int>> SegPoinsInd;
			vector<Point_3> InterSectPoints;
			int iNum=GeometryAlgorithm::GetClosedCurvePlaneIntersection(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurve),
				vecCurveNetwork.at(iPlane).plane,SegPoinsInd,InterSectPoints);
			if (iNum)
			{
				iIntersectNumBA=iIntersectNumBA+iNum;
				InterSectPointsBA.insert(InterSectPointsBA.end(),InterSectPoints.begin(),InterSectPoints.end());
				SegPoinsIndBA.insert(SegPoinsIndBA.end(),SegPoinsInd.begin(),SegPoinsInd.end());
				for (unsigned int j=0;j<InterSectPoints.size();j++)
				{
					vecCurveBA.push_back(iCurve);
				}
			}
		}

		//intersect number don't equal
		if (iIntersectNumAB!=iIntersectNumBA)
		{
			QMessageBox msgBox;
			msgBox.setText("Intersect Num don't equal");
			msgBox.exec();
			cout<<"rule 4 is violated...,Existing CN id: "<<iPlane<<",Specified CN id: "<<iSpecifiedCN<<endl;
			bResult=false;
			break;
		}

		vector<Int_Int_Pair> GroupResult;
		vector<Point_3> vecMidPoint;
		GeometryAlgorithm::GroupNearestPoints(InterSectPointsAB,InterSectPointsBA,GroupResult,vecMidPoint);

		//int: which curve
		//map<int,Point_3> int: which point Point_3: point pos
		map<int,map<int,Point_3>> mapInsertAB,mapInsertBA; 
		
		//for all intersection points
		for (unsigned int iJoint=0;iJoint<GroupResult.size();iJoint++)
		{
			//handle existing curve
			//which intersection point
			int iPointAB=GroupResult.at(iJoint).first;
			//point coordinates
			Point_3 PointAB=InterSectPointsAB.at(iPointAB);
			//which curve
			int iCurveAB=vecCurveAB.at(iPointAB);
			//which seg
			vector<int> SegAB=SegPoinsIndAB.at(iPointAB);
			//check the distance from the two end points of SegAB to the intersection point
			double dDist0=CGAL::squared_distance(PointAB,vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.front()));
			double dDist1=CGAL::squared_distance(PointAB,vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.back()));
			double dMinDist=dDist0<dDist1?dDist0:dDist1;
			//same points
			if (PointAB==vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.front()))
			{
				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.front())=PointAB;
			}
			else if (PointAB==vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.back()))
			{
				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.back())=PointAB;
			}
			//if distance is small, then just move the end point;else, insert the intersection point
			else if ((dMinDist==dDist0) && (dMinDist<0.0001))//0.0001
			{
				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.front())=PointAB;
			}
			else if ((dMinDist==dDist1) && (dMinDist<0.0001))
			{
				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(SegAB.back())=PointAB;
			}
			else
			{
				map<int,map<int,Point_3>>::iterator pFind=mapInsertAB.find(iCurveAB);
				if (pFind==mapInsertAB.end())//new curve
				{
					map<int,Point_3> NewInsertPoint;
					NewInsertPoint.insert(make_pair(SegAB.back(),PointAB));
					mapInsertAB.insert(make_pair(iCurveAB,NewInsertPoint));
				}
				else
				{
					pFind->second.insert(make_pair(SegAB.back(),PointAB));
				}
			}

			//handle specified curve
			//which intersection point
			int iPointBA=GroupResult.at(iJoint).second;
			//which curve
			int iCurveBA=vecCurveBA.at(iPointBA);
			//which seg
			vector<int> SegBA=SegPoinsIndBA.at(iPointBA);
			//check the distance from the two end points of SegBA to the intersection point
			dDist0=CGAL::squared_distance(PointAB,vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.front()));
			dDist1=CGAL::squared_distance(PointAB,vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.back()));
			dMinDist=dDist0<dDist1?dDist0:dDist1;
			//same points
			if (PointAB==vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.front()))
			{
				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.front())=PointAB;
			}
			else if (PointAB==vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.back()))
			{
				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.back())=PointAB;
			}
			//if distance is small, then just move the end point to PointAB directly;else, insert the intersection point
			else if ((dMinDist==dDist0) && (dMinDist<0.0001))
			{
				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.front())=PointAB;
			}
			else if ((dMinDist==dDist1) && (dMinDist<0.0001))
			{
				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(SegBA.back())=PointAB;
			}
			else
			{
				map<int,map<int,Point_3>>::iterator pFind=mapInsertBA.find(iCurveBA);
				if (pFind==mapInsertBA.end())//new curve
				{
					map<int,Point_3> NewInsertPoint;
					NewInsertPoint.insert(make_pair(SegBA.back(),PointAB));
					mapInsertBA.insert(make_pair(iCurveBA,NewInsertPoint));
				}
				else
				{
					pFind->second.insert(make_pair(SegBA.back(),PointAB));
				}
			}
		}//for all intersection points
		//if insert new points,update
		if (!mapInsertAB.empty())
		{
			for (map<int,map<int,Point_3>>::iterator Iter=mapInsertAB.begin();Iter!=mapInsertAB.end();Iter++)
			{
				int iCurveAB=Iter->first;
				//save the partial CS info(if exist) for further update
				vector<pair<Point_3,Point_3>> vecEndPoints;
				map<int,vector<vector<int>>>::iterator PartIter;
				if (!vecCurveNetwork.at(iPlane).PartProfile3D.empty())
				{
					PartIter=vecCurveNetwork.at(iPlane).PartProfile3D.find(iCurveAB);
					if (PartIter!=vecCurveNetwork.at(iPlane).PartProfile3D.end())
					{
						for (unsigned int iPart=0;iPart<PartIter->second.size();iPart++)
						{
							int iStartPointID=PartIter->second.at(iPart).front();
							Point_3 StartPoint=vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(iStartPointID);
							int iEndPointID=PartIter->second.at(iPart).back();
							Point_3 EndPoint=vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).at(iEndPointID);
							vecEndPoints.push_back(make_pair(StartPoint,EndPoint));
						}
					}
				}
				//re-arrange the points on each modified CS
				int iIncrement=0;
				for (map<int,Point_3>::iterator Iter2=Iter->second.begin();Iter2!=Iter->second.end();Iter2++)
				{
					int iPointPos=Iter2->first;
					Point_3 InsertPoint=Iter2->second;
					iPointPos=iPointPos+iIncrement;
					vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).insert(
						vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).begin()+iPointPos,InsertPoint);
					iIncrement++;
				}
				//re-sort the partial CS info(if exist)
				if (!vecEndPoints.empty())
				{
					PartIter->second.clear();
					for (unsigned int iPart=0;iPart<vecEndPoints.size();iPart++)
					{
						//find the new id of the two end points in the new curve,according to the point positions
						Point_3 StartPoint=vecEndPoints.at(iPart).first;
						Point_3 EndPoint=vecEndPoints.at(iPart).second;
						vector<Point_3>::iterator FindStart=find(vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).begin(),
							vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).end(),StartPoint);
						assert(FindStart!=vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).end());
						int iStartPointNewID=distance(vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).begin(),FindStart);
						vector<Point_3>::iterator FindEnd=find(vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).begin(),
							vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).end(),EndPoint);
						assert(FindEnd!=vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).end());
						int iEndPointNewID=distance(vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).begin(),FindEnd);
						vector<int> vecNewPart;
						int iPointNewID=iStartPointNewID;
						//iterate to save the point id on the partial curve 
						while (true)
						{
							vecNewPart.push_back(iPointNewID);
							iPointNewID=(iPointNewID+1)%vecCurveNetwork.at(iPlane).Profile3D.at(iCurveAB).size();
							if (iPointNewID==iEndPointNewID)
							{
								vecNewPart.push_back(iPointNewID);
								break;
							}
						}
						PartIter->second.push_back(vecNewPart);
					}
				}
			}
		}//if insert new points,update
		//if insert new points,update
		if (!mapInsertBA.empty())
		{
			for (map<int,map<int,Point_3>>::iterator Iter=mapInsertBA.begin();Iter!=mapInsertBA.end();Iter++)
			{
				int iCurveBA=Iter->first;
				//save the partial CS info(if exist) for further update
				vector<pair<Point_3,Point_3>> vecEndPoints;
				map<int,vector<vector<int>>>::iterator PartIter;
				if (!vecCurveNetwork.at(iSpecifiedCN).PartProfile3D.empty())
				{
					PartIter=vecCurveNetwork.at(iSpecifiedCN).PartProfile3D.find(iCurveBA);
					if (PartIter!=vecCurveNetwork.at(iSpecifiedCN).PartProfile3D.end())
					{
						for (unsigned int iPart=0;iPart<PartIter->second.size();iPart++)
						{
							int iStartPointID=PartIter->second.at(iPart).front();
							Point_3 StartPoint=vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(iStartPointID);
							int iEndPointID=PartIter->second.at(iPart).back();
							Point_3 EndPoint=vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).at(iEndPointID);
							vecEndPoints.push_back(make_pair(StartPoint,EndPoint));
						}
					}
				}
				//re-arrange the points on each modified CS
				int iIncrement=0;
				for (map<int,Point_3>::iterator Iter2=Iter->second.begin();Iter2!=Iter->second.end();Iter2++)
				{
					int iPointPos=Iter2->first;
					Point_3 InsertPoint=Iter2->second;
					iPointPos=iPointPos+iIncrement;
					vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).insert(
						vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).begin()+iPointPos,InsertPoint);
					iIncrement++;
				}
				//re-sort the partial CS info(if exist)
				if (!vecEndPoints.empty())
				{
					PartIter->second.clear();
					for (unsigned int iPart=0;iPart<vecEndPoints.size();iPart++)
					{
						//find the new id of the two end points in the new curve,according to the point positions
						Point_3 StartPoint=vecEndPoints.at(iPart).first;
						Point_3 EndPoint=vecEndPoints.at(iPart).second;
						vector<Point_3>::iterator FindStart=find(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).begin(),
							vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).end(),StartPoint);
						assert(FindStart!=vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).end());
						int iStartPointNewID=distance(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).begin(),FindStart);
						vector<Point_3>::iterator FindEnd=find(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).begin(),
							vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).end(),EndPoint);
						assert(FindEnd!=vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).end());
						int iEndPointNewID=distance(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).begin(),FindEnd);
						vector<int> vecNewPart;
						int iPointNewID=iStartPointNewID;
						//iterate to save the point id on the partial curve 
						while (true)
						{
							vecNewPart.push_back(iPointNewID);
							iPointNewID=(iPointNewID+1)%vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveBA).size();
							if (iPointNewID==iEndPointNewID)
							{
								vecNewPart.push_back(iPointNewID);
								break;
							}
						}
						PartIter->second.push_back(vecNewPart);
					}
				}
			}
		}//if insert new points,update

		//force the coordinates to be the consistent with the plane parameter,to avoid numerical issue
		ForceCNCoordinates(vecCurveNetwork.at(iPlane));
		ForceCNCoordinates(vecCurveNetwork.at(iSpecifiedCN));

		//update the 2D polygon of existing CN
		vecCurveNetwork.at(iPlane).Profile2D.clear();
		for (unsigned int iPoly2D=0;iPoly2D<vecCurveNetwork.at(iPlane).Profile3D.size();iPoly2D++)
		{
			Polygon_2 NewProfile2D;
			GeometryAlgorithm::PlanarPolygonToXOY(vecCurveNetwork.at(iPlane).Profile3D.at(iPoly2D),
				NewProfile2D,vecCurveNetwork.at(iPlane).ProfilePlaneType);
			vecCurveNetwork.at(iPlane).Profile2D.push_back(NewProfile2D);
		}
		//update the 2D polygon of specified CN
		vecCurveNetwork.at(iSpecifiedCN).Profile2D.clear();
		for (unsigned int iPoly2D=0;iPoly2D<vecCurveNetwork.at(iSpecifiedCN).Profile3D.size();iPoly2D++)
		{
			Polygon_2 NewProfile2D;
			GeometryAlgorithm::PlanarPolygonToXOY(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iPoly2D),
				NewProfile2D,vecCurveNetwork.at(iSpecifiedCN).ProfilePlaneType);
			vecCurveNetwork.at(iSpecifiedCN).Profile2D.push_back(NewProfile2D);
		}
	}
	GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);
	return bResult;
}

void CCrossSectionProc::ForceCNCoordinates(CurveNetwork& CnToSet)
{
	for (unsigned int iCurve=0;iCurve<CnToSet.Profile3D.size();iCurve++)
	{
		for (unsigned int iPoint=0;iPoint<CnToSet.Profile3D.at(iCurve).size();iPoint++)
		{
			if (CnToSet.ProfilePlaneType==CREATION_XOY_PLANE)
			{
				CnToSet.Profile3D.at(iCurve).at(iPoint)=Point_3(CnToSet.Profile3D.at(iCurve).at(iPoint).x(),
					CnToSet.Profile3D.at(iCurve).at(iPoint).y(),
					-CnToSet.plane.d());
			}
			else if (CnToSet.ProfilePlaneType==CREATION_YOZ_PLANE)
			{
				CnToSet.Profile3D.at(iCurve).at(iPoint)=Point_3(-CnToSet.plane.d(),
					CnToSet.Profile3D.at(iCurve).at(iPoint).y(),
					CnToSet.Profile3D.at(iCurve).at(iPoint).z());
			}
			else if (CnToSet.ProfilePlaneType==CREATION_XOZ_PLANE)
			{
				CnToSet.Profile3D.at(iCurve).at(iPoint)=Point_3(CnToSet.Profile3D.at(iCurve).at(iPoint).x(),
					-CnToSet.plane.d(),
					CnToSet.Profile3D.at(iCurve).at(iPoint).z());
			}
		}
	}
}

////before fitting a CN,GetCNIntersectPoints & GetCurvePlaneIntersectPoints must be done
//bool CCrossSectionProc::FitSpecifiedCN(int iSpecifiedCN,Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
//									   vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType)
//{
//	bool bResult=true;
//
//	for (unsigned int iPlane=0;iPlane<vecCurveNetwork.size();iPlane++)
//	{
//		if (iPlane==iSpecifiedCN)//itself,don't check
//		{
//			continue;
//		}
//		if (vecCurveNetwork.at(iPlane).ProfilePlaneType==vecCurveNetwork.at(iSpecifiedCN).ProfilePlaneType)//parallel,don't check
//		{
//			continue;
//		}
//
//		vector<int>::iterator::difference_type CurrentNum;//how many intersection points between current CN and plane of specified CN
//		CurrentNum = count(vecCurveNetwork.at(iPlane).IntersectCNInd.begin(), 
//			vecCurveNetwork.at(iPlane).IntersectCNInd.end(), iSpecifiedCN);
//		vector<int>::iterator::difference_type BackNum;//how many intersection points between last specified and plane of current CN
//		BackNum = count(vecCurveNetwork.at(iSpecifiedCN).IntersectCNInd.begin(), 
//			vecCurveNetwork.at(iSpecifiedCN).IntersectCNInd.end(), iPlane);
//		if (CurrentNum!=BackNum)//intersect number don't equal
//		{
//			AfxMessageBox("Intersect Num don't equal");
//			DBWindowWrite("rule 4 is violated...\n");
//			//DeleteLastCS(RefPlane,vecCurveNetwork,vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);			
//			bResult=false;
//			break;
//		}
//		else if (CurrentNum==0)//both have no intersect
//		{
//			continue;
//		}
//		//intersect number equal
//		//index of first intersection point between current CN and plane of last CN
//		vector<int>::iterator pFindCurrent=find(vecCurveNetwork.at(iPlane).IntersectCNInd.begin(),vecCurveNetwork.at(iPlane).IntersectCNInd.end(),
//			iSpecifiedCN);
//		int iFindCurrent=pFindCurrent-vecCurveNetwork.at(iPlane).IntersectCNInd.begin();
//		//index of first intersection point between last CN and plane of current CN
//		vector<int>::iterator pFindBack=find(vecCurveNetwork.at(iSpecifiedCN).IntersectCNInd.begin(),vecCurveNetwork.at(iSpecifiedCN).IntersectCNInd.end(),
//			iPlane);
//		int iFindBack=pFindBack-vecCurveNetwork.at(iSpecifiedCN).IntersectCNInd.begin();
//
//		vector<Point_3> CurrentInterPoint,BackInterPoint;
//		//all the intersection points between current CN and plane of last CN
//		CurrentInterPoint.insert(CurrentInterPoint.end(),vecCurveNetwork.at(iPlane).CurvePlaneIntersect.begin()+iFindCurrent,
//			vecCurveNetwork.at(iPlane).CurvePlaneIntersect.begin()+iFindCurrent+CurrentNum);
//		//all the intersection points between last CN and plane of current CN
//		BackInterPoint.insert(BackInterPoint.end(),vecCurveNetwork.at(iSpecifiedCN).CurvePlaneIntersect.begin()+iFindBack,
//			vecCurveNetwork.at(iSpecifiedCN).CurvePlaneIntersect.begin()+iFindBack+BackNum);
//		vector<vector<int>> GroupResult;
//		GeometryAlgorithm::GroupNearestPoints(CurrentInterPoint,BackInterPoint,GroupResult);//,vecMidPoint
//		//compute the destination point between each pair of nearest points
//		//here to avoid the re-movement of the point on existing curve,which will cause the originally intersected
//		//curves become disconnected
//		//set the destination point to the one on the existing curve directly, other than the one
//		//on the specified curve(i.e. only deform the last curve)
//		vector<Point_3> vecMidPoint;
//		for (unsigned int i=0;i<GroupResult.size();i++)
//		{
//			vecMidPoint.push_back(CurrentInterPoint.at(GroupResult.at(i).front()));
//		}
//
//		//indices of curves and points of the current curve network
//		//which are to be moved to a new positions
//		//move them to the new positions again after deformation
//		//to avoid their unwanted movement when they belong to the ROI of other
//		//constraint points
//		vector<int> viCurrentConstrPointCurveIndex;
//		vector<int> viCurrentConstrPointIndex;
//		//same guys for the specified curve network
//		vector<int> viSpeConstrPointCurveIndex;
//		vector<int> viSpeConstrPointIndex;
//
//		//const int iCurveDeformROIRange=6;
//		const int iCurveDeformROIRange=1;
//		for (unsigned int iJoint=0;iJoint<vecMidPoint.size();iJoint++)
//		{
//			//for current curve network
//			//which curve
//			int iCurveIndex=vecCurveNetwork.at(iPlane).IntersectCurveIndex.at(iFindCurrent+GroupResult.at(iJoint).front());
//			//index of handle point
//			int iHandleIndex;
//			if (CGAL::has_larger_distance_to_point(CurrentInterPoint.at(iJoint),
//				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveIndex).at(vecCurveNetwork.at(iPlane).NeighborInd.at(iFindCurrent+GroupResult.at(iJoint).front()).front()),
//				vecCurveNetwork.at(iPlane).Profile3D.at(iCurveIndex).at(vecCurveNetwork.at(iPlane).NeighborInd.at(iFindCurrent+GroupResult.at(iJoint).front()).back())))
//			{
//				iHandleIndex=vecCurveNetwork.at(iPlane).NeighborInd.at(iFindCurrent+GroupResult.at(iJoint).front()).back();
//			} 
//			else
//			{
//				iHandleIndex=vecCurveNetwork.at(iPlane).NeighborInd.at(iFindCurrent+GroupResult.at(iJoint).front()).front();
//			}
//			viCurrentConstrPointCurveIndex.push_back(iCurveIndex);
//			viCurrentConstrPointIndex.push_back(iHandleIndex);
//			//plane type
//			int iCurrentPlaneType=vecCurveNetwork.at(iPlane).ProfilePlaneType;
//			vector<int> iTemp0;iTemp0.push_back(iHandleIndex);
//			vector<Point_3> pTemp0;pTemp0.push_back(vecMidPoint.at(iJoint));
//			//only deform the handle point
//			vecCurveNetwork.at(iPlane).Profile3D.at(iCurveIndex).at(iHandleIndex)=vecMidPoint.at(iJoint);
//			//CCurveDeform::OpenCurveNaiveLaplacianDeform(vecCurveNetwork.at(iPlane).Profile3D.at(iCurveIndex),iTemp0,
//			//	pTemp0,iCurveDeformROIRange,iCurrentPlaneType);
//
//			//for specified curve network
//			//which curve
//			iCurveIndex=vecCurveNetwork.at(iSpecifiedCN).IntersectCurveIndex.at(iFindBack+GroupResult.at(iJoint).back());
//			//index of handle point
//			if (CGAL::has_larger_distance_to_point(BackInterPoint.at(iJoint),
//				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveIndex).at(vecCurveNetwork.at(iSpecifiedCN).NeighborInd.at(iFindBack+GroupResult.at(iJoint).back()).front()),
//				vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveIndex).at(vecCurveNetwork.at(iSpecifiedCN).NeighborInd.at(iFindBack+GroupResult.at(iJoint).back()).back())))
//			{
//				iHandleIndex=vecCurveNetwork.at(iSpecifiedCN).NeighborInd.at(iFindBack+GroupResult.at(iJoint).back()).back();
//			} 
//			else
//			{
//				iHandleIndex=vecCurveNetwork.at(iSpecifiedCN).NeighborInd.at(iFindBack+GroupResult.at(iJoint).back()).front();
//			}
//			//store the indices
//			viSpeConstrPointCurveIndex.push_back(iCurveIndex);
//			viSpeConstrPointIndex.push_back(iHandleIndex);
//			//plane type
//			iCurrentPlaneType=vecCurveNetwork.at(iSpecifiedCN).ProfilePlaneType;
//			vector<int> iTemp1;iTemp1.push_back(iHandleIndex);
//			vector<Point_3> pTemp1;pTemp1.push_back(vecMidPoint.at(iJoint));
//			CCurveDeform::OpenCurveNaiveLaplacianDeform(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iCurveIndex),iTemp1,
//				pTemp1,iCurveDeformROIRange,iCurrentPlaneType);
//		}
//		//force the constraint points to the new positions again after deformation
//		//to avoid their unwanted movement when they belong to the ROI of other
//		//constraint points
//		assert(viCurrentConstrPointCurveIndex.size()==vecMidPoint.size());
//		assert(viCurrentConstrPointIndex.size()==vecMidPoint.size());
//		assert(viSpeConstrPointCurveIndex.size()==vecMidPoint.size());
//		assert(viSpeConstrPointIndex.size()==vecMidPoint.size());
//		for (unsigned int iJoint=0;iJoint<vecMidPoint.size();iJoint++)
//		{
//			vecCurveNetwork.at(iPlane).Profile3D.at(viCurrentConstrPointCurveIndex.at(iJoint)).at(viCurrentConstrPointIndex.at(iJoint))
//				=vecMidPoint.at(iJoint);
//			vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(viSpeConstrPointCurveIndex.at(iJoint)).at(viSpeConstrPointIndex.at(iJoint))
//				=vecMidPoint.at(iJoint);
//		}
//		//don't forget to update the 2d polygon
//		vecCurveNetwork.at(iPlane).Profile2D.clear();
//		for (unsigned int iPoly2D=0;iPoly2D<vecCurveNetwork.at(iPlane).Profile3D.size();iPoly2D++)
//		{
//			Polygon_2 NewProfile2D;
//			GeometryAlgorithm::PlanarPolygonToXOY(vecCurveNetwork.at(iPlane).Profile3D.at(iPoly2D),
//				NewProfile2D,vecCurveNetwork.at(iPlane).ProfilePlaneType);
//			vecCurveNetwork.at(iPlane).Profile2D.push_back(NewProfile2D);
//		}
//		GetCNIntersectPoints(vecCurveNetwork);
//	}
//	//don't forget to update the 2d polygon
//	vecCurveNetwork.at(iSpecifiedCN).Profile2D.clear();
//	for (unsigned int iPoly2D=0;iPoly2D<vecCurveNetwork.at(iSpecifiedCN).Profile3D.size();iPoly2D++)
//	{
//		Polygon_2 NewProfile2D;
//		GeometryAlgorithm::PlanarPolygonToXOY(vecCurveNetwork.at(iSpecifiedCN).Profile3D.at(iPoly2D),
//			NewProfile2D,vecCurveNetwork.at(iSpecifiedCN).ProfilePlaneType);
//		vecCurveNetwork.at(iSpecifiedCN).Profile2D.push_back(NewProfile2D);
//	}
//	GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);
//
//	return bResult;
//}

void CCrossSectionProc::AddComputedCNToTotalCN(vector<vector<Point_3>> vecComputedCS,vector<CurveNetwork>& vecCurveNetwork, 
											   Plane_3 RefPlane[3],int iPlaneType,
											   vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType)
{
	//if the computed CN overlaps with an existing CN(possible when PartProfile3D of the CN is not empty),
	//just delete the PartProfile3D of the CN
	for (unsigned int i=0;i<vecCurveNetwork.size();i++)
	{
		if (vecCurveNetwork.at(i).Profile3D==vecComputedCS)
		{
			vecCurveNetwork.at(i).PartProfile3D.clear();
			return;
		}
	}

	CurveNetwork NewCN;
	NewCN.Profile3D=vecComputedCS;
	//the plane which the computed curves lie on must be the current DrawingProfilePlane
	//otherwise the computed curves won't be rendered and selected
	for (unsigned int iPoly2D=0;iPoly2D<NewCN.Profile3D.size();iPoly2D++)
	{
		Polygon_2 NewProfile2D;
		GeometryAlgorithm::PlanarPolygonToXOY(NewCN.Profile3D.at(iPoly2D),NewProfile2D,iPlaneType);
		NewCN.Profile2D.push_back(NewProfile2D);
	}
	NewCN.plane=RefPlane[iPlaneType];
	NewCN.ProfilePlaneType=iPlaneType;
	vecCurveNetwork.push_back(NewCN);
	SetCNOrientation(vecCurveNetwork.back());
	//don't forget to update the 2d polygon
	vecCurveNetwork.back().Profile2D.clear();
	for (unsigned int iPoly2D=0;iPoly2D<vecCurveNetwork.back().Profile3D.size();iPoly2D++)
	{
		Polygon_2 NewProfile2D;
		GeometryAlgorithm::PlanarPolygonToXOY(vecCurveNetwork.back().Profile3D.at(iPoly2D),
			NewProfile2D,vecCurveNetwork.back().ProfilePlaneType);
		vecCurveNetwork.back().Profile2D.push_back(NewProfile2D);
	}
	//CCrossSectionProc::GetCNIntersectPoints(vecCurveNetwork);
	CCrossSectionProc::GetCurvePlaneIntersectPoints(vecCurveNetwork,RefPlane,
		vecCurvePlaneIntersectPoint,vecCurvePlaneIntersectType);
}

int CCrossSectionProc::GetCSToModify(vector<vector<Point_3>> vecCS,vector<Point_3> InputCurve)
{
	//if only one CS,no need to compute
	if (vecCS.size()==1)
	{
		return 0;
	}
	vector<double> vecAveDist;
	for (unsigned int i=0;i<vecCS.size();i++)
	{
		//find two ends points on the current CS corresponds to the modify curve
		double dDistBegin,dDistEnd;
		dDistBegin=dDistEnd=9999;
		int iBeginInd,iEndInd;
		iBeginInd=iEndInd=0;
		for (unsigned int j=0;j<vecCS.at(i).size();j++)
		{
			double dCurrentDistBegin=CGAL::squared_distance(InputCurve.front(),vecCS.at(i).at(j));
			if (dCurrentDistBegin<dDistBegin)
			{
				iBeginInd=j;
				dDistBegin=dCurrentDistBegin;
			}
			double dCurrentDistEnd=CGAL::squared_distance(InputCurve.back(),vecCS.at(i).at(j));
			if (dCurrentDistEnd<dDistEnd)
			{
				iEndInd=j;
				dDistEnd=dCurrentDistEnd;
			}

		}
		if (iBeginInd!=iEndInd)//record the average distance 
		{
			vecAveDist.push_back((dDistBegin+dDistEnd)/2.0);
		}
		else//if the two points are the same,the current CS is excluded
		{
			vecAveDist.push_back(9999);
		}
	}
	double dMinAveDist=9999;
	int iMinDistCS=-1;
	for (unsigned int i=0;i<vecAveDist.size();i++)
	{
		if (vecAveDist.at(i)<dMinAveDist)
		{
			dMinAveDist=vecAveDist.at(i);
			iMinDistCS=i;
		}
	}
	return iMinDistCS;
}

void CCrossSectionProc::ReverseModifyPointsID(int iCSLen,vector<int>& vecModifiedPointInd)
{
	vector<int> TempResult;
	//get the new id after the CS has been reversed
	for (unsigned int i=0;i<vecModifiedPointInd.size();i++)
	{
		TempResult.push_back(iCSLen-1-vecModifiedPointInd.at(i));
	}
	//reverse the TempResult
	reverse(TempResult.begin(),TempResult.end());
	vecModifiedPointInd=TempResult;
}

void CCrossSectionProc::SavePartialCSInfo(CurveNetwork& CnToSave,int iCSInd,vector<int> PartialPointInd)
{
	//find if the CS contains partial part
	std::map<int,std::vector<std::vector<int>>>::iterator pFindMap=CnToSave.PartProfile3D.find(iCSInd);
	//no,save it directly
	if (pFindMap==CnToSave.PartProfile3D.end())
	{
		vector<vector<int>> TotalPartialSeg;
		TotalPartialSeg.push_back(PartialPointInd);
		CnToSave.PartProfile3D.insert(make_pair(iCSInd,TotalPartialSeg));
	}
	//yes,judge if the new partial cs has overlap with stored partial cs
	else
	{
		vector<vector<int>> TotalPartialSeg=pFindMap->second;
		//check if the new partial cs overlaps with each exsiting partial cs
		vector<int> CombineResult=PartialPointInd;
		vector<int> DisjointPart;//id of existing partial cs that do NOT overlap with new partial cs
		//judge the relationship of each existing partial with the new partial
		//if have overlap,update the new partial only
		//else,record the id of existing partial
		for (unsigned int i=0;i<TotalPartialSeg.size();i++)
		{
			vector<int>::iterator pFindFront=find(TotalPartialSeg.at(i).begin(),TotalPartialSeg.at(i).end(),CombineResult.front());
			vector<int>::iterator pFindBack=find(TotalPartialSeg.at(i).begin(),TotalPartialSeg.at(i).end(),CombineResult.back());
			//frontal part of the new partial CS overlaps with existing partial CS, rear part does not
			if (pFindFront!=TotalPartialSeg.at(i).end() && pFindBack==TotalPartialSeg.at(i).end())
			{
				for (unsigned int j=0;j<CombineResult.size();j++)
				{
					vector<int>::iterator pFindStart=find(TotalPartialSeg.at(i).begin(),TotalPartialSeg.at(i).end(),CombineResult.at(j));
					if (pFindStart==TotalPartialSeg.at(i).end())
					{
						vector<int>::iterator IterStart=CombineResult.begin()+j;
						TotalPartialSeg.at(i).insert(TotalPartialSeg.at(i).end(),IterStart,CombineResult.end());
						CombineResult=TotalPartialSeg.at(i);
						break;
					}
				}
			}
			//rear part of the new partial CS overlaps with existing partial CS, frontal part does not
			else if (pFindFront==TotalPartialSeg.at(i).end() && pFindBack!=TotalPartialSeg.at(i).end())
			{
				for (unsigned int j=0;j<TotalPartialSeg.at(i).size();j++)
				{
					vector<int>::iterator pFindStart=find(CombineResult.begin(),CombineResult.end(),TotalPartialSeg.at(i).at(j));
					if (pFindStart==CombineResult.end())
					{
						vector<int>::iterator IterStart=TotalPartialSeg.at(i).begin()+j;
						CombineResult.insert(CombineResult.end(),IterStart,TotalPartialSeg.at(i).end());
						break;
					}
				}
			}
			//both part of the new partial CS overlaps with existing partial CS(new partial inside existing completely)
			else if (pFindFront!=TotalPartialSeg.at(i).end() && pFindBack!=TotalPartialSeg.at(i).end())
			{
				CombineResult=TotalPartialSeg.at(i);
			}
			//existing partial inside new completely or do not overlap at all
			else
			{
				vector<int>::iterator pFindOld=find(CombineResult.begin(),CombineResult.end(),TotalPartialSeg.at(i).front());
				//if the start point of the existing partial CS is inside the new partial CS,
				//then existing partial is completely inside new 
				//otherwise,the two partial do not overlap at all
				if (pFindOld==CombineResult.end())
				{
					DisjointPart.push_back(i);
				}
			}
		}
		vector<vector<int>> TempTotalPartialSeg;
		for (unsigned int i=0;i<DisjointPart.size();i++)
		{
			TempTotalPartialSeg.push_back(TotalPartialSeg.at(DisjointPart.at(i)));
		}
		TempTotalPartialSeg.push_back(CombineResult);
		pFindMap->second=TempTotalPartialSeg;
	}
}

bool CCrossSectionProc::ImportCrossSections(const char* fnames,vector<CurveNetwork>& vecCurveNetwork)
{
	FILE* fin = fopen(fnames, "r");
	//	FILE* fin = fopen( "sshape.contour", "r");

	if( fin == NULL )
	{
		cout<<"Unable to open file"<<endl;
		return false;
	}

	//contour number
	int ctrnum = 0;
	fscanf( fin, "%d", &ctrnum );

	vecCurveNetwork.clear();
	//read contours on each plane
	for( int i = 0; i < ctrnum; i ++)
	{
		CurveNetwork CurrentCN;
		//plane parameter
		float tparam[ 4 ];
		fscanf( fin, "%f %f %f %f\r\n", tparam, tparam + 1, tparam+2, tparam+3);
		//plane parameter for the current curve network
		CurrentCN.plane=Plane_3(tparam[0],tparam[1],tparam[2],tparam[3]);
		//type of plane for the current curve network
		if (CurrentCN.plane.a()!=0)
		{
			CurrentCN.ProfilePlaneType=CREATION_YOZ_PLANE;
		}
		else if (CurrentCN.plane.b()!=0)
		{
			CurrentCN.ProfilePlaneType=CREATION_XOZ_PLANE;
		}
		else if (CurrentCN.plane.c()!=0)
		{
			CurrentCN.ProfilePlaneType=CREATION_XOY_PLANE;
		}
		else
		{
			cout<<"error: unknown plane type!"<<endl;
		}

		//vertex number, edge number
		int vernum, edgenum;
		fscanf( fin, "%d %d\r\n", &vernum,&edgenum);

		//vertices
		vector<Point_3> vecPoint;
		for( int j = 0; j < vernum; j ++)
		{
			float tver[ 3 ];
			fscanf( fin, "%f %f %f\r\n", tver, tver+1, tver+2);
			vecPoint.push_back(Point_3(tver[0],tver[1],tver[2]));
		}

		//edges
		int iVerIndBase=0;
		vector<Point_3> CurrentProfile3D;
		for( int j = 0; j < edgenum; j ++ )
		{
			int tedge[ 4 ];
			fscanf( fin, "%d %d %d %d\r\n", tedge, tedge + 1, tedge + 2, tedge + 3 );
			CurrentProfile3D.push_back(vecPoint.at(tedge[0]));

			if (tedge[1]==iVerIndBase)
			{
				iVerIndBase=iVerIndBase+CurrentProfile3D.size();
				CurrentCN.Profile3D.push_back(CurrentProfile3D);
				//2d polygon data
				Polygon_2 NewProfile2D;
				GeometryAlgorithm::PlanarPolygonToXOY(CurrentProfile3D,NewProfile2D,CurrentCN.ProfilePlaneType);
				CurrentCN.Profile2D.push_back(NewProfile2D);

				CurrentProfile3D.clear();
			}
		}
		vecCurveNetwork.push_back(CurrentCN);
		SetCNOrientation(vecCurveNetwork.back());
	}
	fclose( fin );

	//kw added, if the CNs which should intersect do not intersect,this is necessary 
	//CCrossSectionProc::GetCNIntersectPoints(vecCurveNetwork);

	//find if partial CSs exist
	string PartFileName(fnames);
	PartFileName=PartFileName+".part";
	FILE* pPartFile=fopen(PartFileName.c_str(),"r");
	if( pPartFile == NULL )
	{
		cout<<"Unable to open part file"<<endl;
		return true;
	}
	int iPartialCN=0;
	//num of CN which contains partial CSs
	fscanf(pPartFile, "%d\r\n", &iPartialCN);
	for (int i=0;i<iPartialCN;i++)
	{
		//id of CN & num of partial CSs
		int iCN=-1;
		int iNumPartCS=-1;
		fscanf(pPartFile, "%d %d\r\n", &iCN,&iNumPartCS);
		map<int,vector<vector<int>>> ReadPartProfile3D;
		for (int j=0;j<iNumPartCS;j++)
		{
			//which CS & num of partials
			int iCS=-1;
			int iNumPart=-1;
			fscanf(pPartFile, "%d %d\r\n", &iCS,&iNumPart);
			vector<vector<int>> vecvecPart;
			for (int k=0;k<iNumPart;k++)
			{
				//num of points on the current partial
				int iNumPartPoints=-1;
				fscanf(pPartFile, "%d\r\n",&iNumPartPoints);
				vector<int> vecPart;
				for (int m=0;m<iNumPartPoints;m++)
				{
					int iPointId=-1;
					fscanf(pPartFile,"%d",&iPointId);
					vecPart.push_back(iPointId);
				}
				fscanf(pPartFile,"\r\n");
				vecvecPart.push_back(vecPart);
			}
			ReadPartProfile3D.insert(make_pair(iCS,vecvecPart));
		}
		vecCurveNetwork.at(iCN).PartProfile3D=ReadPartProfile3D;
	}
	fclose(pPartFile);
	return true;
}

bool CCrossSectionProc::ExportCrossSections(const char* fnames,vector<CurveNetwork> vecCurveNetwork)
{
	if (vecCurveNetwork.empty())
	{
		return false;
	}
	
	//output standard CN
	FILE* pFile=fopen(fnames,"w");
	if( pFile == NULL )
	{
		cout<<"Unable to open file"<<endl;
		return false;
	}

	//number of planes
	fprintf(pFile, "%d\n", vecCurveNetwork.size());

	//num of CN which contains partial CSs
	int iPartialCN=0;

	for (unsigned int iCN=0;iCN<vecCurveNetwork.size();iCN++)
	{
		if (!vecCurveNetwork.at(iCN).PartProfile3D.empty())
		{
			iPartialCN++;
		}
		//plane parameter
		Plane_3 CurrentPlane=vecCurveNetwork.at(iCN).plane;
		fprintf(pFile, "%.3f %.3f %.3f %.3f\n",CurrentPlane.a(),CurrentPlane.b(),CurrentPlane.c(),CurrentPlane.d());
		//vertex number,edge number (equal)
		int iVerNum=0;
		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iCN).Profile3D.size();iCurve++)
		{
			iVerNum=iVerNum+vecCurveNetwork.at(iCN).Profile3D.at(iCurve).size();
		}
		fprintf(pFile, "%d %d\n",iVerNum,iVerNum);
		//vertex positions
		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iCN).Profile3D.size();iCurve++)
		{
			for (unsigned int iPoint=0;iPoint<vecCurveNetwork.at(iCN).Profile3D.at(iCurve).size();iPoint++)
			{
				fprintf(pFile,"%.4f %.4f %.4f\n",vecCurveNetwork.at(iCN).Profile3D.at(iCurve).at(iPoint).x(),
					vecCurveNetwork.at(iCN).Profile3D.at(iCurve).at(iPoint).y(),
					vecCurveNetwork.at(iCN).Profile3D.at(iCurve).at(iPoint).z());
			}
		}
		//edges (indices of vertices)
		int iCurrentIndex=0;
		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iCN).Profile3D.size();iCurve++)
		{
			for (unsigned int iPoint=0;iPoint<vecCurveNetwork.at(iCN).Profile3D.at(iCurve).size();iPoint++)
			{
				if (iPoint==vecCurveNetwork.at(iCN).Profile3D.at(iCurve).size()-1)//the last edge
				{
					fprintf(pFile,"%d %d %d %d\n",iCurrentIndex,iCurrentIndex+1-vecCurveNetwork.at(iCN).Profile3D.at(iCurve).size(),1,2);
				}
				else
				{
					fprintf(pFile,"%d %d %d %d\n",iCurrentIndex,iCurrentIndex+1,1,2);
				}
				iCurrentIndex++;
			}
		}
		fprintf(pFile,"\n");
	}
	fclose(pFile);

	//output partial CSs if exist
	if (iPartialCN==0)
	{
		return true;
	}

	string PartFileName(fnames);
	PartFileName=PartFileName+".part";
	FILE* pPartFile=fopen(PartFileName.c_str(),"w");
	if( pPartFile == NULL )
	{
		cout<<"Unable to open file"<<endl;
		return false;
	}
	//num of CN which contains partial CSs
	fprintf(pPartFile, "%d\n",iPartialCN);
	for (unsigned int iCN=0;iCN<vecCurveNetwork.size();iCN++)
	{
		if (vecCurveNetwork.at(iCN).PartProfile3D.empty())
		{
			continue;
		}
		//id of CN & num of partial CSs
		fprintf(pPartFile, "%d %d\n",iCN,vecCurveNetwork.at(iCN).PartProfile3D.size());
		map<int,vector<vector<int>>>::iterator MapIter=vecCurveNetwork.at(iCN).PartProfile3D.begin();
		while(MapIter!=vecCurveNetwork.at(iCN).PartProfile3D.end())
		{
			vector<vector<int>> vecPartialSeg=MapIter->second;
			//which CS & num of partials
			fprintf(pPartFile, "%d %d\n",MapIter->first,vecPartialSeg.size());
			for (unsigned int i=0;i<vecPartialSeg.size();i++)
			{
				//num of points on the current partial
				fprintf(pPartFile, "%d\n",vecPartialSeg.at(i).size());
				for (unsigned int j=0;j<vecPartialSeg.at(i).size();j++)
				{
					fprintf(pPartFile,"%d ",vecPartialSeg.at(i).at(j));
				}
				fprintf(pPartFile,"\n");
			}
			MapIter++;
		}
	}
	fclose(pPartFile);
	return true;
}