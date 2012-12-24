#include "WedgeEdgeBasedDeform.h"
#include "../OBJHandle.h"
#include "DeformationAlgorithm.h"

void CWedgeEdgeBasedDeform::GetEdgeInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices, 
										vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle>& ROIEdges,
										vector<Halfedge_handle>& AnchorEdges)
{

	//get edges of handle vertices
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<Halfedge_handle>::iterator pFind=find(ROIEdges.begin(),ROIEdges.end(),Havc);
			vector<Halfedge_handle>::iterator pFindOpp=find(ROIEdges.begin(),ROIEdges.end(),Havc->opposite());
			if ((pFind==ROIEdges.end())&&(pFindOpp==ROIEdges.end()))
			{
				ROIEdges.push_back(Havc);
			}
			Havc++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
	}

	//get edges of roi vertices
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<Halfedge_handle>::iterator pFind=find(ROIEdges.begin(),ROIEdges.end(),Havc);
			vector<Halfedge_handle>::iterator pFindOpp=find(ROIEdges.begin(),ROIEdges.end(),Havc->opposite());
			if ((pFind==ROIEdges.end())&&(pFindOpp==ROIEdges.end()))
			{
				ROIEdges.push_back(Havc);
			}
			Havc++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
	}

	////re-order the ROIEdges acoording to their indices
	//vector<int> vecEdgeIndex;
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	vecEdgeIndex.push_back(ROIEdges.at(i)->GetEdgeIndex());
	//}
	//sort(vecEdgeIndex.begin(),vecEdgeIndex.end());

	//vector<Halfedge_handle>temp=ROIEdges;
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	vector<int>::iterator pFind=find(vecEdgeIndex.begin(),vecEdgeIndex.end(),ROIEdges.at(i)->GetEdgeIndex());
	//	assert(pFind!=vecEdgeIndex.end());
	//	int iNewPos=pFind-vecEdgeIndex.begin();
	//	temp.at(iNewPos)=ROIEdges.at(i);
	//}
	//ROIEdges=temp;

	//the anchor vertex edges,whose both end vertices are anchor vertices
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=vecAnchorVertices.at(i)->vertex_begin();
		do 
		{
			Vertex_handle VerOpp=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator pFindHandle=find(vecAnchorVertices.begin(),vecAnchorVertices.end(),VerOpp);
			if (pFindHandle!=vecAnchorVertices.end())//another end vertex is handle or roi
			{
				vector<Halfedge_handle>::iterator pFind=find(AnchorEdges.begin(),AnchorEdges.end(),Havc);
				vector<Halfedge_handle>::iterator pFindOpp=find(AnchorEdges.begin(),AnchorEdges.end(),Havc->opposite());
				if ((pFind==AnchorEdges.end())&&(pFindOpp==AnchorEdges.end()))
				{
					AnchorEdges.push_back(Havc);
				}
			}
			Havc++;
		} while(Havc!=vecAnchorVertices.at(i)->vertex_begin());
	}

	////re-order the AnchorEdges acoording to their indices
	//vecEdgeIndex.clear();
	//for (unsigned int i=0;i<AnchorEdges.size();i++)
	//{
	//	vecEdgeIndex.push_back(AnchorEdges.at(i)->GetEdgeIndex());
	//}
	//sort(vecEdgeIndex.begin(),vecEdgeIndex.end());

	//temp=AnchorEdges;
	//for (unsigned int i=0;i<AnchorEdges.size();i++)
	//{
	//	vector<int>::iterator pFind=find(vecEdgeIndex.begin(),vecEdgeIndex.end(),AnchorEdges.at(i)->GetEdgeIndex());
	//	assert(pFind!=vecEdgeIndex.end());
	//	int iNewPos=pFind-vecEdgeIndex.begin();
	//	temp.at(iNewPos)=ROIEdges.at(i);
	//}

	//AnchorEdges=temp;
}

void CWedgeEdgeBasedDeform::GetEdgeTopo(vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>>& NeighborEdges, 
										vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>>& AnchorNeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		assert(!ROIEdges.at(i)->is_border_edge());
		vector<Halfedge_handle> CurrentNbEdges;
		Vertex_handle EndVertex0=ROIEdges.at(i)->vertex();
		Vertex_handle EndVertex1=ROIEdges.at(i)->opposite()->vertex();

		vector<Halfedge_handle> Nb0;
		Halfedge_around_vertex_circulator Havc=EndVertex0->vertex_begin();
		do 
		{
			Nb0.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex0->vertex_begin());
		reverse(Nb0.begin(),Nb0.end());
		vector<Halfedge_handle>::iterator pFind=find(Nb0.begin(),Nb0.end(),ROIEdges.at(i));
		int iIndex=pFind-Nb0.begin();
		rotate(Nb0.begin(),Nb0.begin()+iIndex,Nb0.end());
		Nb0.erase(Nb0.begin());

		vector<Halfedge_handle> Nb1;
		Havc=EndVertex1->vertex_begin();
		do 
		{
			Nb1.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex1->vertex_begin());
		reverse(Nb1.begin(),Nb1.end());
		pFind=find(Nb1.begin(),Nb1.end(),ROIEdges.at(i)->opposite());
		iIndex=pFind-Nb1.begin();
		rotate(Nb1.begin(),Nb1.begin()+iIndex,Nb1.end());
		Nb1.erase(Nb1.begin());

		//there may be more vertices who connect both EndVertex0 and EndVertex1 !
		assert(Nb0.front()->opposite()->vertex()==Nb1.back()->opposite()->vertex());
		assert(Nb0.back()->opposite()->vertex()==Nb1.front()->opposite()->vertex());
		assert(Nb0.size()+Nb1.size()==EndVertex0->vertex_degree()+EndVertex1->vertex_degree()-2);

//		CurrentNbEdges=Nb0;
//		CurrentNbEdges.insert(CurrentNbEdges.end(),Nb1.begin(),Nb1.end());

		//only a wedge neighborhood is considered
		CurrentNbEdges.push_back(Nb0.front());
		CurrentNbEdges.push_back(Nb0.back());
		CurrentNbEdges.push_back(Nb1.front());
		CurrentNbEdges.push_back(Nb1.back());
		NeighborEdges.push_back(CurrentNbEdges);
	}

	for (unsigned int i=0;i<AnchorEdges.size();i++)
	{
		assert(!AnchorEdges.at(i)->is_border_edge());
		vector<Halfedge_handle> CurrentNbEdges;
		Vertex_handle EndVertex0=AnchorEdges.at(i)->vertex();
		Vertex_handle EndVertex1=AnchorEdges.at(i)->opposite()->vertex();

		vector<Halfedge_handle> Nb0;
		Halfedge_around_vertex_circulator Havc=EndVertex0->vertex_begin();
		do 
		{
			Nb0.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex0->vertex_begin());
		reverse(Nb0.begin(),Nb0.end());
		vector<Halfedge_handle>::iterator pFind=find(Nb0.begin(),Nb0.end(),AnchorEdges.at(i));
		int iIndex=pFind-Nb0.begin();
		rotate(Nb0.begin(),Nb0.begin()+iIndex,Nb0.end());
		Nb0.erase(Nb0.begin());

		vector<Halfedge_handle> Nb1;
		Havc=EndVertex1->vertex_begin();
		do 
		{
			Nb1.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex1->vertex_begin());
		reverse(Nb1.begin(),Nb1.end());
		pFind=find(Nb1.begin(),Nb1.end(),AnchorEdges.at(i)->opposite());
		iIndex=pFind-Nb1.begin();
		rotate(Nb1.begin(),Nb1.begin()+iIndex,Nb1.end());
		Nb1.erase(Nb1.begin());

		//there may be more vertices who connect both EndVertex0 and EndVertex1 !
		assert(Nb0.front()->opposite()->vertex()==Nb1.back()->opposite()->vertex());
		assert(Nb0.back()->opposite()->vertex()==Nb1.front()->opposite()->vertex());
		assert(Nb0.size()+Nb1.size()==EndVertex0->vertex_degree()+EndVertex1->vertex_degree()-2);

		//		CurrentNbEdges=Nb0;
		//		CurrentNbEdges.insert(CurrentNbEdges.end(),Nb1.begin(),Nb1.end());

		//only a wedge neighborhood is considered
		CurrentNbEdges.push_back(Nb0.front());
		CurrentNbEdges.push_back(Nb0.back());
		CurrentNbEdges.push_back(Nb1.front());
		CurrentNbEdges.push_back(Nb1.back());
		AnchorNeighborEdges.push_back(CurrentNbEdges);
	}
}

void CWedgeEdgeBasedDeform::ComputeEdgeLaplacian(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Point_3> NbPoints;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			NbPoints.push_back(NbPoint);
		}
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		if (iType==1)
		{
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				dLaplacian[0]=dLaplacian[0]+CurrentPoint.x()-NbPoints.at(j).x();
				dLaplacian[1]=dLaplacian[1]+CurrentPoint.y()-NbPoints.at(j).y();
				dLaplacian[2]=dLaplacian[2]+CurrentPoint.z()-NbPoints.at(j).z();
			}
			ROIEdges.at(i)->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		}
		else if (iType==3)
		{
			vector<double> EdgeWeights;
			double dPosSumWeight,dNegSumWeight;
			dPosSumWeight=dNegSumWeight=0;
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				Point_3 Vertex1=NbPoints.at(j);
				double dCurrentWeight=0;
				Point_3 Vertex1Prev,Vertex1Next;
				if (j==0)
				{
					Vertex1Prev=NbPoints.back();
					Vertex1Next=NbPoints.at(j+1);
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				else if (j==NbPoints.size()-1)
				{
					Vertex1Prev=NbPoints.at(j-1);
					Vertex1Next=NbPoints.front();
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				else
				{
					Vertex1Prev=NbPoints.at(j-1);
					Vertex1Next=NbPoints.at(j+1);
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Vertex1.x();
				dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Vertex1.y();
				dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Vertex1.z();
				EdgeWeights.push_back(dCurrentWeight);
				if (j<ROIEdges.at(i)->vertex()->vertex_degree()-1)
				{
					dPosSumWeight=dPosSumWeight+dCurrentWeight;
				}
				else
				{
					dNegSumWeight=dNegSumWeight+dCurrentWeight;
				}
			}
			dLaplacian[0]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.x()-dLaplacian[0];
			dLaplacian[1]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.y()-dLaplacian[1];
			dLaplacian[2]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.z()-dLaplacian[2];

			ROIEdges.at(i)->SetWeightedLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
			ROIEdges.at(i)->SetEdgeWeights(EdgeWeights);
			ROIEdges.at(i)->SetPosSumWeight(dPosSumWeight);
			ROIEdges.at(i)->SetNegSumWeight(dNegSumWeight);
		}
	}
}

void CWedgeEdgeBasedDeform::GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
										  vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
										  vector<vector<Halfedge_handle>> NeighborEdges,SparseMatrix& LaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	LaplacianMatrix.m=vecAllVertex.size();

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Vertex_handle EndVer0,EndVer1;//two end points of the edge
		EndVer0=ROIEdges.at(i)->vertex();
		EndVer1=ROIEdges.at(i)->opposite()->vertex();

		for (unsigned int j=0;j<vecAllVertex.size();j++)
		{
			if (iType==1)//uniform weight
			{
				if (vecAllVertex.at(j)==EndVer0)
				{
					LaplacianMatrix[i][j]=1;
				}
				else if (vecAllVertex.at(j)==EndVer1)
				{
					LaplacianMatrix[i][j]=1;
				}
				else if (NeighborEdges.at(i).front()->opposite()->vertex()==vecAllVertex.at(j)
					|| NeighborEdges.at(i).at(1)->opposite()->vertex()==vecAllVertex.at(j))
				{
					LaplacianMatrix[i][j]=-1.0;
				}
				else
				{
					//0, so do nothing
				}
			}
			else if (iType==3)//weighted
			{
				//if (vecAllVertex.at(j)==EndVer0)
				//{
				//	LaplacianMatrix[i][j]=0.5*ROIEdges.at(i)->GetNegSumWeight();
				//}
				//else if (vecAllVertex.at(j)==EndVer1)
				//{
				//	LaplacianMatrix[i][j]=0.5*ROIEdges.at(i)->GetPosSumWeight();
				//}
				//else
				//{
				//	int iNbIndex0=GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j));
				//	int iNbIndex1=GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j));
				//	if (iNbIndex0!=0||iNbIndex1!=0)
				//	{
				//		//may contain bugs!!
				//		float fTemp=0.0;
				//		for (unsigned int k=0;k<NeighborEdges.at(i).size();k++)
				//		{
				//			if (NeighborEdges.at(i).at(k)->opposite()->vertex()==vecAllVertex.at(j))
				//			{
				//				fTemp=fTemp-0.5*(ROIEdges.at(i)->GetEdgeWeights().at(k));
				//			}
				//		}
				//		if (fTemp!=0.0)
				//		{
				//			LaplacianMatrix[i][j]=fTemp;
				//		}
				//	}
				//	else
				//	{
				//		//0, so do nothing
				//	}
				//}
			}
		}
	}
}

void CWedgeEdgeBasedDeform::GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint, 
															vector<Vertex_handle> vecHandleNb,
															vector<Vertex_handle> ROIVertices, 
															vector<Vertex_handle> vecAnchorVertices,
															SparseMatrix& AnchorConstraintMatrix, 
															SparseMatrix& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	AnchorConstraintMatrix.m=HandleConstraintMatrix.m=iColumn;
	int iAnchorRow=(int)vecAnchorVertices.size();
	for (int i=0;i<iAnchorRow;i++)
	{
		for (int j=0;j<iColumn;j++)
		{
			if (j-i==iColumn-iAnchorRow)
			{
				AnchorConstraintMatrix[i][j]=1;
			} 
			else
			{
				//0, do nothing
			}
		}
	}

	int iHandleRow=(int)vecHandlePoint.size();
	for (int i=0;i<iHandleRow;i++)
	{
		HandlePointStruct CurrentHandlePoint=vecHandlePoint.at(i);
		for (unsigned int j=0;j<CurrentHandlePoint.vecVertexIndex.size();j++)
		{
			HandleConstraintMatrix[i][CurrentHandlePoint.vecVertexIndex.at(j)]=WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*CurrentHandlePoint.vecPara.at(j);
		}
	}
}

void CWedgeEdgeBasedDeform::BackUpEdgeVectorsForRigidDeform(vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges, 
															vector<Halfedge_handle>& AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Vector_3> EdgeVectors;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 CurrentEdge=CurrentPoint-NbPoint;
			EdgeVectors.push_back(CurrentEdge);
		}
		ROIEdges.at(i)->SetOldEdgeVectors(EdgeVectors);
		ROIEdges.at(i)->opposite()->SetOldEdgeVectors(EdgeVectors);
	}

	for (unsigned int i=0;i<AnchorEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(AnchorEdges.at(i)->vertex()->point(),AnchorEdges.at(i)->opposite()->vertex()->point());
		vector<Vector_3> EdgeVectors;
		for (unsigned int j=0;j<AnchorNeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(AnchorNeighborEdges.at(i).at(j)->vertex()->point(),
				AnchorNeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 CurrentEdge=CurrentPoint-NbPoint;
			EdgeVectors.push_back(CurrentEdge);
		}
		AnchorEdges.at(i)->SetOldEdgeVectors(EdgeVectors);
		AnchorEdges.at(i)->opposite()->SetOldEdgeVectors(EdgeVectors);
	}
}

void CWedgeEdgeBasedDeform::ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
														  vector<Vertex_handle> vecAnchorVertices, vector<Point_3> vecDeformCurvePoint3d,
														  vector<Halfedge_handle> ROIEdges, vector<vector<double> >& LaplacianRightHandSide, 
														  vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//edge laplacian
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(ROIEdges.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(ROIEdges.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(ROIEdges.at(i)->GetUniformLaplacian().z());
		}
		else if (iType==3)
		{
			Laplacian[0].push_back(ROIEdges.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(ROIEdges.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(ROIEdges.at(i)->GetWeightedLaplacian().z());
		}
	}

	//anchor and handle constraint
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecAnchorVertices.at(i)->point().x());
		Anchor[1].push_back(vecAnchorVertices.at(i)->point().y());
		Anchor[2].push_back(vecAnchorVertices.at(i)->point().z());
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIEdges.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CWedgeEdgeBasedDeform::ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
													vector<Point_3> AnchorConstraints, vector<Point_3> vecDeformCurvePoint3d,
													vector<Halfedge_handle> ROIEdges, vector<vector<Halfedge_handle>> NeighborEdges, 
													vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//edge laplacian
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIEdges.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIEdges.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIEdges.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIEdges.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIEdges.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIEdges.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIEdges.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=ROIEdges.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			vector<double> NbRotationMatrix=NeighborEdges.at(i).at(j)->GetRigidDeformRotationMatrix();
			//assume edges other than handle and roi are fixed
			if (NbRotationMatrix.empty())
			{
				NbRotationMatrix.push_back(1);NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(0);
				NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(1);NbRotationMatrix.push_back(0);
				NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(1);
			}
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}
			double dProduct[3];
			for (int k=0;k<3;k++)
			{
				dProduct[k]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+0)+NbRotationMatrix.at(3*k+0))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+1)+NbRotationMatrix.at(3*k+1))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+2)+NbRotationMatrix.at(3*k+2))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).z());
				CurrentRigid[k]=CurrentRigid[k]+dProduct[k];
			}
		}
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}

	for (unsigned int i=0;i<AnchorConstraints.size();i++)
	{
		Anchor[0].push_back(AnchorConstraints.at(i).x());
		Anchor[1].push_back(AnchorConstraints.at(i).y());
		Anchor[2].push_back(AnchorConstraints.at(i).z());
	}

	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIEdges.size()+AnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CWedgeEdgeBasedDeform::ComputeRotationForRigidDeform(int iType,vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges, 
														  vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges)
{
	//combine ROIEdges and AnchorEdges for short
	ROIEdges.insert(ROIEdges.end(),AnchorEdges.begin(),AnchorEdges.end());
	NeighborEdges.insert(NeighborEdges.end(),AnchorNeighborEdges.begin(),AnchorNeighborEdges.end());

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<vector<double> > TotalMatrix,IdentityMatrix;
		vector<double> tempRow;
		for (int j=0;j<3;j++)
		{
			tempRow.push_back(0);
		}
		for (int j=0;j<3;j++)
		{
			TotalMatrix.push_back(tempRow);
		}
		IdentityMatrix=TotalMatrix;

		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),
			ROIEdges.at(i)->opposite()->vertex()->point());

		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 NewEdge=CurrentPoint-NbPoint;

			//get corresponding old edge
			Vector_3 OldEdge=ROIEdges.at(i)->GetOldEdgeVectors().at(j);
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}

			//construct convariance matrix S
			vector<vector<double> > CurrentMatrix=IdentityMatrix;
			double dCurrentMatrix[9];
			dCurrentMatrix[0]=dCurrentWeight*OldEdge.x()*NewEdge.x();
			dCurrentMatrix[1]=dCurrentWeight*OldEdge.x()*NewEdge.y();
			dCurrentMatrix[2]=dCurrentWeight*OldEdge.x()*NewEdge.z();
			dCurrentMatrix[3]=dCurrentWeight*OldEdge.y()*NewEdge.x();
			dCurrentMatrix[4]=dCurrentWeight*OldEdge.y()*NewEdge.y();
			dCurrentMatrix[5]=dCurrentWeight*OldEdge.y()*NewEdge.z();
			dCurrentMatrix[6]=dCurrentWeight*OldEdge.z()*NewEdge.x();
			dCurrentMatrix[7]=dCurrentWeight*OldEdge.z()*NewEdge.y();
			dCurrentMatrix[8]=dCurrentWeight*OldEdge.z()*NewEdge.z();
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					CurrentMatrix.at(j).at(k)=dCurrentMatrix[3*j+k];
				}
			}
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					TotalMatrix.at(j).at(k)=TotalMatrix.at(j).at(k)+CurrentMatrix.at(j).at(k);
				}
			}
		}


		vector<vector<double> > OutPutU,OutPutSigma,OutPutV;
		CMath::ComputeSVD(TotalMatrix,OutPutU,OutPutSigma,OutPutV);

		//change the sign of the column of U which corresponds to the smallest singular value
		int iMinSVIndex=0;
		for (int j=0;j<3;j++)
		{
			if (OutPutSigma.at(j).at(j)!=0)
			{
				iMinSVIndex=j;
			}
			else
			{
				break;
			}
		}

		vector<double> RotationMatrix;
		bool bSign=false;
		while (true)//ensure that det(RotationMatrix)==1
		{
			if (bSign)
			{
				for (int j=0;j<3;j++)
				{
					OutPutU.at(j).at(iMinSVIndex)=-OutPutU.at(j).at(iMinSVIndex);
				}
			}
			//convert U to UT
			vector<vector<double> > UT=OutPutU;
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					UT.at(j).at(k)=OutPutU.at(k).at(j);
				}
			}
			//multiply V and UT
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					double dCurrentValue=0;
					for (int m=0;m<3;m++)
					{
						dCurrentValue=dCurrentValue+OutPutV.at(j).at(m)*UT.at(m).at(k);
					}
					RotationMatrix.push_back(dCurrentValue);
				}
			}
			double dDeterminant=RotationMatrix.at(0)*RotationMatrix.at(4)*RotationMatrix.at(8)
				+RotationMatrix.at(1)*RotationMatrix.at(5)*RotationMatrix.at(6)
				+RotationMatrix.at(2)*RotationMatrix.at(7)*RotationMatrix.at(3)
				-RotationMatrix.at(2)*RotationMatrix.at(4)*RotationMatrix.at(6)
				-RotationMatrix.at(1)*RotationMatrix.at(3)*RotationMatrix.at(8)
				-RotationMatrix.at(0)*RotationMatrix.at(7)*RotationMatrix.at(5);
			//DBWindowWrite("%f\t",dDeterminant);
			if (dDeterminant>0)
			{
				//DBWindowWrite("\n");
				break;
			}
			else
			{
				RotationMatrix.clear();
				bSign=true;
			}
		}

		//store rotation matrix
		ROIEdges.at(i)->SetRigidDeformRotationMatrix(RotationMatrix);
		ROIEdges.at(i)->opposite()->SetRigidDeformRotationMatrix(RotationMatrix);
	}
}

void CWedgeEdgeBasedDeform::ComputeScaleFactor(int iType,vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges, 
											   vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges,
											   bool bTestIsoScale/* =false */)
{
	//combine ROIEdges and AnchorEdges for short
	ROIEdges.insert(ROIEdges.end(),AnchorEdges.begin(),AnchorEdges.end());
	NeighborEdges.insert(NeighborEdges.end(),AnchorNeighborEdges.begin(),AnchorNeighborEdges.end());

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<double> RotationMatrix=ROIEdges.at(i)->GetRigidDeformRotationMatrix();
		int iOldEdgeIndex=0;
		double dNumerator[3],dDenominator[3];
		for (int j=0;j<3;j++)
		{
			dNumerator[j]=dDenominator[j]=0;
		}

		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),
			ROIEdges.at(i)->opposite()->vertex()->point());

		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 NewEdge=CurrentPoint-NbPoint;
			//get corresponding old edge
			Vector_3 OldEdge=ROIEdges.at(i)->GetOldEdgeVectors().at(j);
			//get RotatedOldEdge=RotationMatrix*OldEdge
			double dRotatedOldEdge[3];
			dRotatedOldEdge[0]=RotationMatrix.at(0)*OldEdge.x()+
				RotationMatrix.at(1)*OldEdge.y()+
				RotationMatrix.at(2)*OldEdge.z();
			dRotatedOldEdge[1]=RotationMatrix.at(3)*OldEdge.x()+
				RotationMatrix.at(4)*OldEdge.y()+
				RotationMatrix.at(5)*OldEdge.z();
			dRotatedOldEdge[2]=RotationMatrix.at(6)*OldEdge.x()+
				RotationMatrix.at(7)*OldEdge.y()+
				RotationMatrix.at(8)*OldEdge.z();
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}
			//get numerator
			dNumerator[0]=dNumerator[0]+dCurrentWeight*dRotatedOldEdge[0]*NewEdge.x();
			dNumerator[1]=dNumerator[1]+dCurrentWeight*dRotatedOldEdge[1]*NewEdge.y();
			dNumerator[2]=dNumerator[2]+dCurrentWeight*dRotatedOldEdge[2]*NewEdge.z();
			//get denominator
			dDenominator[0]=dDenominator[0]+dCurrentWeight*dRotatedOldEdge[0]*dRotatedOldEdge[0];
			dDenominator[1]=dDenominator[1]+dCurrentWeight*dRotatedOldEdge[1]*dRotatedOldEdge[1];
			dDenominator[2]=dDenominator[2]+dCurrentWeight*dRotatedOldEdge[2]*dRotatedOldEdge[2];
		}
		double dScale[3];
		for (int j=0;j<3;j++)
		{
			dScale[j]=(double)(dNumerator[j]/dDenominator[j]);
		}
		if (bTestIsoScale)
		{
			double dUniformScale=sqrt((dScale[0]*dScale[0]+dScale[1]*dScale[1]+dScale[2]*dScale[2])/3);
			dScale[0]=dScale[1]=dScale[2]=dUniformScale;
		}
		//store scaling matrix
		ROIEdges.at(i)->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
		ROIEdges.at(i)->opposite()->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
	}
}

void CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, vector<HandlePointStruct>& vecHandlePoint,
												 vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
												 vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints,bool bTestIsoScale/* =false */)
{
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges,AnchorEdges;
	GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,AnchorEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges,AnchorNeighborEdges;
	GetEdgeTopo(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);

	//compute edge laplacian
	ComputeEdgeLaplacian(iType,ROIEdges,NeighborEdges);

	//test
	//testPoints.clear();
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	Point_3 MidPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
	//	testPoints.push_back(MidPoint);
	//}
	cout<<"related edge num: "<<ROIEdges.size()<<endl;
	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;

	//get Laplacian matrix
	SparseMatrix LaplacianMatrix(ROIEdges.size());
	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	clock_t TotalBegin=clock();   

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		clock_t IterationBegin=clock();   

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
				vecDeformCurvePoint3d,ROIEdges,NeighborEdges,
				LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		clock_t RHS=clock();
		cout<<"time for computing RHS: "<<float(RHS-IterationBegin)<<endl;

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
		//		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);

		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

		if (bResult)
		{
			//vecDeformCurvePoint3d.clear();
			//calculated result of handle 
			for (unsigned int i=0;i<vecHandleNb.size();i++)
			{
				vecHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<ROIVertices.size();i++)
			{
				ROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+i),
					Result.at(1).at(vecHandleNb.size()+i),
					Result.at(2).at(vecHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecAnchorVertices.size();i++)
			{
				vecAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+i));
			}
		}

		clock_t LSE=clock();   
		//DBWindowWrite("time for computing LSE: %f\n",float(LSE-RHS));

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			ComputeScaleFactor(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges,bTestIsoScale);
		}
	}

	TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;
}

void CWedgeEdgeBasedDeform::WedgeEdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
												 vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
												 vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d,
												 vector<Point_3>& testPoints)
{
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges,AnchorEdges;
	GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,AnchorEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges,AnchorNeighborEdges;
	GetEdgeTopo(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);

	//compute edge laplacian
	ComputeEdgeLaplacian(iType,ROIEdges,NeighborEdges);

	//test
	//testPoints.clear();
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	Point_3 MidPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
	//	testPoints.push_back(MidPoint);
	//}
	cout<<"related edge num: "<<ROIEdges.size()<<endl;
	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;

	//get Laplacian matrix
	SparseMatrix LaplacianMatrix(ROIEdges.size());
	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandleNb.size());
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	clock_t TotalBegin=clock();   

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		clock_t IterationBegin=clock();   

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
				vecDeformCurvePoint3d,ROIEdges,NeighborEdges,
				LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		clock_t RHS=clock();   
		cout<<"time for computing RHS: "<<float(RHS-IterationBegin)<<endl;

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
		//		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);

		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

		if (bResult)
		{
			//vecDeformCurvePoint3d.clear();
			//calculated result of handle 
			for (unsigned int i=0;i<vecHandleNb.size();i++)
			{
				vecHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<ROIVertices.size();i++)
			{
				ROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+i),
					Result.at(1).at(vecHandleNb.size()+i),
					Result.at(2).at(vecHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecAnchorVertices.size();i++)
			{
				vecAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+i));
			}
		}

		clock_t LSE=clock();   
		//DBWindowWrite("time for computing LSE: %f\n",float(LSE-RHS));

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			ComputeScaleFactor(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
		}
	}

	TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();   
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;
}

void CWedgeEdgeBasedDeform::GetEdgeDomainMesh(KW_Mesh& PrimalMesh,map<Point_3,vector<Point_3>>& EdgeMesh)
{
	vector<Vertex_handle> vecHandleNb,ROIVertices,vecAnchorVertices;
	for (Vertex_iterator i=PrimalMesh.vertices_begin();i!=PrimalMesh.vertices_end();i++)
	{
		ROIVertices.push_back(i);
	}
	
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges,AnchorEdges;
	GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,AnchorEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges,AnchorNeighborEdges;
	GetEdgeTopo(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CenterPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Point_3> NeighborPoints;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			NeighborPoints.push_back(CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point()));
		}
		EdgeMesh.insert(pair <Point_3, vector<Point_3>>(CenterPoint,NeighborPoints));
	}

}

void CWedgeEdgeBasedDeform::BuildWedgeEdgeMesh(KW_Mesh& PrimalMesh,KW_Mesh& WedgeEdgeMesh)
{
	if (!PrimalMesh.is_closed())
	{
		return;
	}
	
	WedgeEdgeMesh.clear();

	//collect vertex information of WedgeEdgeMesh
	//set the index of each edge to -1 first
	Halfedge_iterator  HI=PrimalMesh.halfedges_begin();
	do 
	{
		HI->SetEdgeIndex(-1);
		HI++;
	} while(HI!=PrimalMesh.halfedges_end());
	//set the index of each edge starting from 0
	int iIndex=0;
	HI=PrimalMesh.halfedges_begin();
	do 
	{
		if (HI->GetEdgeIndex()==-1 || HI->opposite()->GetEdgeIndex()==-1)
		{
			HI->SetEdgeIndex(iIndex);
			HI->opposite()->SetEdgeIndex(iIndex);
			iIndex++;
		}
		HI++;
	} while(HI!=PrimalMesh.halfedges_end());

	KW_Polyhedron model;
	//put the vertices in
	HI=PrimalMesh.halfedges_begin();
	int iLastIndex=-1;
	do 
	{
		if (HI->GetEdgeIndex()!=iLastIndex)
		{
			if (HI->GetEdgeIndex()==iLastIndex+1)//confirm the order
			{
				model.vecVertex.push_back(CGAL::midpoint(HI->vertex()->point(),HI->opposite()->vertex()->point()));
			}
			else
			{
				cout<<"Error when building wedge edge mesh"<<endl;
			}
		}
		iLastIndex=HI->GetEdgeIndex();
		HI++;
	} while(HI!=PrimalMesh.halfedges_end());

	//put the faces in
	//the faces formed by the midpoints of edges around each vertex
	Vertex_iterator VI=PrimalMesh.vertices_begin();
	vector<int> VerIndex;
	do 
	{
		VerIndex.clear();
		Halfedge_around_vertex_circulator Havc=VI->vertex_begin();
		do 
		{
			VerIndex.push_back(Havc->GetEdgeIndex());
			Havc++;
		} while(Havc!=VI->vertex_begin());
		reverse(VerIndex.begin(),VerIndex.end());
		assert(VerIndex.size()==VI->vertex_degree());
		model.vecFacet.push_back(VerIndex);
		VI++;
	} while(VI!=PrimalMesh.vertices_end());
	//the faces formed by the midpoints of edges of each triangle
	Facet_iterator FI=PrimalMesh.facets_begin();
	do 
	{
		VerIndex.clear();
		Halfedge_around_facet_circulator Hafc = FI->facet_begin();
		do 
		{
			VerIndex.push_back(Hafc->GetEdgeIndex());
		} while(++Hafc != FI->facet_begin());
		model.vecFacet.push_back(VerIndex);
		FI++;
	} while(FI!=PrimalMesh.facets_end());

	Build_WedgeEdgeMesh<HalfedgeDS> poly(model);
	WedgeEdgeMesh.delegate(poly);
}























































///////////////////////////////////////////////////////////////////////////////////////
void CWedgeEdgeBasedDeform::Test(double dLamda,int iType,int iIterNum,KW_Mesh& PrimalMesh, vector<Vertex_handle>& vecHandleNb, 
								 vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
								 vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints)
{
	//compute edge mesh 
	KW_Mesh WedgeEdgeMesh;
	BuildWedgeEdgeMesh(PrimalMesh,WedgeEdgeMesh);

	OBJHandle::UnitizeCGALPolyhedron(WedgeEdgeMesh,false,false);
	GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(WedgeEdgeMesh);

	//get roi and anchor vertices in edge mesh
	set<int> setEdgeHandleNb;
	vector<Vertex_handle> vecEdgeROIVertices,vecEdgeAnchorVertices;
	SparseMatrix VerToEdgeMatrix;
	TestGetROIandAnchorInEdgeMesh(PrimalMesh,vecHandleNb,ROIVertices,vecAnchorVertices,
		WedgeEdgeMesh,setEdgeHandleNb,vecEdgeROIVertices,vecEdgeAnchorVertices,VerToEdgeMatrix);
	
	//for (unsigned int i=0;i<vecEdgeAnchorVertices.size();i++)
	//{
	//	testPoints.push_back(vecEdgeAnchorVertices.at(i)->point());
	//}

	//apply flexible deform to edge mesh
	CDeformationAlgorithm::TestFlexibleDeform(dLamda,iType,iIterNum,WedgeEdgeMesh,setEdgeHandleNb,
		vecEdgeROIVertices,vecEdgeAnchorVertices,vecDeformCurvePoint3d);

	//output edge mesh
	OBJHandle::UnitizeCGALPolyhedron(WedgeEdgeMesh,false,false);
	std::ofstream out("1tempout.obj",ios_base::out | ios_base::trunc);
	print_polyhedron_wavefront(out,WedgeEdgeMesh);

	//convert edge mesh back to primal mesh
	CMath TAUCSSolver;
	SparseMatrix AT(VerToEdgeMatrix.NCols());
	TAUCSSolver.TAUCSFactorize(VerToEdgeMatrix,AT);
	vector<vector<double> > RightHandSide;
	vector<double> temp[3];
	for (unsigned int i=0;i<vecEdgeROIVertices.size();i++)
	{
		temp[0].push_back(vecEdgeROIVertices.at(i)->point().x());
		temp[1].push_back(vecEdgeROIVertices.at(i)->point().y());
		temp[2].push_back(vecEdgeROIVertices.at(i)->point().z());
	}
	RightHandSide.push_back(temp[0]);RightHandSide.push_back(temp[1]);RightHandSide.push_back(temp[2]);
	
	temp[0].clear();temp[1].clear();temp[2].clear();
	for (unsigned int i=0;i<vecEdgeAnchorVertices.size();i++)
	{
		temp[0].push_back(vecEdgeAnchorVertices.at(i)->point().x());
		temp[1].push_back(vecEdgeAnchorVertices.at(i)->point().y());
		temp[2].push_back(vecEdgeAnchorVertices.at(i)->point().z());
	}

	for (int i=0;i<3;i++)
	{
		RightHandSide.at(i).insert(RightHandSide.at(i).end(),temp[i].begin(),
			temp[i].end());
	}

	vector<vector<double>> Result;
	bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);
	if (bResult)
	{
		for (unsigned int i=0;i<vecHandleNb.size();i++)
		{
			vecHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
		}
		//calculated result of ROI 
		for (unsigned int i=0;i<ROIVertices.size();i++)
		{
			ROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+i),
				Result.at(1).at(vecHandleNb.size()+i),
				Result.at(2).at(vecHandleNb.size()+i));
		}
		//calculated result of anchor 
		for (unsigned int i=0;i<vecAnchorVertices.size();i++)
		{
			vecAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i),
				Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+i),
				Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+i));
		}
	}

}

void CWedgeEdgeBasedDeform::TestGetROIandAnchorInEdgeMesh(KW_Mesh& PrimalMesh, vector<Vertex_handle>& vecHandleNb, 
														  vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
														  KW_Mesh& EdgeMesh,set<int>& vecEdgeHandleNb, 
														  vector<Vertex_handle>& vecEdgeROIVertices,vector<Vertex_handle>& vecEdgeAnchorVertices,
														  SparseMatrix& VerToEdgeMatrix)
{
	//get roi vertices in edge mesh
	//get the indices first
	set<int> setEdgeROIIndex;
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		Vertex_handle CurrentVer=ROIVertices.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentVer->vertex_begin();
		do 
		{
			setEdgeROIIndex.insert(Havc->GetEdgeIndex());
			Havc++;
		} while(Havc!=CurrentVer->vertex_begin());
	}

	set<int> setEdgeAnchorIndex;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Vertex_handle CurrentVer=vecAnchorVertices.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentVer->vertex_begin();
		do 
		{
			vector<Vertex_handle>::iterator pFind=find(vecAnchorVertices.begin(),vecAnchorVertices.end(),Havc->opposite()->vertex());
			if (pFind!=vecAnchorVertices.end())
			{
				setEdgeAnchorIndex.insert(Havc->GetEdgeIndex());
			}
			Havc++;
		} while(Havc!=CurrentVer->vertex_begin());
	}
	//get the handle vertices of edge mesh which are around the handle vertex of primal mesh
	vecEdgeHandleNb.clear();
	vector<int> vecTemp;
	vecTemp.insert(vecTemp.end(),setEdgeROIIndex.begin(),setEdgeROIIndex.end());
	Vertex_handle PrimalHandleVer=vecHandleNb.front();
	Halfedge_around_vertex_circulator Havc=PrimalHandleVer->vertex_begin();
	do 
	{
		vector<int>::iterator pFind=find(vecTemp.begin(),vecTemp.end(),Havc->GetEdgeIndex());
		assert(pFind!=vecTemp.end());
		int temp=pFind-vecTemp.begin();
		vecEdgeHandleNb.insert(temp);
		Havc++;
	} while(Havc!=PrimalHandleVer->vertex_begin());


	cout<<"edge roi num: "<<setEdgeROIIndex.size()<<endl;
	cout<<"edge anchor num: "<<setEdgeAnchorIndex.size()<<endl;

	//get the pointer from the index
	Vertex_iterator VI=EdgeMesh.vertices_begin();
	int iIndex=0;
	do 
	{
		set<int>::iterator pFindROI=find(setEdgeROIIndex.begin(),setEdgeROIIndex.end(),iIndex);
		if (pFindROI!=setEdgeROIIndex.end())
		{
			vecEdgeROIVertices.push_back(VI);
		}
		set<int>::iterator pFindAnchor=find(setEdgeAnchorIndex.begin(),setEdgeAnchorIndex.end(),iIndex);
		if (pFindAnchor!=setEdgeAnchorIndex.end())
		{
			vecEdgeAnchorVertices.push_back(VI);
		}

		iIndex++;
		VI++;
	} while(VI!=EdgeMesh.vertices_end());

	SparseMatrix Temp(setEdgeROIIndex.size()+setEdgeAnchorIndex.size());
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	Temp.m=vecAllVertices.size();
	//for each primal edge, find the corresponding two vertex indices
	int iRow=0;
	for (set<int>::iterator EdgeROIIter=setEdgeROIIndex.begin();EdgeROIIter!=setEdgeROIIndex.end();EdgeROIIter++)
	{
		//get the edge which has index *EdgeROIIter
		Halfedge_iterator HI=PrimalMesh.halfedges_begin();
		for (int i=0;i<*EdgeROIIter;i++)
		{
			HI++;
			HI++;
		}
		Vertex_handle Ver0=HI->vertex();
		Vertex_handle Ver1=HI->opposite()->vertex();
		vector<Vertex_handle>::iterator pFind0=find(vecAllVertices.begin(),vecAllVertices.end(),Ver0);
		assert(pFind0!=vecAllVertices.end());
		int iVer0=pFind0-vecAllVertices.begin();
		vector<Vertex_handle>::iterator pFind1=find(vecAllVertices.begin(),vecAllVertices.end(),Ver1);
		assert(pFind1!=vecAllVertices.end());
		int iVer1=pFind1-vecAllVertices.begin();
		Temp[iRow][iVer0]=0.5;
		Temp[iRow][iVer1]=0.5;
		iRow++;
	}
	for (set<int>::iterator EdgeAnchorIter=setEdgeAnchorIndex.begin();EdgeAnchorIter!=setEdgeAnchorIndex.end();EdgeAnchorIter++)
	{
		//get the edge which has index *EdgeROIIter
		Halfedge_iterator HI=PrimalMesh.halfedges_begin();
		for (int i=0;i<*EdgeAnchorIter;i++)
		{
			HI++;
			HI++;
		}
		Vertex_handle Ver0=HI->vertex();
		Vertex_handle Ver1=HI->opposite()->vertex();
		vector<Vertex_handle>::iterator pFind0=find(vecAllVertices.begin(),vecAllVertices.end(),Ver0);
		assert(pFind0!=vecAllVertices.end());
		int iVer0=pFind0-vecAllVertices.begin();
		vector<Vertex_handle>::iterator pFind1=find(vecAllVertices.begin(),vecAllVertices.end(),Ver1);
		assert(pFind1!=vecAllVertices.end());
		int iVer1=pFind1-vecAllVertices.begin();
		Temp[iRow][iVer0]=0.5;
		Temp[iRow][iVer1]=0.5;
		iRow++;
	}

	VerToEdgeMatrix=Temp;

	////verify
	//vector<vector<double> > MatrixA,Matrixx;
	//for (unsigned int i=0;i<VerToEdgeMatrix.NRows();i++)
	//{
	//	vector<double> CurrentRow;
	//	for (unsigned int j=0;j<VerToEdgeMatrix.NCols();j++)
	//	{
	//		CurrentRow.push_back(VerToEdgeMatrix[i][j]);
	//	}
	//	MatrixA.push_back(CurrentRow);
	//}
	//for (unsigned int i=0;i<vecAllVertices.size();i++)
	//{
	//	vector<double> CurrentRow;
	//	CurrentRow.push_back(vecAllVertices.at(i)->point().x());
	//	CurrentRow.push_back(vecAllVertices.at(i)->point().y());
	//	CurrentRow.push_back(vecAllVertices.at(i)->point().z());
	//	Matrixx.push_back(CurrentRow);
	//}
	//vector<vector<double> > Result;
	//bool bTest=CMath::SparseMatrixMultiply(MatrixA,Matrixx,Result);

	//vector<Vertex_handle> vecEdgeAllVertices;
	//vecEdgeAllVertices.insert(vecEdgeAllVertices.end(),vecEdgeROIVertices.begin(),vecEdgeROIVertices.end());
	//vecEdgeAllVertices.insert(vecEdgeAllVertices.end(),vecEdgeAnchorVertices.begin(),vecEdgeAnchorVertices.end());
	//FILE* pfile=fopen("1compare.txt","w");
	//FILE* pfile2=fopen("2compare.txt","w");
	//for (unsigned int i=0;i<vecEdgeAllVertices.size();i++)
	//{
	//	fprintf(pfile,"%f,%f,%f\n",Result.at(i).at(0),Result.at(i).at(1),Result.at(i).at(2));
	//	fprintf(pfile2,"%f,%f,%f\n",vecEdgeAllVertices.at(i)->point().x(),
	//		vecEdgeAllVertices.at(i)->point().y(),vecEdgeAllVertices.at(i)->point().z());
	//}
	//fclose(pfile);
	//fclose(pfile2);

}









