#include "DualMeshDeform.h"
#include "DeformationAlgorithm.h"
#include "../OBJHandle.h"

CDualMeshDeform::CDualMeshDeform(void)
{
}

CDualMeshDeform::~CDualMeshDeform(void)
{
}

void CDualMeshDeform::ComputeDualLaplacianMatrix(int iType,KW_Mesh& Mesh,vector<Vertex_handle> vecHandleNb,
												 vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices, 
												 vector<vector<double> >& vecvecLaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	//laplacian matrix is of size iRow*iColumn
	int iRow=(int)(vecHandleNb.size()+ROIVertices.size());
	int iColumn=vecAllVertices.size();
	for (int i=0;i<iRow;i++)//for all rows
	{
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)//for each column of CurrentRow
		{
			//vertex i and vertex j are neighbors
			int iNbIndex=GeometryAlgorithm::JudgeIfNeighbors(vecAllVertices.at(i),vecAllVertices.at(j));
			if (iNbIndex)
			{
				//CurrentRow.push_back(-1/(double)vecAllVertices.at(i)->vertex_degree());
				if (iType==1)
				{
					CurrentRow.push_back(-1);
				} 
				else
				{
					double dCurrentWeight=vecAllVertices.at(i)->GetEdgeWeights().at(iNbIndex-1);
					CurrentRow.push_back(-dCurrentWeight);
				}
			}
			else if (i==j)
			{
				//CurrentRow.push_back(1);
				if (iType==1)
				{
					CurrentRow.push_back((double)vecAllVertices.at(i)->vertex_degree());
				}
				else
				{
					CurrentRow.push_back((double)vecAllVertices.at(i)->GetWeightedLaplacianSumWeight());
				}
			}
			else
			{
				CurrentRow.push_back(0);
			}
		}
		vecvecLaplacianMatrix.push_back(CurrentRow);
	}
	assert(vecvecLaplacianMatrix.size()==iRow);
}

void CDualMeshDeform::UpdateDualMeshGeometry(vector<Vertex_handle> vecPrimalHandleNb,
												   vector<Vertex_handle> PrimalROIVertices,
												   vector<Vertex_handle> vecPrimalAnchorVertices,
												   vector<Vertex_handle> vecDualHandle,
												   vector<Vertex_handle> DualROIVertices, 
												   vector<Vertex_handle> vecDualAnchorVertices,
												   vector<vector<double> > PrimalToDualMatrix)
{
	vector<Vertex_handle> vecDualAll=vecDualHandle;
	vecDualAll.insert(vecDualAll.end(),DualROIVertices.begin(),DualROIVertices.end());
	vecDualAll.insert(vecDualAll.end(),vecDualAnchorVertices.begin(),vecDualAnchorVertices.end());

	vector<Vertex_handle> vecPrimalAll=vecPrimalHandleNb;
	vecPrimalAll.insert(vecPrimalAll.end(),PrimalROIVertices.begin(),PrimalROIVertices.end());
	vecPrimalAll.insert(vecPrimalAll.end(),vecPrimalAnchorVertices.begin(),vecPrimalAnchorVertices.end());

	vector<vector<double>> PrimalMatrix;
	for (unsigned int i=0;i<vecPrimalAll.size();i++)
	{
		vector<double> CurrentRow;
		CurrentRow.push_back(vecPrimalAll.at(i)->point().x());
		CurrentRow.push_back(vecPrimalAll.at(i)->point().y());
		CurrentRow.push_back(vecPrimalAll.at(i)->point().z());
		PrimalMatrix.push_back(CurrentRow);
	}

	vector<vector<double>> Result;
	CMath::SparseMatrixMultiply(PrimalToDualMatrix,PrimalMatrix,Result);
//	GeometryAlgorithm::MultiplyMatrixAandMatrixB(PrimalToDualMatrix,PrimalMatrix,Result);

	for (unsigned int i=0;i<vecDualAll.size();i++)
	{
		vecDualAll.at(i)->point()=Point_3(Result.at(i).at(0),Result.at(i).at(1),Result.at(i).at(2));
	}

}

void CDualMeshDeform::ComputeFlexibleDualRightHandSide(double dLamda,int iType,vector<Vertex_handle>& vecDualHandleNb,
															 vector<Vertex_handle>& DualROIVertices,
															 vector<Vertex_handle>& vecDualAnchorVertices, 
															 vector<Point_3> vecPrimalDeformCurvePoint3d,
															 vector<Vertex_handle> vecPrimalAnchorVertices, 
															 vector<vector<double> >& RigidRightHandSide,
															 vector<vector<double> >& AnchorRightHandSide, 
															 vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecDualHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecDualHandleNb.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecDualHandleNb.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecDualHandleNb.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecDualHandleNb.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecDualHandleNb.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecDualHandleNb.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecDualHandleNb.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=vecDualHandleNb.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecDualHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=vecDualHandleNb.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecDualHandleNb.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}


	for (unsigned int i=0;i<DualROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=DualROIVertices.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*DualROIVertices.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*DualROIVertices.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*DualROIVertices.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*DualROIVertices.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*DualROIVertices.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*DualROIVertices.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=DualROIVertices.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=DualROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=DualROIVertices.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=DualROIVertices.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecPrimalAnchorVertices.at(i)->point().x());
		Anchor[1].push_back(vecPrimalAnchorVertices.at(i)->point().y());
		Anchor[2].push_back(vecPrimalAnchorVertices.at(i)->point().z());
	}
	for (unsigned int i=0;i<vecPrimalDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecDualHandleNb.size()+DualROIVertices.size()+vecPrimalAnchorVertices.size()+vecPrimalDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDualMeshDeform::ComputeDualRigidRightHandSide(int iType,vector<Vertex_handle>& vecDualHandleNb, 
														  vector<Vertex_handle>& DualROIVertices,
														  vector<Vertex_handle>& vecDualAnchorVertices, 
														  vector<Point_3> vecPrimalDeformCurvePoint3d,
														  vector<Vertex_handle> vecPrimalAnchorVertices, 
														  vector<vector<double> >& RigidRightHandSide,
														  vector<vector<double> >& AnchorRightHandSide, 
														  vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecDualHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecDualHandleNb.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		CurrentLaplacian[0]=CurrentLaplacian[1]=CurrentLaplacian[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecDualHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=vecDualHandleNb.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecDualHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentLaplacian[j]=CurrentLaplacian[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecDualHandleNb.at(i)->vertex_begin());
		Laplacian[0].push_back(CurrentLaplacian[0]);
		Laplacian[1].push_back(CurrentLaplacian[1]);
		Laplacian[2].push_back(CurrentLaplacian[2]);
	}
	for (unsigned int i=0;i<DualROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=DualROIVertices.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		CurrentLaplacian[0]=CurrentLaplacian[1]=CurrentLaplacian[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=DualROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=DualROIVertices.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(DualROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentLaplacian[j]=CurrentLaplacian[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=DualROIVertices.at(i)->vertex_begin());
		Laplacian[0].push_back(CurrentLaplacian[0]);
		Laplacian[1].push_back(CurrentLaplacian[1]);
		Laplacian[2].push_back(CurrentLaplacian[2]);
	}
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecPrimalAnchorVertices.at(i)->point().x());
		Anchor[1].push_back(vecPrimalAnchorVertices.at(i)->point().y());
		Anchor[2].push_back(vecPrimalAnchorVertices.at(i)->point().z());
	}
	for (unsigned int i=0;i<vecPrimalDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecDualHandleNb.size()+DualROIVertices.size()+vecPrimalAnchorVertices.size()+vecPrimalDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDualMeshDeform::ComputeNaiveDualRightHandSide(int iType,vector<Vertex_handle> vecDualHandleNb,
														  vector<Vertex_handle> DualROIVertices,
														  vector<Point_3> vecPrimalDeformCurvePoint3d,
														  vector<Vertex_handle> vecPrimalAnchorVertices,
														  vector<vector<double> >& LaplacianRightHandSide,
														  vector<vector<double> >& AnchorRightHandSide,
														  vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecDualHandleNb.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(vecDualHandleNb.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(vecDualHandleNb.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(vecDualHandleNb.at(i)->GetUniformLaplacian().z());
		}
		else
		{
			Laplacian[0].push_back(vecDualHandleNb.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(vecDualHandleNb.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(vecDualHandleNb.at(i)->GetWeightedLaplacian().z());
		}
	}
	for (unsigned int i=0;i<DualROIVertices.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(DualROIVertices.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(DualROIVertices.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(DualROIVertices.at(i)->GetUniformLaplacian().z());
		} 
		else
		{
			Laplacian[0].push_back(DualROIVertices.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(DualROIVertices.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(DualROIVertices.at(i)->GetWeightedLaplacian().z());
		}
	}
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecPrimalAnchorVertices.at(i)->point().x());
		Anchor[1].push_back(vecPrimalAnchorVertices.at(i)->point().y());
		Anchor[2].push_back(vecPrimalAnchorVertices.at(i)->point().z());
	}
	for (unsigned int i=0;i<vecPrimalDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecPrimalDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecDualHandleNb.size()+DualROIVertices.size()+vecPrimalAnchorVertices.size()+vecPrimalDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}


void CDualMeshDeform::GetDualConstraintMatrix(vector<Vertex_handle> vecDualHandle,
													vector<Vertex_handle> DualROIVertices,
													vector<Vertex_handle> vecDualAnchorVertices,
													vector<vector<double> >& AnchorConstraintMatrix, 
													vector<vector<double> >& HandleConstraintMatrix)
{
	int iColumn=vecDualHandle.size()+DualROIVertices.size()+vecDualAnchorVertices.size();
	int iHandleRow=vecDualHandle.size();
	for (int i=0;i<iHandleRow;i++)
	{
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			if (j==i)
			{
				CurrentRow.push_back(1);
			}
			else
			{
				CurrentRow.push_back(0);
			}
		}
		HandleConstraintMatrix.push_back(CurrentRow);
	}

	int iAnchorRow=vecDualAnchorVertices.size();
	for (int i=0;i<iAnchorRow;i++)
	{
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			if (j-i==iColumn-iAnchorRow)
			{
				CurrentRow.push_back(1);
			} 
			else
			{
				CurrentRow.push_back(0);
			}
		}
		AnchorConstraintMatrix.push_back(CurrentRow);
	}
}

void CDualMeshDeform::GetPrimalToDualMatrix(vector<Vertex_handle> vecPrimalHandleNb,
												  vector<Vertex_handle> PrimalROIVertices,
												  vector<Vertex_handle> vecPrimalAnchorVertices,
												  vector<Vertex_handle> vecDualHandle,
												  vector<Vertex_handle> DualROIVertices, 
												  vector<Vertex_handle> vecDualAnchorVertices,
												  vector<vector<double> >& PrimalToDualMatrix)
{
	PrimalToDualMatrix.clear();
	vector<Vertex_handle> vecDualAll=vecDualHandle;
	vecDualAll.insert(vecDualAll.end(),DualROIVertices.begin(),DualROIVertices.end());
	vecDualAll.insert(vecDualAll.end(),vecDualAnchorVertices.begin(),vecDualAnchorVertices.end());
	vector<vector<double> > LeftMatrix;
	//topology matrix from primal to dual
	int iRowSize=vecDualAll.size();
	int iColSize=vecPrimalHandleNb.size()+PrimalROIVertices.size()+vecPrimalAnchorVertices.size();
	for (int i=0;i<iRowSize;i++)
	{
		vector<double> vecCurrentRow;
		for (int j=0;j<iColSize;j++)
		{
			vecCurrentRow.push_back(0);
		}
		for (unsigned int j=0;j<vecDualAll.at(i)->GetPrimalVertexIndices().size();j++)
		{
			vecCurrentRow.at(vecDualAll.at(i)->GetPrimalVertexIndices().at(j))=1.0/3.0;
		}
		PrimalToDualMatrix.push_back(vecCurrentRow);
	}
}

void CDualMeshDeform::ConvertDualMeshToPrimal(vector<Vertex_handle>& vecPrimalHandleNb,
													vector<Vertex_handle>& PrimalROIVertices, 
													vector<Vertex_handle>& vecPrimalAnchorVertices,
													vector<Point_3> vecPrimalHandleNbNewPos, 
													vector<Vertex_handle>& vecDualHandle,
													vector<Vertex_handle>& DualROIVertices,
													vector<Vertex_handle>& vecDualAnchorVertices)
{
	vector<Vertex_handle> vecDualAll=vecDualHandle;
	vecDualAll.insert(vecDualAll.end(),DualROIVertices.begin(),DualROIVertices.end());
	vecDualAll.insert(vecDualAll.end(),vecDualAnchorVertices.begin(),vecDualAnchorVertices.end());
	vector<vector<double> > LeftMatrix;
	//topology matrix from primal to dual
	int iRowSize=vecDualAll.size();
	int iColSize=vecPrimalHandleNb.size()+PrimalROIVertices.size()+vecPrimalAnchorVertices.size();
	for (int i=0;i<iRowSize;i++)
	{
		vector<double> vecCurrentRow;
		for (int j=0;j<iColSize;j++)
		{
			vecCurrentRow.push_back(0);
		}
		for (unsigned int j=0;j<vecDualAll.at(i)->GetPrimalVertexIndices().size();j++)
		{
			vecCurrentRow.at(vecDualAll.at(i)->GetPrimalVertexIndices().at(j))=1.0/3.0;
		}
		LeftMatrix.push_back(vecCurrentRow);
	}
	//primal handle constraint matrix
	iRowSize=vecPrimalHandleNb.size();
	for (int i=0;i<iRowSize;i++)
	{
		vector<double> vecCurrentRow;
		for (int j=0;j<iColSize;j++)
		{
			if (j==i)
			{
				vecCurrentRow.push_back(20*1);
			}
			else
			{
				vecCurrentRow.push_back(0);
			}
		}
		LeftMatrix.push_back(vecCurrentRow);
	}
	//primal anchor constraint matrix
	iRowSize=vecPrimalAnchorVertices.size();
	for (int i=0;i<iRowSize;i++)
	{
		vector<double> vecCurrentRow;
		for (int j=0;j<iColSize;j++)
		{
			if (j-i==iColSize-vecPrimalAnchorVertices.size())
			{
				vecCurrentRow.push_back(1);
			}
			else
			{
				vecCurrentRow.push_back(0);
			}
		}
		LeftMatrix.push_back(vecCurrentRow);
	}

	vector<vector<double> > RightHandSide;
	vector<double> RightCol[3];
	//dual mesh result
	for (unsigned int i=0;i<vecDualAll.size();i++)
	{
		RightCol[0].push_back(vecDualAll.at(i)->point().x());
		RightCol[1].push_back(vecDualAll.at(i)->point().y());
		RightCol[2].push_back(vecDualAll.at(i)->point().z());
	}
	//handle constraint result
	for (unsigned int i=0;i<vecPrimalHandleNbNewPos.size();i++)
	{
		RightCol[0].push_back(20*vecPrimalHandleNbNewPos.at(i).x());
		RightCol[1].push_back(20*vecPrimalHandleNbNewPos.at(i).y());
		RightCol[2].push_back(20*vecPrimalHandleNbNewPos.at(i).z());
	}
	//anchor constraint result
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		RightCol[0].push_back(vecPrimalAnchorVertices.at(i)->point().x());
		RightCol[1].push_back(vecPrimalAnchorVertices.at(i)->point().y());
		RightCol[2].push_back(vecPrimalAnchorVertices.at(i)->point().z());
	}
	for (int i=0;i<3;i++)
	{
		RightHandSide.push_back(RightCol[i]);
	}

	vector<vector<double> > Result;
	bool bResult=CMath::ComputeLSE(LeftMatrix,RightHandSide,Result);
	if (bResult)
	{
		for (unsigned int i=0;i<vecPrimalHandleNb.size();i++)
		{
			vecPrimalHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
		}
		for (unsigned int i=0;i<PrimalROIVertices.size();i++)
		{
			PrimalROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+i),
				Result.at(1).at(vecPrimalHandleNb.size()+i),
				Result.at(2).at(vecPrimalHandleNb.size()+i));
		}
		for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
		{
			vecPrimalAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
				Result.at(1).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
				Result.at(2).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i));
		}
	}
}

void CDualMeshDeform::ComputeDualHandleNewPos(vector<Point_3> vecHandleNbNewPos,
													vector<Vertex_handle>& vecDualHandle,
													vector<Point_3>& vecNewDualHandlePos)
{
	for (unsigned int i=0;i<vecDualHandle.size();i++)
	{
		Point_3 NewPos;
		vector<Point_3> vecPrimalVertex;
		for (unsigned int j=0;j<vecDualHandle.at(i)->GetPrimalVertexIndices().size();j++)
		{
			vecPrimalVertex.push_back(vecHandleNbNewPos.at(vecDualHandle.at(i)->GetPrimalVertexIndices().at(j)));
		}
		NewPos=CGAL::centroid(vecPrimalVertex.begin(),vecPrimalVertex.end());
		vecNewDualHandlePos.push_back(NewPos);
		//test
		//vecDualHandle.at(i)->point()=NewPos;
	}
}


void CDualMeshDeform::BuildDualMesh(KW_Mesh& PrimalMesh,vector<Vertex_handle>& vecPrimalHandleNb, 
										  vector<Vertex_handle>& PrimalROIVertices,
										  vector<Vertex_handle>& vecPrimalAnchorVertices, 
										  KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
										  vector<Vertex_handle>& DualROIVertices,
										  vector<Vertex_handle>& vecDualAnchorVertices)
{
	DualMesh.clear();
	Build_DualMesh<HalfedgeDS> BuildDualMesh(PrimalMesh);
	DualMesh.delegate(BuildDualMesh);
	OBJHandle::UnitizeCGALPolyhedron(DualMesh,false,false);

	vector<Vertex_handle> vecAllVertex;
	vecAllVertex.insert(vecAllVertex.end(),vecPrimalHandleNb.begin(),vecPrimalHandleNb.end());
	vecAllVertex.insert(vecAllVertex.end(),PrimalROIVertices.begin(),PrimalROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecPrimalAnchorVertices.begin(),vecPrimalAnchorVertices.end());

	//find all related primal facet
	vector<Facet_handle> vecAllPrimalFacet;
	for (unsigned int i=0;i<vecAllVertex.size();i++)
	{
		Vertex_handle CurrentVertex=vecAllVertex.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentVertex->vertex_begin();
		do 
		{
			Facet_handle CurrentFacet=Havc->facet();
			vector<Facet_handle>::iterator pFind=find(vecAllPrimalFacet.begin(),vecAllPrimalFacet.end(),CurrentFacet);
			if (pFind==vecAllPrimalFacet.end())
			{
				vecAllPrimalFacet.push_back(CurrentFacet);
			}
			Havc++;
		} while(Havc!=CurrentVertex->vertex_begin());
	}
	//find one more layer anchor vertices
	vector<Vertex_handle> vecOneMoreAnchor;
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		Vertex_handle CurrentAnchor=vecPrimalAnchorVertices.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentAnchor->vertex_begin();
		do 
		{
			Vertex_handle AnchorNeighbor=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator pFind=find(vecOneMoreAnchor.begin(),vecOneMoreAnchor.end(),AnchorNeighbor);
			if (pFind==vecOneMoreAnchor.end())
			{
				vector<Vertex_handle>::iterator pFind2=find(vecAllVertex.begin(),vecAllVertex.end(),AnchorNeighbor);
				if (pFind2==vecAllVertex.end())
				{
					vecOneMoreAnchor.push_back(AnchorNeighbor);
				}
			}
			Havc++;
		} while(Havc!=CurrentAnchor->vertex_begin());
	}
	vecAllVertex.insert(vecAllVertex.end(),vecOneMoreAnchor.begin(),vecOneMoreAnchor.end());
	//insert one more layer of anchor
	vecPrimalAnchorVertices.insert(vecPrimalAnchorVertices.end(),vecOneMoreAnchor.begin(),vecOneMoreAnchor.end());

	//get dual handle,roi and anchor
	for (Vertex_iterator i=DualMesh.vertices_begin();i!=DualMesh.vertices_end();i++)
	{
		//get primal facet 
		Facet_iterator PrimalFacet=PrimalMesh.facets_begin();
		for (int j=0;j<i->GetFacetOnPrimalMeshIndex();j++)
		{
			PrimalFacet++;
		}
		//check if related to deformation
		vector<Facet_handle>::iterator pFind=find(vecAllPrimalFacet.begin(),vecAllPrimalFacet.end(),PrimalFacet);
		if (pFind==vecAllPrimalFacet.end())
		{
			continue;
		}

		//get info from primal facet vertex

		//calculate the num of handle,roi and anchor Primal vertices
		vector<int> vecPrimalVertexIndex;
		int iPrimalHandle,iPrimalROI,iPrimalAnchor;
		iPrimalHandle=iPrimalROI=iPrimalAnchor=0;

		Halfedge_around_facet_circulator Hafc=PrimalFacet->facet_begin();
		do 
		{
			Vertex_handle CurrentPrimalVertex=Hafc->vertex();
			vector<Vertex_handle>::iterator pFind=find(vecAllVertex.begin(),vecAllVertex.end(),CurrentPrimalVertex);
			if (pFind!=vecAllVertex.end())
			{
				int iIndex=pFind-vecAllVertex.begin();
				vecPrimalVertexIndex.push_back(iIndex);
				if (iIndex<(int)vecPrimalHandleNb.size())
				{
					iPrimalHandle++;
				} 
				else if (iIndex>=(int)vecPrimalHandleNb.size()&&iIndex<(int)(vecPrimalHandleNb.size()+PrimalROIVertices.size()))
				{
					iPrimalROI++;
				}
				else
				{
					iPrimalAnchor++;
				}
			}
			Hafc++;
		} while(Hafc!=PrimalFacet->facet_begin());

		i->SetPrimalVertexIndices(vecPrimalVertexIndex);
		//classify related dual mesh vertices
		//if the primal facet contains a ROI vertex,the dual vertex must be ROI
		if (iPrimalROI!=0)
		{
			DualROIVertices.push_back(i);
		}
		//only if all vertices of the primal facet are handle,the dual vertex is handle
		else if(iPrimalHandle==PrimalFacet->facet_degree())
		{
			vecDualHandle.push_back(i);
		}
		//only if all vertices of the primal facet are anchor,the dual vertex is anchor
		else if(iPrimalAnchor==PrimalFacet->facet_degree())
		{
			vecDualAnchorVertices.push_back(i);
		}
		//in case no ROI exist(some vertices of the primal facet are handles and others are anchors),be careful!
		else
		{
			DualROIVertices.push_back(i);
		}
	}

	cout<<"Dual Mesh size: "<<DualMesh.size_of_vertices()<<endl;
	cout<<"Primal:  Handle: "<<vecPrimalHandleNb.size()<<",ROI: "<<PrimalROIVertices.size()<<",Anchor: "<<vecPrimalAnchorVertices.size()<<endl;
	cout<<"Dual:    Handle: "<<vecDualHandle.size()<<",ROI: "<<DualROIVertices.size()<<",Anchor: "<<vecDualAnchorVertices.size()<<endl;
}

void CDualMeshDeform::GetDualMeshInfo(KW_Mesh& PrimalMesh,vector<Vertex_handle>& vecHandleNb, 
											vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
											vector<DualMeshVertexStruct>& DualMesh,int* pDualMeshVertexNum)
{
	vector<Vertex_handle> vecAllVertex;
	vecAllVertex.insert(vecAllVertex.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	//find all related primal facet
	vector<Facet_handle> vecAllPrimalFacet;
	for (unsigned int i=0;i<vecAllVertex.size();i++)
	{
		Vertex_handle CurrentVertex=vecAllVertex.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentVertex->vertex_begin();
		do 
		{
			Facet_handle CurrentFacet=Havc->facet();
			vector<Facet_handle>::iterator pFind=find(vecAllPrimalFacet.begin(),vecAllPrimalFacet.end(),CurrentFacet);
			if (pFind==vecAllPrimalFacet.end())
			{
				vecAllPrimalFacet.push_back(CurrentFacet);
			}
			Havc++;
		} while(Havc!=CurrentVertex->vertex_begin());
	}

	//find one more layer anchor vertices
	vector<Vertex_handle> vecOneMoreAnchor;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Vertex_handle CurrentAnchor=vecAnchorVertices.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentAnchor->vertex_begin();
		do 
		{
			Vertex_handle AnchorNeighbor=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator pFind=find(vecOneMoreAnchor.begin(),vecOneMoreAnchor.end(),AnchorNeighbor);
			if (pFind==vecOneMoreAnchor.end())
			{
				vector<Vertex_handle>::iterator pFind2=find(vecAllVertex.begin(),vecAllVertex.end(),AnchorNeighbor);
				if (pFind2==vecAllVertex.end())
				{
					vecOneMoreAnchor.push_back(AnchorNeighbor);
				}
			}
			Havc++;
		} while(Havc!=CurrentAnchor->vertex_begin());
	}
	vecAllVertex.insert(vecAllVertex.end(),vecOneMoreAnchor.begin(),vecOneMoreAnchor.end());
	//insert one more layer of anchor
	vecAnchorVertices.insert(vecAnchorVertices.end(),vecOneMoreAnchor.begin(),vecOneMoreAnchor.end());

	//iterate primal facets to construct dual mesh vertex
	//note that vecAllPrimalFacet has the same order as DualMesh
	for (unsigned int i=0;i<vecAllPrimalFacet.size();i++)
	{
		Facet_handle CurrentPrimalFacet=vecAllPrimalFacet.at(i);
		DualMeshVertexStruct CurrentDualMeshVertex;
		CurrentDualMeshVertex.PrimalFacet=CurrentPrimalFacet;

		vector<Vertex_handle> tempVertex;
		vector<Point_3> tempPoint;
		Halfedge_around_facet_circulator Hafc = CurrentPrimalFacet->facet_begin();
		do 
		{
			tempVertex.push_back(Hafc->vertex());
			tempPoint.push_back(Hafc->vertex()->point());

			//construct dual mesh topology
			Facet_handle NeighborFacet=Hafc->opposite()->facet();
			vector<Facet_handle>::iterator pFind=find(vecAllPrimalFacet.begin(),vecAllPrimalFacet.end(),NeighborFacet);
			if (pFind!=vecAllPrimalFacet.end())
			{
				int iNeighborIndex=pFind-vecAllPrimalFacet.begin();
				CurrentDualMeshVertex.vecDualNeighborIndex.push_back(iNeighborIndex);
			}
			Hafc++;
		} while(Hafc!=CurrentPrimalFacet->facet_begin());

		//attention,whether to use centroid or barycenter is uncertian yet!!
		CurrentDualMeshVertex.PointPos=CGAL::centroid(tempPoint.begin(),tempPoint.end());

		//calculate the num of handle,roi and anchor Primal vertices
		int iPrimalHandle,iPrimalROI,iPrimalAnchor;
		iPrimalHandle=iPrimalROI=iPrimalAnchor=0;

		for (unsigned int j=0;j<tempVertex.size();j++)
		{
			vector<Vertex_handle>::iterator pFind=find(vecAllVertex.begin(),vecAllVertex.end(),tempVertex.at(j));	
			assert(pFind!=vecAllVertex.end());
			int iIndex=pFind-vecAllVertex.begin();
			CurrentDualMeshVertex.vecPrimalVertexIndex.push_back(iIndex);

			if (iIndex<(int)vecHandleNb.size())
			{
				iPrimalHandle++;
			} 
			else if (iIndex>=(int)vecHandleNb.size()&&iIndex<(int)(vecHandleNb.size()+ROIVertices.size()))
			{
				iPrimalROI++;
			}
			else
			{
				iPrimalAnchor++;
			}
		}

		//set type
		//if the primal facet contains a ROI vertex,the dual vertex must be ROI
		if (iPrimalROI!=0)
		{
			CurrentDualMeshVertex.iType=2;
			pDualMeshVertexNum[1]++;
		}
		//only if all vertices of the primal facet are handle,the dual vertex is handle
		else if(iPrimalHandle==vecAllPrimalFacet.at(i)->facet_degree())
		{
			CurrentDualMeshVertex.iType=1;
			pDualMeshVertexNum[0]++;
		}
		//only if all vertices of the primal facet are anchor,the dual vertex is anchor
		else if(iPrimalAnchor==vecAllPrimalFacet.at(i)->facet_degree())
		{
			CurrentDualMeshVertex.iType=3;
			pDualMeshVertexNum[2]++;
		}
		//in case no ROI exist(some vertices of the primal facet are handles and others are anchors),be careful!
		else
		{
			CurrentDualMeshVertex.iType=2;
			pDualMeshVertexNum[1]++;
		}

		DualMesh.push_back(CurrentDualMeshVertex);
	}

	//compute weight for each edge of dual mesh
	for (unsigned int i=0;i<DualMesh.size();i++)
	{

	}

	//print Dualmesh info
	//for (unsigned int i=0;i<DualMesh.size();i++)
	//{
	//DBWindowWrite("Pos: x: %f,y: %f,z: %f\n",DualMesh.at(i).PointPos.x(),DualMesh.at(i).PointPos.y(),DualMesh.at(i).PointPos.z());
	//for (unsigned int j=0;j<DualMesh.at(i).vecPrimalVertexIndex.size();j++)
	//{
	//	DBWindowWrite("Index on Primal Mesh: %d\n",DualMesh.at(i).vecPrimalVertexIndex.at(j));
	//}
	//for (unsigned int j=0;j<DualMesh.at(i).vecDualNeighborIndex.size();j++)
	//{
	//	DBWindowWrite("Index on Dual Mesh: %d\n",DualMesh.at(i).vecDualNeighborIndex.at(j));
	//}
	//DBWindowWrite("\n");
	//}
	cout<<"Dual Mesh size: "<<DualMesh.size()<<endl;
	cout<<"Handle: "<<pDualMeshVertexNum[0]<<",ROI: "<<pDualMeshVertexNum[1]<<",Anchor: "<<pDualMeshVertexNum[2]<<endl;

}

void CDualMeshDeform::ComputeHandleNbNewPos(KW_Mesh& Mesh,vector<HandlePointStruct>& vecHandlePoint, 
												  vector<Vertex_handle>& vecHandleNb,vector<Point_3>& vecDeformCurvePoint3d,
												  vector<Point_3>& vecHandleNbNewPos)
{
	//compute left coefficient matrix
	vector<vector<double> > LeftMatrix;
	for (unsigned int i=0;i<vecHandlePoint.size();i++)
	{
		vector<double> CurrentRow;
		for (unsigned int j=0;j<vecHandleNb.size();j++)
		{
			CurrentRow.push_back(0);
		}
		for (unsigned int j=0;j<vecHandlePoint.at(i).vecVertexIndex.size();j++)
		{
			CurrentRow.at(vecHandlePoint.at(i).vecVertexIndex.at(j))=vecHandlePoint.at(i).vecPara.at(j);
		}
		LeftMatrix.push_back(CurrentRow);
	}

	vector<vector<double> > RightHandSide;
	vector<double> RightHandeCol[3];
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		RightHandeCol[0].push_back(vecDeformCurvePoint3d.at(i).x());
		RightHandeCol[1].push_back(vecDeformCurvePoint3d.at(i).y());
		RightHandeCol[2].push_back(vecDeformCurvePoint3d.at(i).z());
	}
	for (int i=0;i<3;i++)
	{
		RightHandSide.push_back(RightHandeCol[i]);
	}

	vector<vector<double> > Result;
	bool bResult=CMath::ComputeLSE(LeftMatrix,RightHandSide,Result);
	if (bResult)
	{
		for (unsigned int i=0;i<vecHandleNb.size();i++)
		{
			vecHandleNbNewPos.push_back(Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i)));
			//test
			//vecHandleNb.at(i)->point()=vecHandleNbNewPos.at(i);
			//vecHandleNbNewPos.at(i)=vecHandleNb.at(i)->point();
		}
	}

}

void CDualMeshDeform::FlexibleDualMeshDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
												   vector<HandlePointStruct>& vecPrimalHandlePoint,
												   vector<Vertex_handle>& vecPrimalHandleNb, 
												   vector<Vertex_handle>& PrimalROIVertices,
												   vector<Vertex_handle>& vecPrimalAnchorVertices, 
												   vector<Point_3>& vecPrimalDeformCurvePoint3d, 
												   KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle, 
												   vector<Vertex_handle>& DualROIVertices,
												   vector<Vertex_handle>& vecDualAnchorVertices)
{
	if (DualMesh.empty())
	{
		return;
	}
	//get the matrix from primal to dual
	vector<vector<double> > PrimalToDualMatrix;
	GetPrimalToDualMatrix(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);
	//compute left Laplacian matrix
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeDualLaplacianMatrix(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
		vecvecLaplacianMatrix);
	//Matrix laplacian*Matrix primalToDual
	vector<vector<double> > LeftTopoMatrix;
	//	bool bTest=GeometryAlgorithm::MultiplyMatrixAandMatrixB(vecvecLaplacianMatrix,PrimalToDualMatrix,LeftTopoMatrix);
	bool bTest=CMath::SparseMatrixMultiply(vecvecLaplacianMatrix,PrimalToDualMatrix,LeftTopoMatrix);

	//compute left Constraint matrx
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecPrimalHandlePoint,vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	//combine left topo and Constraiant matrix to get left matrix
	vector<vector<double> > LeftHandMatrixA;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),LeftTopoMatrix.begin(),LeftTopoMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//begin iterations
	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
//			CDeformationAlgorithm::BackUpEdgeVectorsForRigidDeform(DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			ComputeNaiveDualRightHandSide(iType,vecDualHandle,DualROIVertices,vecPrimalDeformCurvePoint3d,
				vecPrimalAnchorVertices,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleDualRightHandSide(dLamda,iType,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
				vecPrimalDeformCurvePoint3d,vecPrimalAnchorVertices,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		
		vector<vector<double> > Result;
		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);

		if (bResult)
		{
			//calculated result of handle 
			for (unsigned int i=0;i<vecPrimalHandleNb.size();i++)
			{
				vecPrimalHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<PrimalROIVertices.size();i++)
			{
				PrimalROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
			{
				vecPrimalAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i));
			}
		}

		//FILE* pfile=fopen("1right.txt","w");
		//for (unsigned int iCol=0;iCol<Result.front().size();iCol++)
		//{
		//	for (unsigned int iRow=0;iRow<3;iRow++)
		//	{
		//		fprintf(pfile,"%f,",Result.at(iRow).at(iCol));
		//	}
		//	fprintf(pfile,"\n");
		//}
		//fclose(pfile);

		break;
		//std::ofstream out("1.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		UpdateDualMeshGeometry(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
			vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			clock_t RotationBegin=clock();   
			CDeformationAlgorithm::ComputeRotationForRigidDeform(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			clock_t RotationEnd=clock();   
			cout<<"time for computing rotation: "<<float(RotationEnd-RotationBegin)<<endl;
			CDeformationAlgorithm::ComputeScaleFactor(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
		}
	}
}

//dual mesh rigid deformation
void CDualMeshDeform::DualMeshRigidDeform(int iType,int iIterNum,KW_Mesh& Mesh, 
													vector<HandlePointStruct>& vecPrimalHandlePoint,
													vector<Vertex_handle>& vecPrimalHandleNb,
													vector<Vertex_handle>& PrimalROIVertices,
													vector<Vertex_handle>& vecPrimalAnchorVertices,
													vector<Point_3>& vecPrimalDeformCurvePoint3d, 
													KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
													vector<Vertex_handle>& DualROIVertices,
													vector<Vertex_handle>& vecDualAnchorVertices)
{
	//test
	//compute the new Primal handleNb vertex positions from deformcurve
	//vector<Point_3> vecPrimalHandleNbNewPos;
	//ComputeHandleNbNewPos(Mesh,vecPrimalHandlePoint,vecPrimalHandleNb,vecPrimalDeformCurvePoint3d,vecPrimalHandleNbNewPos);
	
	//get the matrix from primal to dual
	vector<vector<double> > PrimalToDualMatrix;
	GetPrimalToDualMatrix(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);
	//compute left Laplacian matrix
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeDualLaplacianMatrix(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
		vecvecLaplacianMatrix);
	//Matrix laplacian*Matrix primalToDual
	vector<vector<double> > LeftTopoMatrix;
	bool bTest=GeometryAlgorithm::MultiplyMatrixAandMatrixB(vecvecLaplacianMatrix,PrimalToDualMatrix,LeftTopoMatrix);

	//compute left Constraint matrx
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecPrimalHandlePoint,vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	//combine left topo and Constraiant matrix to get left matrix
	vector<vector<double> > LeftHandMatrixA;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),LeftTopoMatrix.begin(),LeftTopoMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());


	//begin iterations
	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			CDeformationAlgorithm::BackUpEdgeVectorsForRigidDeform(DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			ComputeNaiveDualRightHandSide(iType,vecDualHandle,DualROIVertices,vecPrimalDeformCurvePoint3d,
				vecPrimalAnchorVertices,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeDualRigidRightHandSide(iType,vecDualHandle,DualROIVertices,vecDualAnchorVertices,vecPrimalDeformCurvePoint3d,
				vecPrimalAnchorVertices,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
		if (bResult)
		{
			//calculated result of handle 
			for (unsigned int i=0;i<vecPrimalHandleNb.size();i++)
			{
				vecPrimalHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<PrimalROIVertices.size();i++)
			{
				PrimalROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
			{
				vecPrimalAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i));
			}
		}

		//std::ofstream out("1.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		UpdateDualMeshGeometry(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
			vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			clock_t RotationBegin=clock();   
			CDeformationAlgorithm::ComputeRotationForRigidDeform(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			clock_t RotationEnd=clock();   
			cout<<"time for computing rotation: "<<float(RotationEnd-RotationBegin)<<endl;
		}
	}
}

void CDualMeshDeform::DualMeshRigidDeformTest(int iType,int iIterNum,KW_Mesh& Mesh, 
											vector<HandlePointStruct>& vecPrimalHandlePoint,
											vector<Vertex_handle>& vecPrimalHandleNb,
											vector<Vertex_handle>& PrimalROIVertices,
											vector<Vertex_handle>& vecPrimalAnchorVertices,
											vector<Point_3>& vecPrimalDeformCurvePoint3d, 
											KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
											vector<Vertex_handle>& DualROIVertices,
											vector<Vertex_handle>& vecDualAnchorVertices)
{

	//compute the new Primal handleNb vertex positions from deformcurve
	vector<Point_3> vecPrimalHandleNbNewPos;
	ComputeHandleNbNewPos(Mesh,vecPrimalHandlePoint,vecPrimalHandleNb,vecPrimalDeformCurvePoint3d,vecPrimalHandleNbNewPos);

	//compute the new Dual handle vertex positions from the new Primal handleNb
	vector<Point_3> vecNewDualHandlePos;
	ComputeDualHandleNewPos(vecPrimalHandleNbNewPos,vecDualHandle,vecNewDualHandlePos);

	//apply rigid deformation to Dual mesh
	//compute left Laplacian matrix
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeDualLaplacianMatrix(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
		vecvecLaplacianMatrix);
	//compute left Constraint matrx
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetDualConstraintMatrix(vecDualHandle,DualROIVertices,vecDualAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	//combine left Laplacian and Constraiant matrix to get left matrix
	vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
	//begin iterations
	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			CDeformationAlgorithm::BackUpEdgeVectorsForRigidDeform(DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			CDeformationAlgorithm::ComputeNaiveLaplacianRightHandSide(iType,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
				vecNewDualHandlePos,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			CDeformationAlgorithm::ComputeRigidRightHandSide(iType,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
				vecNewDualHandlePos,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
		if (bResult)
		{
			//calculated result of handle 
			for (unsigned int i=0;i<vecDualHandle.size();i++)
			{
				vecDualHandle.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<DualROIVertices.size();i++)
			{
				DualROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecDualHandle.size()+i),
					Result.at(1).at(vecDualHandle.size()+i),
					Result.at(2).at(vecDualHandle.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecDualAnchorVertices.size();i++)
			{
				vecDualAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecDualHandle.size()+DualROIVertices.size()+i),
					Result.at(1).at(vecDualHandle.size()+DualROIVertices.size()+i),
					Result.at(2).at(vecDualHandle.size()+DualROIVertices.size()+i));
			}
		}

		//std::ofstream out("1.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			clock_t RotationBegin=clock();   
			CDeformationAlgorithm::ComputeRotationForRigidDeform(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices);
			clock_t RotationEnd=clock();   
			cout<<"time for computing rotation: "<<float(RotationEnd-RotationBegin)<<endl;
		}
	}



	//reconstruct new Primal mesh from Dual mesh
	ConvertDualMeshToPrimal(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,vecPrimalHandleNbNewPos,
		vecDualHandle,DualROIVertices,vecDualAnchorVertices);
}


void CDualMeshDeform::IterativeDualLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh,
						 vector<HandlePointStruct>& vecPrimalHandlePoint,vector<Vertex_handle>& vecPrimalHandleNb, 
						 vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
						 vector<Point_3>& vecPrimalDeformCurvePoint3d,
						 KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle, 
						 vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices)
{
	if (DualMesh.empty())
	{
		return;
	}
	//get the matrix from primal to dual
	vector<vector<double> > PrimalToDualMatrix;
	GetPrimalToDualMatrix(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);
	//compute left Laplacian matrix
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeDualLaplacianMatrix(iType,DualMesh,vecDualHandle,DualROIVertices,vecDualAnchorVertices,
		vecvecLaplacianMatrix);
	//Matrix laplacian*Matrix primalToDual
	vector<vector<double> > LeftTopoMatrix;
	//	bool bTest=GeometryAlgorithm::MultiplyMatrixAandMatrixB(vecvecLaplacianMatrix,PrimalToDualMatrix,LeftTopoMatrix);
	bool bTest=CMath::SparseMatrixMultiply(vecvecLaplacianMatrix,PrimalToDualMatrix,LeftTopoMatrix);

	//compute left Constraint matrx
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecPrimalHandlePoint,vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	//combine left topo and Constraiant matrix to get left matrix
	vector<vector<double> > LeftHandMatrixA;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),LeftTopoMatrix.begin(),LeftTopoMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the primal anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> PrimalAnchorPosConstraints;
	for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
	{
		PrimalAnchorPosConstraints.push_back(vecPrimalAnchorVertices.at(i)->point());
	}

	//begin iterations
	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		clock_t IterationBegin=clock();   

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			ComputeNaiveDualRightHandSide(iType,vecDualHandle,DualROIVertices,vecPrimalDeformCurvePoint3d,
				vecPrimalAnchorVertices,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			IterativeUpdateDualLaplacianRightHandSide(iType,vecDualHandle,DualROIVertices,PrimalAnchorPosConstraints,vecPrimalDeformCurvePoint3d,
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
		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
		if (bResult)
		{
			//calculated result of handle 
			for (unsigned int i=0;i<vecPrimalHandleNb.size();i++)
			{
				vecPrimalHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<PrimalROIVertices.size();i++)
			{
				PrimalROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecPrimalAnchorVertices.size();i++)
			{
				vecPrimalAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(1).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i),
					Result.at(2).at(vecPrimalHandleNb.size()+PrimalROIVertices.size()+i));
			}
		}

		clock_t LSE=clock();   
		cout<<"time for computing LSE: "<<float(LSE-RHS)<<endl;

		UpdateDualMeshGeometry(vecPrimalHandleNb,PrimalROIVertices,vecPrimalAnchorVertices,
			vecDualHandle,DualROIVertices,vecDualAnchorVertices,PrimalToDualMatrix);

		clock_t IterationEnd=clock();   
		cout<<"time for updating dual mesh: "<<float(IterationEnd-LSE)<<endl;
	}
}

void CDualMeshDeform::IterativeUpdateDualLaplacianRightHandSide(int iType,vector<Vertex_handle> vecDualHandleNb,
																vector<Vertex_handle> DualROIVertices,vector<Point_3> PrimalAnchorConstraints,
																vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide,
																vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	vector<Vertex_handle> vecAll=vecDualHandleNb;
	vecAll.insert(vecAll.end(),DualROIVertices.begin(),DualROIVertices.end());
	for (unsigned int i=0;i<vecAll.size();i++)
	{
		if (iType==1)
		{
			//compute new laplacian and new average normal of facets
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			vector<Point_3> NbVertex;
			Halfedge_around_vertex_circulator Havc=vecAll.at(i)->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();

				NbVertex.push_back(Havc->opposite()->vertex()->point());
				Havc++;
			} while(Havc!=vecAll.at(i)->vertex_begin());

			double dNeighborNum=vecAll.at(i)->vertex_degree();
			dLaplacian[0]=dNeighborNum*(vecAll.at(i)->point().x())-dLaplacian[0]; 
			dLaplacian[1]=dNeighborNum*(vecAll.at(i)->point().y())-dLaplacian[1]; 
			dLaplacian[2]=dNeighborNum*(vecAll.at(i)->point().z())-dLaplacian[2];
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);
			Vector_3 NewLaplacianNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);

			Triangle_3 NewTri(NbVertex.at(0),NbVertex.at(1),NbVertex.at(2));
			double dNewSumArea=std::sqrt(NewTri.squared_area());
	
			double dOldMagnitude=std::sqrt(vecAll.at(i)->GetUniformLaplacian()*vecAll.at(i)->GetUniformLaplacian());

			//scale old laplacian magnitude
			dOldMagnitude=dOldMagnitude*sqrt(dNewSumArea/vecAll.at(i)->GetSumArea());

			//update laplacian with new direction and new magnitude
			dLaplacian[0]=NewLaplacianNormal.x()*dOldMagnitude;Laplacian[0].push_back(dLaplacian[0]);
			dLaplacian[1]=NewLaplacianNormal.y()*dOldMagnitude;Laplacian[1].push_back(dLaplacian[1]);
			dLaplacian[2]=NewLaplacianNormal.z()*dOldMagnitude;Laplacian[2].push_back(dLaplacian[2]);
		} 
		else if (iType==3)
		{
			//compute new laplacian
			double dLaplacian[3];
			vector<double> EdgeWeights;
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			vector<Point_3> NbVertex;
			int iTemp=0;
			Halfedge_around_vertex_circulator Havc=vecAll.at(i)->vertex_begin();
			do 
			{
				double dCurrentWeight=vecAll.at(i)->GetEdgeWeights().at(iTemp);
				iTemp++;
				dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Havc->opposite()->vertex()->point().z();

				NbVertex.push_back(Havc->opposite()->vertex()->point());
				Havc++;
			} while(Havc!=vecAll.at(i)->vertex_begin());

			dLaplacian[0]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().x())-dLaplacian[0]; 
			dLaplacian[1]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().y())-dLaplacian[1]; 
			dLaplacian[2]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().z())-dLaplacian[2]; 
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);
			Vector_3 NewLaplacianNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);

			Triangle_3 NewTri(NbVertex.at(0),NbVertex.at(1),NbVertex.at(2));
			double dNewSumArea=std::sqrt(NewTri.squared_area());

			double dOldMagnitude=std::sqrt(vecAll.at(i)->GetWeightedLaplacian()*vecAll.at(i)->GetWeightedLaplacian());

			//scale old laplacian magnitude
			dOldMagnitude=dOldMagnitude*sqrt(dNewSumArea/vecAll.at(i)->GetSumArea());

			//update laplacian with new direction and new magnitude
			dLaplacian[0]=NewLaplacianNormal.x()*dOldMagnitude;Laplacian[0].push_back(dLaplacian[0]);
			dLaplacian[1]=NewLaplacianNormal.y()*dOldMagnitude;Laplacian[1].push_back(dLaplacian[1]);
			dLaplacian[2]=NewLaplacianNormal.z()*dOldMagnitude;Laplacian[2].push_back(dLaplacian[2]);
		}
	}

	for (unsigned int i=0;i<PrimalAnchorConstraints.size();i++)
	{
		Anchor[0].push_back(PrimalAnchorConstraints.at(i).x());
		Anchor[1].push_back(PrimalAnchorConstraints.at(i).y());
		Anchor[2].push_back(PrimalAnchorConstraints.at(i).z());
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecDualHandleNb.size()+DualROIVertices.size()+PrimalAnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////

void CDualMeshDeform::GetFacetInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices, 
								   vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle>& ROIFacets)
{
	//get facets around handle vertices
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<Facet_handle>::iterator pFind=find(ROIFacets.begin(),ROIFacets.end(),Havc->facet());
			if (pFind==ROIFacets.end())
			{
				ROIFacets.push_back(Havc->facet());
			}
			Havc++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
	}

	//get facets around roi vertices
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<Facet_handle>::iterator pFind=find(ROIFacets.begin(),ROIFacets.end(),Havc->facet());
			if (pFind==ROIFacets.end())
			{
				ROIFacets.push_back(Havc->facet());
			}
			Havc++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
	}
}

void CDualMeshDeform::GetFacetTopo(vector<Facet_handle> ROIFacets,vector<vector<Facet_handle>>& NeighborFacets)
{
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		Halfedge_around_facet_circulator Hafc = ROIFacets.at(i)->facet_begin();
		vector<Facet_handle> CurrentFacets;
		do 
		{
			CurrentFacets.push_back(Hafc->opposite()->facet());
		} while(++Hafc != ROIFacets.at(i)->facet_begin());
		NeighborFacets.push_back(CurrentFacets);
	}
}

void CDualMeshDeform::ComputeFacetLaplacian(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets)
{
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		Point_3 CurrentPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));
		vector<Point_3> NbPoints;
		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			Point_3 NbPoint=GeometryAlgorithm::GetFacetCenter(NeighborFacets.at(i).at(j));
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
			ROIFacets.at(i)->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		}
		else if (iType==3)
		{
		}
	}

}

void CDualMeshDeform::GetHalfMoreLayerAnchor(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
											 vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle>& ROIFacets,
											 vector<vector<Facet_handle>> NeighborFacets, vector<Vertex_handle>& OneMoreLayerAnchor)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	
	for (unsigned int i=0;i<NeighborFacets.size();i++)
	{
		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			vector<Facet_handle>::iterator pFind=find(ROIFacets.begin(),ROIFacets.end(),NeighborFacets.at(i).at(j));
			if (pFind==ROIFacets.end())
			{
				Halfedge_around_facet_circulator Hafc=NeighborFacets.at(i).at(j)->facet_begin();
				do 
				{
					vector<Vertex_handle>::iterator pFind2=find(vecAllVertex.begin(),vecAllVertex.end(),Hafc->vertex());
					vector<Vertex_handle>::iterator pFind3=find(OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end(),Hafc->vertex());
					if (pFind2==vecAllVertex.end()&&pFind3==OneMoreLayerAnchor.end())
					{
						OneMoreLayerAnchor.push_back(Hafc->vertex());
					}
				} while(++Hafc != NeighborFacets.at(i).at(j)->facet_begin());
			}
		}
	}

	//for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	//{
	//	Vertex_handle CurrentAnchor=vecAnchorVertices.at(i);
	//	Halfedge_around_vertex_circulator Havc=CurrentAnchor->vertex_begin();
	//	do 
	//	{
	//		Vertex_handle AnchorNeighbor=Havc->opposite()->vertex();
	//		vector<Vertex_handle>::iterator pFind=find(OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end(),AnchorNeighbor);
	//		if (pFind==OneMoreLayerAnchor.end())
	//		{
	//			vector<Vertex_handle>::iterator pFind2=find(vecAllVertex.begin(),vecAllVertex.end(),AnchorNeighbor);
	//			if (pFind2==vecAllVertex.end())
	//			{
	//				OneMoreLayerAnchor.push_back(AnchorNeighbor);
	//			}
	//		}
	//		Havc++;
	//	} while(Havc!=CurrentAnchor->vertex_begin());
	//}
}

void CDualMeshDeform::GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
										 vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle> ROIFacets,
										 vector<vector<Facet_handle>> NeighborFacets,SparseMatrix& LaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	LaplacianMatrix.m=vecAllVertex.size();

	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		Halfedge_around_facet_circulator Hafc = ROIFacets.at(i)->facet_begin();
		//may have problem
		Vertex_handle CurrentFacetVertex[3],NbFacetVertex[3];
		int iIndex=0;
		do 
		{
			CurrentFacetVertex[iIndex]=Hafc->vertex();
			NbFacetVertex[iIndex]=Hafc->opposite()->next()->vertex();
			iIndex++;
		} while(++Hafc != ROIFacets.at(i)->facet_begin());

		for (unsigned int j=0;j<vecAllVertex.size();j++)
		{
			if (iType==1)//uniform weight
			{
				if (vecAllVertex.at(j)==CurrentFacetVertex[0]
				||vecAllVertex.at(j)==CurrentFacetVertex[1]
				||vecAllVertex.at(j)==CurrentFacetVertex[2])
				{
					LaplacianMatrix[i][j]=1.0/3;
				}
				//two neighbor facetes of the center facet share a same edge
				else if (vecAllVertex.at(j)==NbFacetVertex[0] && vecAllVertex.at(j)==NbFacetVertex[1])
				{
					LaplacianMatrix[i][j]=-2.0/3;
				}
				else if (vecAllVertex.at(j)==NbFacetVertex[1] && vecAllVertex.at(j)==NbFacetVertex[2])
				{
					LaplacianMatrix[i][j]=-2.0/3;
				}
				else if (vecAllVertex.at(j)==NbFacetVertex[0] && vecAllVertex.at(j)==NbFacetVertex[2])
				{
					LaplacianMatrix[i][j]=-2.0/3;
				}
				else if (vecAllVertex.at(j)==NbFacetVertex[0]
				||vecAllVertex.at(j)==NbFacetVertex[1]
				||vecAllVertex.at(j)==NbFacetVertex[2])
				{
					LaplacianMatrix[i][j]=-1.0/3;
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

void CDualMeshDeform::BackUpEdgeVectorsForRigidDeform(vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets)
{
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		Point_3 CurrentPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));
		vector<Vector_3> EdgeVectors;
		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			Point_3 NbPoint=GeometryAlgorithm::GetFacetCenter(NeighborFacets.at(i).at(j));
			Vector_3 CurrentEdge=CurrentPoint-NbPoint;
			EdgeVectors.push_back(CurrentEdge);
		}
		ROIFacets.at(i)->SetOldEdgeVectors(EdgeVectors);
	}
}

void CDualMeshDeform::ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
														 vector<Vertex_handle> vecAnchorVertices, vector<Point_3> vecDeformCurvePoint3d,
														 vector<Facet_handle> ROIFacets, vector<vector<Facet_handle>> NeighborFacets,
														 vector<vector<double> >& LaplacianRightHandSide, 
														 vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//dual laplacian
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(ROIFacets.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(ROIFacets.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(ROIFacets.at(i)->GetUniformLaplacian().z());
		}
		else if (iType==3)
		{
			Laplacian[0].push_back(ROIFacets.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(ROIFacets.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(ROIFacets.at(i)->GetWeightedLaplacian().z());
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
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIFacets.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDualMeshDeform::ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb, 
												   vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
												   vector<Point_3> vecDeformCurvePoint3d,vector<Facet_handle> ROIFacets, 
												   vector<vector<Facet_handle>> NeighborFacets, vector<vector<double> >& RigidRightHandSide,
												   vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//dual laplacian
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIFacets.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIFacets.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIFacets.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIFacets.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIFacets.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIFacets.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIFacets.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=ROIFacets.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			vector<double> NbRotationMatrix=NeighborFacets.at(i).at(j)->GetRigidDeformRotationMatrix();
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
				dCurrentWeight=ROIFacets.at(i)->GetEdgeWeights().at(j);
			}
			double dProduct[3];
			for (int k=0;k<3;k++)
			{
				dProduct[k]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+0)+NbRotationMatrix.at(3*k+0))*
					(ROIFacets.at(i)->GetOldEdgeVectors().at(j).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+1)+NbRotationMatrix.at(3*k+1))*
					(ROIFacets.at(i)->GetOldEdgeVectors().at(j).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+2)+NbRotationMatrix.at(3*k+2))*
					(ROIFacets.at(i)->GetOldEdgeVectors().at(j).z());
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
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIFacets.size()+AnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDualMeshDeform::ComputeRotationForRigidDeform(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets)
{
	for (unsigned int i=0;i<ROIFacets.size();i++)
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

		Point_3 CurrentPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));

		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=GeometryAlgorithm::GetFacetCenter(NeighborFacets.at(i).at(j));
			Vector_3 NewEdge=CurrentPoint-NbPoint;
			//get corresponding old edge
			Vector_3 OldEdge=ROIFacets.at(i)->GetOldEdgeVectors().at(j);
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=ROIFacets.at(i)->GetEdgeWeights().at(j);
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
		ROIFacets.at(i)->SetRigidDeformRotationMatrix(RotationMatrix);
	}
}

void CDualMeshDeform::ComputeScaleFactor(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets,bool bTestIsoScale/* =false */)
{
	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		vector<double> RotationMatrix=ROIFacets.at(i)->GetRigidDeformRotationMatrix();
		int iOldEdgeIndex=0;
		double dNumerator[3],dDenominator[3];
		for (int j=0;j<3;j++)
		{
			dNumerator[j]=dDenominator[j]=0;
		}

		Point_3 CurrentPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));

		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=GeometryAlgorithm::GetFacetCenter(NeighborFacets.at(i).at(j));
			Vector_3 NewEdge=CurrentPoint-NbPoint;
			//get corresponding old edge
			Vector_3 OldEdge=ROIFacets.at(i)->GetOldEdgeVectors().at(j);
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
				dCurrentWeight=ROIFacets.at(i)->GetEdgeWeights().at(j);
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
		//store rotation matrix
		ROIFacets.at(i)->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
	}
}

void CDualMeshDeform::FlexibleDualMeshDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
											 vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
											 vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
											 vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints,
											 bool bTestIsoScale/* =false */)
{
	//collect deform related edge info first
	vector<Facet_handle> ROIFacets;
	GetFacetInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIFacets);

	//get facet topo
	vector<vector<Facet_handle>> NeighborFacets;
	GetFacetTopo(ROIFacets,NeighborFacets);

	//compute facet laplacian
	ComputeFacetLaplacian(iType,ROIFacets,NeighborFacets);

	////test
	//testPoints.clear();
	//for (unsigned int i=0;i<ROIFacets.size();i++)
	//{
	//	Point_3 MidPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));
	//	testPoints.push_back(MidPoint);
	//}
	cout<<"related facet num: "<<ROIFacets.size()<<endl;

	//get one more layer anchor vertices
	vector<Vertex_handle> OneMoreLayerAnchor;
	GetHalfMoreLayerAnchor(vecHandleNb,ROIVertices,vecAnchorVertices,ROIFacets,NeighborFacets,OneMoreLayerAnchor);
	vecAnchorVertices.insert(vecAnchorVertices.end(),OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end());

	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;

	//get Laplacian matrix
	SparseMatrix LaplacianMatrix(ROIFacets.size());
	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIFacets,NeighborFacets,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
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
			BackUpEdgeVectorsForRigidDeform(ROIFacets,NeighborFacets);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIFacets,NeighborFacets,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
				vecDeformCurvePoint3d,ROIFacets,NeighborFacets,
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

		
		//bool bResult=CMath::ComputeLSE(LeftHandMatrixATEST,RightHandSide,Result);
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
			ComputeRotationForRigidDeform(iType,ROIFacets,NeighborFacets);
			ComputeScaleFactor(iType,ROIFacets,NeighborFacets);
		}
	}

	//test matrix
	//TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();   
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;
}

void CDualMeshDeform::GetDualMeshTest(KW_Mesh& PrimalMesh,map<Point_3,vector<Point_3>>& DualMesh)
{
	vector<Vertex_handle> vecHandleNb,ROIVertices,vecAnchorVertices;
	for (Vertex_iterator i=PrimalMesh.vertices_begin();i!=PrimalMesh.vertices_end();i++)
	{
		ROIVertices.push_back(i);
	}

	//collect deform related edge info first
	vector<Facet_handle> ROIFacets;
	GetFacetInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIFacets);

	//get edge topo
	vector<vector<Facet_handle>> NeighborFacets;
	GetFacetTopo(ROIFacets,NeighborFacets);

	for (unsigned int i=0;i<ROIFacets.size();i++)
	{
		Point_3 CenterPoint=GeometryAlgorithm::GetFacetCenter(ROIFacets.at(i));
		vector<Point_3> NeighborPoints;
		for (unsigned int j=0;j<NeighborFacets.at(i).size();j++)
		{
			NeighborPoints.push_back(GeometryAlgorithm::GetFacetCenter(NeighborFacets.at(i).at(j)));
		}
		DualMesh.insert(pair <Point_3, vector<Point_3>>(CenterPoint,NeighborPoints));
	}

}