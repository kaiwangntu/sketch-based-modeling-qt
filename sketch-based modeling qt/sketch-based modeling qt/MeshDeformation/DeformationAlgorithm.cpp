#include "DeformationAlgorithm.h"
#include "../OBJHandle.h"

CDeformationAlgorithm::CDeformationAlgorithm(void)
{
}

CDeformationAlgorithm::~CDeformationAlgorithm(void)
{
}

void CDeformationAlgorithm::BackUpMeshGeometry(KW_Mesh Mesh,vector<Point_3>& CurrentPos)
{
	for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)
	{
		CurrentPos.push_back(i->point());
	}
}

void CDeformationAlgorithm::RestoreMeshGeometry(KW_Mesh& Mesh,vector<Point_3> OldPos)
{
	int iIndex=0;
	for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)
	{
		i->point()=OldPos.at(iIndex);
		iIndex++;
	}
}

void CDeformationAlgorithm::BackUpMeshLaplacian(int iWeightType,vector<Vertex_handle>& AllVertices,vector<Vector_3>& CurrentLaplacian)
{
	CurrentLaplacian.clear();
	for (unsigned int i=0;i<AllVertices.size();i++)
	{
		if (iWeightType==1)
		{
			CurrentLaplacian.push_back(AllVertices.at(i)->GetUniformLaplacian());
		}
		else if (iWeightType=3)
		{
			CurrentLaplacian.push_back(AllVertices.at(i)->GetWeightedLaplacian());
		}
	}
}

void CDeformationAlgorithm::RestoreMeshLaplacian(int iWeightType,vector<Vertex_handle>& AllVertices,vector<Vector_3> OldLaplacian)
{
	for (unsigned int i=0;i<OldLaplacian.size();i++)
	{
		if (iWeightType==1)
		{
			AllVertices.at(i)->SetUniformLaplacian(OldLaplacian.at(i));
		}
		else if (iWeightType==3)
		{
			AllVertices.at(i)->SetWeightedLaplacian(OldLaplacian.at(i));
		}
	}
}

void CDeformationAlgorithm::NaiveLaplacianDeform(int iType,KW_Mesh& Mesh,
												 vector<HandlePointStruct>& vecHandlePoint,
												 vector<Vertex_handle>& vecHandleNb,
												 vector<Vertex_handle>& ROIVertices,
												 vector<Vertex_handle>& vecAnchorVertices, 
												 vector<Point_3>& vecDeformCurvePoint3d)
{
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
		vecvecLaplacianMatrix);
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	//for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	//{
	//	vecDeformCurvePoint3d.at(i)=vecHandlePoint.at(i).PointPos;
	//}
	vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
	ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,vecDeformCurvePoint3d,
		LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);

	cout<<"HandlePoint: "<<vecHandlePoint.size()<<endl;
	cout<<"HandleNB: "<<vecHandleNb.size()<<endl;
	cout<<"ROI: "<<ROIVertices.size()<<endl;
	cout<<"Anchor: "<<vecAnchorVertices.size()<<endl;

	//vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
	//vector<vector<double> > RightHandSide=LaplacianRightHandSide;
	//for (int i=0;i<3;i++)
	//{
	//	RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
	//		AnchorRightHandSide.at(i).end());
	//	RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
	//		HandleRightHandSide.at(i).end());
	//}
	//vector<vector<double> > Result;
	//bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
	//if (bResult)


	vector<vector<double> > LeftHandConstrainedMatrix=AnchorConstraintMatrix;
	LeftHandConstrainedMatrix.insert(LeftHandConstrainedMatrix.end(),
		HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
	vector<vector<double> > RightHandConstrainedVector;
	for (int i=0;i<3;i++)
	{
		vector<double> CurrentVec=AnchorRightHandSide.at(i);
		CurrentVec.insert(CurrentVec.end(),
			HandleRightHandSide.at(i).begin(),HandleRightHandSide.at(i).end());
		RightHandConstrainedVector.push_back(CurrentVec);
	}

	cout<<endl;
	cout<<"A size: "<<vecvecLaplacianMatrix.size()<<"*"<<vecvecLaplacianMatrix.front().size()<<endl;
	cout<<"B size: "<<LaplacianRightHandSide.front().size()<<endl;
	cout<<"C size: "<<LeftHandConstrainedMatrix.size()<<"*"<<LeftHandConstrainedMatrix.front().size()<<endl;
	cout<<"D size: "<<RightHandConstrainedVector.front().size()<<endl;
	
	vector<vector<double> > Result;
	for (int i=0;i<3;i++)
	{
		vector<double> CurrentResult;
		bool bResult=CMath::ComputeConstrainedLSE(vecvecLaplacianMatrix,LaplacianRightHandSide.at(i),
			LeftHandConstrainedMatrix,RightHandConstrainedVector.at(i),CurrentResult);
		if (bResult)
		{
			Result.push_back(CurrentResult);
		}
	}

	if (!Result.empty())
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
}

//get laplacian matrix which contains 3 parts:1-handle vertices 2-ROI 3-anchor vertices
void CDeformationAlgorithm::ComputeLaplacianMatrix(int iType,KW_Mesh& Mesh,
												   vector<Vertex_handle> vecHandleNb, 
												   vector<Vertex_handle> ROIVertices,
												   vector<Vertex_handle> vecAnchorVertices, 
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
			if (i==j)
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
			//vertex i and vertex j are neighbors
			else if (GeometryAlgorithm::JudgeIfNeighbors(vecAllVertices.at(i),vecAllVertices.at(j)))
			{
				//CurrentRow.push_back(-1/(double)vecAllVertices.at(i)->vertex_degree());
				if (iType==1)
				{
					CurrentRow.push_back(-1);
				} 
				else
				{
					double dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(vecAllVertices.at(i),
						vecAllVertices.at(j),iType);
					CurrentRow.push_back(-dCurrentWeight);
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

void CDeformationAlgorithm::ComputeLaplacianMatrix(int iType,KW_Mesh& Mesh,
												   vector<Vertex_handle> vecHandleNb, 
												   vector<Vertex_handle> ROIVertices,
												   vector<Vertex_handle> vecAnchorVertices, 
												   SparseMatrix& LaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	//laplacian matrix is of size iRow*iColumn
	int iRow=(int)(vecHandleNb.size()+ROIVertices.size());
	int iColumn=vecAllVertices.size();
	LaplacianMatrix.m=iColumn;

	for (int i=0;i<iRow;i++)//for all rows
	{
		for (int j=0;j<iColumn;j++)//for each column of CurrentRow
		{
			if (i==j)
			{
				//CurrentRow.push_back(1);
				if (iType==1)
				{
					LaplacianMatrix[i][j]=(float)vecAllVertices.at(i)->vertex_degree();
				}
				else
				{
					LaplacianMatrix[i][j]=(float)vecAllVertices.at(i)->GetWeightedLaplacianSumWeight();
				}
			}
			//vertex i and vertex j are neighbors
			else if (GeometryAlgorithm::JudgeIfNeighbors(vecAllVertices.at(i),vecAllVertices.at(j)))
			{
				//CurrentRow.push_back(-1/(double)vecAllVertices.at(i)->vertex_degree());
				if (iType==1)
				{
					LaplacianMatrix[i][j]=-1;
				} 
				else
				{
					double dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(vecAllVertices.at(i),
						vecAllVertices.at(j),iType);
					LaplacianMatrix[i][j]=-dCurrentWeight;
				}
			}
			else
			{
				//0,do nothing
			}
		}
	}
}

//get the constraint matrix of anchor and handle vertices
void CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint, 
																 vector<Vertex_handle> vecHandleNb,
																 vector<Vertex_handle> ROIVertices, 
																 vector<Vertex_handle> vecAnchorVertices,
																 vector<vector<double> >& AnchorConstraintMatrix, 
																 vector<vector<double> >& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
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

	int iHandleRow=(int)vecHandlePoint.size();
	for (int i=0;i<iHandleRow;i++)
	{
		HandlePointStruct CurrentHandlePoint=vecHandlePoint.at(i);
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			CurrentRow.push_back(0);
		}
		for (unsigned int j=0;j<CurrentHandlePoint.vecVertexIndex.size();j++)
		{
			CurrentRow.at(CurrentHandlePoint.vecVertexIndex.at(j))=CONSTRAINED_HANDLE_WEIGHT*CurrentHandlePoint.vecPara.at(j);
		}
		HandleConstraintMatrix.push_back(CurrentRow);
	}
}

void CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint, 
																 vector<Vertex_handle> vecHandleNb,
																 vector<Vertex_handle> ROIVertices, 
																 vector<Vertex_handle> vecAnchorVertices,
																 SparseMatrix& AnchorConstraintMatrix, 
																 SparseMatrix& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
	AnchorConstraintMatrix.m=iColumn;
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
				//do nothing
			}
		}
	}

	int iHandleRow=(int)vecHandlePoint.size();
	HandleConstraintMatrix.m=iColumn;
	for (int i=0;i<iHandleRow;i++)
	{
		HandlePointStruct CurrentHandlePoint=vecHandlePoint.at(i);
		for (unsigned int j=0;j<CurrentHandlePoint.vecVertexIndex.size();j++)
		{
			HandleConstraintMatrix[i][CurrentHandlePoint.vecVertexIndex.at(j)]=CONSTRAINED_HANDLE_WEIGHT*CurrentHandlePoint.vecPara.at(j);
		}
	}
}

void CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices, 
																		 vector<Vertex_handle> vecAnchorVertices,vector<vector<double> >& AnchorConstraintMatrix,
																		 vector<vector<double> >& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
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

	int iHandleRow=(int)vecHandleNb.size();
	for (int i=0;i<iHandleRow;i++)
	{
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			if (i==j)
			{
				CurrentRow.push_back(CONSTRAINED_HANDLE_WEIGHT*1.0);
			}
			else
			{
				CurrentRow.push_back(0);
			}
		}
		HandleConstraintMatrix.push_back(CurrentRow);
	}
}

void CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices, 
																 vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix, 
																 SparseMatrix& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
	AnchorConstraintMatrix.m=iColumn;
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
				//do nothing
			}
		}
	}

	int iHandleRow=(int)vecHandleNb.size();
	HandleConstraintMatrix.m=iColumn;
	for (int i=0;i<iHandleRow;i++)
	{
		for (int j=0;j<iColumn;j++)
		{
			if (i==j)
			{
				HandleConstraintMatrix[i][j]=CONSTRAINED_HANDLE_WEIGHT*1.0;
			}
			else
			{
				//do nothing
			}
		}
	}
}

//compute the right hand side of the naive laplacian equation
//It contains 1: the delta value of uniform laplacian of Handle&ROI,2:coordinates value of anchor
//3: the deformed coordinates value of the handle vertices
void CDeformationAlgorithm::ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
															   vector<Vertex_handle> ROIVertices,
															   vector<Vertex_handle> vecAnchorVertices, 
															   vector<Point_3> vecDeformCurvePoint3d,
															   vector<vector<double> >& LaplacianRightHandSide, 
															   vector<vector<double> >& AnchorRightHandSide,
															   vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(vecHandleNb.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(vecHandleNb.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(vecHandleNb.at(i)->GetUniformLaplacian().z());
		}
		else
		{
			Laplacian[0].push_back(vecHandleNb.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(vecHandleNb.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(vecHandleNb.at(i)->GetWeightedLaplacian().z());
		}
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(ROIVertices.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(ROIVertices.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(ROIVertices.at(i)->GetUniformLaplacian().z());
		} 
		else
		{
			Laplacian[0].push_back(ROIVertices.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(ROIVertices.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(ROIVertices.at(i)->GetWeightedLaplacian().z());
		}
	}

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
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}


//rigid deformation 
void CDeformationAlgorithm::RigidDeform(int iType,int iIterNum,KW_Mesh& Mesh, 
										vector<HandlePointStruct>& vecHandlePoint,
										vector<Vertex_handle>& vecHandleNb, 
										vector<Vertex_handle>& ROIVertices,
										vector<Vertex_handle>& vecAnchorVertices, 
										vector<Point_3>& vecDeformCurvePoint3d)
{
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
		vecvecLaplacianMatrix);
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeRigidRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//vector<vector<double> > LeftHandConstrainedMatrix=AnchorConstraintMatrix;
		//LeftHandConstrainedMatrix.insert(LeftHandConstrainedMatrix.end(),
		//	HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
		//vector<vector<double> > RightHandConstrainedVector;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentVec=AnchorRightHandSide.at(i);
		//	CurrentVec.insert(CurrentVec.end(),
		//		HandleRightHandSide.at(i).begin(),HandleRightHandSide.at(i).end());
		//	RightHandConstrainedVector.push_back(CurrentVec);
		//}

		//vector<vector<double> > Result;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentResult;
		//	bool bResult=CMath::ComputeConstrainedLSE(vecvecLaplacianMatrix,LaplacianRightHandSide.at(i),
		//		LeftHandConstrainedMatrix,RightHandConstrainedVector.at(i),CurrentResult);
		//	if (bResult)
		//	{
		//		Result.push_back(CurrentResult);
		//	}
		//}

		//if (!Result.empty())
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

		//std::ofstream out("1.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			clock_t RotationBegin=clock();   
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			clock_t RotationEnd=clock();   
			cout<<"time for computing rotation: "<<float(RotationEnd-RotationBegin)<<endl;
		}
	}
}

//backup edge vectors for computing ratation in rigid deformation
void CDeformationAlgorithm::BackUpEdgeVectorsForRigidDeform(KW_Mesh& Mesh, 
															vector<Vertex_handle>& vecHandleNb,
															vector<Vertex_handle>& ROIVertices,
															vector<Vertex_handle>& vecAnchorVertices)
{
	if (vecAnchorVertices.empty())
	{
		GeometryAlgorithm::ComputeOldEdgeVectors(Mesh);
	}
	else
	{
		vector<Vertex_handle> vecAllVertices;
		vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
		vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
		vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

		GeometryAlgorithm::ComputeOldEdgeVectors(vecAllVertices);
	}
}

//compute Rotation in rigid deformation
void CDeformationAlgorithm::ComputeRotationForRigidDeform(int iType,KW_Mesh& Mesh,
														  vector<Vertex_handle>& vecHandleNb,
														  vector<Vertex_handle>& ROIVertices,
														  vector<Vertex_handle>& vecAnchorVertices)
{
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	for (unsigned int i=0;i<vecAllVertices.size();i++)
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
		int iOldEdgeIndex=0;

		Halfedge_around_vertex_circulator Havc=vecAllVertices.at(i)->vertex_begin();
		do 
		{
			//compute new edge
			Vector_3 NewEdge=vecAllVertices.at(i)->point()-Havc->opposite()->vertex()->point();

			//get corresponding old edge
			Vector_3 OldEdge=vecAllVertices.at(i)->GetOldEdgeVectors().at(iOldEdgeIndex);
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=vecAllVertices.at(i)->GetEdgeWeights().at(iOldEdgeIndex);
			}

			Havc++;
			iOldEdgeIndex++;
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
		} while(Havc!=vecAllVertices.at(i)->vertex_begin());
		
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
		vecAllVertices.at(i)->SetRigidDeformRotationMatrix(RotationMatrix);
	}
}

//compute second Rotation in rigid deformation
void CDeformationAlgorithm::ComputeSecondRotationForRigidDeform(int iType,KW_Mesh& Mesh,
														  vector<Vertex_handle>& vecHandleNb,
														  vector<Vertex_handle>& ROIVertices,
														  vector<Vertex_handle>& vecAnchorVertices)
{
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	for (unsigned int i=0;i<vecAllVertices.size();i++)
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
		int iOldEdgeIndex=0;

		Halfedge_around_vertex_circulator Havc=vecAllVertices.at(i)->vertex_begin();
		vector<double> FirstRotationMatrix=vecAllVertices.at(i)->GetRigidDeformRotationMatrix();
		Vector_3 ScaleFactor=vecAllVertices.at(i)->GetScaleFactor();
		do 
		{
			//compute new edge
			Vector_3 NewEdge=vecAllVertices.at(i)->point()-Havc->opposite()->vertex()->point();
			//get corresponding old edge
			Vector_3 OldEdge=vecAllVertices.at(i)->GetOldEdgeVectors().at(iOldEdgeIndex);
			//get rotated and scaled old edge=ScaleMatrix*FirstRotationMatrix*OldEdge
			double dRotatedOldEdge[3];
			dRotatedOldEdge[0]=FirstRotationMatrix.at(0)*OldEdge.x()+
				FirstRotationMatrix.at(1)*OldEdge.y()+
				FirstRotationMatrix.at(2)*OldEdge.z();
			dRotatedOldEdge[1]=FirstRotationMatrix.at(3)*OldEdge.x()+
				FirstRotationMatrix.at(4)*OldEdge.y()+
				FirstRotationMatrix.at(5)*OldEdge.z();
			dRotatedOldEdge[2]=FirstRotationMatrix.at(6)*OldEdge.x()+
				FirstRotationMatrix.at(7)*OldEdge.y()+
				FirstRotationMatrix.at(8)*OldEdge.z();
			double dScaledRotatedOldEdge[3];
			dScaledRotatedOldEdge[0]=ScaleFactor.x()*dRotatedOldEdge[0];
			dScaledRotatedOldEdge[1]=ScaleFactor.y()*dRotatedOldEdge[1];
			dScaledRotatedOldEdge[2]=ScaleFactor.z()*dRotatedOldEdge[2];

			OldEdge=Vector_3(dScaledRotatedOldEdge[0],dScaledRotatedOldEdge[1],dScaledRotatedOldEdge[2]);

			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=vecAllVertices.at(i)->GetEdgeWeights().at(iOldEdgeIndex);
			}

			Havc++;
			iOldEdgeIndex++;
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
		} while(Havc!=vecAllVertices.at(i)->vertex_begin());

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
		vecAllVertices.at(i)->SetSecondRigidDeformRotationMatrix(RotationMatrix);
	}
}

//compute the right hand side of the rigid equation
//It contains 1: the delta value of uniform rigid of Handle&ROI,2:coordinates value of anchor
//3: the deformed coordinates value of the handle vertices
void CDeformationAlgorithm::ComputeRigidRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb,
													  vector<Vertex_handle>& ROIVertices,
													  vector<Vertex_handle>& vecAnchorVertices,
													  vector<Point_3> vecDeformCurvePoint3d,
													  vector<vector<double> >& RigidRightHandSide, 
													  vector<vector<double> >& AnchorRightHandSide,
													  vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecHandleNb.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		CurrentLaplacian[0]=CurrentLaplacian[1]=CurrentLaplacian[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
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
				dCurrentWeight=vecHandleNb.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentLaplacian[j]=CurrentLaplacian[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
		Laplacian[0].push_back(CurrentLaplacian[0]);
		Laplacian[1].push_back(CurrentLaplacian[1]);
		Laplacian[2].push_back(CurrentLaplacian[2]);
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIVertices.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		CurrentLaplacian[0]=CurrentLaplacian[1]=CurrentLaplacian[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
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
				dCurrentWeight=ROIVertices.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentLaplacian[j]=CurrentLaplacian[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
		Laplacian[0].push_back(CurrentLaplacian[0]);
		Laplacian[1].push_back(CurrentLaplacian[1]);
		Laplacian[2].push_back(CurrentLaplacian[2]);
	}
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
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

//rotated laplacian deformation with roration considered
void CDeformationAlgorithm::RotatedLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh, 
												   vector<HandlePointStruct>& vecHandlePoint,
												   vector<Vertex_handle>& vecHandleNb, 
												   vector<Vertex_handle>& ROIVertices,
												   vector<Vertex_handle>& vecAnchorVertices,
												   vector<Point_3>& vecDeformCurvePoint3d)
{
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
		vecvecLaplacianMatrix);
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeRotatedLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//vector<vector<double> > LeftHandConstrainedMatrix=AnchorConstraintMatrix;
		//LeftHandConstrainedMatrix.insert(LeftHandConstrainedMatrix.end(),
		//	HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
		//vector<vector<double> > RightHandConstrainedVector;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentVec=AnchorRightHandSide.at(i);
		//	CurrentVec.insert(CurrentVec.end(),
		//		HandleRightHandSide.at(i).begin(),HandleRightHandSide.at(i).end());
		//	RightHandConstrainedVector.push_back(CurrentVec);
		//}

		//vector<vector<double> > Result;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentResult;
		//	bool bResult=CMath::ComputeConstrainedLSE(vecvecLaplacianMatrix,LaplacianRightHandSide.at(i),
		//		LeftHandConstrainedMatrix,RightHandConstrainedVector.at(i),CurrentResult);
		//	if (bResult)
		//	{
		//		Result.push_back(CurrentResult);
		//	}
		//}

		//if (!Result.empty())
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

		//std::ofstream out("1.obj",ios_base::out | ios_base::trunc);
		//print_polyhedron_wavefront(out,Mesh);

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
		}
	}
}

//compute the right hand side of the rotated laplacian equation
//It contains 1: the delta value of uniform rigid of Handle&ROI,2:coordinates value of anchor
//3: the deformed coordinates value of the handle vertices
void CDeformationAlgorithm::ComputeRotatedLaplacianRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb, 
																 vector<Vertex_handle>& ROIVertices,
																 vector<Vertex_handle>& vecAnchorVertices,
																 vector<Point_3> vecDeformCurvePoint3d,
																 vector<vector<double> >& RigidRightHandSide,
																 vector<vector<double> >& AnchorRightHandSide,
																 vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecHandleNb.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetWeightedLaplacian().z();
			}
			Laplacian[j].push_back(CurrentLaplacian[j]);
		}
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIVertices.at(i)->GetRigidDeformRotationMatrix();
		double CurrentLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetWeightedLaplacian().z();
			}
			Laplacian[j].push_back(CurrentLaplacian[j]);
		}
	}
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
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

//flexible deformation 
void CDeformationAlgorithm::FlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
										   vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
										   vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
										   vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale)
{
	//vector<vector<double> > vecvecLaplacianMatrix;
	//ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
	//	vecvecLaplacianMatrix);
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);


	//vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	//GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
	//	AnchorConstraintMatrix,HandleConstraintMatrix);
	//vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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
//		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);

		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

		if (bResult)
		//vector<vector<double> > LeftHandConstrainedMatrix=AnchorConstraintMatrix;
		//LeftHandConstrainedMatrix.insert(LeftHandConstrainedMatrix.end(),
		//	HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
		//vector<vector<double> > RightHandConstrainedVector;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentVec=AnchorRightHandSide.at(i);
		//	CurrentVec.insert(CurrentVec.end(),
		//		HandleRightHandSide.at(i).begin(),HandleRightHandSide.at(i).end());
		//	RightHandConstrainedVector.push_back(CurrentVec);
		//}

		//vector<vector<double> > Result;
		//for (int i=0;i<3;i++)
		//{
		//	vector<double> CurrentResult;
		//	bool bResult=CMath::ComputeConstrainedLSE(vecvecLaplacianMatrix,LaplacianRightHandSide.at(i),
		//		LeftHandConstrainedMatrix,RightHandConstrainedVector.at(i),CurrentResult);
		//	if (bResult)
		//	{
		//		Result.push_back(CurrentResult);
		//	}
		//}

		//if (!Result.empty())
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

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,bTestIsoScale);
		}
	}

	TAUCSSolver.TAUCSClear();

//	GetInterpolationResult(vecDeformCurvePoint3d,vecHandlePoint,vecHandleNb);
}

void CDeformationAlgorithm::FlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
										   vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
										   vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d)
{
	cout<<"num of handle: "<<vecHandleNb.size()<<endl;
	cout<<"num of roi: "<<ROIVertices.size()<<endl;
	cout<<"num of anchor: "<<vecAnchorVertices.size()<<endl;

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandleNb.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::FlexibleRSRDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
											  vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
											  vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
											  vector<Point_3>& vecDeformCurvePoint3d)
{
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);


	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRSRRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeSecondRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::ComputeScaleFactor(int iType,KW_Mesh& Mesh, vector<Vertex_handle>& vecHandleNb,
											   vector<Vertex_handle>& ROIVertices, vector<Vertex_handle>& vecAnchorVertices,
											   bool bTestIsoScale)
{
	vector<Vertex_handle> vecAllVertices;
	vecAllVertices.insert(vecAllVertices.end(),vecHandleNb.begin(),vecHandleNb.end());
	vecAllVertices.insert(vecAllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	//	vecAllVertices.insert(vecAllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	for (unsigned int i=0;i<vecAllVertices.size();i++)
	{
		vector<double> RotationMatrix=vecAllVertices.at(i)->GetRigidDeformRotationMatrix();
		int iOldEdgeIndex=0;
		double dNumerator[3],dDenominator[3];
		for (int j=0;j<3;j++)
		{
			dNumerator[j]=dDenominator[j]=0;
		}
		Halfedge_around_vertex_circulator Havc=vecAllVertices.at(i)->vertex_begin();
		do 
		{
			//compute new edge
			Vector_3 NewEdge=vecAllVertices.at(i)->point()-Havc->opposite()->vertex()->point();
			//get corresponding old edge
			Vector_3 OldEdge=vecAllVertices.at(i)->GetOldEdgeVectors().at(iOldEdgeIndex);
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
				dCurrentWeight=vecAllVertices.at(i)->GetEdgeWeights().at(iOldEdgeIndex);
			}
			//get numerator
			dNumerator[0]=dNumerator[0]+dCurrentWeight*dRotatedOldEdge[0]*NewEdge.x();
			dNumerator[1]=dNumerator[1]+dCurrentWeight*dRotatedOldEdge[1]*NewEdge.y();
			dNumerator[2]=dNumerator[2]+dCurrentWeight*dRotatedOldEdge[2]*NewEdge.z();
			//get denominator
			dDenominator[0]=dDenominator[0]+dCurrentWeight*dRotatedOldEdge[0]*dRotatedOldEdge[0];
			dDenominator[1]=dDenominator[1]+dCurrentWeight*dRotatedOldEdge[1]*dRotatedOldEdge[1];
			dDenominator[2]=dDenominator[2]+dCurrentWeight*dRotatedOldEdge[2]*dRotatedOldEdge[2];

			Havc++;
			iOldEdgeIndex++;
			//construct convariance matrix S
		} while(Havc!=vecAllVertices.at(i)->vertex_begin());
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
		vecAllVertices.at(i)->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
	}
}

void CDeformationAlgorithm::ComputeFlexibleRightHandSide(double dLamda,int iType,
														 vector<Vertex_handle>& vecHandleNb, 
														 vector<Vertex_handle>& ROIVertices,
														 vector<Point_3>& vecAnchorVertices, 
														 vector<Point_3> vecDeformCurvePoint3d,
														 vector<vector<double> >& RigidRightHandSide,
														 vector<vector<double> >& AnchorRightHandSide,
														 vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecHandleNb.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=vecHandleNb.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
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
				dCurrentWeight=vecHandleNb.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}


	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIVertices.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=ROIVertices.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
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
				dCurrentWeight=ROIVertices.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecAnchorVertices.at(i).x());//->point()
		Anchor[1].push_back(vecAnchorVertices.at(i).y());//->point()
		Anchor[2].push_back(vecAnchorVertices.at(i).z());//->point()
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDeformationAlgorithm::ComputeFlexibleRSRRightHandSide(double dLamda,int iType,
														 vector<Vertex_handle>& vecHandleNb, 
														 vector<Vertex_handle>& ROIVertices,
														 vector<Point_3>& vecAnchorVertices, 
														 vector<Point_3> vecDeformCurvePoint3d,
														 vector<vector<double> >& RigidRightHandSide,
														 vector<vector<double> >& AnchorRightHandSide,
														 vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecHandleNb.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleNb.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleNb.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleNb.at(i)->GetWeightedLaplacian().z();
			}
		}
		//compute scaled rotated Laplacian
		Vector_3 ScaleFactor=vecHandleNb.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();
		//DBWindowWrite("before 2nd rotation: %f,%f,%f\n",CurrentRotatedLaplacian[0],CurrentRotatedLaplacian[1],CurrentRotatedLaplacian[2]);
		//compute rotated scaled rotated Laplacian
		vector<double> CurrentSecondRotationMatrix=vecHandleNb.at(i)->GetSecondRigidDeformRotationMatrix();
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentSecondRotationMatrix.at(3*j+0)*CurrentRotatedLaplacian[0]
					+CurrentSecondRotationMatrix.at(3*j+1)*CurrentRotatedLaplacian[1]
					+CurrentSecondRotationMatrix.at(3*j+2)*CurrentRotatedLaplacian[2];
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentSecondRotationMatrix.at(3*j+0)*CurrentRotatedLaplacian[0]
					+CurrentSecondRotationMatrix.at(3*j+1)*CurrentRotatedLaplacian[1]
					+CurrentSecondRotationMatrix.at(3*j+2)*CurrentRotatedLaplacian[2];
			}
		}
		//DBWindowWrite("after 2nd rotation: %f,%f,%f\n",CurrentRotatedLaplacian[0],CurrentRotatedLaplacian[1],CurrentRotatedLaplacian[2]);

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
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
				dCurrentWeight=vecHandleNb.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}


	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIVertices.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIVertices.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIVertices.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIVertices.at(i)->GetWeightedLaplacian().z();
			}
		}
		//compute scaled rotated Laplacian
		Vector_3 ScaleFactor=ROIVertices.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();
		//compute rotated scaled rotated Laplacian
		vector<double> CurrentSecondRotationMatrix=ROIVertices.at(i)->GetSecondRigidDeformRotationMatrix();
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentSecondRotationMatrix.at(3*j+0)*CurrentRotatedLaplacian[0]
				+CurrentSecondRotationMatrix.at(3*j+1)*CurrentRotatedLaplacian[1]
				+CurrentSecondRotationMatrix.at(3*j+2)*CurrentRotatedLaplacian[2];
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentSecondRotationMatrix.at(3*j+0)*CurrentRotatedLaplacian[0]
				+CurrentSecondRotationMatrix.at(3*j+1)*CurrentRotatedLaplacian[1]
				+CurrentSecondRotationMatrix.at(3*j+2)*CurrentRotatedLaplacian[2];
			}
		}

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
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
				dCurrentWeight=ROIVertices.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecAnchorVertices.at(i).x());//->point()
		Anchor[1].push_back(vecAnchorVertices.at(i).y());//->point()
		Anchor[2].push_back(vecAnchorVertices.at(i).z());//->point()
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDeformationAlgorithm::GetInterpolationResult(vector<Point_3> vecDeformCurvePoint3d, 
												   vector<HandlePointStruct>& vecHandlePoint,
												   vector<Vertex_handle>& vecHandleNb)
{
	cout<<"Interpolation Result: "<<endl;
	for (unsigned int i=0;i<vecHandlePoint.size();i++)
	{
		Point_3 OldPoint=vecDeformCurvePoint3d.at(i);

		double dNewPos[3];
		dNewPos[0]=dNewPos[1]=dNewPos[2]=0;

		for (unsigned int j=0;j<vecHandlePoint.at(i).vecVertexIndex.size();j++)
		{
			dNewPos[0]=dNewPos[0]+
				vecHandleNb.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().x()*vecHandlePoint.at(i).vecPara.at(j);
			dNewPos[1]=dNewPos[1]+
				vecHandleNb.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().y()*vecHandlePoint.at(i).vecPara.at(j);
			dNewPos[2]=dNewPos[2]+
				vecHandleNb.at(vecHandlePoint.at(i).vecVertexIndex.at(j))->point().z()*vecHandlePoint.at(i).vecPara.at(j);
		}

		Point_3 NewPoint(dNewPos[0],dNewPos[1],dNewPos[2]);

		double dDistance=sqrt(CGAL::squared_distance(OldPoint,NewPoint));

		cout<<i<<" point: "<<dDistance<<endl;
	}
}


void CDeformationAlgorithm::IterativeLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh,vector<HandlePointStruct>& vecHandlePoint,
													 vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
													 vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d)
{
	vector<vector<double> > vecvecLaplacianMatrix;
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
		vecvecLaplacianMatrix);
	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	CMath TAUCSSolver;
	vector<vector<double>> AT;
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			IterativeUpdateLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,AnchorPosConstraints,vecDeformCurvePoint3d,LaplacianRightHandSide,
												AnchorRightHandSide,HandleRightHandSide);
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
	}

	TAUCSSolver.TAUCSClear();

}

void CDeformationAlgorithm::IterativeUpdateLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
																  vector<Point_3> AnchorConstraints, vector<Point_3> vecDeformCurvePoint3d,
																  vector<vector<double> >& LaplacianRightHandSide, 
																  vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	vector<Vertex_handle> vecAll=vecHandleNb;
	vecAll.insert(vecAll.end(),ROIVertices.begin(),ROIVertices.end());
	for (unsigned int i=0;i<vecAll.size();i++)
	{
		if (iType==1)
		{
			//compute new laplacian and new average normal of facets
			double dLaplacian[3];
			dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
			Vector_3 NewFacetNormal=CGAL::NULL_VECTOR;
			double dNewSumArea=0;
			Halfedge_around_vertex_circulator Havc=vecAll.at(i)->vertex_begin();
			do 
			{
				dLaplacian[0]=dLaplacian[0]+Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+Havc->opposite()->vertex()->point().z();

				if (!Havc->is_border())
				{
					NewFacetNormal=NewFacetNormal+Havc->facet()->normal();
				}

				Facet_handle CurrentFacet=Havc->facet();
				if (CurrentFacet->is_triangle())
				{
					Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
					dNewSumArea=dNewSumArea+std::sqrt(TriFace.squared_area());
				}

				Havc++;
			} while(Havc!=vecAll.at(i)->vertex_begin());
			NewFacetNormal = NewFacetNormal / std::sqrt(NewFacetNormal*NewFacetNormal);

			double dNeighborNum=vecAll.at(i)->vertex_degree();
			dLaplacian[0]=dNeighborNum*(vecAll.at(i)->point().x())-dLaplacian[0]; 
			dLaplacian[1]=dNeighborNum*(vecAll.at(i)->point().y())-dLaplacian[1]; 
			dLaplacian[2]=dNeighborNum*(vecAll.at(i)->point().z())-dLaplacian[2];
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);

			//adjust new laplacian orientation
			double dOldMagnitude=std::sqrt(vecAll.at(i)->GetUniformLaplacian()*vecAll.at(i)->GetUniformLaplacian());
			Vector_3 InitFacetNormal=vecAll.at(i)->normal();
			Vector_3 InitLaplacianNormal(vecAll.at(i)->GetUniformLaplacian().x()/dOldMagnitude,
										vecAll.at(i)->GetUniformLaplacian().y()/dOldMagnitude,
										vecAll.at(i)->GetUniformLaplacian().z()/dOldMagnitude);
			Vector_3 NewLaplacianNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);

			double dSign=(NewLaplacianNormal*NewFacetNormal)*(InitFacetNormal*InitLaplacianNormal);
			if (dSign<0)
			{
				cout<<"<0"<<endl;
				NewLaplacianNormal=Vector_3(-NewLaplacianNormal.x(),-NewLaplacianNormal.y(),-NewLaplacianNormal.z());
			}
			else if (dSign==0)
			{
				cout<<"==0"<<endl;
				if (InitFacetNormal*InitLaplacianNormal>0)
				{
					NewLaplacianNormal=NewFacetNormal;
				}
				else
				{
					NewLaplacianNormal=Vector_3(-NewFacetNormal.x(),-NewFacetNormal.y(),-NewFacetNormal.z());
				}
			}
			else
			{
				//do not change
			}

			//scale old laplacian magnitude
//			dOldMagnitude=dOldMagnitude*sqrt(dNewSumArea/vecAll.at(i)->GetSumArea());

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
			Vector_3 NewFacetNormal=CGAL::NULL_VECTOR;
			double dNewSumArea=0;
			int iTemp=0;
			Halfedge_around_vertex_circulator Havc=vecAll.at(i)->vertex_begin();
			do 
			{
				double dCurrentWeight=vecAll.at(i)->GetEdgeWeights().at(iTemp);
				iTemp++;
				dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Havc->opposite()->vertex()->point().x();
				dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Havc->opposite()->vertex()->point().y();
				dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Havc->opposite()->vertex()->point().z();

				if (!Havc->is_border())
				{
					NewFacetNormal=NewFacetNormal+Havc->facet()->normal();
				}

				Facet_handle CurrentFacet=Havc->facet();
				if (CurrentFacet->is_triangle())
				{
					Triangle_3 TriFace(Havc->vertex()->point(),Havc->next()->vertex()->point(),Havc->prev()->vertex()->point());
					dNewSumArea=dNewSumArea+std::sqrt(TriFace.squared_area());
				}

				Havc++;
			} while(Havc!=vecAll.at(i)->vertex_begin());
			NewFacetNormal = NewFacetNormal / std::sqrt(NewFacetNormal*NewFacetNormal);

			dLaplacian[0]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().x())-dLaplacian[0]; 
			dLaplacian[1]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().y())-dLaplacian[1]; 
			dLaplacian[2]=vecAll.at(i)->GetWeightedLaplacianSumWeight()*(vecAll.at(i)->point().z())-dLaplacian[2]; 
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);
			//adjust new laplacian orientation
			double dOldMagnitude=std::sqrt(vecAll.at(i)->GetWeightedLaplacian()*vecAll.at(i)->GetWeightedLaplacian());
			Vector_3 InitFacetNormal=vecAll.at(i)->normal();
			Vector_3 InitLaplacianNormal(vecAll.at(i)->GetWeightedLaplacian().x()/dOldMagnitude,
				vecAll.at(i)->GetWeightedLaplacian().y()/dOldMagnitude,
				vecAll.at(i)->GetWeightedLaplacian().z()/dOldMagnitude);
			Vector_3 NewLaplacianNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);
			
			double dSign=(NewLaplacianNormal*NewFacetNormal)*(InitFacetNormal*InitLaplacianNormal);
			if (dSign<0)
			{
				cout<<"<0"<<endl;
				NewLaplacianNormal=Vector_3(-NewLaplacianNormal.x(),-NewLaplacianNormal.y(),-NewLaplacianNormal.z());
			}
			else if (dSign==0)
			{
				cout<<"==0"<<endl;
				if (InitFacetNormal*InitLaplacianNormal>0)
				{
					NewLaplacianNormal=NewFacetNormal;
				}
				else
				{
					NewLaplacianNormal=Vector_3(-NewFacetNormal.x(),-NewFacetNormal.y(),-NewFacetNormal.z());
				}
			}
			else
			{
				//do not change
			}

			//scale old laplacian magnitude
//			dOldMagnitude=dOldMagnitude*sqrt(dNewSumArea/vecAll.at(i)->GetSumArea());

			//update laplacian with new direction and new magnitude
			dLaplacian[0]=NewLaplacianNormal.x()*dOldMagnitude;Laplacian[0].push_back(dLaplacian[0]);
			dLaplacian[1]=NewLaplacianNormal.y()*dOldMagnitude;Laplacian[1].push_back(dLaplacian[1]);
			dLaplacian[2]=NewLaplacianNormal.z()*dOldMagnitude;Laplacian[2].push_back(dLaplacian[2]);
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
		vecHandleNb.size()+ROIVertices.size()+AnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDeformationAlgorithm::IterativeFlexibleDeform(double dLamda,int iType,int iInterpoNum,int iIterNum,KW_Mesh& Mesh, 
													vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
													vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
													vector<Point_3>& vecDeformCurvePoint3d)
{
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int k=0;k<iInterpoNum;k++)
	{
		//compute current handle constraints 
		vector<Point_3> vecCurrentHandleDest;
		for (unsigned int j=0;j<vecHandlePoint.size();j++)
		{
			Point_3 CurrentHandlePos=vecHandlePoint.at(j).PointPos;
			Point_3 FinalHandlePos=vecDeformCurvePoint3d.at(j);
			Point_3 CurrentHandleDest(CurrentHandlePos.x()+(FinalHandlePos.x()-CurrentHandlePos.x())*(double)(k+1)/(double)iInterpoNum,
				CurrentHandlePos.y()+(FinalHandlePos.y()-CurrentHandlePos.y())*(double)(k+1)/(double)iInterpoNum,
				CurrentHandlePos.z()+(FinalHandlePos.z()-CurrentHandlePos.z())*(double)(k+1)/(double)iInterpoNum);
			vecCurrentHandleDest.push_back(CurrentHandleDest);
		}

		for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
		{
			vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
			if (iCurrent==0)
			{
				if (k==0)
				{
					BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
					ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
						vecCurrentHandleDest,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
				}
				else
				{
					continue;
				}
			}
			else
			{
				ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
					vecCurrentHandleDest,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

			//compute Rotation for Handle+ROI+Anchor
			if (iCurrent!=iIterNum)
			{
				ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			}
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::FlexibleLinearInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh, 
														vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
														vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
														vector<Point_3>& vecDeformCurvePoint3d)
{
	if (vecAnchorVertices.empty())//the whole mesh involves in the computation
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
		vector<Vertex_handle> temp=vecHandleNb;
		temp.insert(temp.end(),ROIVertices.begin(),ROIVertices.end());
		temp.insert(temp.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
		if (iType==1)
		{
			GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
		}
		else
		{
			GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(temp,iType);
		}
	}

	//vector<vector<double> > vecvecLaplacianMatrix;
	//ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,
	//	vecvecLaplacianMatrix);
	//vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	//GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
	//	AnchorConstraintMatrix,HandleConstraintMatrix);
	//vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);
	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());



	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	vector<Point_3> OldPos,Lambda0Pos,Lambda1Pos;
	BackUpMeshGeometry(Mesh,OldPos);

	//compute the two boundary result(lambda==0 and lambda==1)
	for (int iBoundaryLambda=0;iBoundaryLambda<2;iBoundaryLambda++)
	{
		for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
		{
			vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
			if (iCurrent==0)
			{
				BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
			}
			else
			{
				ComputeFlexibleRightHandSide(iBoundaryLambda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

			//compute Rotation for Handle+ROI+Anchor
			if (iCurrent!=iIterNum)
			{
				ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			}
		}
		if (iBoundaryLambda==0)
		{
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			std::ofstream out0("00Linearflexible.obj",ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out0,Mesh);

			BackUpMeshGeometry(Mesh,Lambda0Pos);
			RestoreMeshGeometry(Mesh,OldPos);
		}
		else
		{
			BackUpMeshGeometry(Mesh,Lambda1Pos);
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			std::ofstream out1("10Linearflexible.obj",ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out1,Mesh);
		}
	}
	TAUCSSolver.TAUCSClear();
	
	//do the linear interpolation
	for (int i=0;i<iInterpoNum;i++)
	{
		vector<Point_3> CurrentPos;
		for (unsigned int j=0;j<Lambda0Pos.size();j++)
		{
			Point_3 Lambda0Point=Lambda0Pos.at(j);
			Point_3 Lambda1Point=Lambda1Pos.at(j);
			Point_3 CurrentPoint(Lambda0Point.x()+(Lambda1Point.x()-Lambda0Point.x())*(i+1)/(iInterpoNum+1),
				Lambda0Point.y()+(Lambda1Point.y()-Lambda0Point.y())*(i+1)/(iInterpoNum+1),
				Lambda0Point.z()+(Lambda1Point.z()-Lambda0Point.z())*(i+1)/(iInterpoNum+1));
			CurrentPos.push_back(CurrentPoint);
		}
		RestoreMeshGeometry(Mesh,CurrentPos);
		//output
		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
		QString FileName;
		FileName.sprintf("%d%d%s",0,(i+1),"Linearflexible.obj");
		std::ofstream out(FileName.toUtf8().constData(),ios_base::out | ios_base::trunc);
		print_polyhedron_wavefront(out,Mesh);
	}

	RestoreMeshGeometry(Mesh,OldPos);
}

void CDeformationAlgorithm::FlexibleLambdaInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh, 
														vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
														vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
														vector<Point_3>& vecDeformCurvePoint3d)
{
	if (vecAnchorVertices.empty())//the whole mesh involves in the computation
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
		vector<Vertex_handle> temp=vecHandleNb;
		temp.insert(temp.end(),ROIVertices.begin(),ROIVertices.end());
		temp.insert(temp.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
		if (iType==1)
		{
			GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);
		}
		else
		{
			GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(temp,iType);
		}
	}

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);
	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<Point_3> OldPos;
	BackUpMeshGeometry(Mesh,OldPos);

	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	//compute the inbetween results
	for (int iLambda=0;iLambda<(2+iInterpoNum);iLambda++)
	{
		double dLambda=0.0+1.0*(double)iLambda/(double)(iInterpoNum+1);
		for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
		{
			vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
			if (iCurrent==0)
			{
				BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
			}
			else
			{
				ComputeFlexibleRightHandSide(dLambda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

			//compute Rotation for Handle+ROI+Anchor
			if (iCurrent!=iIterNum)
			{
				ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			}
		}
		if (dLambda!=1.0)
		{
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			QString FileName;
			FileName.sprintf("%d%d%s",0,iLambda,"Lambdaflexible.obj");
			std::ofstream out(FileName.toUtf8().constData(),ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out,Mesh);

			RestoreMeshGeometry(Mesh,OldPos);
		}
		else
		{
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			std::ofstream out("10Lambdaflexible.obj",ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out,Mesh);
		}
	}
	TAUCSSolver.TAUCSClear();

	RestoreMeshGeometry(Mesh,OldPos);
}

void CDeformationAlgorithm::FlexibleLaplacianInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh, 
														   vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
														   vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
														   vector<Point_3>& vecDeformCurvePoint3d)
{
	vector<Vertex_handle> AllVertices=vecHandleNb;
	AllVertices.insert(AllVertices.end(),ROIVertices.begin(),ROIVertices.end());
	AllVertices.insert(AllVertices.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	if (iType==1)
	{
		GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(AllVertices);
	}
	else
	{
		GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(AllVertices,iType);
	}

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);
	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<Point_3> OldPos;
	BackUpMeshGeometry(Mesh,OldPos);

	vector<Vector_3> InitialLaplacian,Lambda0Laplacian,Lambda1Laplacian;
	BackUpMeshLaplacian(iType,AllVertices,InitialLaplacian);

	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	//compute the two boundary result(lambda==0 and lambda==1)
	for (int iBoundaryLambda=0;iBoundaryLambda<2;iBoundaryLambda++)
	{
		for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
		{
			vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
			if (iCurrent==0)
			{
				BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
			}
			else
			{
				ComputeFlexibleRightHandSide(iBoundaryLambda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
					vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

			//compute Rotation for Handle+ROI+Anchor
			if (iCurrent!=iIterNum)
			{
				ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
				ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			}
		}
		if (iBoundaryLambda==0)
		{
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			std::ofstream out0("00Laplacianflexible.obj",ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out0,Mesh);

			if (iType==1)
			{
				GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(AllVertices);
			}
			else
			{
				GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(AllVertices,iType);
			}
			BackUpMeshLaplacian(iType,AllVertices,Lambda0Laplacian);
			RestoreMeshGeometry(Mesh,OldPos);
			RestoreMeshLaplacian(iType,AllVertices,InitialLaplacian);
		}
		else
		{
			if (iType==1)
			{
				GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(AllVertices);
			}
			else
			{
				GeometryAlgorithm::ComputeCGALMeshWeightedLaplacian(AllVertices,iType);
			}
			BackUpMeshLaplacian(iType,AllVertices,Lambda1Laplacian);
			//output
			OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
			std::ofstream out1("10Laplacianflexible.obj",ios_base::out | ios_base::trunc);
			print_polyhedron_wavefront(out1,Mesh);

			RestoreMeshGeometry(Mesh,OldPos);
		}
	}
	
	//do the linear interpolation
	for (int i=0;i<iInterpoNum;i++)
	{
		//compute new Laplacian
		vector<Vector_3> CurrentLaplacian;
		for (unsigned int j=0;j<Lambda0Laplacian.size();j++)
		{
			Vector_3 Lambda0Lap=Lambda0Laplacian.at(j);
			Vector_3 Lambda1Lap=Lambda1Laplacian.at(j);
			Vector_3 CurrentLap(Lambda0Lap.x()+(Lambda1Lap.x()-Lambda0Lap.x())*(i+1)/(iInterpoNum+1),
				Lambda0Lap.y()+(Lambda1Lap.y()-Lambda0Lap.y())*(i+1)/(iInterpoNum+1),
				Lambda0Lap.z()+(Lambda1Lap.z()-Lambda0Lap.z())*(i+1)/(iInterpoNum+1));
			CurrentLaplacian.push_back(CurrentLap);
		}
		RestoreMeshLaplacian(iType,AllVertices,CurrentLaplacian);
		//calculate new mesh with given Laplacian
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
			vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int j=0;j<3;j++)
		{
			RightHandSide.at(j).insert(RightHandSide.at(j).end(),AnchorRightHandSide.at(j).begin(),
				AnchorRightHandSide.at(j).end());
			RightHandSide.at(j).insert(RightHandSide.at(j).end(),HandleRightHandSide.at(j).begin(),
				HandleRightHandSide.at(j).end());
		}
		vector<vector<double> > Result;
		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);
		if (bResult)
		{
			//vecDeformCurvePoint3d.clear();
			//calculated result of handle 
			for (unsigned int j=0;j<vecHandleNb.size();j++)
			{
				vecHandleNb.at(j)->point()=Point_3(Result.at(0).at(j),Result.at(1).at(j),Result.at(2).at(j));
			}
			//calculated result of ROI 
			for (unsigned int j=0;j<ROIVertices.size();j++)
			{
				ROIVertices.at(j)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+j),
					Result.at(1).at(vecHandleNb.size()+j),
					Result.at(2).at(vecHandleNb.size()+j));
			}
			//calculated result of anchor 
			for (unsigned int j=0;j<vecAnchorVertices.size();j++)
			{
				vecAnchorVertices.at(j)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+j),
					Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+j),
					Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+j));
			}
		}

		//output
		OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
		QString FileName;
		FileName.sprintf("%d%d%s",0,(i+1),"Laplacianflexible.obj");
		std::ofstream out(FileName.toUtf8().constData(),ios_base::out | ios_base::trunc);
		print_polyhedron_wavefront(out,Mesh);
		//restore mesh geometry
		RestoreMeshGeometry(Mesh,OldPos);
	}
	TAUCSSolver.TAUCSClear();
}


///////////////////////////////////////////////////////////////////////////////////////////////
void CDeformationAlgorithm::FlexibleTransPropDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
													vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
													vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
													vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale)
{
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,bTestIsoScale);
		}
	}

	TAUCSSolver.TAUCSClear();
}



void CDeformationAlgorithm::FlexibleImpDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
											  vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
											  vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
											  vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale)
{
	cout<<"num of handle: "<<vecHandleNb.size()<<endl;
	cout<<"num of roi: "<<ROIVertices.size()<<endl;
	cout<<"num of anchor: "<<vecAnchorVertices.size()<<endl;

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	vector<Vertex_handle> vecHandleROI=vecHandleNb;
	vecHandleROI.insert(vecHandleROI.end(),ROIVertices.begin(),ROIVertices.end());
	vector<Vertex_handle> vecALL=vecHandleROI;
	vecALL.insert(vecALL.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	//used for the second step
	BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);

	//Mi[V0 ... VN]* = [Vi-VBar,Vj1-VBar,...,Vjn-VBar]*
	//Mi [V0' ... VN']* = Mi [V0 ... VN]* Ti*
	//let A=Mi [V0 ... VN]* = U Sigma V*
	//let B=V Sigma+ U* 
	//Ti*= B Mi [V0' ... VN']*
	//(Li-[deltaX,deltaY,deltaZ] B Mi) [V0' ... VN']* = [0...0]

	//matrix B=V Sigma+ U*,also useful for the second step
	vector<GeneralMatrix> vecCoeffMat;
	vector<vector<double> > AnchorRightHandSide,HandleRightHandSide;
	FlexibleImpFirstStep(iType,vecHandleROI,vecALL,vecAnchorVertices,vecDeformCurvePoint3d,LaplacianMatrix,AnchorConstraintMatrix,HandleConstraintMatrix,
		AnchorRightHandSide,HandleRightHandSide,vecCoeffMat);

	//compute righthand side of second system
	//compute deformation gradient 
	//vecCoeffMat (B)
	//vecVectorMat -- need update
	//compute its transpose
	//extract rotation and scale
	//compute RHS
	//solve
//	dLamda=0;
//	FlexibleImpSecondStep(iType,dLamda,vecHandleROI,vecALL,LaplacianMatrix,AnchorConstraintMatrix,HandleConstraintMatrix,
//		AnchorRightHandSide,HandleRightHandSide,vecCoeffMat);

}

void CDeformationAlgorithm::FlexibleImpFirstStep(int iType,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL, 
												 vector<Vertex_handle>& vecAnchorVertices,vector<Point_3>& vecDeformCurvePoint3d,
												 SparseMatrix& LaplacianMatrix, SparseMatrix& AnchorConstraintMatrix,SparseMatrix& HandleConstraintMatrix,
												 vector<vector<double> >& AnchorRightHandSide, vector<vector<double> >& HandleRightHandSide,
												 vector<GeneralMatrix>& vecCoeffMat)
{
	//get matrix converting coordinates to vectors at each vertex (Mi)
	//vector<GeneralMatrix> vecCo2VecMat;
	//vecCo2VecMat.resize(vecHandleROI.size());
	//GetCo2VecMatrix(vecHandleROI,vecALL,vecCo2VecMat);

	//get the vector matrix at each vertex (A=Mi [V0 ... VN]*)
	vector<GeneralMatrix> vecVectorMat;
	vecVectorMat.resize(vecHandleROI.size());
	GetVecMatrix(vecHandleROI,vecVectorMat);

	//test if vector matrix is right
	//VerifyVecMatrix(vecCo2VecMat,vecALL,vecVectorMat);

	//do svd of vector matrix (A=U Sigma V*) and get coefficient matrix (B)
	FactorizeVecMatrix(vecVectorMat,vecCoeffMat);

	//get second part of system matrix ([deltaX,deltaY,deltaZ] B Mi)
	//may have memory problem,quite slow!!!
	SparseMatrix SubSysMat(LaplacianMatrix.NRows());
	SubSysMat.m=LaplacianMatrix.NCols();
	GetSubSysMat(iType,vecHandleROI,vecALL,vecCoeffMat,SubSysMat);

	SparseMatrix SysMat=LaplacianMatrix-SubSysMat;

	SparseMatrix LeftHandMatrixA=SysMat;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());


//	CMath TAUCSSolver;
//	SparseMatrix AT(LeftHandMatrixA.NCols());
//	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//compute righthand side of first system
	vector<vector<double> > LaplacianRightHandSide;
	GetFirstRHS(vecHandleROI,vecAnchorVertices,
		vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
	vector<vector<double> > RightHandSide=LaplacianRightHandSide;
	for (int i=0;i<3;i++)
	{
		RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
			AnchorRightHandSide.at(i).end());
		RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
			HandleRightHandSide.at(i).end());
	}
	//solve
	vector<vector<double> > Result;
//	bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);
	bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
	if (bResult)
	{
		//calculated result of handle 
		for (unsigned int i=0;i<vecALL.size();i++)
		{
			vecALL.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
		}
	}
//	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::FlexibleImpSecondStep(int iType,double dLambda,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL, 
												  SparseMatrix& LaplacianMatrix,SparseMatrix& AnchorConstraintMatrix,
												  SparseMatrix& HandleConstraintMatrix,vector<vector<double>>& AnchorRightHandSide,
												  vector<vector<double>>& HandleRightHandSide,vector<GeneralMatrix>& vecCoeffMat)
{
	//get the vector matrix at each vertex (Mi [V0' ... VN']*)
	vector<GeneralMatrix> vecVectorMat;
	vecVectorMat.resize(vecHandleROI.size());
	GetVecMatrix(vecHandleROI,vecVectorMat);
	//get deformation gradienet	Ti through Ti*= B Mi [V0' ... VN']*
	vector<GeneralMatrix> vecDGMat;
	GetDGMat(vecCoeffMat,vecVectorMat,vecDGMat);
	//extract rotation and uniform scale from deformation gradient Ti
	ExtractRotScal(vecDGMat,vecHandleROI);
	//compute new right hand side
	vector<vector<double> > LaplacianRightHandSide;
	GetSecondRHS(iType,dLambda,vecHandleROI,LaplacianRightHandSide);

	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//compute righthand side of second system
	vector<vector<double> > RightHandSide=LaplacianRightHandSide;
	for (int i=0;i<3;i++)
	{
		RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
			AnchorRightHandSide.at(i).end());
		RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
			HandleRightHandSide.at(i).end());
	}
	//solve
	vector<vector<double> > Result;
	bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);
	if (bResult)
	{
		//calculated result of handle 
		for (unsigned int i=0;i<vecALL.size();i++)
		{
			vecALL.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
		}
	}
	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::GetCo2VecMatrix(vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL,vector<GeneralMatrix>& vecCo2VecMat)
{
	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		vecCo2VecMat.at(i).ReSize(vecHandleROI.at(i)->vertex_degree()+1,vecALL.size());
		//get the indices of the vertex and its neighbors in vecROI
		//can be further improved
		vector<int> vecInd;
		vector<Vertex_handle>::iterator Iter=find(vecHandleROI.begin(),vecHandleROI.end(),vecHandleROI.at(i));
		assert(Iter!=vecHandleROI.end());
		vecInd.push_back(distance(vecHandleROI.begin(),Iter));
		Halfedge_around_vertex_circulator Havc=vecHandleROI.at(i)->vertex_begin();
		do 
		{
			Iter=find(vecALL.begin(),vecALL.end(),Havc->opposite()->vertex());
			assert(Iter!=vecALL.end());
			vecInd.push_back(distance(vecALL.begin(),Iter));
			Havc++;
		} while(Havc!=vecHandleROI.at(i)->vertex_begin());
		assert(vecInd.size()==vecCo2VecMat.at(i).nRow());

		//build Mi
		for (unsigned int j=0;j<vecInd.size();j++)
		{
			for (unsigned int k=0;k<vecInd.size();k++)
			{
				if (j==0)//Vi-VBar
				{
					if (k==0)
					{
						vecCo2VecMat.at(i)[j][vecInd.at(k)]=1;
					}
					else
					{
						vecCo2VecMat.at(i)[j][vecInd.at(k)]=-1.0/(double)vecHandleROI.at(i)->vertex_degree();
					}
				}
				else//Vj-VBar (j \in N(i))
				{
					if ((k==j)&&(k!=0))
					{
						vecCo2VecMat.at(i)[j][vecInd.at(k)]=1.0-1.0/(double)vecHandleROI.at(i)->vertex_degree();
					}
					else if (k!=0)
					{
						vecCo2VecMat.at(i)[j][vecInd.at(k)]=-1.0/(double)vecHandleROI.at(i)->vertex_degree();
					}
				}
			}
		}
	}
}

void CDeformationAlgorithm::GetVecMatrix(vector<Vertex_handle>& vecHandleROI,vector<GeneralMatrix>& vecVectorMat)
{
	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		vecVectorMat.at(i).ReSize(vecHandleROI.at(i)->vertex_degree()+1,3);
		//compute the center of neighbors
		double dMeanX,dMeanY,dMeanZ;
		dMeanX=dMeanY=dMeanZ=0;
		Halfedge_around_vertex_circulator Havc=vecHandleROI.at(i)->vertex_begin();
		do 
		{
			dMeanX=dMeanX+Havc->opposite()->vertex()->point().x();
			dMeanY=dMeanY+Havc->opposite()->vertex()->point().y();
			dMeanZ=dMeanZ+Havc->opposite()->vertex()->point().z();
			Havc++;
		} while(Havc!=vecHandleROI.at(i)->vertex_begin());
		dMeanX=dMeanX/(double)vecHandleROI.at(i)->vertex_degree();
		dMeanY=dMeanY/(double)vecHandleROI.at(i)->vertex_degree();
		dMeanZ=dMeanZ/(double)vecHandleROI.at(i)->vertex_degree();

		//fill in matrix
		vecVectorMat.at(i)[0][0]=vecHandleROI.at(i)->point().x()-dMeanX;
		vecVectorMat.at(i)[0][1]=vecHandleROI.at(i)->point().y()-dMeanY;
		vecVectorMat.at(i)[0][2]=vecHandleROI.at(i)->point().z()-dMeanZ;
		int iTempRow=1;
		Havc=vecHandleROI.at(i)->vertex_begin();
		do 
		{
			vecVectorMat.at(i)[iTempRow][0]=Havc->opposite()->vertex()->point().x()-dMeanX;
			vecVectorMat.at(i)[iTempRow][1]=Havc->opposite()->vertex()->point().y()-dMeanY;
			vecVectorMat.at(i)[iTempRow][2]=Havc->opposite()->vertex()->point().z()-dMeanZ;
			iTempRow++;
			Havc++;
		} while(Havc!=vecHandleROI.at(i)->vertex_begin());
		assert(iTempRow==vecVectorMat.at(i).nRow());
	}
}

void CDeformationAlgorithm::VerifyVecMatrix(vector<GeneralMatrix>& vecCo2VecMat,vector<Vertex_handle>& vecALL,vector<GeneralMatrix>& vecVectorMat)
{
	GeneralMatrix CoMat(vecALL.size(),3);
	for (unsigned int i=0;i<vecALL.size();i++)
	{
		CoMat[i][0]=vecALL.at(i)->point().x();
		CoMat[i][1]=vecALL.at(i)->point().y();
		CoMat[i][2]=vecALL.at(i)->point().z();
	}

	for (unsigned int i=0;i<vecCo2VecMat.size();i++)
	{
		GeneralMatrix ResultMat=vecCo2VecMat.at(i)*CoMat;
		GeneralMatrix DifMat=ResultMat-vecVectorMat.at(i);

		for (int j=0;j<DifMat.nRow();j++)
		{
			for (int k=0;k<DifMat.nCol();k++)
			{
				cout<<DifMat[j][k]<<" ";
			}
			cout<<endl;
		}
		cout<<endl;
	}
}

void CDeformationAlgorithm::FactorizeVecMatrix(vector<GeneralMatrix>& vecVectorMat,vector<GeneralMatrix>& vecCoeffMat)
{
	for (unsigned int i=0;i<vecVectorMat.size();i++)
	{
		GeneralMatrix OutputU(vecVectorMat.at(i).nRow(),vecVectorMat.at(i).nRow());
		GeneralMatrix OutputSigma(vecVectorMat.at(i).nRow(),vecVectorMat.at(i).nCol());
		GeneralMatrix OutputV(vecVectorMat.at(i).nCol(),vecVectorMat.at(i).nCol());
		CMath::ComputeSVD(vecVectorMat.at(i),OutputU,OutputSigma,OutputV);

		//pseudo inverse of sigma
		for (int j=0;j<OutputSigma.nRow();j++)
		{
			for (int k=0;k<OutputSigma.nCol();k++)
			{
				if (OutputSigma[j][k]!=0)
				{
					OutputSigma[j][k]=1.0/OutputSigma[j][k];
				}
			}
		}

		OutputSigma.T_Self();
		OutputU.T_Self();

		vecCoeffMat.push_back(OutputV*OutputSigma*OutputU);
	}
}

void CDeformationAlgorithm::GetSubSysMat(int iType,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL, 
										 vector<GeneralMatrix>& vecCoeffMat,SparseMatrix& SubSysMat)
{
	float MiTime=0;
	float BuildTime=0;
	float FillTime=0;
	float SubSysTime=0;
	float Multip1=0;
	float Multip2=0;

	GeometryAlgorithm::SetOrderForVer(vecALL);

	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		clock_t MiBegin=clock();   

		GeneralMatrix Co2VecMat(vecHandleROI.at(i)->vertex_degree()+1,vecALL.size());

		clock_t BuildEnd=clock();   
		BuildTime=BuildTime+BuildEnd-MiBegin;


		//build Mi
		Co2VecMat[0][i]=1;//i==vecHandleROI.at(i)->GetReserved()
		int iRowTemp=1;
		Halfedge_around_vertex_circulator Havc=vecHandleROI.at(i)->vertex_begin();
		do 
		{
			Vertex_handle VhNeighbor=Havc->opposite()->vertex();
			for (int j=0;j<(int)vecHandleROI.at(i)->vertex_degree()+1;j++)
			{
				if (j!=iRowTemp)
				{
					//fill in other row
					Co2VecMat[j][VhNeighbor->GetReserved()]=-1.0/(double)vecHandleROI.at(i)->vertex_degree();
				}
				else
				{
					//fill in current row
					Co2VecMat[iRowTemp][VhNeighbor->GetReserved()]=1.0-1.0/(double)vecHandleROI.at(i)->vertex_degree();
				}
			}
			iRowTemp++;
			Havc++;
		} while(Havc!=vecHandleROI.at(i)->vertex_begin());



		clock_t MiEnd=clock();   
		MiTime=MiTime+MiEnd-MiBegin;

		FillTime=FillTime+MiEnd-BuildEnd;

		
		GeneralMatrix LpCo(1,3);
		if (iType==1)
		{
			LpCo[0][0]=vecHandleROI.at(i)->GetUniformLaplacian().x();
			LpCo[0][1]=vecHandleROI.at(i)->GetUniformLaplacian().y();
			LpCo[0][2]=vecHandleROI.at(i)->GetUniformLaplacian().z();
		}

		clock_t MultipBegin=clock();   

//		GeneralMatrix CurrentRow=LpCo*vecCoeffMat.at(i)*Co2VecMat;
		GeneralMatrix GMTemp=LpCo*vecCoeffMat.at(i);

		clock_t Multip1End=clock();   
		Multip1=Multip1+Multip1End-MultipBegin;

		GeneralMatrix CurrentRow=GMTemp*Co2VecMat;

		clock_t Multip2End=clock();   
		Multip2=Multip2+Multip2End-Multip1End;

		assert(CurrentRow.nRow()==1);
		assert(CurrentRow.nCol()==SubSysMat.NCols());

		for (int j=0;j<CurrentRow.nCol();j++)
		{
			if (CurrentRow[0][j]!=0)
			{
				SubSysMat[i][j]=CurrentRow[0][j];
			}
		}

		clock_t SubSysEnd=clock();   
		SubSysTime=SubSysTime+SubSysEnd-MiEnd;
	}
	cout<<"Mi Time: "<<MiTime<<endl;
	cout<<"Build Time: "<<BuildTime<<endl;
	cout<<"Fill Time: "<<FillTime<<endl;
	cout<<"SubSys Time: "<<SubSysTime<<endl;
	cout<<"Multip1 Time: "<<Multip1<<endl;
	cout<<"Multip2 Time: "<<Multip2<<endl;
}

void CDeformationAlgorithm::GetFirstRHS(vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecAnchorVertices, 
										vector<Point_3>& vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide, 
										vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		Laplacian[0].push_back(0);
		Laplacian[1].push_back(0);
		Laplacian[2].push_back(0);
	}
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
		vecHandleROI.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CDeformationAlgorithm::GetDGMat(vector<GeneralMatrix>& vecCoeffMat,vector<GeneralMatrix>& vecVectorMat,vector<GeneralMatrix>& vecDGMat)
{
	assert(vecCoeffMat.size()==vecVectorMat.size());
	for (unsigned int i=0;i<vecCoeffMat.size();i++)
	{
		GeneralMatrix DGMatTran=vecCoeffMat.at(i)*vecVectorMat.at(i);
		DGMatTran.T_Self();
		vecDGMat.push_back(DGMatTran);
	}
}

void CDeformationAlgorithm::ExtractRotScal(vector<GeneralMatrix>& vecDGMat,vector<Vertex_handle>& vecHandleROI)
{
	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		GeneralMatrix OutputU(vecDGMat.at(i).nRow(),vecDGMat.at(i).nRow());
		GeneralMatrix OutputSigma(vecDGMat.at(i).nRow(),vecDGMat.at(i).nCol());
		GeneralMatrix OutputV(vecDGMat.at(i).nCol(),vecDGMat.at(i).nCol());
		CMath::ComputeSVD(vecDGMat.at(i),OutputU,OutputSigma,OutputV);
		//rotation=U*VT
		GeneralMatrix GMRotMat=OutputU*(OutputV.T_Self());
		vector<double> vecRotMat;
		for (int j=0;j<3;j++)
		{
			for (int k=0;k<3;k++)
			{
				vecRotMat.push_back(GMRotMat[j][k]);
			}
		}
		vecHandleROI.at(i)->SetRigidDeformRotationMatrix(vecRotMat);
		//scale
		double dScale=sqrt((OutputSigma[0][0]*OutputSigma[0][0]+OutputSigma[1][1]*OutputSigma[1][1]
							+OutputSigma[2][2]*OutputSigma[2][2])/3.0);
		vecHandleROI.at(i)->SetScaleFactor(Vector_3(dScale,dScale,dScale));
	}
}

void CDeformationAlgorithm::GetSecondRHS(int iType,double dLamda,vector<Vertex_handle>& vecHandleROI,vector<vector<double> >& LaplacianRightHandSide)
{
	vector<double> Laplacian[3];
	for (unsigned int i=0;i<vecHandleROI.size();i++)
	{
		vector<double> CurrentRotationMatrix=vecHandleROI.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleROI.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleROI.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleROI.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*vecHandleROI.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*vecHandleROI.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*vecHandleROI.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=vecHandleROI.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecHandleROI.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
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
				dCurrentWeight=vecHandleROI.at(i)->GetEdgeWeights().at(iIndex);
			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(vecHandleROI.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(vecHandleROI.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(vecHandleROI.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecHandleROI.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}
	assert(Laplacian[0].size()==vecHandleROI.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
	}
}


















































///////////////////////////////////////////////////////////////
void CDeformationAlgorithm::TestFlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, set<int>& setEdgeHandleNb,
											   vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
											   vector<Point_3>& vecDeformCurvePoint3d)
{
	cout<<"num of handle: "<<setEdgeHandleNb.size()<<endl;
	cout<<"num of roi: "<<ROIVertices.size()<<endl;
	cout<<"num of anchor: "<<vecAnchorVertices.size()<<endl;

	vector<Vertex_handle> vecHandleNb;
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(1);
	TESTGetConstraintsMatrixToNaiveLaplacian(setEdgeHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);

	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
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

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CDeformationAlgorithm::TESTGetConstraintsMatrixToNaiveLaplacian(set<int> setHandleIndex,vector<Vertex_handle> ROIVertices, 
																	 vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix, 
																	 SparseMatrix& HandleConstraintMatrix)
{
	int iColumn=(int)(ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
	AnchorConstraintMatrix.m=iColumn;
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
				//do nothing
			}
		}
	}

	int iHandleRow=1;
	HandleConstraintMatrix.m=iColumn;
	double dDegree=setHandleIndex.size();
	for (int i=0;i<iHandleRow;i++)
	{

		for (int j=0;j<iColumn;j++)
		{
			set<int>::iterator pFind=setHandleIndex.find(j);
			if (pFind!=setHandleIndex.end())
			{
				HandleConstraintMatrix[i][j]=CONSTRAINED_HANDLE_WEIGHT*(1.0/dDegree);
			}
			else
			{
				//do nothing
			}
		}
	}
}