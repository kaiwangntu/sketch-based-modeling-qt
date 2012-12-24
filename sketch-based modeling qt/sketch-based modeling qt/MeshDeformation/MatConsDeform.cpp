#include "MatConsDeform.h"
#include "DeformationAlgorithm.h"
#include "WedgeEdgeBasedDeform.h"
#include "../OBJHandle.h"

CMatConsDeform::CMatConsDeform(void)
{
}

CMatConsDeform::~CMatConsDeform(void)
{
}

void CMatConsDeform::SetUniformMaterial(double dUniMat,KW_Mesh& Mesh,vector<Vertex_handle>& vecHandleNb, 
										vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices)
{
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vecHandleNb.at(i)->SetMaterial(dUniMat);
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		ROIVertices.at(i)->SetMaterial(dUniMat);
	}
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		vecAnchorVertices.at(i)->SetMaterial(dUniMat);
	}
}

void CMatConsDeform::SetHarmonicMaterial(int iType,KW_Mesh& Mesh, 
										 vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb, 
										 vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices)
{
	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	CDeformationAlgorithm::ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
	ComputeHarmonicRightHandSide(iType,vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
	LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);

	vector<vector<double> > RightHandSide=LaplacianRightHandSide;
	RightHandSide.at(0).insert(RightHandSide.at(0).end(),AnchorRightHandSide.at(0).begin(),
			AnchorRightHandSide.at(0).end());
	RightHandSide.at(0).insert(RightHandSide.at(0).end(),HandleRightHandSide.at(0).begin(),
			HandleRightHandSide.at(0).end());

	vector<vector<double> > Result;
	bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

	if (bResult)
	{
		//calculated result of handle 
		for (unsigned int i=0;i<vecHandleNb.size();i++)
		{
			if (Result.at(0).at(i)>1.0)
			{
				Result.at(0).at(i)=1.0;
			}
			vecHandleNb.at(i)->SetMaterial(Result.at(0).at(i));
		}
		//calculated result of ROI 
		for (unsigned int i=0;i<ROIVertices.size();i++)
		{
			if (Result.at(0).at(vecHandleNb.size()+i)>1.0)
			{
				Result.at(0).at(vecHandleNb.size()+i)=1.0;
			}
			ROIVertices.at(i)->SetMaterial(Result.at(0).at(vecHandleNb.size()+i));
		}
		//calculated result of anchor 
		for (unsigned int i=0;i<vecAnchorVertices.size();i++)
		{
			if (Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i)>1.0)
			{
				Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i)=1.0;
			}
			vecAnchorVertices.at(i)->SetMaterial(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i));
		}
	}
}

void CMatConsDeform::SetMatColor(vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
									 vector<Vertex_handle>& vecAnchorVertices)
{
	//compute and set
	//lambda==0 ->blue, lambda==1 ->red
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		double dBlueCol=vecHandleNb.at(i)->GetMaterial();
		double dRedCol=1.0-dBlueCol;
		vector<double> vecNewColor;
		vecNewColor.push_back(dRedCol);
		vecNewColor.push_back(0);//vecHandleNb.at(i)->GetColor().at(1)
		vecNewColor.push_back(dBlueCol);
		vecNewColor.push_back(1.0);
		vecHandleNb.at(i)->SetColor(vecNewColor);
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		double dBlueCol=ROIVertices.at(i)->GetMaterial();
		double dRedCol=1.0-dBlueCol;
		vector<double> vecNewColor;
		vecNewColor.push_back(dRedCol);
		vecNewColor.push_back(0);//ROIVertices.at(i)->GetColor().at(1)
		vecNewColor.push_back(dBlueCol);
		vecNewColor.push_back(1.0);
		ROIVertices.at(i)->SetColor(vecNewColor);
	}
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		double dBlueCol=vecAnchorVertices.at(i)->GetMaterial();
		double dRedCol=1.0-dBlueCol;
		vector<double> vecNewColor;
		vecNewColor.push_back(dRedCol);
		vecNewColor.push_back(0);//vecAnchorVertices.at(i)->GetColor().at(1)
		vecNewColor.push_back(dBlueCol);
		vecNewColor.push_back(1.0);
		vecAnchorVertices.at(i)->SetColor(vecNewColor);
	}
}

void CMatConsDeform::SetMatColor(KW_Mesh& Mesh)
{
	//compute and set
	//lambda==0 ->blue, lambda==1 ->red
	for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)	
	{
		if (i->GetMaterial()<0 || i->GetMaterial()>1)
		{
//			DBWindowWrite("mat value: %f\n",i->GetMaterial());
			vector<double> vecDefaultColor;
			vecDefaultColor.push_back(0.5);
			vecDefaultColor.push_back(0.9);
			vecDefaultColor.push_back(0.4);
			vecDefaultColor.push_back(1.0);
			i->SetColor(vecDefaultColor);
			continue;
		}
		double dBlueCol=i->GetMaterial();
		double dRedCol=1.0-dBlueCol;
		vector<double> vecNewColor;
		vecNewColor.push_back(dRedCol);
		vecNewColor.push_back(0);
		vecNewColor.push_back(dBlueCol);
		vecNewColor.push_back(1.0);
		i->SetColor(vecNewColor);
	}
}

void CMatConsDeform::SetMatAndColor(vector<Vertex_handle>& vecVer,double dMaterial)
{
	//compute color
	double dBlueCol=dMaterial;
	double dRedCol=1.0-dBlueCol;
	vector<double> vecNewColor;
	vecNewColor.push_back(dRedCol);
	vecNewColor.push_back(0);//vecHandleNb.at(i)->GetColor().at(1)
	vecNewColor.push_back(dBlueCol);
	vecNewColor.push_back(1.0);

	//lambda==0 ->blue, lambda==1 ->red
	for (unsigned int i=0;i<vecVer.size();i++)
	{
		//set material
		vecVer.at(i)->SetMaterial(dMaterial);
		//set color
		vecVer.at(i)->SetColor(vecNewColor);
	}
}

void CMatConsDeform::ComputeHarmonicRightHandSide(int iType,vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle> vecHandleNb, 
												  vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices, 
												  vector<vector<double> >& LaplacianRightHandSide,vector<vector<double> >& AnchorRightHandSide, 
												  vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian,Anchor,Handle;
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		if (iType==1)
		{
			Laplacian.push_back(0.0);
		}
		else
		{
			Laplacian.push_back(0.0);
		}
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		if (iType==1)
		{
			Laplacian.push_back(0.0);
		} 
		else
		{
			Laplacian.push_back(0.0);
		}
	}

	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Anchor.push_back(0.0);
	}
	for (unsigned int i=0;i<vecHandlePoint.size();i++)
	{
		Handle.push_back(1.0);
	}

	assert(Laplacian.size()+Anchor.size()+Handle.size()==
		vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size()+vecHandlePoint.size());

	LaplacianRightHandSide.push_back(Laplacian);
	AnchorRightHandSide.push_back(Anchor);
	HandleRightHandSide.push_back(Handle);
}

void CMatConsDeform::MatConsVerFlexibleDeform(int iType,int iIterNum,KW_Mesh& Mesh, vector<HandlePointStruct>& vecHandlePoint,
										   vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
										   vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale)
{
	vector<Vertex_handle> temp=vecHandleNb;
	temp.insert(temp.end(),ROIVertices.begin(),ROIVertices.end());
	temp.insert(temp.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(temp);

	SparseMatrix LaplacianMatrix(vecHandleNb.size()+ROIVertices.size());
	CDeformationAlgorithm::ComputeLaplacianMatrix(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	CDeformationAlgorithm::GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
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
			CDeformationAlgorithm::BackUpEdgeVectorsForRigidDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			CDeformationAlgorithm::ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeMatConsVerFlexibleRightHandSide(iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
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
			CDeformationAlgorithm::ComputeRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			CDeformationAlgorithm::ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices,bTestIsoScale);
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CMatConsDeform::MatConsEdgeFlexibleDeform(int iType,int iIterNum,KW_Mesh& Mesh, vector<HandlePointStruct>& vecHandlePoint,
											   vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
											   vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale)
{
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges,AnchorEdges;
	CWedgeEdgeBasedDeform::GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,AnchorEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges,AnchorNeighborEdges;
	CWedgeEdgeBasedDeform::GetEdgeTopo(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);

	//compute edge laplacian
	CWedgeEdgeBasedDeform::ComputeEdgeLaplacian(iType,ROIEdges,NeighborEdges);

	cout<<"related edge num: "<<ROIEdges.size()<<endl;
	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;

	//set the material for each ROI edge
	SetEdgeMaterial(ROIEdges);

	//get Laplacian matrix
	SparseMatrix LaplacianMatrix(ROIEdges.size());
	CWedgeEdgeBasedDeform::GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,LaplacianMatrix);

	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	CWedgeEdgeBasedDeform::GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
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
			CWedgeEdgeBasedDeform::BackUpEdgeVectorsForRigidDeform(ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			CWedgeEdgeBasedDeform::ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeMatConsEdgeFlexibleRightHandSide(iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
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
			CWedgeEdgeBasedDeform::ComputeRotationForRigidDeform(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges);
			CWedgeEdgeBasedDeform::ComputeScaleFactor(iType,ROIEdges,NeighborEdges,AnchorEdges,AnchorNeighborEdges,bTestIsoScale);
		}
	}

	TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();   
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;

}

void CMatConsDeform::ComputeMatConsVerFlexibleRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
														 vector<Point_3>& vecAnchorVertices, vector<Point_3> vecDeformCurvePoint3d,
														 vector<vector<double> >& RigidRightHandSide, vector<vector<double> >& AnchorRightHandSide,
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
			double dValue=vecHandleNb.at(i)->GetMaterial()*CurrentRigid[j]+(1.0-vecHandleNb.at(i)->GetMaterial())*CurrentRotatedLaplacian[j];
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
			double dValue=ROIVertices.at(i)->GetMaterial()*CurrentRigid[j]+(1.0-ROIVertices.at(i)->GetMaterial())*CurrentRotatedLaplacian[j];
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

void CMatConsDeform::ComputeMatConsEdgeFlexibleRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
															 vector<Point_3> AnchorConstraints, vector<Point_3> vecDeformCurvePoint3d,
															 vector<Halfedge_handle> ROIEdges, vector<vector<Halfedge_handle>> NeighborEdges, 
															 vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,
															 vector<vector<double> >& HandleRightHandSide)
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
			assert((ROIEdges.at(i)->GetMaterial()>=0.0) && (ROIEdges.at(i)->GetMaterial()<=1.0));
			double dValue=ROIEdges.at(i)->GetMaterial()*CurrentRigid[j]+(1.0-ROIEdges.at(i)->GetMaterial())*CurrentRotatedLaplacian[j];
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

void CMatConsDeform::SetEdgeMaterial(vector<Halfedge_handle>& ROIEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		double dMat0=ROIEdges.at(i)->vertex()->GetMaterial();
		double dMat1=ROIEdges.at(i)->opposite()->vertex()->GetMaterial();
		double dEdgeMat=(dMat0+dMat1)/2.0;
		ROIEdges.at(i)->SetMaterial(dEdgeMat);
	}
}

void CMatConsDeform::ExportMat(KW_Mesh& Mesh,QString MatFileName)
{
	FILE* pfile=fopen(MatFileName.toUtf8().constData(),"w");
	if (pfile!=NULL)
	{
		for (Vertex_iterator Vi=Mesh.vertices_begin();Vi!=Mesh.vertices_end();Vi++)	
		{
			fprintf(pfile,"%f\n",Vi->GetMaterial());
		}
	}
	fclose(pfile);
}

void CMatConsDeform::ImportMat(KW_Mesh& Mesh,QString MatFileName)
{
	FILE* pfile=fopen(MatFileName.toUtf8().constData(),"r");
	if (pfile!=NULL)
	{
		for (Vertex_iterator Vi=Mesh.vertices_begin();Vi!=Mesh.vertices_end();Vi++)	
		{
			float dMat=0;
			fscanf(pfile,"%f",&dMat);
			Vi->SetMaterial(dMat);
		}
	}
	fclose(pfile);
}

void CMatConsDeform::LearnMatFromSrc(int iType,KW_Mesh& Mesh,vector<QString> vecTmpName)
{
	//compute Laplacian of source model
	GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(Mesh);

	vector<vector<double>> vecvecLearnedMat;
	for (unsigned int i=0;i<vecTmpName.size();i++)
	{
		cout<<"learning from model: "<<i<<endl;
		KW_Mesh TmpModel;
		vector<double> Tempclr;
		Tempclr.push_back(0);Tempclr.push_back(0);Tempclr.push_back(0);Tempclr.push_back(0);
		OBJHandle::glmReadOBJNew(vecTmpName.at(i).toUtf8().constData(),TmpModel,false,false,Tempclr,false);
		assert(Mesh.size_of_vertices()==TmpModel.size_of_vertices());
		assert(Mesh.size_of_halfedges()==TmpModel.size_of_halfedges());
		assert(Mesh.size_of_facets()==TmpModel.size_of_facets());
		//compute Laplacian of deformed source model
		GeometryAlgorithm::ComputeCGALMeshUniformLaplacian(TmpModel);
		//compute Rotation Ri
		LearnRotation(Mesh,TmpModel);
		//compute Scale Si
		LearnScale(Mesh,TmpModel);
		//solve
		vector<double> vecLearnedMat;
		ComputeMat(Mesh,TmpModel,vecLearnedMat);
		//save
		vecvecLearnedMat.push_back(vecLearnedMat);
	}
	for (unsigned int i=0;i<vecvecLearnedMat.size();i++)
	{
		int iIndex=0;
		for (Vertex_iterator ViSrc=Mesh.vertices_begin();ViSrc!=Mesh.vertices_end();ViSrc++)	
		{
			ViSrc->SetMaterial(vecvecLearnedMat.at(i).at(iIndex));
			iIndex++;
		}
	}
}

void CMatConsDeform::LearnRotation(KW_Mesh& SrcMesh,KW_Mesh& DefModel)
{
	//compute edge vectors for source model
	GeometryAlgorithm::ComputeOldEdgeVectors(SrcMesh);
	//compute the rotation which transforms the source mesh into its deformed shape
	//almost the same with CDeformationAlgorithm::ComputeRotationForRigidDeform
	for (Vertex_iterator ViSrc=SrcMesh.vertices_begin(),ViDef=DefModel.vertices_begin();
		ViSrc!=SrcMesh.vertices_end(),ViDef!=DefModel.vertices_end();ViSrc++,ViDef++)	
	{
		vector<vector<double> > TotalMatrix,IdentityMatrix;
		vector<double> tempRow;
		for (int i=0;i<3;i++)
		{
			tempRow.push_back(0);
		}
		for (int i=0;i<3;i++)
		{
			TotalMatrix.push_back(tempRow);
		}
		IdentityMatrix=TotalMatrix;
		int iOldEdgeIndex=0;

		Halfedge_around_vertex_circulator Havc=ViDef->vertex_begin();
		do 
		{
			//compute new edge
			Vector_3 NewEdge=ViDef->point()-Havc->opposite()->vertex()->point();
			//get corresponding old edge
			Vector_3 OldEdge=ViSrc->GetOldEdgeVectors().at(iOldEdgeIndex);
			//get weight for the edge
			double dCurrentWeight;
//			if (iType==1)
//			{
				dCurrentWeight=1;
//			}
//			else
//			{
//				dCurrentWeight=ViSrc->GetEdgeWeights().at(iOldEdgeIndex);
//			}

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
			for (int i=0;i<3;i++)
			{
				for (int j=0;j<3;j++)
				{
					CurrentMatrix.at(i).at(j)=dCurrentMatrix[3*i+j];
				}
			}
			for (int i=0;i<3;i++)
			{
				for (int j=0;j<3;j++)
				{
					TotalMatrix.at(i).at(j)=TotalMatrix.at(i).at(j)+CurrentMatrix.at(i).at(j);
				}
			}
		} while(Havc!=ViDef->vertex_begin());

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
		ViSrc->SetRigidDeformRotationMatrix(RotationMatrix);
	}
}

void CMatConsDeform::LearnScale(KW_Mesh& SrcMesh,KW_Mesh& DefModel)
{
	//compute the scale which transforms Laplacian of the source mesh 
	//into that of its deformed shape
	for (Vertex_iterator ViSrc=SrcMesh.vertices_begin(),ViDef=DefModel.vertices_begin();
		ViSrc!=SrcMesh.vertices_end(),ViDef!=DefModel.vertices_end();ViSrc++,ViDef++)	
	{
		Vector_3 SrcLap=ViSrc->GetUniformLaplacian();
		Vector_3 DefLap=ViDef->GetUniformLaplacian();
		double dScale=sqrt(DefLap.squared_length())/sqrt(SrcLap.squared_length());
		ViSrc->SetScaleFactor(Vector_3(dScale,dScale,dScale));
	}
}

void CMatConsDeform::ComputeMat(KW_Mesh& SrcMesh,KW_Mesh& DefModel,vector<double>& vecLearnedMat)
{
	//compute lambda for each vertex
	//lambda*Ri*SrcRigid+(1-lambda)*SrcScale*SrcRigid*SrcLaplacian-DefLaplacian=0
	//similar to CDeformationAlgorithm::ComputeFlexibleRightHandSide
	for (Vertex_iterator ViSrc=SrcMesh.vertices_begin(),ViDef=DefModel.vertices_begin();
		ViSrc!=SrcMesh.vertices_end(),ViDef!=DefModel.vertices_end();ViSrc++,ViDef++)	
	{
		vector<double> CurrentRotationMatrix=ViSrc->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
//			if (iType==1)
//			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ViSrc->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ViSrc->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ViSrc->GetUniformLaplacian().z();
//			}
//			else
//			{
//				CurrentRotatedLaplacian[j]=
//					CurrentRotationMatrix.at(3*j+0)*ViSrc->GetWeightedLaplacian().x()
//					+CurrentRotationMatrix.at(3*j+1)*ViSrc->GetWeightedLaplacian().y()
//					+CurrentRotationMatrix.at(3*j+2)*ViSrc->GetWeightedLaplacian().z();
//			}
		}
		Vector_3 ScaleFactor=ViSrc->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=ViSrc->vertex_begin();
		do 
		{
			vector<double> NbRotationMatrix=Havc->opposite()->vertex()->GetRigidDeformRotationMatrix();
			double dCurrentWeight;
//			if (iType==1)
//			{
				dCurrentWeight=1;
//			} 
//			else
//			{
//				dCurrentWeight=ViSrc->GetEdgeWeights().at(iIndex);
//			}
			double dProduct[3];
			for (int j=0;j<3;j++)
			{
				dProduct[j]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+0)+NbRotationMatrix.at(3*j+0))*
					(ViSrc->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+1)+NbRotationMatrix.at(3*j+1))*
					(ViSrc->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*j+2)+NbRotationMatrix.at(3*j+2))*
					(ViSrc->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=ViSrc->vertex_begin());

		double dLearnedMatX,dLearnedMatY,dLearnedMatZ;//for x, y and z respectively

		//DBWindowWrite("Nume, Deno: %f,%f\n",ViDef->GetUniformLaplacian().x()-CurrentRotatedLaplacian[0],CurrentRigid[0]-CurrentRotatedLaplacian[0]);
		//DBWindowWrite("Nume, Deno: %f,%f\n",ViDef->GetUniformLaplacian().y()-CurrentRotatedLaplacian[1],CurrentRigid[1]-CurrentRotatedLaplacian[1]);
		//DBWindowWrite("Nume, Deno: %f,%f\n",ViDef->GetUniformLaplacian().z()-CurrentRotatedLaplacian[2],CurrentRigid[2]-CurrentRotatedLaplacian[2]);


		dLearnedMatX=(ViDef->GetUniformLaplacian().x()-CurrentRotatedLaplacian[0])/(CurrentRigid[0]-CurrentRotatedLaplacian[0]);
		dLearnedMatY=(ViDef->GetUniformLaplacian().y()-CurrentRotatedLaplacian[1])/(CurrentRigid[1]-CurrentRotatedLaplacian[1]);
		dLearnedMatZ=(ViDef->GetUniformLaplacian().z()-CurrentRotatedLaplacian[2])/(CurrentRigid[2]-CurrentRotatedLaplacian[2]);

		double dResultMat=sqrt(dLearnedMatX*dLearnedMatX+dLearnedMatY*dLearnedMatY+dLearnedMatZ*dLearnedMatZ/3.0);
		//handle exception
		//if (dResultMat<0)
		//{
		//	dResultMat=0;
		//}
		//else if (dResultMat>1)
		//{
		//	dResultMat=1;
		//}
		vecLearnedMat.push_back(dResultMat);
	}
}