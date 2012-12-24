#include "RSRCellDeform.h"
#include "DeformationAlgorithm.h"


void CRSRCellDeform::RSRCellDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, 
											  vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
											  vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
											  vector<Point_3>& vecDeformCurvePoint3d)
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
			ComputeRSRRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,//vecAnchorVertices,
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
			ComputeScaleFactor(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			CDeformationAlgorithm::ComputeSecondRotationForRigidDeform(iType,Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
			ComputeRSRMatrixForDeform(Mesh,vecHandleNb,ROIVertices,vecAnchorVertices);
		}
	}

	TAUCSSolver.TAUCSClear();
}

void CRSRCellDeform::ComputeScaleFactor(int iType,KW_Mesh& Mesh, 
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
		//		double dUniformScale=(dScale[0]+dScale[1]+dScale[2])/3.0;
		//		dScale[0]=dScale[1]=dScale[2]=dUniformScale;
		//store rotation matrix
		vecAllVertices.at(i)->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
	}
}

void CRSRCellDeform::ComputeRSRRightHandSide(double dLamda,int iType,
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
		vector<double> CurrentRSRMatrix=vecHandleNb.at(i)->GetRSRTransformMatrix();
		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRSRMatrix=Havc->opposite()->vertex()->GetRSRTransformMatrix();
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
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+0)+NbRSRMatrix.at(3*j+0))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+1)+NbRSRMatrix.at(3*j+1))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+2)+NbRSRMatrix.at(3*j+2))*
					(vecHandleNb.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			Laplacian[j].push_back(CurrentRigid[j]);
		}
	}


	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		vector<double> CurrentRSRMatrix=ROIVertices.at(i)->GetRSRTransformMatrix();
		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		int iIndex=0;
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<double> NbRSRMatrix=Havc->opposite()->vertex()->GetRSRTransformMatrix();
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
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+0)+NbRSRMatrix.at(3*j+0))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).x())
					+
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+1)+NbRSRMatrix.at(3*j+1))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).y())
					+
					dCurrentWeight*0.5*(CurrentRSRMatrix.at(3*j+2)+NbRSRMatrix.at(3*j+2))*
					(ROIVertices.at(i)->GetOldEdgeVectors().at(iIndex).z());
				CurrentRigid[j]=CurrentRigid[j]+dProduct[j];
			}
			Havc++;
			iIndex++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
		for (int j=0;j<3;j++)
		{
			Laplacian[j].push_back(CurrentRigid[j]);
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

void CRSRCellDeform::ComputeRSRMatrixForDeform(KW_Mesh& Mesh,
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
		vector<double> FirstRotationMatrix=vecAllVertices.at(i)->GetRigidDeformRotationMatrix();
		vector<double> SecondRotationMatrix=vecAllVertices.at(i)->GetSecondRigidDeformRotationMatrix();
		Vector_3 ScaleFactor=vecAllVertices.at(i)->GetScaleFactor();
		double R2SProduct[9];
		R2SProduct[0]=SecondRotationMatrix.at(0)*ScaleFactor.x();
		R2SProduct[1]=SecondRotationMatrix.at(1)*ScaleFactor.y();
		R2SProduct[2]=SecondRotationMatrix.at(2)*ScaleFactor.z();
		R2SProduct[3]=SecondRotationMatrix.at(3)*ScaleFactor.x();
		R2SProduct[4]=SecondRotationMatrix.at(4)*ScaleFactor.y();
		R2SProduct[5]=SecondRotationMatrix.at(5)*ScaleFactor.z();
		R2SProduct[6]=SecondRotationMatrix.at(6)*ScaleFactor.x();
		R2SProduct[7]=SecondRotationMatrix.at(7)*ScaleFactor.y();
		R2SProduct[8]=SecondRotationMatrix.at(8)*ScaleFactor.z();
		double R2SR1Product[9];
		R2SR1Product[0]=R2SProduct[0]*FirstRotationMatrix.at(0)+R2SProduct[1]*FirstRotationMatrix.at(3)+R2SProduct[2]*FirstRotationMatrix.at(6);
		R2SR1Product[1]=R2SProduct[0]*FirstRotationMatrix.at(1)+R2SProduct[1]*FirstRotationMatrix.at(4)+R2SProduct[2]*FirstRotationMatrix.at(7);
		R2SR1Product[2]=R2SProduct[0]*FirstRotationMatrix.at(2)+R2SProduct[1]*FirstRotationMatrix.at(5)+R2SProduct[2]*FirstRotationMatrix.at(8);
		R2SR1Product[3]=R2SProduct[3]*FirstRotationMatrix.at(0)+R2SProduct[4]*FirstRotationMatrix.at(3)+R2SProduct[5]*FirstRotationMatrix.at(6);
		R2SR1Product[4]=R2SProduct[3]*FirstRotationMatrix.at(1)+R2SProduct[4]*FirstRotationMatrix.at(4)+R2SProduct[5]*FirstRotationMatrix.at(7);
		R2SR1Product[5]=R2SProduct[3]*FirstRotationMatrix.at(2)+R2SProduct[4]*FirstRotationMatrix.at(5)+R2SProduct[5]*FirstRotationMatrix.at(8);
		R2SR1Product[6]=R2SProduct[6]*FirstRotationMatrix.at(0)+R2SProduct[7]*FirstRotationMatrix.at(3)+R2SProduct[8]*FirstRotationMatrix.at(6);
		R2SR1Product[7]=R2SProduct[6]*FirstRotationMatrix.at(1)+R2SProduct[7]*FirstRotationMatrix.at(4)+R2SProduct[8]*FirstRotationMatrix.at(7);
		R2SR1Product[8]=R2SProduct[6]*FirstRotationMatrix.at(2)+R2SProduct[7]*FirstRotationMatrix.at(5)+R2SProduct[8]*FirstRotationMatrix.at(8);
		//double SR1Product[9];
		//SR1Product[0]=ScaleFactor.x()*FirstRotationMatrix[0];
		//SR1Product[1]=ScaleFactor.x()*FirstRotationMatrix[1];
		//SR1Product[2]=ScaleFactor.x()*FirstRotationMatrix[2];
		//SR1Product[3]=ScaleFactor.y()*FirstRotationMatrix[3];
		//SR1Product[4]=ScaleFactor.y()*FirstRotationMatrix[4];
		//SR1Product[5]=ScaleFactor.y()*FirstRotationMatrix[5];
		//SR1Product[6]=ScaleFactor.z()*FirstRotationMatrix[6];
		//SR1Product[7]=ScaleFactor.z()*FirstRotationMatrix[7];
		//SR1Product[8]=ScaleFactor.z()*FirstRotationMatrix[8];


		vector<double> RSRTransformMatrix;
		for (int j=0;j<9;j++)
		{
			RSRTransformMatrix.push_back(R2SR1Product[j]);
//			RSRTransformMatrix.push_back(SR1Product[j]);
		}
		vecAllVertices.at(i)->SetRSRTransformMatrix(RSRTransformMatrix);
	}
}

