#include "CurveDeform.h"


CCurveDeform::CCurveDeform(void)
{
}

CCurveDeform::~CCurveDeform(void)
{
}

void CCurveDeform::ClosedCurveNaiveLaplacianDeform(vector<Point_3>& vecCurvePoint, vector<int> vecHandleIndex,
												   vector<Point_3> vecDeformCurvePoint,int iPlaneType/* =3 */)
{
	SparseMatrix LaplacianMatrix(vecCurvePoint.size());
	ComputeLaplacianMatrix(vecCurvePoint.size(),LaplacianMatrix);


	SparseMatrix HandleConstraintMatrix(vecHandleIndex.size());
	GetConstraintsMatrix(vecCurvePoint.size(),vecHandleIndex,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	vector<vector<double> > LaplacianRightHandSide,HandleRightHandSide;
	ComputeNaiveLaplacianRightHandSide(vecCurvePoint,vecHandleIndex,vecDeformCurvePoint,iPlaneType,
		LaplacianRightHandSide,HandleRightHandSide);
	vector<vector<double> > RightHandSide=LaplacianRightHandSide;
	if (iPlaneType!=3)
	{
		for (int i=0;i<2;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
	}
	vector<vector<double> > Result;
	bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

	if (bResult)
	{
		if (iPlaneType==0)//xoy
		{
			for (unsigned int i=0;i<vecCurvePoint.size();i++)
			{
				vecCurvePoint.at(i)=Point_3(Result.at(0).at(i),Result.at(1).at(i),vecCurvePoint.at(i).z());
			}
		}
		else if (iPlaneType==1)//xoz
		{
			for (unsigned int i=0;i<vecCurvePoint.size();i++)
			{
				vecCurvePoint.at(i)=Point_3(Result.at(0).at(i),vecCurvePoint.at(i).y(),Result.at(1).at(i));
			}
		}
		else if (iPlaneType==2)//yoz
		{
			for (unsigned int i=0;i<vecCurvePoint.size();i++)
			{
				vecCurvePoint.at(i)=Point_3(vecCurvePoint.at(i).x(),Result.at(0).at(i),Result.at(1).at(i));
			}
		}
		else
		{

		}
		//hard constraint
		for (unsigned int i=0;i<vecHandleIndex.size();i++)
		{
			vecCurvePoint.at(vecHandleIndex.at(i))=vecDeformCurvePoint.at(i);
		}
	}
}

void CCurveDeform::OpenCurveNaiveLaplacianDeform(vector<Point_3>& vecCurvePoint, vector<int> vecHandleIndex,
												 vector<Point_3> vecDeformCurvePoint,int iROIRange, int iPlaneType/* =3 */)
{

	for (unsigned int i=0;i<vecHandleIndex.size();i++)
	{
		//eg: handle index  25,roi range 3,so roi: 22,23,24,26,27,28,
		//anchor:21,29
		//get roi points
		vector<int> vecROIIndex;
		int iBegin=vecHandleIndex.at(i)-1;
		if (vecHandleIndex.at(i)==0)
		{
			iBegin=vecCurvePoint.size()-1;
		}
		for (int j=0;j<iROIRange;j++)
		{
			vecROIIndex.push_back(iBegin);
			if (iBegin==0)
			{
				iBegin=vecCurvePoint.size()-1;
			}
			else
			{
				iBegin--;
			}
		}
		reverse(vecROIIndex.begin(),vecROIIndex.end());
		iBegin=vecHandleIndex.at(i)+1;
		if (vecHandleIndex.at(i)==vecCurvePoint.size()-1)
		{
			iBegin=0;
		}
		for (int j=0;j<iROIRange;j++)
		{
			vecROIIndex.push_back(iBegin);
			if (iBegin==vecCurvePoint.size()-1)
			{
				iBegin=0;
			}
			else
			{
				iBegin++;
			}
		}

		//get anchor points
		vector<int> vecAnchorIndex;
		iBegin=vecROIIndex.front()-1;
		if (vecROIIndex.front()==0)
		{
			iBegin=vecCurvePoint.size()-1;
		}
		vecAnchorIndex.push_back(iBegin);
		iBegin=vecROIIndex.back()+1;
		if (vecROIIndex.back()==vecCurvePoint.size()-1)
		{
			iBegin=0;
		}
		vecAnchorIndex.push_back(iBegin);

		vector<int> vecTotalIndex;
		vecTotalIndex.push_back(vecAnchorIndex.at(0));
		vecTotalIndex.insert(vecTotalIndex.end(),vecROIIndex.begin(),vecROIIndex.begin()+vecROIIndex.size()/2);
		vecTotalIndex.push_back(vecHandleIndex.at(i));
		vecTotalIndex.insert(vecTotalIndex.end(),vecROIIndex.begin()+vecROIIndex.size()/2,vecROIIndex.begin()+vecROIIndex.size());
		vecTotalIndex.push_back(vecAnchorIndex.at(1));
		assert(vecTotalIndex.size()==1+2+vecROIIndex.size());

		SparseMatrix LaplacianMatrix(iROIRange*2+1);//num of row
		ComputeOpenLaplacianMatrix(iROIRange,LaplacianMatrix);

		SparseMatrix ConstraintMatrix(3);
		GetOpenConstraintsMatrix(iROIRange*2+1+2,ConstraintMatrix);
		SparseMatrix LeftHandMatrixA=LaplacianMatrix;
		LeftHandMatrixA.insert(LeftHandMatrixA.end(),ConstraintMatrix.begin(),ConstraintMatrix.end());

		CMath TAUCSSolver;
		SparseMatrix AT(LeftHandMatrixA.NCols());
		TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

		vector<vector<double> > LaplacianRightHandSide,ConstraintRightHandSide;
		ComputeOpenNaiveLaplacianRightHandSide(vecCurvePoint,vecHandleIndex.at(i),vecDeformCurvePoint.at(i),
			vecROIIndex,vecAnchorIndex,vecTotalIndex,iPlaneType,
			LaplacianRightHandSide,ConstraintRightHandSide);
		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		if (iPlaneType!=3)
		{
			for (int j=0;j<2;j++)
			{
				RightHandSide.at(j).insert(RightHandSide.at(j).end(),ConstraintRightHandSide.at(j).begin(),
					ConstraintRightHandSide.at(j).end());
			}
		}
		vector<vector<double> > Result;
		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);
		if (bResult)
		{
			if (iPlaneType==0)//xoy
			{
				for (unsigned int j=0;j<vecTotalIndex.size();j++)
				{
					vecCurvePoint.at(vecTotalIndex.at(j))=Point_3(Result.at(0).at(j),Result.at(1).at(j),vecCurvePoint.at(vecTotalIndex.at(j)).z());
				}
			}
			else if (iPlaneType==1)//xoz
			{
				for (unsigned int j=0;j<vecTotalIndex.size();j++)
				{
					vecCurvePoint.at(vecTotalIndex.at(j))=Point_3(Result.at(0).at(j),vecCurvePoint.at(vecTotalIndex.at(j)).y(),Result.at(1).at(j));
				}
			}
			else if (iPlaneType==2)//yoz
			{
				for (unsigned int j=0;j<vecTotalIndex.size();j++)
				{
					vecCurvePoint.at(vecTotalIndex.at(j))=Point_3(vecCurvePoint.at(vecTotalIndex.at(j)).x(),Result.at(0).at(j),Result.at(1).at(j));
				}
			}
			else
			{

			}
			//hard constraint
			vecCurvePoint.at(vecHandleIndex.at(i))=vecDeformCurvePoint.at(i);
		}
	}
}

void CCurveDeform::ComputeLaplacianMatrix(int iTotalPointsNum,SparseMatrix& LaplacianMatrix)
{
	//laplacian matrix is of size iRow*iColumn
	int iRow=iTotalPointsNum;
	int iColumn=iTotalPointsNum;
	LaplacianMatrix.m=iColumn;//num of col

	for (int i=0;i<iRow;i++)//for all rows
	{
		for (int j=0;j<iColumn;j++)//for each column of CurrentRow
		{
			if (j==i)
			{
				LaplacianMatrix[i][j]=2;
			}
			else if (i==0 && j==iTotalPointsNum-1)
			{
				LaplacianMatrix[i][j]=-1;
			}
			else if (i==iTotalPointsNum-1 && j==0)
			{
				LaplacianMatrix[i][j]=-1;
			}
			else if (j==i-1 || j==i+1)
			{
				LaplacianMatrix[i][j]=-1;
			}
			else
			{
				//0,do nothing
			}
		}
	}
}

void CCurveDeform::ComputeOpenLaplacianMatrix(int iROIRange,SparseMatrix& LaplacianMatrix)
{
	//laplacian matrix is of size iRow*iColumn
	int iRow=1+iROIRange*2;
	int iColumn=1+iROIRange*2+2;
	LaplacianMatrix.m=iColumn;//num of col

	for (int i=0;i<iRow;i++)//for all rows
	{
		for (int j=0;j<iColumn;j++)//for each column of CurrentRow
		{
			if (j==i+1)
			{
				LaplacianMatrix[i][j]=2;
			}
			else if (j==i || j==i+2)
			{
				LaplacianMatrix[i][j]=-1;
			}
			else
			{
				//0,do nothing
			}
		}
	}
}

void CCurveDeform::GetConstraintsMatrix(int iTotalPointsNum, vector<int> vecHandleIndex,
										SparseMatrix& HandleConstraintMatrix)
{
	int iHandleRow=(int)vecHandleIndex.size();
	HandleConstraintMatrix.m=iTotalPointsNum;
	for (int i=0;i<iHandleRow;i++)
	{
		HandleConstraintMatrix[i][vecHandleIndex.at(i)]=1;
	}
}

void CCurveDeform::GetOpenConstraintsMatrix(int iTotalPointsNum,SparseMatrix& HandleConstraintMatrix)
{
	HandleConstraintMatrix.m=iTotalPointsNum;//num of col
	HandleConstraintMatrix[0][0]=1;
	HandleConstraintMatrix[1][(iTotalPointsNum-1)/2]=1;
	HandleConstraintMatrix[2][iTotalPointsNum-1]=1;
}

void CCurveDeform::ComputeNaiveLaplacianRightHandSide(vector<Point_3> vecCurvePoint, vector<int> vecHandleIndex,
													  vector<Point_3> vecDeformCurvePoint,
													  int iPlaneType, vector<vector<double> >& LaplacianRightHandSide, 
													  vector<vector<double> >& HandleRightHandSide)
{
	if (iPlaneType==0)//xoy 
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecCurvePoint.size();i++)
		{
			if (i==0)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.back().x()-vecCurvePoint.at(i+1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.back().y()-vecCurvePoint.at(i+1).y());
			}
			else if (i==vecCurvePoint.size()-1)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.front().x()-vecCurvePoint.at(i-1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.front().y()-vecCurvePoint.at(i-1).y());
			}
			else
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.at(i+1).x()-vecCurvePoint.at(i-1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.at(i+1).y()-vecCurvePoint.at(i-1).y());
			}
		}
		//constraint
		for (unsigned int i=0;i<vecHandleIndex.size();i++)
		{
			Handle[0].push_back(vecDeformCurvePoint.at(i).x());
			Handle[1].push_back(vecDeformCurvePoint.at(i).y());
		}

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else if (iPlaneType==1)//xoz
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecCurvePoint.size();i++)
		{
			if (i==0)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.back().x()-vecCurvePoint.at(i+1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.back().z()-vecCurvePoint.at(i+1).z());
			}
			else if (i==vecCurvePoint.size()-1)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.front().x()-vecCurvePoint.at(i-1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.front().z()-vecCurvePoint.at(i-1).z());
			}
			else
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).x()*2-vecCurvePoint.at(i+1).x()-vecCurvePoint.at(i-1).x());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.at(i+1).z()-vecCurvePoint.at(i-1).z());
			}
		}
		//constraint
		for (unsigned int i=0;i<vecHandleIndex.size();i++)
		{
			Handle[0].push_back(vecDeformCurvePoint.at(i).x());
			Handle[1].push_back(vecDeformCurvePoint.at(i).z());
		}

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else if (iPlaneType==2)//yoz
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecCurvePoint.size();i++)
		{
			if (i==0)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.back().y()-vecCurvePoint.at(i+1).y());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.back().z()-vecCurvePoint.at(i+1).z());
			}
			else if (i==vecCurvePoint.size()-1)
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.front().y()-vecCurvePoint.at(i-1).y());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.front().z()-vecCurvePoint.at(i-1).z());
			}
			else
			{
				Laplacian[0].push_back(vecCurvePoint.at(i).y()*2-vecCurvePoint.at(i+1).y()-vecCurvePoint.at(i-1).y());
				Laplacian[1].push_back(vecCurvePoint.at(i).z()*2-vecCurvePoint.at(i+1).z()-vecCurvePoint.at(i-1).z());
			}
		}
		//constraint
		for (unsigned int i=0;i<vecHandleIndex.size();i++)
		{
			Handle[0].push_back(vecDeformCurvePoint.at(i).y());
			Handle[1].push_back(vecDeformCurvePoint.at(i).z());
		}

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else//non plananr curve
	{

	}
}

void CCurveDeform::ComputeOpenNaiveLaplacianRightHandSide(vector<Point_3> vecCurvePoint, int iHandleIndex,Point_3 DeformCurvePoint, 
														  vector<int> vecROIIndex,vector<int> vecAnchorIndex, vector<int> vecTotalIndex,
														  int iPlaneType, vector<vector<double> >& LaplacianRightHandSide, vector<vector<double> >& HandleRightHandSide)
{
	if (iPlaneType==0)//xoy 
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecROIIndex.size()+1;i++)
		{
			Laplacian[0].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).x()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).x()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).x());
			Laplacian[1].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).y()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).y()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).y());
		}
		//constraint
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.front()).x());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.front()).y());
		Handle[0].push_back(DeformCurvePoint.x());
		Handle[1].push_back(DeformCurvePoint.y());
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.back()).x());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.back()).y());

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else if (iPlaneType==1)//xoz
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecROIIndex.size()+1;i++)
		{
			Laplacian[0].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).x()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).x()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).x());
			Laplacian[1].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).z()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).z()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).z());
		}
		//constraint
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.front()).x());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.front()).z());
		Handle[0].push_back(DeformCurvePoint.x());
		Handle[1].push_back(DeformCurvePoint.z());
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.back()).x());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.back()).z());

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else if (iPlaneType==2)//yoz
	{
		vector<double> Laplacian[2],Handle[2];
		//Laplacian
		for (unsigned int i=0;i<vecROIIndex.size()+1;i++)
		{
			Laplacian[0].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).y()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).y()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).y());
			Laplacian[1].push_back(vecCurvePoint.at(vecTotalIndex.at(i+1)).z()*2-
				vecCurvePoint.at(vecTotalIndex.at(i)).z()-
				vecCurvePoint.at(vecTotalIndex.at(i+2)).z());
		}
		//constraint
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.front()).y());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.front()).z());
		Handle[0].push_back(DeformCurvePoint.y());
		Handle[1].push_back(DeformCurvePoint.z());
		Handle[0].push_back(vecCurvePoint.at(vecTotalIndex.back()).y());
		Handle[1].push_back(vecCurvePoint.at(vecTotalIndex.back()).z());

		for (int i=0;i<2;i++)
		{
			LaplacianRightHandSide.push_back(Laplacian[i]);
			HandleRightHandSide.push_back(Handle[i]);
		}
	}
	else//non plananr curve
	{

	}
}