#include "../PreDef.h"
#include "Math.h"

/*! dssmatrix constructor without arguments */
KW_SparseMatrix::KW_SparseMatrix()
: m(M), n(N), cap(CAP), vol(VOL), array(Array), indx(Indx), jndx(Jndx)
{
	//////// initialize ////////
	M =N =0;
	CAP =VOL =0;
	Array =NULL;
	Indx =Jndx =NULL;
}

/*! dssmatrix constructor with size specification */
KW_SparseMatrix::KW_SparseMatrix(const long& _m, const long& _n, const long& _c)
: m(M), n(N), cap(CAP), vol(VOL), array(Array), indx(Indx), jndx(Jndx)
{
	//////// initialize ////////
	M =_m; N =_n;
	CAP =_c;
	VOL =0;
	Array =new float[CAP];
	Indx =new long[CAP];
	Jndx =new long[CAP];
}

/*! dssmatrix destructor */
KW_SparseMatrix::~KW_SparseMatrix()
{
	//////// delete Array ////////
	delete [] Array;
	delete [] Indx;
	delete [] Jndx;
}

/*! put value without isListed check and volume cheack */
void KW_SparseMatrix::fput(const long& i, const long& j, const float& v)
{
	Indx[VOL]=i;  Jndx[VOL]=j;  Array[VOL]=v;
	VOL++;
}

/*! add value with isListed check and volume cheack */
void KW_SparseMatrix::add(const long& i, const long& j, const float& v)
{

	//// in case (i,j) already exists ////
	for(long c=0; c<VOL; c++){
		if(Indx[c]==i && Jndx[c]==j){ Array[c]+=v; return; }
	}

	//// in case (i,j) doesnot exist ////
	if(VOL==CAP){
		expand(100);
	}

	Indx[VOL]=i; Jndx[VOL]=j; Array[VOL]=v;
	VOL++;
}

/*! expand the matrix capacity */
void KW_SparseMatrix::expand(const long& dc)
{
	CAP+=dc;
	float* newArray(new float[CAP]);
	long *newIndx(new long[CAP]), *newJndx(new long[CAP]);

	for(int c=0; c<VOL; c++){
		newArray[c] =Array[c];
		newIndx[c] =Indx[c];
		newJndx[c] =Jndx[c];
	}

	delete [] Array;
	delete [] Indx;
	delete [] Jndx;
	Array =newArray;
	Indx =newIndx;
	Jndx =newJndx;
}

/*! dssmatrix*dssmatrix operator */
void KW_SparseMatrix::KW_multiply(KW_SparseMatrix& matA,KW_SparseMatrix& matB,KW_SparseMatrix& Result)
{
	//	KW_SparseMatrix newmat( matA.m, matB.n, 0 );

	for(long c=0; c<matA.vol; c++){
		for(long d=0; d<matB.vol; d++){
			if(matA.jndx[c]==matB.indx[d]){
				Result.add(matA.indx[c], matB.jndx[d], matA.array[c]*matB.array[d]);
			}
		}
	}
	//	return newmat;
}

//SparseMatrix  operator *  (SparseMatrix& other) const
//{
//	assert(this->NCols()==other.NRows());
//	SparseMatrix result(this->NRows());
//	result.m=other.NCols();
//
//	for (unsigned int j=0;j<other.NCols();j++)
//	{
//		for (unsigned int i=0;i<this->NRows();i++)
//		{
//
//		}
//	}
//}
//
SparseMatrix  SparseMatrix::operator +  (SparseMatrix& other) const
{
	assert(this->NRows()==other.NRows());
	assert(this->NCols()==other.NCols());
	SparseMatrix result(this->NRows());
	result.m=this->NCols();
	for (int i=0;i<(int)this->NRows();i++)
	{
		int iTemp=0;
		map<int, double>::const_iterator m=(*this)[i].begin();
		map<int, double>::const_iterator n=other[i].begin();
		for (int j=0;j<(int)this->NCols();j++)
		{
			if ((m!=(*this)[i].end())&&(n!=other[i].end()))
			{
				if ((iTemp<m->first)&&(iTemp<n->first))
				{
					iTemp++;
				}
				else if (m->first==n->first)
				{
					result[i][iTemp]=m->second+n->second;
					iTemp++;m++;n++;
				}
				else if (iTemp==m->first)
				{
					result[i][iTemp]=m->second;
					iTemp++;m++;
				}
				else if (iTemp==n->first)
				{
					result[i][iTemp]=n->second;
					iTemp++;n++;
				}
			}
			else if ((m==(*this)[i].end())&&(n==other[i].end()))
			{
				break;
			}
			else if (m==(*this)[i].end())
			{
				if (iTemp<n->first)
				{
					iTemp++;
				}
				else if (iTemp==n->first)
				{
					result[i][iTemp]=n->second;
					iTemp++;n++;
				}
			}
			else if (n==other[i].end())
			{
				if (iTemp<m->first)
				{
					iTemp++;
				}
				if (iTemp==m->first)
				{
					result[i][iTemp]=m->second;
					iTemp++;m++;
				}
			}
		}
	}
	return result;
}

SparseMatrix  SparseMatrix::operator -  (SparseMatrix& other) const
{
	assert(this->NRows()==other.NRows());
	assert(this->NCols()==other.NCols());
	SparseMatrix result(this->NRows());
	result.m=this->NCols();
	for (int i=0;i<(int)this->NRows();i++)
	{
		int iTemp=0;
		map<int, double>::const_iterator m=(*this)[i].begin();
		map<int, double>::const_iterator n=other[i].begin();
		for (int j=0;j<(int)this->NCols();j++)
		{
			if ((m!=(*this)[i].end())&&(n!=other[i].end()))
			{
				if ((iTemp<m->first)&&(iTemp<n->first))
				{
					iTemp++;
				}
				else if (m->first==n->first)
				{
					result[i][iTemp]=m->second-n->second;
					iTemp++;m++;n++;
				}
				else if (iTemp==m->first)
				{
					result[i][iTemp]=m->second;
					iTemp++;m++;
				}
				else if (iTemp==n->first)
				{
					result[i][iTemp]=-n->second;
					iTemp++;n++;
				}
			}
			else if ((m==(*this)[i].end())&&(n==other[i].end()))
			{
				break;
			}
			else if (m==(*this)[i].end())
			{
				if (iTemp<n->first)
				{
					iTemp++;
				}
				else if (iTemp==n->first)
				{
					result[i][iTemp]=-n->second;
					iTemp++;n++;
				}
			}
			else if (n==other[i].end())
			{
				if (iTemp<m->first)
				{
					iTemp++;
				}
				if (iTemp==m->first)
				{
					result[i][iTemp]=m->second;
					iTemp++;m++;
				}
			}
		}
	}
	return result;	
}


CMath::CMath(void)
{
}

CMath::~CMath(void)
{
}

#define MAX_Memo 999999
//minimize sqr(|| c - A*x ||)   subject to   B*x = d
bool CMath::ComputeConstrainedLSE(vector<vector<double> > LeftMatrixA,vector<double> RightMatrixC,
					   vector<vector<double> > LeftMatrixB,vector<double> RightMatrixD, 
					   vector<double>& Result)
{
//	int temp=(int)LeftMatrixA.size();
//	integer iMatARow=temp;
//	temp=(int)LeftMatrixA.front().size();
//	integer iMatACol=temp;
//	temp=(int)LeftMatrixB.size();
//	integer iMatBRow=temp;
//
//	if (iMatARow*iMatACol>MAX_Memo)
//	{
//		AfxMessageBox("not enough memo for matrix");
//		return false;
//	}
//	double* A=new double[MAX_Memo];
//	memset(A,0x00,MAX_Memo*sizeof(double));
//	int iIndex=0;
//	for (unsigned int i=0;i<LeftMatrixA.front().size();i++)
//	{
//		for (unsigned int j=0;j<LeftMatrixA.size();j++)
//		{
//			A[iIndex]=LeftMatrixA.at(j).at(i);
//			iIndex++;
//		}
//	}
//
//	integer LDA=iMatARow;
//
//	double* B=new double[MAX_Memo];
//	memset(B,0x00,MAX_Memo*sizeof(double));
//	iIndex=0;
//	for (unsigned int i=0;i<LeftMatrixB.front().size();i++)
//	{
//		for (unsigned int j=0;j<LeftMatrixB.size();j++)
//		{
//			B[iIndex]=LeftMatrixB.at(j).at(i);
//			iIndex++;
//		}
//	}
//
//	
//	integer LDB=iMatBRow;
//
//	double* C=new double[MAX_Memo];
//	memset(C,0x00,MAX_Memo*sizeof(double));
//	for (unsigned int i=0;i<RightMatrixC.size();i++)
//	{
//		C[i]=RightMatrixC.at(i);
//	}
//
//	double* D=new double[MAX_Memo];
//	memset(D,0x00,MAX_Memo*sizeof(double));
//	for (unsigned int i=0;i<RightMatrixD.size();i++)
//	{
//		D[i]=RightMatrixD.at(i);
//	}
//
//
//	double* X=new double[MAX_Memo];
//	memset(X,0x00,MAX_Memo*sizeof(double));
//
//	double* WORK=new double[MAX_Memo];
//	memset(WORK,0x00,MAX_Memo*sizeof(double));
//
//	integer LWORK=iMatARow+iMatACol+iMatBRow+10;
////	integer LWORK=max(1,iMatARow+iMatACol+iMatBRow)+10;
////	integer LWORK=iMatBRow+min(iMatARow,iMatACol)+max(iMatARow,iMatACol)*256;
//	integer INFO=0;
//
//	dgglse_(&iMatARow,&iMatACol,&iMatBRow,A,&LDA,B,&LDB,C,D,X,WORK,&LWORK,&INFO);
//
//	if (INFO==0)
//	{
//		for (unsigned int i=0;i<LeftMatrixA.front().size();i++)
//		{
//			Result.push_back(X[i]);
//		}
//		delete [] A;
//		delete [] B;
//		delete [] C;
//		delete [] D;
//		delete [] X;
//		delete [] WORK;
//		return true;
//	}
//
//	delete [] A;
//	delete [] B;
//	delete [] C;
//	delete [] D;
//	delete [] X;
//	delete [] WORK;
	return false;
}

//minimize sqr(|| B - A*x ||), B may contain more than 1 column,RightMatrixB stores in columnsize
bool CMath::ComputeLSE(std::vector<std::vector<double> > LeftMatrixA,std::vector<std::vector<double> > RightMatrixB,
					   std::vector<std::vector<double> >& Result)
{
	//CPPL::dgematrix A(LeftMatrixA.size(),LeftMatrixA.front().size());
	//for (unsigned int i=0;i<LeftMatrixA.size();i++)
	//{
	//	for (unsigned int j=0;j<LeftMatrixA.front().size();j++)
	//	{
	//		A(i,j)=LeftMatrixA.at(i).at(j);
	//	}
	//}
	//CPPL::dgematrix B(RightMatrixB.front().size(),RightMatrixB.size());

	//for (unsigned int i=0;i<RightMatrixB.size();i++)
	//{
	//	for (unsigned int j=0;j<RightMatrixB.front().size();j++)
	//	{
	//		B(j,i)=RightMatrixB.at(i).at(j);
	//	}
	//}

	//int iResult=A.dgels(B);

	//if (iResult!=0)
	//{
	//	return false;
	//}

	//CPPL::dgematrix C=t(B);

	//for (int i=0;i<C.m;i++)
	//{
	//	vector<double> CurrentRow;
	//	for (int j=0;j<C.n;j++)
	//	{
	//		CurrentRow.push_back(C(i,j));
	//	}
	//	Result.push_back(CurrentRow);
	//}
	//return true;

	char TRANS='N';
	int temp=LeftMatrixA.size();
	integer iMatARow=temp;
//	long iMatARow=temp;
	temp=LeftMatrixA.front().size();
	integer iMatACol=temp;
//	long iMatACol=temp;
	temp=RightMatrixB.size();
	integer iMatBCol=temp;
//	long iMatBCol=temp;

	bool bOverDetermine=false;
	if (iMatARow>iMatACol)//overdetermined 
	{
		bOverDetermine=true;
	}
	int iFunctionNum=(int)LeftMatrixA.size();
	int iUnknownNumInEachGroup=(int)LeftMatrixA.front().size();
	int iGroupNum=(int)RightMatrixB.size();



	//if (iMatARow*iMatACol>1999*1999)
	//{
	//	AfxMessageBox("not enough memo for matrix");
	//	return false;
	//}
	float* A=new float[LeftMatrixA.size()*LeftMatrixA.front().size()];
	memset(A,0x00,LeftMatrixA.size()*LeftMatrixA.front().size()*sizeof(float));
	int iIndex=0;
	for (unsigned int i=0;i<LeftMatrixA.front().size();i++)
	{
		for (unsigned int j=0;j<LeftMatrixA.size();j++)
		{
			A[iIndex]=LeftMatrixA.at(j).at(i);
			iIndex++;
		}
	}

	integer LDA=iMatARow;
//	long LDA=iMatARow;

	float* B=new float[RightMatrixB.size()*RightMatrixB.front().size()];
	memset(B,0x00,RightMatrixB.size()*RightMatrixB.front().size()*sizeof(float));
	iIndex=0;
	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		for (unsigned int j=0;j<RightMatrixB.front().size();j++)
		{
			B[iIndex]=RightMatrixB.at(i).at(j);
			iIndex++;
		}
		if (!bOverDetermine)//since B is of dimension (LDB,NRHS)!!!
		{
			for (int j=0;j<iUnknownNumInEachGroup-iFunctionNum;j++)
			{
				iIndex++;
			}
		}
	}

	integer LDB=max(iMatARow,iMatACol);
//	long LDB=max(iMatARow,iMatACol);

	long iWorkSize=min(LeftMatrixA.size(),LeftMatrixA.front().size())+max(min(LeftMatrixA.size(),LeftMatrixA.front().size()),RightMatrixB.size());
	float* WORK=new float[iWorkSize];
	memset(WORK,0x00,iWorkSize*sizeof(float));
//	integer LWORK=min(iMatARow,iMatACol)+max(iMatARow*iMatACol,iMatBCol)*256;
	integer LWORK=iWorkSize;
//	long LWORK=iWorkSize;
	integer INFO=0;
//	long INFO=0;

//	dgels_(&TRANS,&iMatARow,&iMatACol,&iMatBCol,A,&LDA,B,&LDB,WORK,&LWORK,&INFO);

//	dgels_('N',iMatARow,iMatACol,iMatBCol,(double*)A,LDA,(double*)B,LDB,(double*)WORK,LWORK,INFO);
	sgels_(&TRANS,&iMatARow,&iMatACol,&iMatBCol,A,&LDA,B,&LDB,WORK,&LWORK,&INFO);
	

	if (INFO==0)
	{
		iIndex=0;
		for (unsigned int i=0;i<RightMatrixB.size();i++)
		{
			vector<double> CurrentResult;
			for (int j=0;j<iUnknownNumInEachGroup;j++)
			{
				CurrentResult.push_back(B[iIndex]);
				iIndex++;
			}
			if (bOverDetermine)
			{
				for (int j=0;j<iFunctionNum-iUnknownNumInEachGroup;j++)
				{
					//these B[iIndex] stores the residual sum of squares for the
					//solution for each group,so skip it
					iIndex++;
				}
			}
			Result.push_back(CurrentResult);
		}
		delete [] A;
		delete [] B;
		delete [] WORK;
		return true;
	}

	delete [] A;
	delete [] B;
	delete [] WORK;
	return false;
}

bool CMath::ComputeLSE(SparseMatrix LeftMatrixA, std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result)
{
	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		if (LeftMatrixA.NRows()!=RightMatrixB.at(i).size())
		{
			return false;
		}
	}
	clock_t SolveBegin=clock();   

	char TRANS='N';
	int temp=LeftMatrixA.NRows();
	integer iMatARow=temp;
	//	long iMatARow=temp;
	temp=LeftMatrixA.NCols();
	integer iMatACol=temp;
	//	long iMatACol=temp;
	temp=RightMatrixB.size();
	integer iMatBCol=temp;
	//	long iMatBCol=temp;

	bool bOverDetermine=false;
	if (iMatARow>iMatACol)//overdetermined 
	{
		bOverDetermine=true;
	}
	int iFunctionNum=(int)LeftMatrixA.NRows();
	int iUnknownNumInEachGroup=(int)LeftMatrixA.NCols();
	int iGroupNum=(int)RightMatrixB.size();

	//row-wise
	float* ATemp=new float[LeftMatrixA.NRows()*LeftMatrixA.NCols()];
	memset(ATemp,0x00,LeftMatrixA.NRows()*LeftMatrixA.NCols()*sizeof(float));
	int iIndex=0;
	for (unsigned int i=0;i<LeftMatrixA.NRows();i++)
	{
		map<int, double>::const_iterator m=LeftMatrixA.at(i).begin();
		for (int j=0;j<(int)LeftMatrixA.NCols();j++)
		{
			if (m==LeftMatrixA.at(i).end())
			{
				ATemp[iIndex]=0;
			}
			else if(j<m->first)
			{
				ATemp[iIndex]=0;
			}
			else
			{
				ATemp[iIndex]=m->second;
				m++;
			}
			iIndex++;
		}
	}

	//convert to column-wise
	float* A=new float[LeftMatrixA.NRows()*LeftMatrixA.NCols()];
	memset(A,0x00,LeftMatrixA.NRows()*LeftMatrixA.NCols()*sizeof(float));
	iIndex=0;
	for (unsigned int i=0;i<LeftMatrixA.NCols();i++)
	{
		for (unsigned int j=0;j<LeftMatrixA.NRows();j++)
		{
			A[iIndex]=ATemp[i+j*LeftMatrixA.NCols()];
			iIndex++;
		}
	}
	ATemp=NULL;
	delete [] ATemp;

	integer LDA=iMatARow;
	//	long LDA=iMatARow;

	temp=LeftMatrixA.NCols();
	if (bOverDetermine)
	{
		temp=LeftMatrixA.NRows();
	}
	float* B=new float[temp*RightMatrixB.size()];
	memset(B,0x00,LeftMatrixA.NRows()*RightMatrixB.size()*sizeof(float));
	iIndex=0;
	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		for (unsigned int j=0;j<RightMatrixB.front().size();j++)
		{
			B[iIndex]=RightMatrixB.at(i).at(j);
			iIndex++;
		}
		if (!bOverDetermine)//since B is of dimension (LDB,NRHS)!!!
		{
			for (int j=0;j<iUnknownNumInEachGroup-iFunctionNum;j++)
			{
				iIndex++;
			}
		}
	}

	integer LDB=max(iMatARow,iMatACol);
	//	long LDB=max(iMatARow,iMatACol);

	//integer LWORK=5*(LeftMatrixA.NRows()+LeftMatrixA.NCols());
	//float* WORK=new float[5*(LeftMatrixA.NRows()+LeftMatrixA.NCols())];
	//memset(WORK,0x00,5*(LeftMatrixA.NRows()+LeftMatrixA.NCols())*sizeof(float));

	long iWorkSize=min(LeftMatrixA.NRows(),LeftMatrixA.NCols())+max(min(LeftMatrixA.NRows(),LeftMatrixA.NCols()),RightMatrixB.size());
	float* WORK=new float[iWorkSize];
	memset(WORK,0x00,iWorkSize*sizeof(float));
	integer LWORK=iWorkSize;
	integer INFO=0;
	//	long INFO=0;

	//	dgels_(&TRANS,&iMatARow,&iMatACol,&iMatBCol,A,&LDA,B,&LDB,WORK,&LWORK,&INFO);

	//	dgels_('N',iMatARow,iMatACol,iMatBCol,(double*)A,LDA,(double*)B,LDB,(double*)WORK,LWORK,INFO);
	sgels_(&TRANS,&iMatARow,&iMatACol,&iMatBCol,A,&LDA,B,&LDB,WORK,&LWORK,&INFO);

	if (INFO==0)
	{
		iIndex=0;
		for (unsigned int i=0;i<RightMatrixB.size();i++)
		{
			vector<double> CurrentResult;
			for (int j=0;j<iUnknownNumInEachGroup;j++)
			{
				CurrentResult.push_back(B[iIndex]);
				iIndex++;
			}
			if (bOverDetermine)
			{
				for (int j=0;j<iFunctionNum-iUnknownNumInEachGroup;j++)
				{
					//these B[iIndex] stores the residual sum of squares for the
					//solution for each group,so skip it
					iIndex++;
				}
			}
			Result.push_back(CurrentResult);
		}
		A=NULL;B=NULL;WORK=NULL;
		delete [] A;
		delete [] B;
		delete [] WORK;

		clock_t SolveEnd=clock();
		cout<<"Solving time: %f"<<float(SolveEnd-SolveBegin)<<endl;

		return true;
	}

	A=NULL;B=NULL;WORK=NULL;
	delete [] A;
	delete [] B;
	delete [] WORK;
	return false;
}

bool CMath::ComputeLSESmallSize(std::vector<std::vector<double> > LeftMatrixA, 
								std::vector<std::vector<double> > RightMatrixB,
								std::vector<std::vector<double> >& Result)
{
	//CPPL::dgematrix A(LeftMatrixA.size(),LeftMatrixA.front().size());

	//for (unsigned int i=0;i<LeftMatrixA.size();i++)
	//{
	//	for (unsigned int j=0;j<LeftMatrixA.front().size();j++)
	//	{
	//		A(i,j)=LeftMatrixA.at(i).at(j);
	//	}
	//}
	//
	//CPPL::dgematrix B(RightMatrixB.front().size(),RightMatrixB.size());

	//for (unsigned int i=0;i<RightMatrixB.size();i++)
	//{
	//	for (unsigned int j=0;j<RightMatrixB.front().size();j++)
	//	{
	//		B(j,i)=RightMatrixB.at(i).at(j);
	//	}
	//}
	//
	//int iResult=A.dgels(B);

	//if (iResult!=0)
	//{
	//	return false;
	//}

	//CPPL::dgematrix C=t(B);

	//for (int i=0;i<C.m;i++)
	//{
	//	vector<double> CurrentRow;
	//	for (int j=0;j<C.n;j++)
	//	{
	//		CurrentRow.push_back(C(i,j));
	//	}
	//	Result.push_back(CurrentRow);
	//}
	//return true;

	char TRANS='N';
	int temp=LeftMatrixA.size();
	integer iMatARow=temp;
	temp=LeftMatrixA.front().size();
	integer iMatACol=temp;
	temp=RightMatrixB.size();
	integer iMatBCol=temp;

	bool bOverDetermine=false;
	if (iMatARow>iMatACol)//overdetermined 
	{
		bOverDetermine=true;
	}
	int iFunctionNum=(int)LeftMatrixA.size();
	int iUnknownNumInEachGroup=(int)LeftMatrixA.front().size();
	int iGroupNum=(int)RightMatrixB.size();



	if (iMatARow*iMatACol>3*3)
	{
		cout<<"not enough memo for matrix"<<endl;
		return false;
	}
	double* A=new double[3*3];
	memset(A,0x00,3*3*sizeof(double));
	int iIndex=0;
	for (unsigned int i=0;i<LeftMatrixA.front().size();i++)
	{
		for (unsigned int j=0;j<LeftMatrixA.size();j++)
		{
			A[iIndex]=LeftMatrixA.at(j).at(i);
			iIndex++;
		}
	}

	integer LDA=iMatARow;

	double* B=new double[3*3];
	memset(B,0x00,3*3*sizeof(double));
	iIndex=0;
	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		for (unsigned int j=0;j<RightMatrixB.front().size();j++)
		{
			B[iIndex]=RightMatrixB.at(i).at(j);
			iIndex++;
		}
		if (!bOverDetermine)//since B is of dimension (LDB,NRHS)!!!
		{
			for (int j=0;j<iUnknownNumInEachGroup-iFunctionNum;j++)
			{
				iIndex++;
			}
		}
	}

	integer LDB=max(iMatARow,iMatACol);

	double* WORK=new double[3*3];
	memset(WORK,0x00,3*3*sizeof(double));
	integer LWORK=min(iMatARow,iMatACol)+max(iMatARow*iMatACol,iMatBCol)*256;
	integer INFO=0;

	dgels_(&TRANS,&iMatARow,&iMatACol,&iMatBCol,A,&LDA,B,&LDB,WORK,&LWORK,&INFO);

	if (INFO==0)
	{
		iIndex=0;
		for (int i=0;i<iGroupNum;i++)
		{
			vector<double> CurrentResult;
			for (int j=0;j<iUnknownNumInEachGroup;j++)
			{
				CurrentResult.push_back(B[iIndex]);
				iIndex++;
			}
			if (bOverDetermine)
			{
				for (int j=0;j<iFunctionNum-iUnknownNumInEachGroup;j++)
				{
					//these B[iIndex] stores the residual sum of squares for the
					//solution for each group,so skip it
					iIndex++;
				}
			}
			Result.push_back(CurrentResult);
		}
		delete [] A;
		delete [] B;
		delete [] WORK;
		return true;
	}

	delete [] A;
	delete [] B;
	delete [] WORK;
	return false;
}

//compute svd of 3*3 matrix.For larger size matrix,the memory allocated inside this function should be larger
bool CMath::ComputeSVD(std::vector<std::vector<double> > InputMatrix, 
					   std::vector<std::vector<double> >& OutputU,
					   std::vector<std::vector<double> >& OutputSigma, 
					   std::vector<std::vector<double> >& OutputV)
{
	//CPPL::dgematrix A(InputMatrix.size(),InputMatrix.front().size());

	//for (unsigned int i=0;i<InputMatrix.size();i++)
	//{
	//	for (unsigned int j=0;j<InputMatrix.front().size();j++)
	//	{
	//		A(i,j)=InputMatrix.at(i).at(j);
	//	}
	//}

	//CPPL::dcovector Sigma;
	//CPPL::dgematrix U,VT;

	//int iResult=A.dgesvd(Sigma,U,VT);
	//if (iResult!=0)
	//{
	//	return false;
	//}

	//CPPL::dgematrix V=t(VT);

	//for (int i=0;i<U.m;i++)
	//{
	//	vector<double> CurrentRow;
	//	for (int j=0;j<U.n;j++)
	//	{
	//		CurrentRow.push_back(U(i,j));
	//	}
	//	OutputU.push_back(CurrentRow);
	//}
	//for (int i=0;i<V.m;i++)
	//{
	//	vector<double> CurrentRow;
	//	for (int j=0;j<V.n;j++)
	//	{
	//		CurrentRow.push_back(V(i,j));
	//	}
	//	OutputV.push_back(CurrentRow);
	//}

	//int iIndex=0;
	//for (int i=0;i<Sigma.l;i++)
	//{
	//	vector<double> CurrentRow;
	//	for (int j=0;j<Sigma.l;j++)
	//	{
	//		if (i==j)
	//		{
	//			CurrentRow.push_back(Sigma.array[iIndex]);
	//			iIndex++;
	//		}
	//		else
	//		{
	//			CurrentRow.push_back(0);
	//		}
	//	}
	//	OutputSigma.push_back(CurrentRow);
	//}
	//return true;

	char JOBU='A';
	char JOBVT='A';

	int temp=InputMatrix.size();
	integer iARow=temp;
	temp=InputMatrix.front().size();
	integer iACol=temp;
	double* A=new double[4*4];
	memset(A,0x00,4*4*sizeof(double));
	int iIndex=0;
	for (unsigned int i=0;i<InputMatrix.front().size();i++)
	{
		for (unsigned int j=0;j<InputMatrix.size();j++)
		{
			A[iIndex]=InputMatrix.at(j).at(i);
			iIndex++;
		}
	}

	clock_t Begintest=clock();

	integer LDA=max(1,iARow);

	double* S=new double[4*4];
	memset(S,0x00,4*4*sizeof(double));

	double* U=new double[4*4];
	memset(U,0x00,4*4*sizeof(double));
	integer LDU=iARow;

	double* VT=new double[4*4];
	memset(VT,0x00,4*4*sizeof(double));
	integer LDVT=iACol;

	double* WORK=new double[4*4];
	memset(WORK,0x00,4*4*sizeof(double));
	integer LWORK=max(3*min(iARow,iACol)+max(iARow,iACol),5*min(iARow,iACol))+1000;

	integer INFO=0;

	dgesvd_( &JOBU, &JOBVT, &iARow, &iACol, A, &LDA, S, U, &LDU, VT, &LDVT, WORK, &LWORK, &INFO);

	if (INFO==0)
	{
		for (unsigned int i=0;i<InputMatrix.size();i++)
		{
			vector<double> CurrentRow;
			for (unsigned int j=0;j<InputMatrix.size();j++)
			{
				CurrentRow.push_back(U[i+j*InputMatrix.size()]);
			}
			OutputU.push_back(CurrentRow);
		}
		for (unsigned int i=0;i<InputMatrix.size();i++)
		{
			vector<double> CurrentRow;
			for (unsigned int j=0;j<InputMatrix.front().size();j++)
			{
				if (i==j)
				{
					CurrentRow.push_back(S[i]);
				}
				else
				{
					CurrentRow.push_back(0);
				}
			}
			OutputSigma.push_back(CurrentRow);
		}
		for (unsigned int i=0;i<InputMatrix.front().size();i++)
		{
			vector<double> CurrentRow;
			for (unsigned int j=0;j<InputMatrix.front().size();j++)
			{
				CurrentRow.push_back(VT[i*InputMatrix.front().size()+j]);
			}
			OutputV.push_back(CurrentRow);
		}
		delete [] A;
		delete [] S;
		delete [] U;
		delete [] VT;
		delete [] WORK;

		return true;
	}

	delete [] A;
	delete [] S;
	delete [] U;
	delete [] VT;
	delete [] WORK;
	return false;
}

bool CMath::ComputeSVD(GeneralMatrix InputMatrix,GeneralMatrix& OutputU,GeneralMatrix& OutputSigma, GeneralMatrix& OutputV)
{
	char JOBU='A';
	char JOBVT='A';

	int temp=InputMatrix.nRow();
	integer iARow=temp;
	temp=InputMatrix.nCol();
	integer iACol=temp;
	double* A=new double[InputMatrix.nRow()*InputMatrix.nCol()];
	memset(A,0x00,InputMatrix.nRow()*InputMatrix.nCol()*sizeof(double));
	int iIndex=0;
	for (int i=0;i<InputMatrix.nCol();i++)
	{
		for (int j=0;j<(int)InputMatrix.nRow();j++)
		{
			A[iIndex]=InputMatrix[j][i];
			iIndex++;
		}
	}

	integer LDA=max(1,iARow);

	double* S=new double[InputMatrix.nRow()*InputMatrix.nCol()];
	memset(S,0x00,InputMatrix.nRow()*InputMatrix.nCol()*sizeof(double));

	double* U=new double[InputMatrix.nRow()*InputMatrix.nRow()];
	memset(U,0x00,InputMatrix.nRow()*InputMatrix.nRow()*sizeof(double));
	integer LDU=iARow;

	double* VT=new double[InputMatrix.nCol()*InputMatrix.nCol()];
	memset(VT,0x00,InputMatrix.nCol()*InputMatrix.nCol()*sizeof(double));
	integer LDVT=iACol;

//	double* WORK=new double[InputMatrix.nRow()*InputMatrix.nCol()];
//	memset(WORK,0x00,InputMatrix.nRow()*InputMatrix.nCol()*sizeof(double));
	double* WORK=new double[5*(InputMatrix.nRow()+InputMatrix.nCol())];
	memset(WORK,0x00,5*(InputMatrix.nRow()+InputMatrix.nCol())*sizeof(double));
//	integer LWORK=max(1,3*min(iARow,iACol)+max(iARow,iACol),5*min(iARow,iACol))+1000;
	integer LWORK=5*(InputMatrix.nRow()+InputMatrix.nCol());


	integer INFO=0;

	dgesvd_( &JOBU, &JOBVT, &iARow, &iACol, A, &LDA, S, U, &LDU, VT, &LDVT, WORK, &LWORK, &INFO);

	if (INFO==0)
	{
		for (int i=0;i<InputMatrix.nRow();i++)
		{
			for (int j=0;j<InputMatrix.nRow();j++)
			{
				OutputU[i][j]=U[i+j*InputMatrix.nRow()];
			}
		}
		for (int i=0;i<InputMatrix.nRow();i++)
		{
			for (int j=0;j<InputMatrix.nCol();j++)
			{
				if (i==j)
				{
					OutputSigma[i][j]=S[i];
				}
				else
				{
					OutputSigma[i][j]=0;
				}
			}
		}
		for (int i=0;i<InputMatrix.nCol();i++)
		{
			for (int j=0;j<InputMatrix.nCol();j++)
			{
				OutputV[i][j]=VT[i*InputMatrix.nCol()+j];
			}
		}
		delete [] A;
		delete [] S;
		delete [] U;
		delete [] VT;
		delete [] WORK;

		return true;
	}

	delete [] A;
	delete [] S;
	delete [] U;
	delete [] VT;
	delete [] WORK;
	return false;
}

bool CMath::SparseMatrixMultiply(vector<vector<double>> MatrixA,vector<vector<double>> MatrixB,vector<vector<double>>& Result)
{
	if (MatrixA.front().size()!=MatrixB.size())
	{
		return false;
	}
	
	int iANonZeroSize=0;
	for (unsigned int i=0;i<MatrixA.size();i++)
	{
		for (unsigned int j=0;j<MatrixA.front().size();j++)
		{
			if (MatrixA.at(i).at(j)!=0)
			{
				iANonZeroSize++;
			}
		}
	}
//	CPPL::dssmatrix A(MatrixA.size(),MatrixA.front().size(),iANonZeroSize);

	KW_SparseMatrix A(MatrixA.size(),MatrixA.front().size(),iANonZeroSize);
	for (unsigned int i=0;i<MatrixA.size();i++)
	{
		for (unsigned int j=0;j<MatrixA.front().size();j++)
		{
			if (MatrixA.at(i).at(j)!=0)
			{
				A.fput(i,j,MatrixA.at(i).at(j));
			}
		}
	}

	int iBNonZeroSize=0;
	for (unsigned int i=0;i<MatrixB.size();i++)
	{
		for (unsigned int j=0;j<MatrixB.front().size();j++)
		{
			if (MatrixB.at(i).at(j)!=0)
			{
				iBNonZeroSize++;
			}
		}
	}
//	CPPL::dssmatrix B(MatrixB.size(),MatrixB.front().size(),iBNonZeroSize);
	KW_SparseMatrix B(MatrixB.size(),MatrixB.front().size(),iBNonZeroSize);
	for (unsigned int i=0;i<MatrixB.size();i++)
	{
		for (unsigned int j=0;j<MatrixB.front().size();j++)
		{
			if (MatrixB.at(i).at(j)!=0)
			{
				B.fput(i,j,MatrixB.at(i).at(j));
			}
		}
	}


//	CPPL::dssmatrix C=A*B;
	KW_SparseMatrix C(A.m,B.n,0);

	KW_SparseMatrix::KW_multiply(A,B,C);

	int iIndex=0;
	Result.clear();
	for (int i=0;i<C.m;i++)
	{
		vector<double> CurrentRow;
		for (int j=0;j<C.n;j++)
		{
			iIndex=0;
			while (iIndex<C.vol)
			{
				if (C.indx[iIndex]==i&&C.jndx[iIndex]==j)
				{
					break;
				}
				iIndex++;
			}
			if (iIndex!=C.vol)
			{
				CurrentRow.push_back(C.array[iIndex]);
			}
			else
			{
				CurrentRow.push_back(0);
			}
		}
		Result.push_back(CurrentRow);
	}

	return true;
}

void CMath::TAUCSFactorize(vector<vector<double> > LeftMatrixA,vector<vector<double>>& LeftMatrixAT)
{
	clock_t FactorizeBegin=clock();   

	//convert LeftMatrixA^T to SparseMatrix
	SparseMatrix SMLeftMatrixAT(LeftMatrixA.front().size());
	SMLeftMatrixAT.m = LeftMatrixA.size();
	for (unsigned int i=0;i<LeftMatrixA.size();i++)
	{
		for (unsigned int j=0;j<LeftMatrixA.at(i).size();j++)
		{
			if (LeftMatrixA.at(i).at(j)!=0)
			{
				SMLeftMatrixAT[j][i]=LeftMatrixA.at(i).at(j);
			}
		}
	}
	for (unsigned int i=0;i<LeftMatrixA.front().size();i++)
	{
		vector<double> CurrentRow;
		for (unsigned int j=0;j<LeftMatrixA.size();j++)
		{
			CurrentRow.push_back(LeftMatrixA.at(j).at(i));
		}
		LeftMatrixAT.push_back(CurrentRow);
	}
	//LeftMatrixA^T*LeftMatrixA
	SparseMatrix LeftMatrixATA;
	multiplyAAT(SMLeftMatrixAT, LeftMatrixATA);
	//store ATA in taucs_ccs_matrix;
	int dim = LeftMatrixATA.size(); 
	int nnz = 0;
	for (int i = 0; i < dim; ++i)
	{
		int num = LeftMatrixATA[i].size();
		nnz += num;
	}
	taucs_ccs_matrix* A = taucs_dccs_create(dim, dim, nnz);
	convert2taucs(LeftMatrixATA,A,dim,nnz);

	// 1) Reordering
	taucs_ccs_matrix*  Aod;
	taucs_ccs_order(A, &perm, &invperm, "metis");
	Aod = taucs_ccs_permute_symmetrically(A, perm, invperm);

	// 2) Factoring
	F = taucs_ccs_factor_llt_mf(Aod);	
	taucs_ccs_free(Aod);			
	taucs_ccs_free(A);

	clock_t FactorizeEnd=clock();   
	cout<<"Factoriz time: %f\n"<<float(FactorizeEnd-FactorizeBegin);
}

void CMath::TAUCSFactorize(SparseMatrix LeftMatrixA,SparseMatrix& LeftMatrixAT)
{
	clock_t FactorizeBegin=clock();   

	//compute A^T
	LeftMatrixAT.m=LeftMatrixA.NRows();
	for (unsigned int i=0;i<LeftMatrixA.size();i++)
	{
		for (map<int, double>::const_iterator j = LeftMatrixA.at(i).begin(); j != LeftMatrixA.at(i).end(); ++j)
		{
			LeftMatrixAT.at(j->first)[i]=j->second;
		}
	}

	//LeftMatrixA^T*LeftMatrixA
	SparseMatrix LeftMatrixATA;
	multiplyAAT(LeftMatrixAT, LeftMatrixATA);

	//store ATA in taucs_ccs_matrix;
	int dim = LeftMatrixATA.size(); 
	int nnz = 0;
	for (int i = 0; i < dim; ++i)
	{
		int num = LeftMatrixATA[i].size();
		nnz += num;
	}
	taucs_ccs_matrix* A = taucs_dccs_create(dim, dim, nnz);
	
	convert2taucs(LeftMatrixATA,A,dim,nnz);

	// 1) Reordering
	taucs_ccs_matrix*  Aod;
	taucs_ccs_order(A, &perm, &invperm, "metis");
	Aod = taucs_ccs_permute_symmetrically(A, perm, invperm);

	// 2) Factoring
	F = taucs_ccs_factor_llt_mf(Aod);	
	taucs_ccs_free(Aod);			
	taucs_ccs_free(A);

	assert(F!=NULL);

	clock_t FactorizeEnd=clock();
	cout<<"Factoriz time: %f\n"<<float(FactorizeEnd-FactorizeBegin)<<endl;
}

bool CMath::TAUCSComputeLSE(vector<vector<double> > LeftMatrixAT,
							vector<vector<double> > RightMatrixB,
							vector<vector<double> >& Result)
{
	clock_t LSEBegin=clock();   

	assert(LeftMatrixAT.front().size()==RightMatrixB.front().size());


	//compute new RHS
	vector<vector<float>> NewRHS;
	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		vector<float> NewRHSCol;
		for (unsigned int j=0;j<LeftMatrixAT.size();j++)
		{
			float fNewValue=0;
			for (unsigned int k=0;k<LeftMatrixAT.at(j).size();k++)
			{
				if (LeftMatrixAT.at(j).at(k)!=0&&RightMatrixB.at(i).at(k)!=0)
				{
					fNewValue=fNewValue+LeftMatrixAT.at(j).at(k)*RightMatrixB.at(i).at(k);
				}
			}
			NewRHSCol.push_back(fNewValue);
		}
		NewRHS.push_back(NewRHSCol);
	}

	//vector<vector<double>> NewRightMatrixB,NewRHStemp,NewRHS;
	//for (unsigned int i=0;i<RightMatrixB.front().size();i++)
	//{
	//	vector<double> CurrentRow;
	//	for (unsigned int j=0;j<RightMatrixB.size();j++)
	//	{
	//		CurrentRow.push_back(RightMatrixB.at(j).at(i));
	//	}
	//	NewRightMatrixB.push_back(CurrentRow);
	//}
	//SparseMatrixMultiply(LeftMatrixAT,NewRightMatrixB,NewRHStemp);
	//for (unsigned int i=0;i<NewRHStemp.front().size();i++)
	//{
	//	vector<double> CurrentRow;
	//	for (unsigned int j=0;j<NewRHStemp.size();j++)
	//	{
	//		CurrentRow.push_back(NewRHStemp.at(j).at(i));
	//	}
	//	NewRHS.push_back(CurrentRow);
	//}


	clock_t LSEEnd=clock();   
	cout<<"compute new RHS time: %f\n"<<float(LSEEnd-LSEBegin);

	for (unsigned int i=0;i<NewRHS.size();i++)
	{
		float* bod=new float[NewRHS.at(i).size()]; 
		float* result=new float[NewRHS.at(i).size()];
		float* RHS=new float[NewRHS.at(i).size()];
		memset(RHS,0x00,NewRHS.at(i).size()*sizeof(float));
		for (unsigned int j=0;j<NewRHS.at(i).size();j++)
		{
			RHS[j]=NewRHS.at(i).at(j);
		}
		float* xod=new float[NewRHS.at(i).size()];

		taucs_vec_permute(NewRHS.at(i).size(), TAUCS_SINGLE, RHS, bod, perm);
		int iResult=taucs_supernodal_solve_llt(F, xod, bod);	
		taucs_vec_ipermute(NewRHS.at(i).size(), TAUCS_SINGLE, xod, result, perm);

		vector<double> CurrentResult;
		for (unsigned int j=0;j<NewRHS.at(i).size();j++)
		{
			CurrentResult.push_back(result[j]);
		}
		Result.push_back(CurrentResult);

		delete bod;
		delete result;
		delete xod;
		delete RHS;
	}

	clock_t LSESolving=clock();
	cout<<"LSE Solving time: %f\n"<<float(LSESolving-LSEEnd);

	return true;
}

bool CMath::TAUCSComputeLSE(SparseMatrix LeftMatrixAT,vector<vector<double> > RightMatrixB,vector<vector<double> >& Result)
{
	clock_t LSEBegin=clock();   

	assert(LeftMatrixAT.NCols()==RightMatrixB.front().size());


	//compute new RHS
	vector<vector<double>> NewRHS;

	for (unsigned int i=0;i<RightMatrixB.size();i++)
	{
		vector<double> NewRHSCol;
		for (unsigned int j=0;j<LeftMatrixAT.size();j++)
		{
			double fNewValue=0;
			for (map<int, double>::const_iterator k = LeftMatrixAT.at(j).begin(); k != LeftMatrixAT.at(j).end(); ++k)
			{
				fNewValue=fNewValue+(k->second)*(RightMatrixB.at(i).at(k->first));
			}
			NewRHSCol.push_back(fNewValue);
		}
		NewRHS.push_back(NewRHSCol);
	}

	clock_t LSEEnd=clock();
	cout<<"compute new RHS time: %f\n"<<float(LSEEnd-LSEBegin);

	for (unsigned int i=0;i<NewRHS.size();i++)
	{
		double* bod=new double[NewRHS.at(i).size()]; 
		double* result=new double[NewRHS.at(i).size()];
		double* RHS=new double[NewRHS.at(i).size()];
		memset(RHS,0x00,NewRHS.at(i).size()*sizeof(double));
		for (unsigned int j=0;j<NewRHS.at(i).size();j++)
		{
			RHS[j]=NewRHS.at(i).at(j);
		}
		double* xod=new double[NewRHS.at(i).size()];

		taucs_vec_permute(NewRHS.at(i).size(), TAUCS_DOUBLE, RHS, bod, perm);
		int iResult=taucs_supernodal_solve_llt(F, xod, bod);	
		taucs_vec_ipermute(NewRHS.at(i).size(), TAUCS_DOUBLE, xod, result, perm);

		vector<double> CurrentResult;
		for (unsigned int j=0;j<NewRHS.at(i).size();j++)
		{
			CurrentResult.push_back(result[j]);
		}
		Result.push_back(CurrentResult);

		delete bod;
		delete result;
		delete xod;
		delete RHS;
	}

	clock_t LSESolving=clock();
	cout<<"LSE Solving time: %f\n"<<float(LSESolving-LSEEnd);

	return true;
}

void CMath::TAUCSClear()
{
	taucs_supernodal_factor_free(F);
	delete perm;
	delete invperm;
	F=NULL;
	perm=NULL;invperm=NULL;
}

// C = A A'
// A is assumed to be unsymmetric
void CMath::multiplyAAT(SparseMatrix &A, SparseMatrix &C)
{
	// C_ij = Sum_k A_ik*B_kj
	// B = A'  b_kj = a_jk
	// C_ij = Sum_k A_ik*A_jk

	// Let us compute only the upper half of C
	C.clear();
	int dim = A.NRows();
	C.resize(dim);
	C.m = dim;
	for (int i = 0; i < dim; ++i)
	{
		for (int j = i; j < dim; ++j)
		{
			double sum = 0.;
			if (i == j)
			{
				for (map<int, double>::const_iterator k = A[i].begin(); k != A[i].end(); ++k)
					sum += (*k).second*(*k).second;
			}
			else
			{
				for (map<int, double>::const_iterator k = A[i].begin(); k != A[i].end(); ++k)
				{
					map<int, double>::const_iterator kk = A[j].find((*k).first);
					if (kk != A[j].end())
						sum += (*k).second*(*kk).second;
				}
			}
			if (sum != 0)
				C[i][j] = sum;
		}
	}
}

// a = b
void CMath::convert2dense(Matrix& a, SparseMatrix& b)
{
	a.resize(b.NRows(), b.NCols());
	float ftemp=0.0;
	fill(a.begin(), a.end(), ftemp);
	for (unsigned int i = 0; i < b.NRows(); ++i)
		for (map<int, double>::const_iterator j = b[i].begin(); j != b[i].end(); ++j)
			a(i, (*j).first) = (*j).second;
}

void CMath::printMatrix(Matrix &a, char *comment)
{
	cout << comment << endl;
	for (unsigned int i = 0; i < a.NRows(); ++i)
	{
		for (unsigned int j = 0; j < a.NCols(); ++j)
			cout << setw(10) << a(i, j);
		cout << endl;
	}
	cout << endl;
}

// Symmetric matrix C above is actually equivalent to CCS required by TAUCS
// Attention! This is working only for symmetric matrices
void CMath::convert2taucs(SparseMatrix &C,taucs_ccs_matrix* A,int dim,int nnz) 
{
	//int dim = C.size(); 
	//int nnz = 0;
	//for (int i = 0; i < dim; ++i)
	//{
	//	int num = C[i].size();
	//	nnz += num;
	//}
	//A = taucs_dccs_create(dim, dim, nnz);
	A->flags = TAUCS_DOUBLE| TAUCS_SYMMETRIC | TAUCS_LOWER;
	int pos = 0;
	for (int i = 0; i < dim; ++i)
	{
		A->colptr[i] = pos;
		for (map<int, double>::const_iterator j = C[i].begin(); j != C[i].end(); ++j)
		{
			A->rowind[pos] = (*j).first;
			A->values.d[pos] = (*j).second;	
			++pos;
		}
	}
	A->colptr[dim] = nnz;
}

//only support square symmetric matrix
void CMath::convertTaucs2Sparse(taucs_ccs_matrix* A,SparseMatrix &C)
{
	int iPos=0;
	//total non-zero elements number
	int iNZSumNum=0;

	for (int i=0;i<A->n;i++)//for each row
	{
		if (A->colptr[i+1]-iNZSumNum!=0)//if nonzero exist in this row
		{
			for (int j=0;j<A->n;j++)
			{
				if (j==A->rowind[iPos])//if column index equals to that in A
				{
					C[i][j]=A->values.d[iPos];
					iPos++;
					iNZSumNum++;
				}
			}
		}
	}
}

void CMath::General2Sparse(GeneralMatrix GMatrix,SparseMatrix& SMatrix)
{
	for (int i=0;i<GMatrix.nRow();i++)
	{
		for (int j=0;j<GMatrix.nCol();j++)
		{
			if (GMatrix[i][j]!=0)
			{
				SMatrix[i][j]=GMatrix[i][j];
			}
		}
	}
}

void CMath::Sparse2General(SparseMatrix SMatrix,GeneralMatrix& GMatrix)
{
	GMatrix.Zero();
	for (int i=0;i<(int)SMatrix.NRows();i++)
	{
		for (map<int, double>::const_iterator j = SMatrix[i].begin(); j != SMatrix[i].end(); j++)
		{
			GMatrix[i][(*j).first]=(*j).second;
		}
	}
}


void CMath::testCLAPACK()
{
	//char JOBU;
	//char JOBVT;

	//int i;

	////数据类型integer是fortran里的。这里在C++下可以使用的原因是f2c.h文件中已经作了定义
	//integer M = 4;
	//integer N = 4;
	//integer LDA = M;
	//integer LDU = M;
	//integer LDVT = N;
	//integer LWORK;
	//integer INFO;

	//integer mn = min( M, N );

	//integer MN = max( M, N );

	//double a[4*4] = { 16.0, 5.0, 9.0 , 4.0, 2.0, 11.0, 7.0 , 14.0, 3.0, 10.0, 6.0, 15.0, 13.0, 8.0, 12.0, 1.0};
	//double s[4];
	//double wk[201];
	//double uu[4*4];
	//double vt[4*4];

	//JOBU = 'A';

	//JOBVT = 'A';

	//LWORK = 201;

	///* Subroutine int dgesvd_(char *jobu, char *jobvt, integer *m, integer *n,
	//doublereal *a, integer *lda, doublereal *s, doublereal *u, integer *
	//ldu, doublereal *vt, integer *ldvt, doublereal *work, integer *lwork,
	//integer *info)
	//*/
	//dgesvd_( &JOBU, &JOBVT, &M, &N, a, &LDA, s, uu, &LDU, vt, &LDVT, wk, &LWORK, &INFO);

	//printf("INFO=%d \n", INFO );         

	//for ( i= 0; i< 4; i++ ) {
	//	printf("s[ %d ] = %f\n", i, s[ i ] );
	//}
}

void CMath::testTAUCS()
{
	SparseMatrix LeftMatrixA(3);
	LeftMatrixA.m=2;
	LeftMatrixA[0][0]=1;LeftMatrixA[0][1]=2;//LeftMatrixA[0][2]=9;
	LeftMatrixA[1][0]=3;//LeftMatrixA[1][2]=8;
	LeftMatrixA[2][0]=4;LeftMatrixA[2][1]=7;//LeftMatrixA[2][2]=5;
	vector<vector<double>> RightMatrixB;
	RightMatrixB.resize(3);
	vector<double> Row;
	Row.push_back(3);Row.push_back(7);Row.push_back(1);
	RightMatrixB.at(0)=Row;
	Row.clear();Row.push_back(2);Row.push_back(5);Row.push_back(8);
	RightMatrixB.at(1)=Row;
	Row.clear();Row.push_back(4);Row.push_back(9);Row.push_back(3);
	RightMatrixB.at(2)=Row;
	vector<vector<double>> Result;
	ComputeLSE(LeftMatrixA,RightMatrixB,Result);

	int j=0;



//	GeneralMatrix GMMatrix(3,4);
//	GMMatrix[0][0]=4;GMMatrix[0][1]=3;GMMatrix[0][2]=5;GMMatrix[0][3]=9;
//	GMMatrix[1][0]=1;GMMatrix[1][1]=6;GMMatrix[1][2]=9;GMMatrix[1][3]=1;
//	GMMatrix[2][0]=3;GMMatrix[2][1]=0;GMMatrix[2][2]=2;GMMatrix[2][2]=7;
////	GMMatrix[3][0]=7;GMMatrix[3][1]=3;GMMatrix[3][2]=0;
//
//	GeneralMatrix U(3,3),Sigma(3,4),V(4,4);
//
//	ComputeSVD(GMMatrix,U,Sigma,V);
//
//	for (int i=0;i<GMMatrix.nRow();i++)
//	{
//		for (int j=0;j<GMMatrix.nCol();j++)
//		{
//			DBWindowWrite("%f ",GMMatrix[i][j]);
//		}
//		DBWindowWrite("\n");
//	}
//	DBWindowWrite("\n");
//
//	for (int i=0;i<U.nRow();i++)
//	{
//		for (int j=0;j<U.nCol();j++)
//		{
//			DBWindowWrite("%f ",U[i][j]);
//		}
//		DBWindowWrite("\n");
//	}
//	DBWindowWrite("\n");
//
//	for (int i=0;i<Sigma.nRow();i++)
//	{
//		for (int j=0;j<Sigma.nCol();j++)
//		{
//			DBWindowWrite("%f ",Sigma[i][j]);
//		}
//		DBWindowWrite("\n");
//	}
//	DBWindowWrite("\n");
//
//	for (int i=0;i<V.nRow();i++)
//	{
//		for (int j=0;j<V.nCol();j++)
//		{
//			DBWindowWrite("%f ",V[i][j]);
//		}
//		DBWindowWrite("\n");
//	}
//	DBWindowWrite("\n");
//
//	int j=0;

//	vector<vector<double>> A;
//	vector<double> CurrentRow;
//	CurrentRow.push_back(2);CurrentRow.push_back(-1);CurrentRow.push_back(-1);
//	A.push_back(CurrentRow);
//	CurrentRow.clear();
//	CurrentRow.push_back(0);CurrentRow.push_back(1);CurrentRow.push_back(0);
//	A.push_back(CurrentRow);
//	CurrentRow.clear();
//	CurrentRow.push_back(0);CurrentRow.push_back(0);CurrentRow.push_back(1);
//	A.push_back(CurrentRow);
//	CurrentRow.clear();
//	CurrentRow.push_back(1);CurrentRow.push_back(0);CurrentRow.push_back(0);
//	A.push_back(CurrentRow);
//	CurrentRow.clear();
//
//	vector<vector<double>> AT;
//	CMath CMathtest;
//	CMathtest.F=CMathtest.perm=CMathtest.invperm=NULL;
//
//	CMathtest.TAUCSFactorize(A,AT);
//
//	vector<vector<double>> B,Result;
//	CurrentRow.push_back(-3);CurrentRow.push_back(3);CurrentRow.push_back(2);CurrentRow.push_back(1);
//	B.push_back(CurrentRow);
//	CurrentRow.clear();
//	CurrentRow.push_back(1);CurrentRow.push_back(7);CurrentRow.push_back(0);CurrentRow.push_back(4);
//	B.push_back(CurrentRow);
//	CMathtest.TAUCSComputeLSE(AT,B,Result);
//
//	CMathtest.TAUCSClear();
//
//	int iwww=0;
//
////	float value[5];
////	int colptr[4];
////	int rowind[5];
////	float RHS[3];
////
////	// create CCS matrix structure using vector class
////	value[0]=2.0;
////	value[1]=-1.0;
////	value[2]=2.0;
////	value[3]=-1.0;
////	value[4]=2.0;
////
////	colptr[0]=0;
////	colptr[1]=2;
////	colptr[2]=4;
////	colptr[3]=5;
////
////	rowind[0]=0;
////	rowind[1]=1;
////	rowind[2]=1;
////	rowind[3]=2;
////	rowind[4]=2;
////
////	// create right-hand size vector object
////	RHS[0] = 10.0;
////	RHS[1] = 2.0;
////	RHS[2] = 3.0;
////
////	// resize vectors.
////	int dim = 3;
////
////	// create TAUCS matrix from vector objects an, jn and ia
////	taucs_ccs_matrix* A = taucs_dccs_create(dim, dim, 5);
////	
//////	taucs_ccs_matrix  A; // a matrix to solve Ax=b in CCS format
//////	A->n = dim;
//////	A->m = dim;
////	A->flags = (TAUCS_SINGLE | TAUCS_SYMMETRIC | TAUCS_LOWER);
////	A->colptr = colptr;
////	A->rowind = rowind;
////	A->values.s = value;
////
////	// solve the linear system
////	//Using TAUCS low-level routines
////	int*         perm;
////	int*         invperm;
////	taucs_ccs_matrix*  Aod;
////
////	void* F = NULL;
////
////	// 1) Reordering
////	taucs_ccs_order(A, &perm, &invperm, "metis");
////	Aod = taucs_ccs_permute_symmetrically(A, perm, invperm);
////
////	// 2) Factoring
////	F = taucs_ccs_factor_llt_mf(Aod);	
////	taucs_ccs_free(Aod);			
//////	taucs_ccs_free(A);			
////
////
////	// 3) Back substitution and reodering the solution back
////	//RHS related
////	float bod[3]; 
////	// allocate TAUCS solution vector
////	float result[3];
////	float xod[3];
////	taucs_vec_permute(dim, TAUCS_SINGLE, RHS, bod, perm);
////	int i=taucs_supernodal_solve_llt(F, xod, bod);	
////	taucs_vec_ipermute(dim, TAUCS_SINGLE, xod, result, perm);
////
////	RHS[0]=2;RHS[1]=5;RHS[2]=8;
////	taucs_vec_permute(dim, TAUCS_SINGLE, RHS, bod, perm);
////	i=taucs_supernodal_solve_llt(F, xod, bod);	
////	taucs_vec_ipermute(dim, TAUCS_SINGLE, xod, result, perm);
////
////	// deallocate the factorization
////	taucs_supernodal_factor_free(F);
}

void CMath::ConvertMatToQuat(vector<double> Mat,vector<double>& Quat)
{
	assert(Mat.size()==9);
	double dS,dQW,dQX,dQY,dQZ;
	dS=dQW=dQX=dQY=dQZ=0;

	double dTrace=Mat.at(0)+Mat.at(4)+Mat.at(8);
	if (dTrace>0)
	{
		dS=sqrt(dTrace+1.0)*2;// S=4*qw
		dQW=0.25*dS;
		dQX=(Mat.at(7)-Mat.at(5))/dS;
		dQY=(Mat.at(2)-Mat.at(6))/dS;
		dQZ=(Mat.at(3)-Mat.at(1))/dS;
	}
	else if ((Mat.at(0)>Mat.at(4))&&(Mat.at(0)>Mat.at(8)))
	{
		dS=sqrt(1.0+Mat.at(0)-Mat.at(4)-Mat.at(8))*2;// S=4*qy
		dQW=(Mat.at(7)-Mat.at(5))/dS;
		dQX=0.25*dS;
		dQY=(Mat.at(1)+Mat.at(3))/dS;
		dQZ=(Mat.at(2)+Mat.at(6))/dS;
	}
	else if (Mat.at(4)>Mat.at(8))
	{
		dS=sqrt(1.0+Mat.at(4)-Mat.at(0)-Mat.at(8))*2;// S=4*qy
		dQW=(Mat.at(2)-Mat.at(6))/dS;
		dQX=(Mat.at(1)+Mat.at(3))/dS;
		dQY=0.25*dS;
		dQZ=(Mat.at(5)+Mat.at(7))/dS;
	}
	else
	{
		dS=sqrt(1.0+Mat.at(8)-Mat.at(0)-Mat.at(4))*2;// S=4*qz
		dQW=(Mat.at(3)-Mat.at(1))/dS;
		dQX=(Mat.at(2)+Mat.at(6))/dS;
		dQY=(Mat.at(5)+Mat.at(7))/dS;
		dQZ=0.25*dS;
	}

	Quat.clear();
	Quat.push_back(dQW);
	Quat.push_back(dQX);
	Quat.push_back(dQY);
	Quat.push_back(dQZ);
}

void CMath::ConvertQuatToMat(vector<double> Quat,vector<double>& Mat)
{
	assert(Quat.size()==4);
	Mat.clear();
	Mat.push_back(1.0-2.0*Quat.at(2)*Quat.at(2)-2.0*Quat.at(3)*Quat.at(3));
	Mat.push_back(2.0*Quat.at(1)*Quat.at(2)-2.0*Quat.at(0)*Quat.at(3));
	Mat.push_back(2.0*Quat.at(1)*Quat.at(3)+2.0*Quat.at(0)*Quat.at(2));
	Mat.push_back(2.0*Quat.at(1)*Quat.at(2)+2.0*Quat.at(0)*Quat.at(3));
	Mat.push_back(1.0-2.0*Quat.at(1)*Quat.at(1)-2.0*Quat.at(3)*Quat.at(3));
	Mat.push_back(2.0*Quat.at(2)*Quat.at(3)-2.0*Quat.at(0)*Quat.at(1));
	Mat.push_back(2.0*Quat.at(1)*Quat.at(3)-2.0*Quat.at(0)*Quat.at(2));
	Mat.push_back(2.0*Quat.at(2)*Quat.at(3)+2.0*Quat.at(0)*Quat.at(1));
	Mat.push_back(1.0-2.0*Quat.at(1)*Quat.at(1)-2.0*Quat.at(2)*Quat.at(2));
}

void CMath::NormalizeQuat(vector<double>& Quat)
{
	assert(Quat.size()==4);
	double Norm=sqrt(Quat.at(0)*Quat.at(0)+Quat.at(1)*Quat.at(1)
		+Quat.at(2)*Quat.at(2)+Quat.at(3)*Quat.at(3));

	if (Norm==0.0)
	{
		Quat.at(0)=1.0;
		Quat.at(1)=Quat.at(2)=Quat.at(3)=0.0;
	}
	else
	{
		Quat.at(0)=Quat.at(0)/Norm;
		Quat.at(1)=Quat.at(1)/Norm;
		Quat.at(2)=Quat.at(2)/Norm;
		Quat.at(3)=Quat.at(3)/Norm;
	}
}