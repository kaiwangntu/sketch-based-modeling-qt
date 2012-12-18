#include "./PreDef.h"
#include "GeneralMatrix.h"
#include <math.h>


GeneralMatrix::GeneralMatrix()
{  
   pHead = new HeadNode;

     pHead->nRow = 0;
     pHead->nCol = 0;
   pHead->pFirst = NULL;
}

GeneralMatrix::GeneralMatrix(const GeneralMatrix& other)
{
	pHead = new HeadNode;
	
	 pHead->nRow = 0;
	 pHead->nCol = 0;
   pHead->pFirst = NULL;

	  AddRows(other.nRow(),0.0);
	  AddCols(other.nCol()-1,0.0);

	  *this += other; 
}

GeneralMatrix::GeneralMatrix(int iRow,int iCol,ELEMTYPE Val)
{
	pHead = new HeadNode;
	
	 pHead->nRow = 0;
	 pHead->nCol = 0;
   pHead->pFirst = NULL;

	if(iRow<=0||iCol<=0) return;

	  AddRows(iRow,Val);
	  AddCols(iCol-1,Val);
}

GeneralMatrix::GeneralMatrix(int iRow,int iCol)
{
	pHead = new HeadNode;
	
	 pHead->nRow = 0;
	 pHead->nCol = 0;
   pHead->pFirst = NULL;

	if(iRow<=0||iCol<=0) return;

     AddRows(iRow,0.0);
	 AddCols(iCol-1,0.0);
}

int GeneralMatrix::nRow()const
{
return this->pHead->nRow; 
}

int GeneralMatrix::nCol()const
{
return this->pHead->nCol;
}

bool GeneralMatrix::AddRows(int in,ELEMTYPE Val)const
{
         int i,j,index=0;
   ElemNode *p,*pnew,*pL; 
   
   if (in==0) return FALSE;

   if (in<0) return DelRows(in*-1);

   if(pHead->pFirst==NULL)
   {
	   this->pHead->pFirst = new ElemNode;

        if(pHead->pFirst == NULL)
			return FALSE;

       pHead->nRow = 1;
	   pHead->nCol = 1;
	   pHead->pFirst->pDown = NULL;
	   pHead->pFirst->pRight = NULL;
	   pHead->pFirst->val = Val;
	   index = 1;
   }

   i = pHead->nRow-1;
   for(;index<in;index++,i++,pHead->nRow++)
	   for(pL=NULL,j=0;j<pHead->nCol;j++)
	   {
            p = GetElemP(i,j);
         pnew = new ElemNode;
		     if(pnew==NULL) return FALSE;

			 pnew->pDown  = NULL;
			 pnew->pRight = NULL;
			 pnew->val    = Val;
			 
			 if(pL!=NULL)
				 pL->pRight = pnew;
			 
			     pL = pnew;
		   p->pDown = pnew;
	   }

return TRUE;
}

bool GeneralMatrix::AddCols(int in,ELEMTYPE Val)const
{
	int i,j,index=0;
	ElemNode *p,*pnew,*pU; 
	
	if (in==0) return FALSE;
	
	if (in<0) return DelCols(in*-1);
	
	if(pHead->pFirst==NULL)
	{
		pHead->pFirst = new ElemNode;
		
        if(pHead->pFirst == NULL)
			return FALSE;
		
		pHead->nRow = 1;
		pHead->nCol = 1;
		pHead->pFirst->pDown = NULL;
		pHead->pFirst->pRight = NULL;
		pHead->pFirst->val = Val;
		index = 1;
	}

	  j = pHead->nCol-1;
 	for(;index<in;index++,j++,pHead->nCol++)
       for(pU=NULL,i=0;i<pHead->nRow;i++)
	   {
		      p = GetElemP(i,j);
           pnew = new ElemNode;
		       if(pnew==NULL) return FALSE;

		   pnew->pDown  = NULL;
		   pnew->pRight = NULL;
		   pnew->val    = Val;

		   if(pU!=NULL)
              pU->pDown = pnew;

                  pU = pnew;
		   p->pRight = pnew;
	   }
	  
return TRUE;
}

bool GeneralMatrix::DelRows(int in)const
{
    int i,j,n,index;

    if(in<0) return FALSE;

	if(pHead->nRow==0) return FALSE;
	
    index = in>=pHead->nRow ? pHead->nRow-1 : in;

    for (i=pHead->nRow-1,n=0;n<index;i--,n++,pHead->nRow--)
        for (j=0;j<pHead->nCol;j++)
             delete GetElemP(i,j);
    
	for(i=pHead->nRow-1,j=0;j<pHead->nCol;j++)
	    	GetElemP(i,j)->pDown = NULL;

	if(index!=in)
	{
    for (j=pHead->nCol-1;j>=0;j--)
         delete GetElemP(0,j);
	pHead->pFirst = NULL;
	  pHead->nCol = 0;
	  pHead->nRow = 0;
    }
return TRUE;
}

bool GeneralMatrix::DelCols(int in)const
{
     int i,j,n,index;

	 if(in<0) return FALSE;
	 
	 if(pHead->nCol==0) return FALSE;
	 
     index = in>=pHead->nCol ? pHead->nCol-1 : in;

	 for (j=pHead->nCol-1,n=0;n<index;j--,n++,pHead->nCol--)
		 for (i=pHead->nRow-1;i>=0;i--)
             delete GetElemP(i,j);

	 for(j=pHead->nCol-1,i=0;i<pHead->nRow;i++)
	    	GetElemP(i,j)->pRight = NULL;

	   if(index!=in)
	 {
		 for (i=pHead->nRow-1;i>=0;i--)
			 delete GetElemP(i,0);
		 pHead->pFirst = NULL;
		 pHead->nCol = 0;
		 pHead->nRow = 0;
	  }  

return TRUE;
}

bool GeneralMatrix::ReSize(int iRow,int iCol, ELEMTYPE Val)const
{
	if(iRow<0||iCol<0) return FALSE;

	if(iRow==0) iCol = 0;

	AddRows(iRow-nRow(),Val);
	AddCols(iCol-nCol(),Val);

return TRUE;
}

bool GeneralMatrix::ReSize(int iRow,int iCol)const
{
	return ReSize(iRow,iCol,0.0);
}

bool GeneralMatrix::CutIn2_Vertical(GeneralMatrix& Left,GeneralMatrix& Right,int iCol)
{
  int i,j;

   if (nCol()==0)
   {
	    Left.ReSize(0,0);
	   Right.ReSize(0,0);
	   return TRUE;
   }

   if(iCol>nCol()||iCol<0) return FALSE;

    if(!Left.ReSize(nRow(),iCol,ERRORVAL))
		return FALSE;

   if(!Right.ReSize(nRow(),nCol()-iCol,ERRORVAL))
	   return FALSE;

	   for(i=0;i<nRow();i++)
          for(j=0;j<iCol;j++)
               Left[i][j] = GetElem(i,j);
       
	   for(i=0;i<nRow();i++)
		   for(j=0;j<Right.nCol();j++)
			  Right[i][j] = GetElem(i,j+iCol);
			   
   return TRUE;
}

bool GeneralMatrix::CutIn2_Across(GeneralMatrix& Upper,GeneralMatrix& Lower,int iRow)
{
	int i,j;
	
	if (nCol()==0)
	{
		Upper.ReSize(0,0);
		Lower.ReSize(0,0);
		return TRUE;
	}
	
	if(iRow>nRow()||iRow<0) return FALSE;
	
    if(!Upper.ReSize(iRow,nCol(),ERRORVAL))
		return FALSE;
	
	if(!Lower.ReSize(nRow()-iRow,nCol(),ERRORVAL))
		return FALSE;
	
	   for(i=0;i<iRow;i++)
		   for(j=0;j<nCol();j++)
               Upper[i][j] = GetElem(i,j);
		   
		   for(i=0;i<Lower.nRow();i++)
			   for(j=0;j<nCol();j++)
				   Lower[i][j] = GetElem(i+iRow,j);
			   
   return TRUE;
}

bool GeneralMatrix::Jointer_Right(GeneralMatrix& other)
{
    if(other.nRow()!=nRow()) return FALSE;
	
	if(!AddCols(other.nCol(),ERRORVAL))
		return FALSE;

	Paste(other,0,nCol()-other.nCol());
   
return TRUE;
}

bool GeneralMatrix::Jointer_bottom(GeneralMatrix& other)
{
	if(other.nCol()!=nCol()) return FALSE;
	
	if(!AddRows(other.nRow(),ERRORVAL))
		return FALSE;	
	
return Paste(other,nRow()-other.nRow(),0);
}

bool GeneralMatrix::Jointer_Diagonal(GeneralMatrix& other,ELEMTYPE Val)
{
	int iCol;
	
	iCol = nRow()==0 ? other.nCol()-1 : other.nCol();

	if(!AddRows(other.nRow(),Val))
		return FALSE;
	if(!AddCols(iCol,Val))
	    return FALSE;

return Paste(other,nRow()-other.nRow(),nCol()-other.nCol()); 
}

ElemNode* GeneralMatrix::GetElemP(int iRow,int iCol)const
{
	ElemNode *p = NULL;

	int i;

    if (iRow<0||iCol<0)
        return p;

    if((iRow>pHead->nRow - 1)||(iCol>pHead->nCol - 1))
        return p;
	
    p = pHead->pFirst;

	for(i=0;i<iCol;i++)
         p = p->pRight;
	for(i=0;i<iRow;i++)
		 p = p->pDown;
	
return p;
}

bool GeneralMatrix::Paste(GeneralMatrix& other,int top,int left) const
{
 int i,j,iRow,iCol;

 iRow = other.nRow()>(nRow()-top)  ? (nRow()-top) : other.nRow();
 iCol = other.nCol()>(nCol()-left) ? (nCol()-left): other.nCol();
 
 for(i=0;i<iRow;i++)
	 for(j=0;j<iCol;j++)
		 GetElem(i+top,j+left) = other[i][j];

return TRUE;
}

SMatrixChild& GeneralMatrix::operator [](int iRow)
{
    myChild.Head.pFirst = pHead->pFirst;
	  myChild.Head.nRow = pHead->nRow;
	  myChild.Head.nCol = pHead->nCol;
	       myChild.nRow = iRow;
	return myChild;
}

GeneralMatrix& GeneralMatrix::operator = (const GeneralMatrix& other)
{
	int i,j;

	if(other.nRow()!=this->nRow()||other.nCol()!=this->nCol())
	{
		for (i=0;i<this->nRow();i++)
		   for (j=0;j<this->nCol();j++)
		         GetElemP(i,j)->val = ERRORVAL;
	    return (*this);
	}

for(i=0;i<this->nRow();i++)
   for(j=0;j<this->nCol();j++)
	   GetElemP(i,j)->val = other.GetElem(i,j);

return (*this);
}

//////////////////////////////////////////////////////////////////////
// 重载运算符==，判断矩阵是否相等
//
// 参数：
// 1. const GeneralMatrix& other - 用于比较的矩阵
//
// 返回值：BOOL 型，两个矩阵相等则为TRUE，否则为FALSE
//////////////////////////////////////////////////////////////////////
bool GeneralMatrix::operator==(const GeneralMatrix& other) const
{
	// 首先检查行列数是否相等
	if (this->nCol() != other.nCol() || this->nRow() != other.nRow())
		return FALSE;

	for (int i=0; i<nRow(); ++i)
	{
		for (int j=0; j<nCol(); ++j)
		{
			if (this->GetElem(i,j)!=other.GetElem(i,j))
				return FALSE;
		}
	}

	return TRUE;
}

//////////////////////////////////////////////////////////////////////
// 重载运算符!=，判断矩阵是否不相等
//
// 参数：
// 1. const CMatrix& other - 用于比较的矩阵
//
// 返回值：BOOL 型，两个不矩阵相等则为TRUE，否则为FALSE
//////////////////////////////////////////////////////////////////////
bool GeneralMatrix::operator!=(const GeneralMatrix& other) const
{
	return !(*this == other);
}

GeneralMatrix GeneralMatrix::operator + (const GeneralMatrix& other)const
{
	int i,j;

   GeneralMatrix result(*this);

   if(other.nRow()!=this->nRow()||other.nCol()!=this->nCol())
   {
	   for (i=0;i<result.nRow();i++)
		   for (j=0;j<result.nCol();j++)
			   result.GetElemP(i,j)->val = ERRORVAL;
		   return result;
	}

   result += other;

return result;
}

GeneralMatrix& GeneralMatrix::operator += (const GeneralMatrix& other)
{
    int i,j;

	if(other.nRow()!=this->nRow()||other.nCol()!=this->nCol())
	{
		for (i=0;i<this->nRow();i++)
			for (j=0;j<this->nCol();j++)
				GetElemP(i,j)->val = ERRORVAL;
			return *this;
	}

    for(i=0;i<this->nRow();i++)
		for(j=0;j<this->nCol();j++)
			this->GetElem(i,j) += other.GetElem(i,j);

	return *this;
}

GeneralMatrix GeneralMatrix::operator - (const GeneralMatrix& other) const
{
	int i,j;
	
	GeneralMatrix result(*this);
	
	if(other.nRow()!=this->nRow()||other.nCol()!=this->nCol())
	{
		for (i=0;i<result.nRow();i++)
			for (j=0;j<result.nCol();j++)
				result.GetElemP(i,j)->val = ERRORVAL;
			return result;
	}
	
	result -= other;
	
return result;
}

GeneralMatrix  GeneralMatrix::operator -  () const
{
	int i,j;

	GeneralMatrix result(*this);

	for (i=0;i<result.nRow();i++)
	{
		for (j=0;j<result.nCol();j++)
		{
			result[i][j]=-result[i][j];
		}
	}

	return result;
}


GeneralMatrix& GeneralMatrix::operator -= (const GeneralMatrix& other)
{
	int i,j;
	
	if(other.nRow()!=this->nRow()||other.nCol()!=this->nCol())
	{
		for (i=0;i<this->nRow();i++)
			for (j=0;j<this->nCol();j++)
				GetElemP(i,j)->val = ERRORVAL;
			return *this;
	}
	
    for(i=0;i<this->nRow();i++)
		for(j=0;j<this->nCol();j++)
			this->GetElem(i,j) -= other.GetElem(i,j);
		
	return *this;
}

GeneralMatrix GeneralMatrix::operator * (const GeneralMatrix& other) const
{
	int i,j,k;
	
	GeneralMatrix result(this->nRow(),other.nCol());
	
	if(other.nRow()!=this->nCol())
	{
		for (i=0;i<result.nRow();i++)
			for (j=0;j<result.nCol();j++)
				result.GetElemP(i,j)->val = ERRORVAL;
			return result;
	}
	
    for (i=0;i<this->nRow();i++)       //矩阵相乘的计算
	{
		for(j=0;j<other.nCol();j++)
		{
			for(k = 0; k < other.nRow(); k++)
			{
				if ((this->GetElem(i,k)!=0)&&(other.GetElem(k,j)!=0))
				{
					result[i][j] += this->GetElem(i,k) * other.GetElem(k,j); 
				}
			}
		}
	}
	
return result;
}

GeneralMatrix GeneralMatrix::operator * (double value) const
{
   int i,j;
	
	GeneralMatrix result(*this);
	
    for (i=0;i<this->nRow();i++)
		for(j=0;j<this->nCol();j++)
				result[i][j] *= value; 
	
return result;
}

GeneralMatrix operator * (double value ,const GeneralMatrix& other)
{
	int i,j;
	
	GeneralMatrix result(other);
	
    for (i=0;i<other.nRow();i++)
		for(j=0;j<other.nCol();j++)
				result[i][j] *= value; 

return result;
}

GeneralMatrix  GeneralMatrix::Transpose(void) const
{
	int i,j;

	GeneralMatrix result(this->nCol(),this->nRow());

    for (i=0;i<result.nRow();i++)
        for(j=0;j<result.nCol();j++)
			result[i][j] = this->GetElem(j,i);
    
	return result;
}

bool GeneralMatrix::T_Self()
{
    GeneralMatrix Temp(this->nCol(),this->nRow());

	int i,j;

	if(this->nCol()<2&&this->nRow()<2)
		return TRUE;

    Temp = this->Transpose();

    i = this->nCol() - this->nRow();
	j = this->nRow() - this->nCol();

    AddRows(i,0.0);
	AddCols(j,0.0);

	*this = Temp;

 return TRUE;   
}

GeneralMatrix GeneralMatrix::MultiplyTranspose()
{
	GeneralMatrix TranspMat=this->Transpose();

	GeneralMatrix result(this->nRow(),this->nRow());

//	DBWindowWrite("matrix size: %d * %d\n",this->nRow(),this->nRow());
	//only multiply upper half
	for (int i=0;i<this->nRow();i++)
	{
//		DBWindowWrite("row: %d ",i);
//		clock_t RowBegin=clock();
		for(int j=i;j<this->nRow();j++)
		{
			double dSum=0;
			for (int k=0;k<this->nCol();k++)
			{
				if ((this->GetElem(i,k)!=0)&&(TranspMat[k][j]!=0))
				{
					dSum=dSum+this->GetElem(i,k)*TranspMat[k][j];
				}
			}
			result[i][j]=dSum;
		}
//		clock_t RowEnd=clock();
//		DBWindowWrite("compute time: %f\n",float(RowEnd-RowBegin));
	}
	//copy to another half
	for (int i=1;i<result.nRow();i++)
	{
		for (int j=0;j<i;j++)
		{
			result[i][j]=result[j][i];
		}
	}

	return result;
}

ELEMTYPE& GeneralMatrix::GetElem(int iRow,int iCol)const
{
    ElemNode *p;
    ELEMTYPE val = ERRORVAL;

	p = GetElemP(iRow,iCol);

	if(p==NULL)
       return val;
	 
	return p->val;
}

bool GeneralMatrix::ExchangeRows(int iRow1,int iRow2) const
{
         int j;
	ELEMTYPE temp;

    if(this->nRow()<2||iRow1==iRow2)
		return TRUE;

    if(iRow1>=nRow()||iRow2>=nRow())
        return FALSE;
	
    for(j=0;j<nCol();j++)
	{
                 temp = GetElem(iRow1,j);
     GetElem(iRow1,j) = GetElem(iRow2,j);
	 GetElem(iRow2,j) = temp;         
	}

return TRUE;   
}

bool GeneralMatrix::ExchangeCols(int iCol1,int iCol2) const
{
	int i;
	ELEMTYPE temp;

    if(this->nCol()<2||iCol1==iCol2)
		return TRUE;
	
    if(iCol1>=nCol()||iCol2>=nCol())
        return FALSE;

    for(i=0;i<nRow();i++)
	{
		          temp = GetElem(i,iCol1);
      GetElem(i,iCol1) = GetElem(i,iCol2);
      GetElem(i,iCol2) = temp;
	}

return TRUE;
}

bool GeneralMatrix::MakeUnit() const
{
	if(nRow()!=nCol())
		return FALSE;

    Zero();

	for (int i=0;i<nRow();i++)
	    GetElem(i,i) = 1.0;

return TRUE;	
}

bool GeneralMatrix::Zero() const
{
	for(int i=0;i<nRow();i++)
		for(int j=0;j<nCol();j++)
			GetElem(i,j) = 0.0;
return TRUE;
}

bool GeneralMatrix::Fabs() const
{
	int i,j;

	for(i=0;i<nRow();i++)
		for(j=0;j<nCol();j++)
            GetElem(i,j) = fabs(GetElem(i,j));

	return TRUE;
}

bool GeneralMatrix::Invert_Self() const
{
    int k,i,j,icol,irow;
    ELEMTYPE elem,d;

    icol = nCol();
	irow = nRow();

	if (icol!=irow)
		return FALSE;

	int *IS = new int[irow];   //记忆行交换空间
	int *JS = new int[irow];   //记忆列交换空间

	for (k=0; k<irow; k++)
	{
		d = 0.0;
		for (i=k; i<irow; i++)
			for (j=k; j<irow; j++)
			{
				elem = fabs(GetElem(i,j));
				if(elem > d) { d = elem; IS[k] = i; JS[k] = j; }  //记忆行列交换
			}
			if(fabs(d) < 1e-19)    //矩阵奇异，错误返回
			{
				delete []IS;
				delete []JS;
				return FALSE;
			}  
			
			if(IS[k]!=k) //行交换
				ExchangeRows(k,IS[k]);
			if(JS[k]!=k) //列变换
				ExchangeCols(k,JS[k]);
			
			GetElem(k,k) = 1.0 / GetElem(k,k); //归一化
			
			for(j=0; j<irow; j++) //归一化计算Akj*Akk=>Akj
				if(j != k)
					GetElem(k,j) = GetElem(k,k) * GetElem(k,j);
				
				for(i=0; i<irow; i++) //消元计算Aij-Aik*Akj=>Aij                      
					if(i != k)
						for (j=0; j<irow; j++)
							if (j != k)
								GetElem(i,j) = GetElem(i,j) - GetElem(i,k) * GetElem(k,j);
							
							for(i=0; i<irow; i++) //消元计算-Aik*Akk=>Aik 
								if(i != k)
									GetElem(i,k) = GetElem(i,k) * GetElem(k,k) * -1.0; 
	}
	
	for (k=irow-1; k>=0; k--)
	{
		if(JS[k] != k)
			ExchangeRows(k,JS[k]);
        if(IS[k] != k)
		    ExchangeCols(k,IS[k]);	
	}
	delete []IS;
	delete []JS;  

	return TRUE;
}

GeneralMatrix GeneralMatrix::Invert()
{
   GeneralMatrix result(nRow(),nRow(),ERRORVAL);

        if(nRow()!=nCol()) return result;
   
		result = *this;

		result.Invert_Self();

return result;
}

GeneralMatrix GeneralMatrix::GetRowVector(int iRow) const
{
	int j;
		
	GeneralMatrix Vector(1,nCol(),ERRORVAL);

	if(iRow<0||iRow>nRow()-1||nCol()==0) return Vector;

   for(j=0;j<nCol();j++)
	   Vector[0][j] = GetElem(iRow,j);

return Vector;
}

GeneralMatrix GeneralMatrix::GetColVector(int iCol) const
{
	int i;
	GeneralMatrix Vector(nRow(),1,ERRORVAL);
	
	if(iCol<0||iCol>nCol()-1||nRow()==0) return Vector;
	
	for(i=0;i<nRow();i++)
		Vector[i][0] = GetElem(i,iCol);
	
return Vector;
}

GeneralMatrix GeneralMatrix::GetDiagonalVector() const
{
	int i;

	GeneralMatrix Vector(nRow(),1,ERRORVAL);

    if (nRow()!=nCol())
        return Vector;

	for(i=0;i<nRow();i++)
		Vector[i][0] = GetElem(i,i);
return Vector;
}

GeneralMatrix GeneralMatrix::GetPart(int left,int top,int bottom,int right) const
{
    int i,j;

    GeneralMatrix result;

	if(left>right||top>bottom||right>=nCol()||bottom>=nRow()||left<0||top<0||right<0||bottom<0) return result;

    if(!result.ReSize(bottom-top+1,right-left+1,ERRORVAL))
		return result;
	
    for(i=0;i<result.nRow();i++)
		for(j=0;j<result.nCol();j++)
			result[i][j] = GetElem(i+left,j+top);

return result;
}

ELEMTYPE GeneralMatrix::Addup(int iPower) const
{
	   int i,j;
  ELEMTYPE result=0;
    
  for(i=0;i<nRow();i++)
	  for(j=0;j<nCol();j++)
		  result += pow(GetElem(i,j),iPower);
return result;   
}

ELEMTYPE GeneralMatrix::Distance_E(GeneralMatrix& other) const
{
	     int i,j; 
	ELEMTYPE result = ERRORVAL;

	if(nRow()!=other.nRow()||nCol()!=other.nCol()) return result;

    for(result=0,i=0;i<nRow();i++)
		for(j=0;j<nCol();j++)
		   result += pow(GetElem(i,j)-other[i][j],2);

return sqrt(result);
}

GeneralMatrix::~GeneralMatrix()
{
   ElemNode *p;
        int i,j;

   p = pHead->pFirst;

   if(p==NULL) return;

   for (i=pHead->nRow-1;i>=0;i--)
      for (j = pHead->nCol-1;j>=0;j--)
	        delete GetElemP(i,j);

  delete pHead;
}

void GeneralMatrix::Print()
{
	if (nRow()==0||nCol()==0)
	{
		return;
	}
	for (int i=0;i<nRow();i++)
	{
		for (int j=0;j<nCol();j++)
		{
			cout<<"%f "<<GetElem(i,j);
		}
		cout<<endl;
	}
	cout<<endl;
}

void GeneralMatrix::PrintNonZero()
{
	if (nRow()==0||nCol()==0)
	{
		return;
	}
	for (int i=0;i<nRow();i++)
	{
		for (int j=0;j<nCol();j++)
		{
			if (GetElem(i,j)!=0)
			{
				cout<<"%f "<<GetElem(i,j);
			}
		}
		cout<<endl;
	}
	cout<<endl;
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
ELEMTYPE& SMatrixChild::operator [](int iCol)
{
	ElemNode *p = NULL;
	ELEMTYPE val = ERRORVAL;
	
	int i,iRow;

	iRow = nRow;
			  
    if ((iRow>=0&&iCol>=0)&&(iRow<Head.nRow)&&iCol<Head.nCol)        
	{		
	   p = Head.pFirst;

	  for(i=0;i<iCol;i++)
		  p = p->pRight;
	  for(i=0;i<iRow;i++)
		  p = p->pDown;
	}
	
    if(p==NULL)
	   return val;
return p->val;
}