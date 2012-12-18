#ifndef _SMATRIX_H
#define _SMATRIX_H


#ifndef NULL
#define NULL 0
#endif

#ifndef TRUE
#define FALSE 0
#define TRUE 1 
#endif

#define ERRORVAL -9999

//typedef long double ELEMTYPE;
typedef double ELEMTYPE;

typedef struct _ElemNode
{
	_ElemNode* pRight;
	_ElemNode* pDown;
    ELEMTYPE  val;
}ElemNode;

typedef struct
{
	   int nRow,nCol;
 ElemNode* pFirst;
}HeadNode;

class SMatrixChild
{
public:
   ELEMTYPE& operator[](int iCol);
	HeadNode Head;
	     int nRow;
};

class GeneralMatrix
{
public:
	       GeneralMatrix();                                                  //Create a 0��0 Matrix
	       GeneralMatrix(int iRow,int iCol);                                 //create a nRow��nCol Matrix
	       GeneralMatrix(int iRow,int iCol,ELEMTYPE Val);                    //create a nRow��nCol Matrix,initals every menber as Val
	       GeneralMatrix(const GeneralMatrix& other);                              //create a Matrix,size like other
          ~GeneralMatrix();                                                  //����        
                      
       GeneralMatrix& operator =  (const GeneralMatrix& other);
	   bool     operator == (const GeneralMatrix& other) const;
	   bool     operator != (const GeneralMatrix& other) const;
	   GeneralMatrix  operator +  (const GeneralMatrix& other) const;
	   GeneralMatrix& operator += (const GeneralMatrix& other);
	   GeneralMatrix  operator -  (const GeneralMatrix& other) const;
	   GeneralMatrix  operator -  () const;//return a new matrix whose elements are negative of this. DO NOT change the value of this !!
       GeneralMatrix& operator -= (const GeneralMatrix& other);
	   GeneralMatrix  operator *  (const GeneralMatrix& other) const;
	   GeneralMatrix  operator *  (double value)         const;
friend GeneralMatrix  operator *  (double value ,const GeneralMatrix& other);
       
   GeneralMatrix  Transpose(void) const; //����ת�þ���
   GeneralMatrix  Invert(void);                                              //�����������ȫѡ��Ԫ��

      bool T_Self(void);                                               //ԭ��ת��
	  bool Invert_Self(void)const;                                     //ȫѡ��Ԫ��ԭ������

	  GeneralMatrix MultiplyTranspose();//multiply matrix and its transpose

       int nRow()const;                                                //Row
	   int nCol()const;                                                //Col


	 ELEMTYPE& GetElem(int iRow,int iCol)const;                       //�õ�iRow,iCorλ�õ�ֵ������*.[iRow][iCol]���� 
 SMatrixChild& operator [] (int iRow);                                //��������
	      bool Paste(GeneralMatrix& other,int top,int left)const;           //��other�������ϽǶ��뵽left��topλ�ã���ֵ����ǰ����
      ELEMTYPE Addup(int iPower)const;                                //��Ԫ�ص�power�η��ۼ�
	  ELEMTYPE Distance_E(GeneralMatrix& other)const;                       //ŷ�Ͼ���


 	  bool ExchangeRows(int iRow1,int iRo2)const;                      //������ 
 	  bool ExchangeCols(int iCol1,int iCol2)const;                     //������ 
	  bool MakeUnit()const;                                            //ת��Ϊ��λ����
	  bool Zero()const;                                                //����
	  bool Fabs()const;                                                //����ֵȡ����ֵ


	  bool ReSize(int iRow,int iCol)const;                             //�ض����С,������ֲ�Ϊ0
	  bool ReSize(int iRow,int iCol, ELEMTYPE Val)const;               //�ض����С,������ֲ�ΪVal
      bool AddRows(int in,ELEMTYPE Val)const;                          //ԭ��������n�У��Ӻ��� (n<0������Ϊɾ��)
      bool AddCols(int in,ELEMTYPE Val)const;                          //ԭ��������n�У������� (n<0������Ϊɾ��)
      bool DelRows(int in)const;                                       //ԭ����ɾ����n�У�n����ԭ��������ʱ�����Ϊ 0��0
      bool DelCols(int in)const;                                       //ԭ����ɾ����n�У�n����ԭ��������ʱ�����Ϊ 0��0


   GeneralMatrix GetRowVector(int iRow)const;                                      //�õ�һ��������
   GeneralMatrix GetColVector(int iCol)const;                                      //�õ�һ��������
   GeneralMatrix GetDiagonalVector() const;                                        //�õ��Խ�����(������)
   GeneralMatrix GetPart(int left,int top,int bottom,int right) const;             //�����в��ֹ���һ���¾���      
      bool CutIn2_Vertical(GeneralMatrix& Left,GeneralMatrix& Right,int iCol);           //��icolλ�����г�����������(��߲�����iCol��)
      bool CutIn2_Across(GeneralMatrix& Upper,GeneralMatrix& Lower,int iRow);            //��iRowλ�ú��г�����������(�ϱ߲�����iRow��)
      bool Jointer_Right(GeneralMatrix& other);                                    //��other������ڸþ����ұ�
      bool Jointer_bottom(GeneralMatrix& other);                                   //��other������ڸþ�������
	  bool Jointer_Diagonal(GeneralMatrix& other,ELEMTYPE Val);                    //��other������ڸþ�������,�հײ�����Val����

	  void Print();//print out matrix
	  void PrintNonZero();//print out nonzeros
    
private:
   ElemNode* GetElemP(int iRow,int iCol)const;                               //�õ�ĳλ�ýṹ��ָ��
	HeadNode *pHead;
SMatrixChild myChild;
};

#endif