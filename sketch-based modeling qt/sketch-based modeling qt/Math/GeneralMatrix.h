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
	       GeneralMatrix();                                                  //Create a 0×0 Matrix
	       GeneralMatrix(int iRow,int iCol);                                 //create a nRow×nCol Matrix
	       GeneralMatrix(int iRow,int iCol,ELEMTYPE Val);                    //create a nRow×nCol Matrix,initals every menber as Val
	       GeneralMatrix(const GeneralMatrix& other);                              //create a Matrix,size like other
          ~GeneralMatrix();                                                  //析构        
                      
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
       
   GeneralMatrix  Transpose(void) const; //返回转置矩阵
   GeneralMatrix  Invert(void);                                              //返回其逆矩阵，全选主元法

      bool T_Self(void);                                               //原地转置
	  bool Invert_Self(void)const;                                     //全选主元法原地求逆

	  GeneralMatrix MultiplyTranspose();//multiply matrix and its transpose

       int nRow()const;                                                //Row
	   int nCol()const;                                                //Col


	 ELEMTYPE& GetElem(int iRow,int iCol)const;                       //得到iRow,iCor位置的值，可用*.[iRow][iCol]代替 
 SMatrixChild& operator [] (int iRow);                                //设置行数
	      bool Paste(GeneralMatrix& other,int top,int left)const;           //将other矩阵左上角对齐到left、top位置，赋值给当前矩阵
      ELEMTYPE Addup(int iPower)const;                                //各元素的power次方累加
	  ELEMTYPE Distance_E(GeneralMatrix& other)const;                       //欧氏距离


 	  bool ExchangeRows(int iRow1,int iRo2)const;                      //交换行 
 	  bool ExchangeCols(int iCol1,int iCol2)const;                     //交换列 
	  bool MakeUnit()const;                                            //转化为单位矩阵
	  bool Zero()const;                                                //清零
	  bool Fabs()const;                                                //所有值取绝对值


	  bool ReSize(int iRow,int iCol)const;                             //重定义大小,多出部分补为0
	  bool ReSize(int iRow,int iCol, ELEMTYPE Val)const;               //重定义大小,多出部分补为Val
      bool AddRows(int in,ELEMTYPE Val)const;                          //原矩阵增加n行，加后面 (n<0的意义为删除)
      bool AddCols(int in,ELEMTYPE Val)const;                          //原矩阵增加n列，加下面 (n<0的意义为删除)
      bool DelRows(int in)const;                                       //原矩阵删除后n行，n大于原矩阵行数时矩阵变为 0×0
      bool DelCols(int in)const;                                       //原矩阵删除后n列，n大于原矩阵列数时矩阵变为 0×0


   GeneralMatrix GetRowVector(int iRow)const;                                      //得到一个行向量
   GeneralMatrix GetColVector(int iCol)const;                                      //得到一个列向量
   GeneralMatrix GetDiagonalVector() const;                                        //得到对角向量(列向量)
   GeneralMatrix GetPart(int left,int top,int bottom,int right) const;             //从其中部分构造一个新矩阵      
      bool CutIn2_Vertical(GeneralMatrix& Left,GeneralMatrix& Right,int iCol);           //在icol位置纵切成左右两部分(左边部分有iCol列)
      bool CutIn2_Across(GeneralMatrix& Upper,GeneralMatrix& Lower,int iRow);            //在iRow位置横切成上下两部分(上边部分有iRow行)
      bool Jointer_Right(GeneralMatrix& other);                                    //把other矩阵接在该矩阵右边
      bool Jointer_bottom(GeneralMatrix& other);                                   //把other矩阵接在该矩阵下面
	  bool Jointer_Diagonal(GeneralMatrix& other,ELEMTYPE Val);                    //把other矩阵接在该矩阵右下,空白部分用Val补上

	  void Print();//print out matrix
	  void PrintNonZero();//print out nonzeros
    
private:
   ElemNode* GetElemP(int iRow,int iCol)const;                               //得到某位置结构体指针
	HeadNode *pHead;
SMatrixChild myChild;
};

#endif