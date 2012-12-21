#pragma once
#ifndef MY_MATH_H
#define MY_MATH_H


#include "../PreDef.h"
#include "GeneralMatrix.h"
#include "KW-Taucs.h"

//extern"C"
//{
//#include <blaswrap.h>
//#include <f2c.h>
//#include <clapack.h>
//}
//
//#ifdef DEBUG
//#pragma comment(lib,"libf2cd.lib")
//#pragma comment(lib,"BLASd.lib")
//#pragma comment(lib,"clapackd.lib")
//#pragma comment(lib,"tmglibd.lib")
//#else
//#pragma comment(lib,"libf2c.lib")
//#pragma comment(lib,"BLAS.lib")
//#pragma comment(lib,"clapack.lib")
//#pragma comment(lib,"tmglib.lib")
//#endif


//a encapsuled class for sparse matrix multiplication
class KW_SparseMatrix
{
private:
	//// objects ////
	long M; //!< matrix row size (NOT accessible)
	long N; //!< matrix column size (NOT accessible)
	long CAP; //!< the length of data arrays (NOT accessible)
	long VOL; //!< the number of non-zero components (NOT accessible)
	float* Array; //!< 1D array to store non-zero matrix data (NOT accessible)
	long* Indx; //!< 1D array to store the i-index of non-zero matrix components (NOT accessible)
	long* Jndx; //!< 1D array to store the j-index of non-zero matrix components (NOT accessible)

public:
	//// const references ////
	long const& m; //!< matrix row size (readable)
	long const& n; //!< matrix column size (readable)
	long const& cap; //!<  the length of data arrays (readable)
	long const& vol; //!< the number of non-zero components (readable)
	float* const& array; //!< 1D array to store matrix data (readable)
	long* const& indx; //!< 1D array to store the i-index of non-zero matrix components (readable)
	long* const& jndx; //!< 1D array to store the j-index of non-zero matrix components (readable)

	//// constructor ////
	KW_SparseMatrix();
	KW_SparseMatrix(const long&, const long&, const long&);
	~KW_SparseMatrix(); //destructor


	//// io ////
	void fput(const long&, const long&, const float&);
	void add(const long&, const long&, const float&);

	void expand(const long&);

	static void KW_multiply(KW_SparseMatrix& matA,KW_SparseMatrix& matB,KW_SparseMatrix& Result);

};

class SparseMatrix : public vector< map<int, double> >
{
	void init()
	{
		m = 0;
	}
public:	
	size_t m;

	SparseMatrix() {init();}
	explicit SparseMatrix(size_t n_) : vector< map<int, double> >(n_) {init();}
	void clear()
	{
		vector< map<int, double> >::clear();
		init();
	}

	size_t NRows() const {return size();}
	size_t NCols() const {return m;}

	void swap(SparseMatrix &y)
	{
		vector< map<int, double> >::swap(y);
		std::swap(m, y.m);
	}
	void clearMemory()
	{
		SparseMatrix empty;
		swap(empty);
	}

//	SparseMatrix  operator *  (SparseMatrix& other) const;

	SparseMatrix  operator +  (SparseMatrix& other) const;
	SparseMatrix  operator -  (SparseMatrix& other) const;
};

//Dense Matrix for comparison
class Matrix : public std::vector<float>
{
	size_t m;
	size_t n;

public:
	Matrix() : m(0), n(0) {}
	Matrix(size_t m_, size_t n_) : std::vector<float>(m_*n_), m(m_), n(n_) {}
	Matrix(size_t m_, size_t n_, float val) : std::vector<float>(m_*n_, val), m(m_), n(n_) {}

	void resize(size_t m_, size_t n_)
	{m = m_; n = n_; std::vector<float>::resize(m_*n_);}
	void reserve(size_t m_, size_t n_)
	{std::vector<float>::reserve(m_*n_);}
	void clear()
	{m = n = 0; std::vector<float>::clear();}

	size_t NRows() const {return m;}
	size_t NCols() const {return n;}

	float& operator()(size_t i, size_t j)
	{
		return operator[](i + j*m);
	}
	const float& operator()(size_t i, size_t j) const
	{
		return operator[](i + j*m);
	}
	void swap(Matrix &y)
	{
		std::vector<float>::swap(y);
		std::swap(n, y.n);
		std::swap(m, y.m);
	}
	void clearMemory()
	{
		Matrix empty;
		swap(empty);
	}
};


class CMath
{
public:
	CMath(void);
	~CMath(void);

	//minimize sqr(|| c - A*x ||)   subject to   B*x = d
	static bool ComputeConstrainedLSE(std::vector<std::vector<double> > LeftMatrixA,std::vector<double> RightMatrixC,
							std::vector<std::vector<double> > LeftMatrixB,std::vector<double> RightMatrixD,
							std::vector<double>& Result); 
	
	//minimize sqr(|| B - A*x ||), B may contain more than 1 column,RightMatrixB stores in columnsize
	static bool ComputeLSE(std::vector<std::vector<double> > LeftMatrixA,
		std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result); 

	//minimize sqr(|| B - A*x ||), B may contain more than 1 column,RightMatrixB stores in columnsize
	static bool ComputeLSE(SparseMatrix LeftMatrixA,
		std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result); 

	//same as ComputeLSE,for small size matrix
	static bool ComputeLSESmallSize(std::vector<std::vector<double> > LeftMatrixA,
		std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result); 

	//compute svd of 3*3 matrix.For larger size matrix,the memory allocated inside this function should be larger
	static bool ComputeSVD(std::vector<std::vector<double> > InputMatrix,
		std::vector<std::vector<double> >& OutputU,std::vector<std::vector<double> >& OutputSigma,
		std::vector<std::vector<double> >& OutputV);

	static bool ComputeSVD(GeneralMatrix InputMatrix,GeneralMatrix& OutputU,GeneralMatrix& OutputSigma,
		GeneralMatrix& OutputV);

	//multiply sparse matrix
	static bool SparseMatrixMultiply(vector<vector<double>> MatrixA,vector<vector<double>> MatrixB,vector<vector<double>>& Result);

	//compute lse using taucs
	void TAUCSFactorize(std::vector<std::vector<double> > LeftMatrixA,std::vector<std::vector<double>>& LeftMatrixAT); 
	
	void TAUCSFactorize(SparseMatrix LeftMatrixA,SparseMatrix& LeftMatrixAT); 

	bool TAUCSComputeLSE(std::vector<std::vector<double> > LeftMatrixAT,std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result);

	bool TAUCSComputeLSE(SparseMatrix LeftMatrixAT,std::vector<std::vector<double> > RightMatrixB,std::vector<std::vector<double> >& Result);

	void TAUCSClear();
	//

	//conversion between general and sparse
	void General2Sparse(GeneralMatrix GMatrix,SparseMatrix& SMatrix);
	void Sparse2General(SparseMatrix SMatrix,GeneralMatrix& GMatrix);


	static void testCLAPACK();

	void testTAUCS();


	//Quat: s, x, y, z
	//Mat:
	//0 1 2
	//3 4 5
	//6 7 8
	static void ConvertQuatToMat(vector<double> Quat,vector<double>& Mat);
	static void ConvertMatToQuat(vector<double> Mat,vector<double>& Quat);
	static void NormalizeQuat(vector<double>& Quat);


protected:
	void* F;
	int* perm;
	int* invperm;

	void multiplyAAT(SparseMatrix &A, SparseMatrix &C);
	void convert2dense(Matrix& a, SparseMatrix& b);
	void printMatrix(Matrix &a, char *comment);
	void convert2taucs(SparseMatrix &C,taucs_ccs_matrix* A,int dim,int nnz);
	void convertTaucs2Sparse(taucs_ccs_matrix* A,SparseMatrix &C);
};
#endif
