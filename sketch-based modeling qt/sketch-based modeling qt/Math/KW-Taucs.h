#pragma once
#ifndef  KW_TAUCS_H
#define  KW_TAUCS_H
extern "C"
{
#include <taucs.h>
};

#ifdef DEBUG
#pragma comment(lib,"libatlas.lib")
#pragma comment(lib,"libcblas.lib")
#pragma comment(lib,"libf77blas.lib")
//#pragma comment(lib,"liblapack.lib")
#pragma comment(lib,"libmetis-MDd-vs2008.lib")
#pragma comment(lib,"libtaucs-MDd-vs2008.lib")
#pragma comment(lib,"vcf2c.lib")
#else
#pragma comment(lib,"libatlas.lib")
#pragma comment(lib,"libcblas.lib")
#pragma comment(lib,"libf77blas.lib")
//#pragma comment(lib,"liblapack.lib")
#pragma comment(lib,"libmetis-MD-vs2008.lib")
#pragma comment(lib,"libtaucs-MD-vs2008.lib")
#pragma comment(lib,"vcf2c.lib")
#endif

#endif
