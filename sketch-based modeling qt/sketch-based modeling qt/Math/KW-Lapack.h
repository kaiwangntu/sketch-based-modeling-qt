#pragma once
#ifndef  KW_LAPACK_H
#define  KW_LAPACK_H

extern"C"
{
#include <blaswrap.h>
#include <f2c.h>
#include <clapack.h>
}

#ifdef DEBUG
#pragma comment(lib,"libf2cd.lib")
#pragma comment(lib,"BLASd.lib")
#pragma comment(lib,"clapackd.lib")
#pragma comment(lib,"tmglibd.lib")
#else
#pragma comment(lib,"libf2c.lib")
#pragma comment(lib,"BLAS.lib")
#pragma comment(lib,"clapack.lib")
#pragma comment(lib,"tmglib.lib")
#endif

#endif