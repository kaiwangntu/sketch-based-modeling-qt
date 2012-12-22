#ifndef _MYMATH_H_
#define _MYMATH_H_

#include "../config.h"

class MyMath{
public:
	static int getSign( float val );
	static bool isEqualInToler( float num1, float num2, float toler);
	//represented in x, y, z
	static float vectorLen(float x, float y , float z);
	//represented in array
	static float dotProduct( float vec1[3], float vec2[3]);
	//compute the cos of angle pt1 pt0 pt2. angle between vector pt0pt1 and pt0pt2
	static float getCosOfAngle(float pt1[3], float pt2[3], float pt0[3]);
	//get vector between two poitns, and save the vector into vec
	static void getVec(float pt1[3], float pt2[3], float vec[3]);
	static void getNeg(float vec[3]);
	static void getPtOnSeg(float pt1[3], float pt2[2], float ratio, float pt[3]);
	static void getEndPtOfSeg(float ept[ 3 ], float vec[ 3 ], float resultpt[ 3 ]);
	static void getPtOnRay(float pt[ 3 ], float dir[ 3 ], float t, float endpt[ 3 ]);
	//compute the length of the vector between the two points
	static float vectorlen( float pt1[3], float pt2[3]);
	//compute the vector's length
	static float vectorlen(float vec[3]);
	//normalize the svec, and keep it unchanged, save the normalized value into dvec
	static void normalize(float svec[3], float dvec[3]);
	//normalize vec, and save the normalized value into vec
	static void normalize(float vec[3] );
	//compute center of two points p1, and p2, save it into center
	static void center(float p1[3], float p2[3], float center[3]);
	//compute the relative position to center, and unit lenght is len.
	static void getrelativepos(float pt[3], float center[3], float len);
	/**
	* return normalized crossproduct of vec1 and vec2  and save it into cp
	*/	
	static void crossProduct(float vec1[3], float vec2[3], float cp[3]);
	/**
	* return non-normalized cross product of vec1, vec2 and save it in cp
	*/
	static void crossProductNotNorm(float vec1[3], float vec2[3], float cp[3]);
	//a naive algorithm to compute the difference for vertex sver to all the vertices in dver.
	static float computeDiff( float* sver,  float* dver,int dvernum);
	
	static void stretchVec( float vec[ 3 ], float ratio[ 3 ] );
	static void stretchVec( float vec[ 3 ], float ratio);
	//another naive algorithm to compute the difference for vertex sver to another mesh, dvers, dtrians

	//dist: save the resulting distance from the point to the line segment lies on
	//dist = the distance from the point to the line the segment lies on , if the perpendicular line from sver crosses the seg
	//dist = the minimal distance from the point to one of the two endpoints of the segment, otherwise
	static void computeDistPt2Seg( float* sver, float* dver, int ind[ 2 ], float& dist);
	//compute distance from point to a triangle 
	//dist = distance from the point to the plane the triangle is on, if the perpendicular line crosses trian
	//dist = nearest distance from the point to the three segments
	static void computeDistPt2Trian(float* sver, float* dver, int ind[ 3 ], float& dist);
	
	static float computeDiff( float* sver, float* dver, int* dtrians, int triannum);
};

#endif