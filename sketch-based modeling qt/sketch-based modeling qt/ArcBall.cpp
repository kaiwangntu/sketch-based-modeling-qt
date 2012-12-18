/** KempoApi: The Turloc Toolkit *****************************/
/** *    *                                                  **/
/** **  **  Filename: ArcBall.cpp                           **/
/**   **    Version:  Common                                **/
/**   **                                                    **/
/**                                                         **/
/**  Arcball class for mouse manipulation.                  **/
/**                                                         **/
/**                                                         **/
/**                                                         **/
/**                                                         **/
/**                              (C) 1999-2003 Tatewake.com **/
/**   History:                                              **/
/**   08/17/2003 - (TJG) - Creation                         **/
/**   09/23/2003 - (TJG) - Bug fix and optimization         **/
/**   09/25/2003 - (TJG) - Version for NeHe Basecode users  **/
/**                                                         **/
/*************************************************************/

//#include "PreDef.h"
//#include "StdAfx.h"
//#include <windows.h>											// Header File For Windows
//#include <gl\gl.h>												// Header File For The OpenGL32 Library
//#include <gl\glu.h>												// Header File For The GLu32 Library
//#include <gl\glaux.h>											// Header File For The GLaux Library

//#include <math.h>                                               // Needed for sqrtf

#include "ArcBall.h"                                            // ArcBall header

//Arcball sphere constants:
//Diameter is       2.0f
//Radius is         1.0f
//Radius squared is 1.0f
//center is (0,0,0)    by KW

void ArcBall_t::_mapToSphere(const Point2fT* NewPt, Vector3fT* NewVec) const
{
	Point2fT TempPt;
	GLfloat length;

	//Copy paramter into temp point
	TempPt = *NewPt;

	//Adjust point coords and scale down to range of [-1 ... 1]
	TempPt.s.X  =        (TempPt.s.X * this->AdjustWidth)  - 1.0f;
	TempPt.s.Y  = 1.0f - (TempPt.s.Y * this->AdjustHeight);

	//Compute the square of the length of the vector to the point from the center
	length      = (TempPt.s.X * TempPt.s.X) + (TempPt.s.Y * TempPt.s.Y);

	//X,Y and Z >0,i.e. always take the positive part of the unit sphere (add by KW)

	//If the point is mapped outside of the sphere... (length > radius squared)
	if (length > 1.0f)
	{
		GLfloat norm;

		//Compute a normalizing factor (radius / sqrt(length))
		norm    = 1.0f / FuncSqrt(length);

		//Return the "normalized" vector, a point on the sphere
		NewVec->s.X = TempPt.s.X * norm;
		NewVec->s.Y = TempPt.s.Y * norm;
		NewVec->s.Z = 0.0f;
	}
	else    //Else it's on the inside
	{
		//Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
		NewVec->s.X = TempPt.s.X;
		NewVec->s.Y = TempPt.s.Y;
		NewVec->s.Z = FuncSqrt(1.0f - length);
	}
}

//Create/Destroy
ArcBall_t::ArcBall_t()
{
	//Clear initial values
	this->StVec.s.X     =
		this->StVec.s.Y     = 
		this->StVec.s.Z     = 

		this->EnVec.s.X     =
		this->EnVec.s.Y     = 
		this->EnVec.s.Z     = 0.0f;

	//Set initial bounds
	this->setBounds(640, 480);
}

ArcBall_t::ArcBall_t(GLfloat NewWidth, GLfloat NewHeight)
{
	//Clear initial values
	this->StVec.s.X     =
		this->StVec.s.Y     = 
		this->StVec.s.Z     = 

		this->EnVec.s.X     =
		this->EnVec.s.Y     = 
		this->EnVec.s.Z     = 0.0f;

	//Set initial bounds
	this->setBounds(NewWidth, NewHeight);
}

//Mouse down
void    ArcBall_t::click(const Point2fT* NewPt)
{
	//Map the point to the sphere
	this->_mapToSphere(NewPt, &this->StVec);
}

//Mouse drag, calculate rotation
void    ArcBall_t::drag(const Point2fT* NewPt, Quat4fT* NewRot)
{
	//Map the point to the sphere
	this->_mapToSphere(NewPt, &this->EnVec);

	//Return the quaternion equivalent to the rotation
	if (NewRot)
	{
		Vector3fT  Perp;

		//Compute the vector perpendicular to the begin and end vectors
		Vector3fCross(&Perp, &this->StVec, &this->EnVec);

		//Compute the length of the perpendicular vector
		if (Vector3fLength(&Perp) > Epsilon)    //if its non-zero
		{
			//We're ok, so return the perpendicular vector as the transform after all
			NewRot->s.X = Perp.s.X;
			NewRot->s.Y = Perp.s.Y;
			NewRot->s.Z = Perp.s.Z;
			//In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
			NewRot->s.W= Vector3fDot(&this->StVec, &this->EnVec)/2;//除2为了增加旋转速度
		}
		else                                    //if its zero
		{
			//The begin and end vectors coincide, so return an identity transform
			NewRot->s.X = 
				NewRot->s.Y = 
				NewRot->s.Z = 
				NewRot->s.W = 0.0f;
		}
	}
}

bool AutoPlaneRotation::PreCompute(Plane_3 plane,Point_3* PlaneBoundaryPoints,GLdouble* modelview,int iStep,Matrix3fT& CurrentRotIn)
{
	this->iTotalStep=iStep;
	this->iStepCount=1;
	this->LastRot=CurrentRotIn;

	//get the rotated camera position in local coordinates
	GLdouble InverseModelviewMatrix[16];
	memcpy(InverseModelviewMatrix,modelview,16*sizeof(GLdouble));
	GeometryAlgorithm Geo;
	Geo.GetInverseMatrix(InverseModelviewMatrix,4);
	Point3D MovedCameraPostemp;
	MovedCameraPostemp.x=0.0;MovedCameraPostemp.y=0.0;MovedCameraPostemp.z=0.0;
	Geo.ComputeTransformedPointPos(&MovedCameraPostemp,InverseModelviewMatrix);
	Point_3 RotatedCamera=Point_3(MovedCameraPostemp.x,MovedCameraPostemp.y,MovedCameraPostemp.z);
	//get the center of plane boundary points
	double dCenterX,dCenterY,dCenterZ;
	dCenterX=dCenterY=dCenterZ=0;
	for (int i=0;i<4;i++)
	{
		dCenterX=dCenterX+PlaneBoundaryPoints[i].x();
		dCenterY=dCenterY+PlaneBoundaryPoints[i].y();
		dCenterZ=dCenterZ+PlaneBoundaryPoints[i].z();
	}
	Point_3 PlaneCenter=Point_3(dCenterX/4.0,dCenterY/4.0,dCenterZ/4.0);
	//get the unit vector from the plane center to the rotated camera
	Vector_3 CenterCamera(PlaneCenter,RotatedCamera);
	double dSqrtLength=sqrt(CenterCamera.squared_length());
	CenterCamera=Vector_3(CenterCamera.x()/dSqrtLength,CenterCamera.y()/dSqrtLength,CenterCamera.z()/dSqrtLength);

	//get the vectors starting from the plane center along and against the normal
	//select one which has smaller angle with the above vector
	Vector_3 PosVec,NegVec;
	double dPosAngle=GeometryAlgorithm::GetAngleBetweenTwoVectors3d(plane.orthogonal_vector(),CenterCamera,PosVec);
	if (dPosAngle>180)
	{
		dPosAngle=360-dPosAngle;
	}
	double dNegAngle=GeometryAlgorithm::GetAngleBetweenTwoVectors3d(-plane.orthogonal_vector(),CenterCamera,NegVec);
	if (dNegAngle>180)
	{
		dNegAngle=360-dNegAngle;
	}
	double dTotalAngle=0;

	//make sure the minimum angle is rotated
	//the sign of the angle is always positive, the rotation axis takes charge of the rotation direction
	if (abs(dPosAngle)>=abs(dNegAngle))
	{
		dTotalAngle=dNegAngle;
		RotateAxis=NegVec;
	}
	else
	{
		dTotalAngle=dPosAngle;
		RotateAxis=PosVec;
	}
	dSqrtLength=sqrt(RotateAxis.squared_length());
	RotateAxis=Vector_3(RotateAxis.x()/dSqrtLength,RotateAxis.y()/dSqrtLength,RotateAxis.z()/dSqrtLength);

	//if the angle is small, no need to rotate
	if (dTotalAngle<5)
	{
		return false;
	}

	//compute the rotation matrix of each step
	this->dStepAngle=dTotalAngle/(double)this->iTotalStep;

	return true;
}

//pass ThisRot in render class in as the CurrentRot
bool AutoPlaneRotation::Rotate(Matrix4fT& CurrentTransformMatrix,Matrix3fT& CurrentRotIn)
{
	if (this->iStepCount>this->iTotalStep)
	{
		return false;
	}

	//accumulate angle
	double dAccuAngle=this->dStepAngle*this->iStepCount;
	double dAccuRadius=dAccuAngle*CGAL_PI/180.0;

	Matrix3fT AccuRot;
	AccuRot.s.M00=cos(dAccuRadius)+RotateAxis.x()*RotateAxis.x()*(1-cos(dAccuRadius));
	AccuRot.s.M01=RotateAxis.x()*RotateAxis.y()*(1-cos(dAccuRadius))-RotateAxis.z()*sin(dAccuRadius);
	AccuRot.s.M02=RotateAxis.x()*RotateAxis.z()*(1-cos(dAccuRadius))+RotateAxis.y()*sin(dAccuRadius);
	AccuRot.s.M10=RotateAxis.y()*RotateAxis.x()*(1-cos(dAccuRadius))+RotateAxis.z()*sin(dAccuRadius);
	AccuRot.s.M11=cos(dAccuRadius)+RotateAxis.y()*RotateAxis.y()*(1-cos(dAccuRadius));
	AccuRot.s.M12=RotateAxis.y()*RotateAxis.z()*(1-cos(dAccuRadius))-RotateAxis.x()*sin(dAccuRadius);
	AccuRot.s.M20=RotateAxis.z()*RotateAxis.x()*(1-cos(dAccuRadius))-RotateAxis.y()*sin(dAccuRadius);
	AccuRot.s.M21=RotateAxis.z()*RotateAxis.y()*(1-cos(dAccuRadius))+RotateAxis.x()*sin(dAccuRadius);
	AccuRot.s.M22=cos(dAccuRadius)+RotateAxis.z()*RotateAxis.z()*(1-cos(dAccuRadius));

	//important!!: for the rotation of the view (not the object), the new rotation matrix (AccuRot) is added to 
	//the right side of the last rotation matrix!!
	Matrix3fT TempRot=this->LastRot;
	Matrix3fMulMatrix3f(&TempRot, &AccuRot);// Accumulate Last Rotation Into This One
	Matrix4fSetRotationFromMatrix3f(&CurrentTransformMatrix, &TempRot);
	CurrentRotIn=TempRot;
	this->iStepCount++;
	return true;
}