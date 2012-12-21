#pragma once

#include "PreDef.h"
#include <qpoint>

class SketchDoc;

class CTest
{
public:
	CTest(void);
	~CTest(void);

	void Init(SketchDoc* pDataIn);
	void Render(bool bSmoothView,GLdouble* modelview,GLdouble* projection,GLint* viewport);
	void InputCurvePoint2D(QPoint ProfilePoint);
	void Conver2DCurveTo3D(GLdouble* modelview,GLdouble* projection,GLint* viewport);

	void LineFilter();
	void LineResample();
	void LineSmooth();



protected:
	SketchDoc* pDoc;

	vector<QPoint> UserInput2DProfile;
	void Render2DProfile(GLdouble* modelview,GLdouble* projection,GLint* viewport);


};