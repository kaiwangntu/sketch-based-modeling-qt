#pragma once

#include "PreDef.h"
#include <qpoint>

class SketchDoc;

class CMeshEditing
{
public:
	CMeshEditing(void);
	~CMeshEditing(void);

	void Init(SketchDoc* pDataIn);

	void Input2DProfilePoint(QPoint ProfilePoint);
	void Convert2DProfileTo3D();

	void Render(int mode);

private:
	SketchDoc* pDoc;


	vector<QPoint> UserInput2DProfile;
	void Render2DProfile();


};
