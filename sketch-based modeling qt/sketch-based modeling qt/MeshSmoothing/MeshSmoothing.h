#pragma once
//#include "../OBJHandle.h"
//#include "../PaintingOnMesh.h"
#include "../GeometryAlgorithm.h"
#include <Qpoint>

class SketchDoc;

class CMeshSmoothing
{
public:
	CMeshSmoothing(void);
	~CMeshSmoothing(void);

	void Init(SketchDoc* pDataIn);

	void InputCurvePoint2D(QPoint Point2D);

	//find roi by painting strokes
	void PaintROIVertices(KW_Mesh& Mesh,GLdouble* modelview,GLdouble* projection,GLint* viewport);

	void ClearROI();

	void Render(bool bSmoothView,GLdouble* modelview,GLdouble* projection,GLint* viewport,GLenum mode);

	void BilateralSmooth(KW_Mesh& Mesh);

private:
	SketchDoc* pDoc;

	vector<QPoint> CurvePoint2D;//user drawn 2D curve(screen coordinate)

	vector<Vertex_handle> ROIVertices;

	void RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport);
	void RenderROI();
};

