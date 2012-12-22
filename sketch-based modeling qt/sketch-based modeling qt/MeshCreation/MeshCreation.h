#pragma once
#include "../PreDef.h"
#include "MeshCreation_Struct_Def.h"//temparory
#include "../ArcBall.h"
#include <qpoint>
#include "Ctr2SufManager/Ctr2SufManager.h"
//#include "KW_CS2Surf/KW_CS2Surf.h"
//#include "../PaintingOnMesh.h"
//#include "CrossSectionProc.h"

class SketchDoc;


#define SAMPLE_POINT_NUM 50



class CMeshCreation
{
public:
	CMeshCreation(void);
	~CMeshCreation(void);

	void Init(SketchDoc* pDataIn);

	void Render(bool bSmoothView,GLenum mode,GLdouble* modelview,GLdouble* projection,GLint* viewport);

	void SetDrawingPlane();
	int GetDrawingPlane();

	bool* GetRenderRefPlane() {return this->bRenderRefPlane;}
	bool GetRenderCN() {return this->bRenderCN;}
	void SetRenderCN(bool bValueIn) {this->bRenderCN=bValueIn;}
	bool GetRenderOnlyUserSketch() {return this->bRenderOnlyUserSketch;}
	void SetRenderOnlyUserSketch(bool bValueIn) {this->bRenderOnlyUserSketch=bValueIn;}
	
	int GetSurfReconstAlgorithm() {return this->iSurfReconstAlgorithm;}
	void SetSurfReconstAlgorithm(int iParaIn) {this->iSurfReconstAlgorithm=iParaIn;}

	//KW_CS2Surf* GetKW_CS2Surf() {return this->kwcs2surf;}
	Ctr2SufManager* GetCS2Surf() {return this->manager;}
	
	void AdjustPlaneBoundary(int iIncrease);

	void TranslateDrawingPlane(double dOffset);

	//compute the intersected curves of the plane and mesh,
	//done after plane translation
	void StopTranslateDrawingPlane();

	//check if the plane on which to sketch faces the user or not
	//avoid sketching on a unselected plane
	bool CheckPlaneState();


	void Input2DProfilePoint(QPoint ProfilePoint);
	void Convert2DProfileTo3D();

	void CancelLastInput();

	//delete specified cross section
	void DeleteSpecifiedCS();

	bool FitLastPlaneCurves();
	bool bCurvesLeftToFit;

	void AdjustContourView();

	void CopyCNFromLastParaPlane(int iPlaneType);

	void ReadContourFromFile(char* pFileName);

	void WriteContourToFile(char* pFileName);

	void GenerateMesh(KW_Mesh& Mesh,vector<double> vecMeshColor);


	bool GetAutoRotState() {return this->bAutoRot;}
	void SetAutoRotState(bool bValueIn) {this->bAutoRot=bValueIn;}
	//after user action, wait to rotate the plane automatically
	void WaitAutoPlaneRotate();
	//rotate the selected plane
	void RotateSelectedPlane();

private:

	SketchDoc* pDoc;

	//surface reconstruction algorithm
	int iSurfReconstAlgorithm;

	Ctr2SufManager* manager;
	//KW_CS2Surf* kwcs2surf;


	//plane0: xoy plane1:xoz plane2:yoz
	Plane_3 RefPlane[3];
	Point_3 PlaneBoundaryPoints[3][4];
	
	//whether allow auto rotation or not
	bool bAutoRot;
	//time left to start rotation
	int iAutoRotTimeLeft;
	//start to wait before plane auto rotation
	void StartWaitAutoPlaneRot();
	//plane auto rotation
	AutoPlaneRotation PlaneRot;
	

	//render this plane or not
	//plane0: xoy plane1:xoz plane2:yoz
	bool bRenderRefPlane[3];
	
	//render curve network or not
	bool bRenderCN;
	//only render user sketches
	bool bRenderOnlyUserSketch;

	void RenderRefPlanes(bool bSmoothView,GLenum mode);

	//index of selected curve network and cross section
	int iSelectedCN;
	int iSelectedCS;

	//set the drawing plane when a curve is selected
	void CurveSelSetDrawingPlane();

	//0:profile on xoy plane,1:profile on xoz plane,2:profile on yoz plane
	//-1:not drawing
	int iDrawingProfilePlane;
	vector<QPoint> UserInput2DProfile;
	void Render2DProfile(GLdouble* modelview,GLdouble* projection,GLint* viewport);

	//if any plane selected?
	//if yes,if the plane faces towards the user?
	bool bPlaneReadyState;

	vector<CurveNetwork> vecCurveNetwork;

	vector<vector<Point_3>> vecComputedCS;
	void RenderComputedCS(GLenum mode);

	vector<Point_3> vecCurvePlaneIntersectPoint;
	//0: xoy 1:xoz 2:yoz
	vector<int> vecCurvePlaneIntersectType;

	void RenderProfile3D(GLenum mode);

	vector<vector<Point_3> > MeshBoundingProfile3D;
	void RenderMeshBoundingProfile3D(GLenum mode);

	//void RenderCNIntersectPoint();

	void RenderCurvePlaneIntersectPoints();

	void CheckNbFacets(int iFacet,set<int>& CheckScope,set<int>& NbFacetsToCheck);
	void testrender();

	Point_3 OldCenter;
	//test points
	vector<Point_3> vecTestPoint;
	void RenderTestPoint();
};

/*Convert from Mesh to CGAL*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Convert_Mesh_To_CGALPoly : public CGAL::Modifier_base<HDS> {
public:
	Convert_Mesh_To_CGALPoly(const Mesh* modelIn) {model=modelIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface( model->sufvernum,model->suffacenum);
		for (int i=0;i<model->sufvernum;i++)
		{
			B.add_vertex(Point_3(model->sufver[3*i+0],
				model->sufver[3*i+1],
				model->sufver[3*i+2]));
		}

		for (int i=0;i<model->suffacenum;i++)
		{
			B.begin_facet();
			for (int j=0;j<3;j++)
			{
//				B.add_vertex_to_facet(model->sufface[3*i+j]);
				int test=model->sufface[3*i+j];
				B.add_vertex_to_facet(test);
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	const Mesh* model;
};
/*Convert from Mesh to CGAL*/