#pragma once

#include "PreDef.h"
#include "CGALDef.h"
#include <QMessageBox>


#ifndef M_PI
#define M_PI 3.14159265f
#endif

#define GLM_NONE     (0)            /* render with only vertices */
#define GLM_FLAT     (1 << 0)       /* render with facet normals */
#define GLM_SMOOTH   (1 << 1)       /* render with vertex normals */
#define GLM_TEXTURE  (1 << 2)       /* render with texture coords */
#define GLM_COLOR    (1 << 3)       /* render with colors */
#define GLM_MATERIAL (1 << 4)       /* render with materials */


//type define
typedef std::pair <int,int> Int_Int_Pair;


//Edge Structure:each edge is represented by the start and end point(make them a pair)
//each edge is shared by two triangles
//typedef pair <int,int> Int_Int_Pair;//start and end point,defined in SymbolsDef.h 
typedef pair <Int_Int_Pair,vector<int>> Edge_Tri_Pair;
typedef map<Int_Int_Pair,vector<int>> EdgeStruct;//vector<int> means the two triangles

typedef pair <int, vector<int>> Int_Vector_Pair;
typedef pair <int, Point_3> Int_Point_3_Pair;
typedef pair <int, vector<Point_3>> Int_VectorPoint_3_Pair;


//Int_Int_Pair:edge   Point_3:mid point of this edge
typedef pair <Int_Int_Pair,Point_3> Edge_Point_3_Pair;
//Int_Int_Pair:edge   int:mid point Index of this edge
typedef pair <Int_Int_Pair,int> Edge_Int_Pair;

//mesh structures in CPP corresponding to the GLM structures
typedef struct _CPPGLMmaterial
{
	char* name;                   /* name of material */
	GLfloat diffuse[4];           /* diffuse component */
	GLfloat ambient[4];           /* ambient component */
	GLfloat specular[4];          /* specular component */
	GLfloat emmissive[4];         /* emmissive component */
	GLfloat shininess;            /* specular exponent */
} CPPGLMmaterial;

typedef struct _CPPGLMtriangle {
	GLuint vindices[3];           /* array of triangle vertex indices */
	GLuint nindices[3];           /* array of triangle normal indices */
	GLuint tindices[3];           /* array of triangle texcoord indices*/
	GLuint findex;                /* index of triangle facet normal */
} CPPGLMtriangle;


typedef struct _CPPGLMmodel {
	char*    pathname;            /* path to this model */
	char*    mtllibname;          /* name of the material library */
	GLuint   numvertices;         /* number of vertices in model */
	vector<float> vertices;            /* array of vertices  */

	GLuint   numnormals;          /* number of normals in model */
	vector<float> normals;             /* array of normals */

	GLuint   numtexcoords;        /* number of texcoords in model */
	vector<float> texcoords;           /* array of texture coordinates */

	GLuint   numfacetnorms;       /* number of facetnorms in model */
	vector<float> facetnorms;          /* array of facetnorms */

	GLuint       numtriangles;    /* number of triangles in model */
	vector<CPPGLMtriangle> triangles;       /* array of triangles */

	GLuint       nummaterials;    /* number of materials in model */
	vector<CPPGLMmaterial> materials;       /* array of materials */

	GLfloat position[3];          /* position of the model */

} CPPGLMmodel;



/* GLMmaterial: Structure that defines a material in a model. 
*/
typedef struct _GLMmaterial
{
	char* name;                   /* name of material */
	GLfloat diffuse[4];           /* diffuse component */
	GLfloat ambient[4];           /* ambient component */
	GLfloat specular[4];          /* specular component */
	GLfloat emmissive[4];         /* emmissive component */
	GLfloat shininess;            /* specular exponent */
} GLMmaterial;

/* GLMtriangle: Structure that defines a triangle in a model.
*/
typedef struct _GLMtriangle {
	GLuint vindices[3];           /* array of triangle vertex indices */
	GLuint nindices[3];           /* array of triangle normal indices */
	GLuint tindices[3];           /* array of triangle texcoord indices*/
	GLuint findex;                /* index of triangle facet normal */
} GLMtriangle;

/* GLMmodel: Structure that defines a model.
*/
typedef struct _GLMmodel {
	char*    pathname;            /* path to this model */
	char*    mtllibname;          /* name of the material library */

	GLuint   numvertices;         /* number of vertices in model */
	GLfloat* vertices;            /* array of vertices  */

	GLuint   numnormals;          /* number of normals in model */
	GLfloat* normals;             /* array of normals */

	GLuint   numtexcoords;        /* number of texcoords in model */
	GLfloat* texcoords;           /* array of texture coordinates */

	GLuint   numfacetnorms;       /* number of facetnorms in model */
	GLfloat* facetnorms;          /* array of facetnorms */

	GLuint       numtriangles;    /* number of triangles in model */
	GLMtriangle* triangles;       /* array of triangles */

	GLuint       nummaterials;    /* number of materials in model */
	GLMmaterial* materials;       /* array of materials */

	GLfloat position[3];          /* position of the model */

} GLMmodel;

#define T(x) (model->triangles[(x)])


/* _GLMnode: general purpose node */
typedef struct _GLMnode {
	GLuint         index;
	GLboolean      averaged;
	struct _GLMnode* next;
} GLMnode;

//similar to GLMmodel, allowing more than 3 vertices on each face
class KW_Polyhedron
{
public:
	vector<Point_3> vecVertex;
	vector<vector<int>> vecFacet;
};

/*Convert from GLM to CGAL*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Build_triangle : public CGAL::Modifier_base<HDS> {
public:
	Build_triangle(const GLMmodel* modelIn) {model=modelIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		typedef pair<int,int> Int_Int_Pair;
		vector<Int_Int_Pair> VertexIndex;
		B.begin_surface( model->numvertices,model->numtriangles);
		for (unsigned int i=1;i<=model->numvertices;i++)
		{
			B.add_vertex(Point_3(model->vertices[3 * i+0],
				model->vertices[3 * i+1],
				model->vertices[3 * i+2]));
		}

		for (unsigned int i=0;i<model->numtriangles;i++)
		{
			triangle = &T(i);
			B.begin_facet();
			for (int j=0;j<3;j++)
			{
				B.add_vertex_to_facet(triangle->vindices[j]-1);
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	const GLMmodel* model;
	GLMtriangle* triangle;
};
/*Convert from GLM to CGAL*/

/*Convert from KW_Polyhedron to CGAL*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Build_polyhedron : public CGAL::Modifier_base<HDS> {
public:
	Build_polyhedron(KW_Polyhedron modelIn) {model=modelIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface(model.vecVertex.size(),model.vecFacet.size());
		for (unsigned int i=0;i<model.vecVertex.size();i++)
		{
			B.add_vertex(model.vecVertex.at(i));
		}
		for (unsigned int i=0;i<model.vecFacet.size();i++)
		{
			B.begin_facet();
			for (unsigned int j=0;j<model.vecFacet.at(i).size();j++)
			{
				B.add_vertex_to_facet(model.vecFacet.at(i).at(j)-1);
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	KW_Polyhedron model;
};
/*Convert from KW_Polyhedron to CGAL*/

//Build a wedge edge mesh 
template <class HDS>
class Build_WedgeEdgeMesh : public CGAL::Modifier_base<HDS> {
public:
	Build_WedgeEdgeMesh(KW_Polyhedron modelIn) {model=modelIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface(model.vecVertex.size(),model.vecFacet.size());
		for (unsigned int i=0;i<model.vecVertex.size();i++)
		{
			B.add_vertex(model.vecVertex.at(i));
		}
		for (unsigned int i=0;i<model.vecFacet.size();i++)
		{
			B.begin_facet();
			for (unsigned int j=0;j<model.vecFacet.at(i).size();j++)
			{
				B.add_vertex_to_facet(model.vecFacet.at(i).at(j));//take care of the indices!!
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	KW_Polyhedron model;
};
/*Convert from KW_Polyhedron to CGAL*/

class OBJHandle
{
public:
	OBJHandle(void);
	static GLfloat	glmUnitize(GLMmodel* model);
	static GLvoid	glmDimensions(GLMmodel* model, GLfloat* dimensions);
	static GLvoid	glmScale(GLMmodel* model, GLfloat scale);
	static GLvoid	glmReverseWinding(GLMmodel* model);
	static GLvoid	glmFacetNormals(GLMmodel* model);
	static GLvoid	glmVertexNormals(GLMmodel* model, GLfloat angle);
	static GLvoid	glmDelete(GLMmodel* model);
	static GLvoid  glmReadOBJ(char* filename,KW_Mesh& mesh,bool bScale,bool bCenter);

	//with new data structure
	static GLvoid  glmReadOBJNew(const char* filename,KW_Mesh& mesh,bool bScale,bool bCenter,vector<double> vecDefaultColor,bool bSetRenderInfo=true);
	//set color for each vertex
	static void SetUniformMeshColor(KW_Mesh& mesh,std::vector<double> vecColor);

	static GLvoid	glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode);
	static GLvoid	glmDraw(GLMmodel* model, GLuint mode);
	static GLuint 	glmList(GLMmodel* model, GLuint mode);
	static GLvoid 	glmWeld(GLMmodel* model, GLfloat epsilon);
	static GLubyte* glmReadPPM(const char* filename, int* width, int* height);

	static GLvoid ConvertGLMmodeltoCPPGLMmodel(GLMmodel * model,CPPGLMmodel & CPPmodel);
	static GLvoid ConvertCPPGLMmodeltoGLMmodel(CPPGLMmodel CPPmodel,GLMmodel * model);

	//compute the new vertices positions after rotation&translation
	static GLvoid GetCurrentModelVerticePos(CPPGLMmodel&  cppmodel,GLdouble * ModelViewMatrix);
	static GLvoid GetCurrentModelVerticePos(GLMmodel * model,GLdouble * ModelViewMatrix);

	//update the edge&vertices info of the model,
	//called when a new OBJ loaded or the topo of the current model changed
	//variables involved:
	//modelEdgeInfo:  key:edge   value:two neighbor triangles of the edge
	//modelVertexInfo: key:index of vertex  value:neighbor vertices of the vertex
	//static GLvoid UpDateTopoInfo(const GLMmodel * model);
	//static EdgeStruct GetEdgeTopoInfo(){return modelEdgeInfo;}
	//static map<int,vector<int>> GetVertexTopoInfo(){return modelVertexInfo;} 

	//loop subdivision
	//static GLvoid LoopSubdivision(CPPGLMmodel & CPPmodel);

	//deep copy
	static void GetModelCopy(GLMmodel * model,GLMmodel * modelCopy);

	static GLfloat	UnitizeCGALPolyhedron(KW_Mesh& mesh,bool bScale,bool bTranslateToCenter);
	static GLvoid DrawCGALPolyhedron(KW_Mesh * pmesh, int iViewmode, GLuint color,vector<double> vecDefaultColor, GLenum mode);
	static GLvoid LoopSubDivision(KW_Mesh& mesh);
	static GLvoid CCSubDivision(KW_Mesh& mesh);

public:
	~OBJHandle(void);

private:
	static GLfloat	glmMax(GLfloat a, GLfloat b);
	static GLfloat	glmAbs(GLfloat f);
	static GLfloat	glmDot(GLfloat* u, GLfloat* v);
	static GLvoid	glmCross(GLfloat* u, GLfloat* v, GLfloat* n);
	static GLvoid	glmNormalize(GLfloat* v);
	static GLboolean glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon);
	static GLfloat* glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon);
	static GLuint glmFindMaterial(GLMmodel* model, char* name);
	static char* glmDirName(char* path);
	static GLboolean glmReadMTL(GLMmodel* model, char* name);
	static GLvoid glmWriteMTL(GLMmodel* model, char* modelpath, char* mtllibname);
	static GLboolean glmFirstPass(GLMmodel* model, FILE* file);
	static GLvoid glmSecondPass(GLMmodel* model, FILE* file);
	//with new data structure
	static GLvoid glmSecondPass(KW_Polyhedron& model, FILE* file);

	//the indices of the n vertices in C are:1,2,...n
	//the indices of the n vertices in CPP are:0,1,...n-1
	//static EdgeStruct modelEdgeInfo;
	//static EdgeStruct CPPmodelEdgeInfo;
	//map<int,vector<int>> modelVertexInfo;
	//map<int,vector<int>> CPPmodelVertexInfo;
	//get another vertex index of the neighbor triangle
	//static int GetAnotherVertexIndex(const CPPGLMmodel CPPmodel,int iTriIndex,int iKnownVer0,int iKnownVer1);
	//compute the alpha of loop subdivision
	//static double Alpha(int iNeighborNum);

	static GLboolean ConvertToCGALPolyhedron(const GLMmodel* model,KW_Mesh& mesh);

	static GLboolean ConvertToCGALPolyhedronNew(KW_Polyhedron& model,KW_Mesh& mesh);
};
