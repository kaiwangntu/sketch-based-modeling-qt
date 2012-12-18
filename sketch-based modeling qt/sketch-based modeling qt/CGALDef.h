#pragma once

#ifndef CGAL_DEF_H
#define CGAL_DEF_H

//#include "stdafx.h"
#include <CGAL/Cartesian.h>
#include <CGAL/enum.h> 
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/centroid.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/io/print_wavefront.h>
#include <cgal/io/Color.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
//#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Circular_kernel_2.h>
//header files related to implicit surface
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/IO/Complex_2_in_triangulation_3_file_writer.h>
#include <CGAL/Gmpz.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>

//cgal related libs
#pragma comment(lib,"libmpfr-4.lib")
#pragma comment(lib,"libgmp-10.lib")


typedef double               FT;
//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Cartesian<FT>  K;
typedef K::Point_3           Point_3;
typedef K::Point_2           Point_2;
typedef K::Triangle_3        Triangle_3;
typedef K::Triangle_2        Triangle_2;
typedef K::Segment_2		 Segment_2;
typedef K::Segment_3		 Segment_3;
typedef K::Vector_2			 Vector_2;
typedef K::Vector_3			 Vector_3;
typedef K::Plane_3			 Plane_3;
typedef K::Line_2			 Line_2;
typedef K::Line_3			 Line_3;
typedef K::Ray_3			 Ray_3;
typedef K::Direction_3		 Direction_3;
typedef K::Circle_2			 Circle_2;
typedef K::Sphere_3			 Sphere_3;
//typedef CGAL::Polygon_2<K, std::list<Point_2> > Polygon_2;
typedef CGAL::Polygon_2<K>						Polygon_2;
typedef std::list<Polygon_2>                   Polygon_list_2;
typedef CGAL::Polygon_with_holes_2<K>          Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>        Pwh_list_2;
typedef std::vector<Polygon_with_holes_2>        Pwh_vector_2;
typedef Polygon_2::Vertex_iterator          Vertex_iterator_2;
typedef Polygon_2::Edge_const_iterator		Edge_const_iterator_2;
typedef Polygon_with_holes_2::Hole_const_iterator Hole_const_iterator_2;

//typedef CGAL::Homogeneous<CGAL::Gmpz>  GmpKernel;
typedef CGAL::Cartesian<CGAL::Gmpq>  GmpKernel;
typedef CGAL::Polyhedron_3<GmpKernel>  GmpPolyhedron;
typedef GmpPolyhedron::HalfedgeDS GmpHalfedgeDS;
typedef GmpKernel::Point_3 GmpPoint_3;
typedef GmpPolyhedron::Vertex_iterator             GmpVertex_iterator;
typedef GmpPolyhedron::Vertex_handle               GmpVertex_handle;
typedef GmpPolyhedron::Facet_iterator              GmpFacet_iterator;
typedef GmpPolyhedron::Facet_handle                GmpFacet_handle;
typedef GmpPolyhedron::Halfedge_around_facet_circulator GmpHalfedge_around_facet_circulator;
typedef GmpPolyhedron::Halfedge_around_vertex_circulator GmpHalfedge_around_vertex_circulator; 
typedef GmpPolyhedron::Halfedge_handle             GmpHalfedge_handle;
typedef GmpPolyhedron::Halfedge_iterator           GmpHalfedge_iterator;
typedef CGAL::Nef_polyhedron_3<GmpKernel> Nef_polyhedron;


//this one makes more exact results
//typedef CGAL::Exact_predicates_exact_constructions_kernel kernel;
//typedef kernel::Point_3 point3;
//typedef kernel::Point_2 point2;

//items related to implicit surface and its mesher
// default triangulation for Surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 SMDT3;
typedef CGAL::Complex_2_in_triangulation_3<SMDT3> C2T3;
typedef SMDT3::Geom_traits SMDT3GT;
typedef SMDT3GT::Sphere_3 SMDT3GTSphere_3;
typedef SMDT3GT::Point_3 SMDT3GTPoint_3;
typedef SMDT3GT::FT SMDT3GTFT;
typedef SMDT3GTFT (*ImpFunction)(SMDT3GTPoint_3);
typedef CGAL::Implicit_surface_3<SMDT3GT, ImpFunction> ImpSurface_3;
typedef CGAL::Polyhedron_3<SMDT3GT>  ImpPolyhedron;



//typedef CGAL::Exact_circular_kernel_2             Circular_k;
typedef CGAL::Algebraic_kernel_for_circles_2_2<FT>      Algebraic_k;
typedef CGAL::Circular_kernel_2<K,Algebraic_k>   Circular_k;

typedef Circular_k::Point_2                 Circular_Point_2;
typedef Circular_k::Circular_arc_2          Circular_arc_2;
typedef Circular_k::Line_arc_2              Circular_Line_arc_2;
typedef Circular_k::Circular_arc_point_2    Circular_arc_point_2;
typedef Circular_k::Circle_2				Circular_Circle_2;
typedef Circular_k::Segment_2				Circular_Segment_2;


//typedef float               FTf;
//typedef CGAL::Cartesian<FTf>  Kf;
//typedef Kf::Point_3           Point_3f;
//typedef Kf::Triangle_3        Triangle_3f;
//typedef Kf::Plane_3			 Plane_3f;
//typedef Kf::Ray_3			 Ray_3f;

/*CGAL mesh definition*/
// Tie all types together and a small main function using it.
///////////////////////////////////////////////////////////////////////////
struct Facet_normal {
	template <class Facet>
	void operator()( Facet& f) {
		typename Facet::Halfedge_handle h = f.halfedge();
		typename Facet::Vector_3 normal = CGAL::cross_product(
			h->next()->vertex()->point() - h->vertex()->point(),
			h->next()->next()->vertex()->point() - h->next()->vertex()->point());
		f.normal() = normal / std::sqrt( normal * normal);
	}
};

struct Vertex_normal {
	template <class Vertex>
	void operator()( Vertex& v) {
		typename Vector_3 normal = CGAL::NULL_VECTOR;

		//FT laplacian[3];
		//laplacian[0]=laplacian[1]=laplacian[2]=0;

		typedef typename Vertex::Halfedge_around_vertex_const_circulator Circ;
		Circ c = v.vertex_begin();
		Circ d = c;
		CGAL_For_all( c, d) {
			if ( ! c->is_border())
				normal = normal + c->facet()->normal();
		}
		v.normal() = normal / std::sqrt( normal * normal);

		//c = v.vertex_begin();
		//d = c;
		//CGAL_For_all( c, d) {
		//	if ( ! c->is_border())
		//	laplacian[0] = laplacian[0]+c->opposite()->vertex()->point().x();
		//	laplacian[1] = laplacian[1]+c->opposite()->vertex()->point().y();
		//	laplacian[2] = laplacian[2]+c->opposite()->vertex()->point().z();
		//}
		//int iDegree=v.vertex_degree();
		//laplacian[0]=laplacian[0]-v.point().x()*iDegree;
		//laplacian[1]=laplacian[1]-v.point().y()*iDegree;
		//laplacian[2]=laplacian[2]-v.point().z()*iDegree;
		//v.laplacian() = Vertex::Laplacian_3(laplacian[0],laplacian[1],laplacian[2]);

	}
};

template <class Refs, class T, class Norm>
class My_facet : public CGAL::HalfedgeDS_face_base<Refs, T> {
	Norm  norm;
	//dual mesh related
	int iDualMeshVertexIndex;//dual mesh vertex index w.r.t. this primal facet
	//used for dual mesh deform
	Vector_3 UniformLaplacian;
	Vector_3 WeightedLaplacian;
	std::vector<double> FacetWeights;
	double PosWeightedLaplacianSumWeight;//weights related to the neighbor edges of vertex i the halfedge points to
	double NegWeightedLaplacianSumWeight;//weights related to the neighbor edges of vertex j the opposite halfedge points to

	std::vector<Vector_3>  OldFacetVectors;
	std::vector<double> RigidDeformRotationMatrix;//3*3 matrix
	Vector_3 ScaleFactor;

public:
	// no constructors to repeat, since only default constructor mandatory
	typedef Norm Vector_3;
	Vector_3&       normal()       { return norm; }
	const Vector_3& normal() const { return norm; }

	//dual mesh related
	int GetDualMeshVertexIndex(){return iDualMeshVertexIndex;};
	void SetDualMeshVertexIndex(int DataIn){iDualMeshVertexIndex=DataIn;};

	Vector_3 GetUniformLaplacian() {return UniformLaplacian;}
	void SetUniformLaplacian(Vector_3 Lk) {UniformLaplacian=Lk;}

	Vector_3 GetWeightedLaplacian() {return WeightedLaplacian;}
	void SetWeightedLaplacian(Vector_3 Lk) {WeightedLaplacian=Lk;}

	std::vector<double> GetEdgeWeights() {return FacetWeights;}
	void SetEdgeWeights(std::vector<double> DataIn) {FacetWeights=DataIn;};

	double GetPosSumWeight() {return PosWeightedLaplacianSumWeight;}
	void SetPosSumWeight(double Lk) {PosWeightedLaplacianSumWeight=Lk;}

	double GetNegSumWeight() {return NegWeightedLaplacianSumWeight;}
	void SetNegSumWeight(double Lk) {NegWeightedLaplacianSumWeight=Lk;}

	std::vector<Vector_3> GetOldEdgeVectors() {return OldFacetVectors;}
	void SetOldEdgeVectors(std::vector<Vector_3> DataIn) {OldFacetVectors=DataIn;}

	std::vector<double> GetRigidDeformRotationMatrix() {return RigidDeformRotationMatrix;}
	void SetRigidDeformRotationMatrix(std::vector<double> DataIn) {RigidDeformRotationMatrix=DataIn;}

	Vector_3 GetScaleFactor() {return ScaleFactor;}
	void SetScaleFactor(Vector_3 DataIn) {ScaleFactor=DataIn;}

};

// A redefined items class for the Polyhedron_3 with a refined vertex
// class that contains a member for the normal vector and a refined
// facet with a normal vector instead of the plane equation (this is
// an alternative solution instead of using Polyhedron_traits_with_normals_3).
template <class Refs, class T, class P, class Norm>
class My_vertex : public CGAL::HalfedgeDS_vertex_base<Refs, T, P> {

	int iIndex;

	Norm  norm;
	//	Point_3 UniformLaplacian;
	//	Point_3 WeightedLaplacian;
	Vector_3 UniformLaplacian;
	Vector_3 WeightedLaplacian;
	double WeightedLaplacianSumWeight;
	std::vector<double> EdgeWeights;//each weight,the sum is WeightedLaplacianSumWeight
	double dSumArea;
	std::vector<Vector_3>  OldEdgeVectors;
	std::vector<double> RigidDeformRotationMatrix;//3*3 matrix
	std::vector<double> SecondRigidDeformRotationMatrix;//3*3 matrix
	std::vector<double> RSRTransformMatrix;
	Vector_3 ScaleFactor;
	double dMeanCurvature;
	double dGaussianCurvature;
	std::vector<double> vecColor;

	//dual mesh related
	int iFacetOnPrimalMeshIndex;
	//indices of vertices of the primal facet which dual mesh point lies on
	//the vertices are stored in vector<Vertex_Handle> vecHandleNbVertex+ROI+Anchor;
	std::vector<int> vecPrimalVertexIndex;

	//for storing new positions during local refinement
	Point_3 LRNewPos;

	//for defining material
	double dMaterial;

	//reserved,for any temporal use
	int iReserved;

public:
	My_vertex() {} // repeat mandatory constructors
	My_vertex( const P& pt) : CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt) {}

	void SetVertexIndex(int iDataIn){iIndex=iDataIn;}
	int GetVertexIndex(){return iIndex;}

	typedef Norm Vector_3;
	Vector_3&       normal()       { return norm; }
	const Vector_3& normal() const { return norm; }

	Vector_3 GetUniformLaplacian() {return UniformLaplacian;}
	void SetUniformLaplacian(Vector_3 Lk) {UniformLaplacian=Lk;}

	Vector_3 GetWeightedLaplacian() {return WeightedLaplacian;}
	void SetWeightedLaplacian(Vector_3 Lk) {WeightedLaplacian=Lk;}

	std::vector<double> GetEdgeWeights() {return EdgeWeights;}
	void SetEdgeWeights(std::vector<double> DataIn) {EdgeWeights=DataIn;};

	double GetWeightedLaplacianSumWeight() {return WeightedLaplacianSumWeight;}
	void SetWeightedLaplacianSumWeight(double Lk) {WeightedLaplacianSumWeight=Lk;}

	double GetSumArea() {return dSumArea;};
	void SetSumArea(double dDataIn) {dSumArea=dDataIn;}

	std::vector<Vector_3> GetOldEdgeVectors() {return OldEdgeVectors;}
	void SetOldEdgeVectors(std::vector<Vector_3> DataIn) {OldEdgeVectors=DataIn;}

	std::vector<double> GetRigidDeformRotationMatrix() {return RigidDeformRotationMatrix;}
	void SetRigidDeformRotationMatrix(std::vector<double> DataIn) {RigidDeformRotationMatrix=DataIn;}

	std::vector<double> GetSecondRigidDeformRotationMatrix() {return SecondRigidDeformRotationMatrix;}
	void SetSecondRigidDeformRotationMatrix(std::vector<double> DataIn) {SecondRigidDeformRotationMatrix=DataIn;}

	std::vector<double> GetRSRTransformMatrix() {return RSRTransformMatrix;}
	void SetRSRTransformMatrix(std::vector<double> DataIn) {RSRTransformMatrix=DataIn;}

	Vector_3 GetScaleFactor() {return ScaleFactor;}
	void SetScaleFactor(Vector_3 DataIn) {ScaleFactor=DataIn;}

	double GetMeanCurvature() {return dMeanCurvature;}
	void SetMeanCurvature(double dDatain) {dMeanCurvature=dDatain;}

	double GetGaussianCurvature() {return dGaussianCurvature;}
	void SetGaussianCurvature(double dDatain) {dGaussianCurvature=dDatain;}

	std::vector<double> GetColor() {return vecColor;}
	void SetColor(std::vector<double> dDatain) {vecColor=dDatain;}

	//dual mesh related
	//get and set index of facet on primal mesh w.r.t. this dual mesh vertex
	int GetFacetOnPrimalMeshIndex(){return iFacetOnPrimalMeshIndex;};
	void SetFacetOnPrimalMeshIndex(int DataIn){iFacetOnPrimalMeshIndex=DataIn;};

	std::vector<int> GetPrimalVertexIndices() {return vecPrimalVertexIndex;}
	void SetPrimalVertexIndices(std::vector<int> DataIn) {vecPrimalVertexIndex=DataIn;};

	Point_3 GetLRNewPos() {return LRNewPos;}
	void SetLRNewPos(Point_3 DataIn) {LRNewPos=DataIn;}

	double GetMaterial() {return dMaterial;}
	void SetMaterial(double dDataIn) {dMaterial=dDataIn;}

	int GetReserved() {return iReserved;}
	void SetReserved(int iDataIn) {iReserved=iDataIn;}
};

//redefined halfedge
template <class Refs, class T>
class My_halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs, T> {
	//used for edge-based laplacian deform
	Vector_3 UniformLaplacian;
	Vector_3 WeightedLaplacian;
	std::vector<double> EdgeWeights;
	double PosWeightedLaplacianSumWeight;//weights related to the neighbor edges of vertex i the halfedge points to
	double NegWeightedLaplacianSumWeight;//weights related to the neighbor edges of vertex j the opposite halfedge points to

	std::vector<Vector_3>  OldEdgeVectors;
	std::vector<double> RigidDeformRotationMatrix;//3*3 matrix
	Vector_3 ScaleFactor;

	//for storing the latest midpoint position
	//when the update of the positions of the two end vertices is undesired
	Point_3 NewMidPoint;
	//for building wedge edge mesh
	int iEdgeIndex;

	//for defining material
	double dMaterial;

	//reserved,for any temporal use
	int iReserved;

public:
	Vector_3 GetUniformLaplacian() {return UniformLaplacian;}
	void SetUniformLaplacian(Vector_3 Lk) {UniformLaplacian=Lk;}

	Vector_3 GetWeightedLaplacian() {return WeightedLaplacian;}
	void SetWeightedLaplacian(Vector_3 Lk) {WeightedLaplacian=Lk;}

	std::vector<double> GetEdgeWeights() {return EdgeWeights;}
	void SetEdgeWeights(std::vector<double> DataIn) {EdgeWeights=DataIn;};

	double GetPosSumWeight() {return PosWeightedLaplacianSumWeight;}
	void SetPosSumWeight(double Lk) {PosWeightedLaplacianSumWeight=Lk;}

	double GetNegSumWeight() {return NegWeightedLaplacianSumWeight;}
	void SetNegSumWeight(double Lk) {NegWeightedLaplacianSumWeight=Lk;}

	std::vector<Vector_3> GetOldEdgeVectors() {return OldEdgeVectors;}
	void SetOldEdgeVectors(std::vector<Vector_3> DataIn) {OldEdgeVectors=DataIn;}

	std::vector<double> GetRigidDeformRotationMatrix() {return RigidDeformRotationMatrix;}
	void SetRigidDeformRotationMatrix(std::vector<double> DataIn) {RigidDeformRotationMatrix=DataIn;}

	Vector_3 GetScaleFactor() {return ScaleFactor;}
	void SetScaleFactor(Vector_3 DataIn) {ScaleFactor=DataIn;}

	Point_3 GetNewMidPoint() {return NewMidPoint;}
	void SetNewMidPoint(Point_3 DataIn) {NewMidPoint=DataIn;}

	//for building wedge edge mesh
	int GetEdgeIndex() {return iEdgeIndex;}
	void SetEdgeIndex(int iDataIn) {iEdgeIndex=iDataIn;}

	double GetMaterial() {return dMaterial;}
	void SetMaterial(double dDataIn) {dMaterial=dDataIn;}

	int GetReserved() {return iReserved;}
	void SetReserved(int iDataIn) {iReserved=iDataIn;}
};

struct My_items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Vertex_wrapper {
		typedef typename Traits::Point_3  Point;
		typedef typename Traits::Vector_3 Normal;
		typedef My_vertex<Refs, CGAL::Tag_true, Point, Normal> Vertex;
	};
	template <class Refs, class Traits>
	struct Face_wrapper {
		typedef typename Traits::Vector_3 Normal;
		typedef My_facet<Refs, CGAL::Tag_true, Normal> Face;
	};
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef My_halfedge<Refs,CGAL::Tag_true> Halfedge;
	};
};


typedef CGAL::Polyhedron_3<K, My_items>    Polyhedron;

class KW_Mesh : public Polyhedron {
public:
	//set render infor for vertex position,vertex normal,vertex index,face index
	void SetRenderInfo(bool bSetVerInfo,bool bSetNormInfo,bool bSetVerInd,bool bSetFaceInd,bool bSetColorInfo);
	//clear data
	void clear();

protected:
	//set vertex pos and norm for rendering
	void SetRenderVerInfo(bool bSetVerInfo,bool bSetNormInfo,bool bSetColorInfo);
	//set index for each vertex
	void SetVerIndices();
	// Computes auxiliar data 
	void buildFacesIndices();
public:
	//store for rendering
	std::vector<int> vecRenderFaceType;//may be triangle,quad,...
	std::vector<std::vector<int>>	vecvecRenderFaceID;//corresponding to FacetType
	std::vector<double> vecRenderVerPos;//vertex positions,don't forget to update!
	std::vector<double> vecRenderNorm;//vertex normals,don't forget to update!
	std::vector<double> vecRenderVerColor;//vertex colors
};

typedef KW_Mesh::Vertex_iterator                    Vertex_iterator;
typedef KW_Mesh::Vertex_handle                     Vertex_handle;
typedef KW_Mesh::Facet_iterator                    Facet_iterator;
typedef KW_Mesh::Facet_handle                     Facet_handle;
typedef KW_Mesh::Point_iterator                    Point_iterator;
typedef KW_Mesh::Halfedge_around_facet_circulator Halfedge_around_facet_circulator;
typedef KW_Mesh::Halfedge_around_vertex_circulator Halfedge_around_vertex_circulator; 
typedef KW_Mesh::HalfedgeDS             HalfedgeDS;
typedef KW_Mesh::Halfedge_handle                    Halfedge_handle;
typedef KW_Mesh::Halfedge_iterator                    Halfedge_iterator;
/*CGAL mesh definition*/

/* 2D triangulation*/
struct Triangulation2K : CGAL::Exact_predicates_inexact_constructions_kernel {};

typedef CGAL::Triangulation_vertex_base_2<Triangulation2K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<Triangulation2K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Triangulation2K, TDS, Itag> CDT;
typedef CDT::Point          CDTPoint;
/* 2D triangulation*/

#endif