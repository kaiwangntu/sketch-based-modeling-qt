#pragma once

#ifndef CARVE_CSG_DEF_H
#define CARVE_CSG_DEF_H

#undef min
#undef max

#include <carve/input.hpp>
#include <carve/poly_decl.hpp>
#include <carve/csg.hpp>
#include <write_ply.hpp>


#ifdef _DEBUG
#pragma comment(lib,"carvelibdb.lib")
#pragma comment(lib,"fileformatsdb.lib")
#pragma comment(lib,"gloopdb.lib")
#else
#pragma comment(lib,"carvelib.lib")
#pragma comment(lib,"fileformats.lib")
#pragma comment(lib,"gloop.lib")
#endif


typedef carve::poly::Polyhedron CarvePoly;
typedef carve::poly::Vertex<3> CarveVertex;
typedef carve::poly::Edge<3> CarveEdge;
typedef carve::poly::Face<3> CarveFace;






#endif