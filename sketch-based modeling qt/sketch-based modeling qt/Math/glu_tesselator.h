#pragma once
#ifndef GLU_TESSELATOR_H
#define GLU_TESSELATOR_H

#include "../PreDef.h"

//modified from vcg glu_tesselator used in MeshLab version 1.2.3

class glu_tesselator
{
public:

	static inline void tesselate(vector<Point_3>& outlines, vector<int> & indices)
	{
		tess_prim_data_vec t_data;

		do_tesselation(outlines, t_data);

		//int k = 0;
		for (size_t i=0; i<t_data.size(); ++i)
		{
			const size_t st = t_data[i].indices.size();
			if (st < 3) continue;

			switch (t_data[i].type)
			{
			case GL_TRIANGLES:
				for (size_t j=0; j<st; ++j)
				{
					indices.push_back(t_data[i].indices[j]);
				}
				break;

			case GL_TRIANGLE_STRIP:
				{
					int i0 = t_data[i].indices[0];
					int i1 = t_data[i].indices[1];

					bool ccw = true;

					for (size_t j=2; j<st; ++j)
					{
						const int i2 = t_data[i].indices[j];

						indices.push_back(i0);
						indices.push_back(i1);
						indices.push_back(i2);

						if (ccw) i0 = i2;
						else     i1 = i2;

						ccw = !ccw;
					}																	
				}
				break;

			case GL_TRIANGLE_FAN:
				{
					const int first = t_data[i].indices[0];
					int prev = t_data[i].indices[1];

					for (size_t j=2; j<st; ++j)
					{
						const int curr = t_data[i].indices[j];

						indices.push_back(first);
						indices.push_back(prev);
						indices.push_back(curr);

						prev = curr;
					}
				}
				break;

			default:
				break;
			}
		}
	}

protected:

	class tess_prim_data
	{
	public:

		GLenum type;
		std::vector<int> indices;

		tess_prim_data(void) { }
		tess_prim_data(GLenum t) : type(t) { }
	};

	typedef std::vector<tess_prim_data> tess_prim_data_vec;

	static void CALLBACK begin_cb(GLenum type, void * polygon_data)
	{
		tess_prim_data_vec * t_data = (tess_prim_data_vec *)polygon_data;
		t_data->push_back(tess_prim_data(type));
	}

	static void CALLBACK end_cb(void * polygon_data)
	{
		(void)polygon_data;
	}

	static void CALLBACK vertex_cb(void * vertex_data, void * polygon_data)
	{
		tess_prim_data_vec * t_data = (tess_prim_data_vec *)polygon_data;
		t_data->back().indices.push_back((int)((size_t)vertex_data));
	}

	static void do_tesselation(vector<Point_3>& outlines, tess_prim_data_vec & t_data)
	{
		GLUtesselator * tess = gluNewTess();
		gluTessCallback(tess, GLU_TESS_BEGIN_DATA,  (GLvoid (CALLBACK *)())(begin_cb));
		gluTessCallback(tess, GLU_TESS_END_DATA,    (GLvoid (CALLBACK *)())(end_cb));
		gluTessCallback(tess, GLU_TESS_VERTEX_DATA, (GLvoid (CALLBACK *)())(vertex_cb));
		void * polygon_data = (void *)(&t_data);

		GLdouble vertex[3];

		int k = 0;
		gluTessBeginPolygon(tess, polygon_data);
		gluTessBeginContour(tess);
		for (size_t j=0; j<outlines.size(); ++j)
		{
			get_position(outlines[j], vertex);
			gluTessVertex(tess, vertex, (void *)k);
			++k;
		}
		gluTessEndContour(tess);
		gluTessEndPolygon(tess);

		gluDeleteTess(tess);
	}

	static inline void get_position(const Point_3 p, GLdouble * d)
	{
		d[0] = (GLdouble)(p.x());
		d[1] = (GLdouble)(p.y());
		d[2] = (GLdouble)(p.z());
	}
};

#endif
