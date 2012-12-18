#include "OBJHandle.h"

OBJHandle::OBJHandle(void)
{
}

OBJHandle::~OBJHandle(void)
{
//	modelEdgeInfo.clear();
//	CPPmodelEdgeInfo.clear();
//	modelVertexInfo.clear();
//	CPPmodelVertexInfo.clear();
}

/* public functions */


/* glmUnitize: "unitize" a model by translating it to the origin and
* scaling it to fit in a unit cube around the origin.   Returns the
* scalefactor used.
*
* model - properly initialized GLMmodel structure 
*/
GLfloat OBJHandle::glmUnitize(GLMmodel* model)
{
	GLuint  i;
	GLfloat maxx, minx, maxy, miny, maxz, minz;
	GLfloat cx, cy, cz, w, h, d;
	GLfloat scale;

	assert(model);
	assert(model->vertices);

	/* get the max/mins */
	maxx = minx = model->vertices[3 + 0];
	maxy = miny = model->vertices[3 + 1];
	maxz = minz = model->vertices[3 + 2];
	for (i = 1; i <= model->numvertices; i++) {
		if (maxx < model->vertices[3 * i + 0])
			maxx = model->vertices[3 * i + 0];
		if (minx > model->vertices[3 * i + 0])
			minx = model->vertices[3 * i + 0];

		if (maxy < model->vertices[3 * i + 1])
			maxy = model->vertices[3 * i + 1];
		if (miny > model->vertices[3 * i + 1])
			miny = model->vertices[3 * i + 1];

		if (maxz < model->vertices[3 * i + 2])
			maxz = model->vertices[3 * i + 2];
		if (minz > model->vertices[3 * i + 2])
			minz = model->vertices[3 * i + 2];
	}

	/* calculate model width, height, and depth */
	/* modified by neo6 */
#if 0
	w = glmAbs(maxx) + glmAbs(minx);
	h = glmAbs(maxy) + glmAbs(miny);
	d = glmAbs(maxz) + glmAbs(minz);
#endif	
	w = maxx - minx;
	h = maxy - miny;
	d = maxz - minz;

	/* calculate center of the model */
	cx = (maxx + minx) / 2.0;
	cy = (maxy + miny) / 2.0;
	cz = (maxz + minz) / 2.0;

	/* calculate unitizing scale factor */
	/* modified by neo6 */
#if 0
	scale = 2.0 / glmMax(glmMax(w, h), d);
#endif
	scale = 1.0 / glmMax(glmMax(w, h), d);

	/* translate around center then scale */
	for (i = 1; i <= model->numvertices; i++) {
		model->vertices[3 * i + 0] -= cx;
		model->vertices[3 * i + 1] -= cy;
		model->vertices[3 * i + 2] -= cz;
		model->vertices[3 * i + 0] *= scale;
		model->vertices[3 * i + 1] *= scale;
		model->vertices[3 * i + 2] *= scale;
	}

	return scale;
}
/* glmDimensions: Calculates the dimensions (width, height, depth) of
* a model.
*
* model   - initialized GLMmodel structure
* dimensions - array of 3 GLfloats (GLfloat dimensions[3])
*/
GLvoid OBJHandle::glmDimensions(GLMmodel* model, GLfloat* dimensions)
{
	GLuint i;
	GLfloat maxx, minx, maxy, miny, maxz, minz;

	assert(model);
	assert(model->vertices);
	assert(dimensions);

	/* get the max/mins */
	maxx = minx = model->vertices[3 + 0];
	maxy = miny = model->vertices[3 + 1];
	maxz = minz = model->vertices[3 + 2];
	for (i = 1; i <= model->numvertices; i++) {
		if (maxx < model->vertices[3 * i + 0])
			maxx = model->vertices[3 * i + 0];
		if (minx > model->vertices[3 * i + 0])
			minx = model->vertices[3 * i + 0];

		if (maxy < model->vertices[3 * i + 1])
			maxy = model->vertices[3 * i + 1];
		if (miny > model->vertices[3 * i + 1])
			miny = model->vertices[3 * i + 1];

		if (maxz < model->vertices[3 * i + 2])
			maxz = model->vertices[3 * i + 2];
		if (minz > model->vertices[3 * i + 2])
			minz = model->vertices[3 * i + 2];
	}

	/* calculate model width, height, and depth */
	dimensions[0] = glmAbs(maxx) + glmAbs(minx);
	dimensions[1] = glmAbs(maxy) + glmAbs(miny);
	dimensions[2] = glmAbs(maxz) + glmAbs(minz);
}


/* glmScale: Scales a model by a given amount.
* 
* model - properly initialized GLMmodel structure
* scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
*/
GLvoid OBJHandle::glmScale(GLMmodel* model, GLfloat scale)
{
	GLuint i;

	for (i = 1; i <= model->numvertices; i++) {
		model->vertices[3 * i + 0] *= scale;
		model->vertices[3 * i + 1] *= scale;
		model->vertices[3 * i + 2] *= scale;
	}
}

/* glmReverseWinding: Reverse the polygon winding for all polygons in
* this model.   Default winding is counter-clockwise.  Also changes
* the direction of the normals.
* 
* model - properly initialized GLMmodel structure 
*/
GLvoid OBJHandle::glmReverseWinding(GLMmodel* model)
{
	GLuint i, swap;

	assert(model);

	for (i = 0; i < model->numtriangles; i++) {
		swap = T(i).vindices[0];
		T(i).vindices[0] = T(i).vindices[2];
		T(i).vindices[2] = swap;

		if (model->numnormals) {
			swap = T(i).nindices[0];
			T(i).nindices[0] = T(i).nindices[2];
			T(i).nindices[2] = swap;
		}

		if (model->numtexcoords) {
			swap = T(i).tindices[0];
			T(i).tindices[0] = T(i).tindices[2];
			T(i).tindices[2] = swap;
		}
	}

	/* reverse facet normals */
	for (i = 1; i <= model->numfacetnorms; i++) {
		model->facetnorms[3 * i + 0] = -model->facetnorms[3 * i + 0];
		model->facetnorms[3 * i + 1] = -model->facetnorms[3 * i + 1];
		model->facetnorms[3 * i + 2] = -model->facetnorms[3 * i + 2];
	}

	/* reverse vertex normals */
	for (i = 1; i <= model->numnormals; i++) {
		model->normals[3 * i + 0] = -model->normals[3 * i + 0];
		model->normals[3 * i + 1] = -model->normals[3 * i + 1];
		model->normals[3 * i + 2] = -model->normals[3 * i + 2];
	}
}

/* glmFacetNormals: Generates facet normals for a model (by taking the
* cross product of the two vectors derived from the sides of each
* triangle).  Assumes a counter-clockwise winding.
*
* model - initialized GLMmodel structure
*/
GLvoid OBJHandle::glmFacetNormals(GLMmodel* model)
{
	GLuint  i;
	GLfloat u[3];
	GLfloat v[3];

	assert(model);
	assert(model->vertices);

	/* clobber any old facetnormals */
	if (model->facetnorms)
		free(model->facetnorms);

	/* allocate memory for the new facet normals */
	model->numfacetnorms = model->numtriangles;
	model->facetnorms = (GLfloat*)malloc(sizeof(GLfloat) *
		3 * (model->numfacetnorms + 1));

	for (i = 0; i < model->numtriangles; i++) {
		model->triangles[i].findex = i+1;

		u[0] = model->vertices[3 * T(i).vindices[1] + 0] -
			model->vertices[3 * T(i).vindices[0] + 0];
		u[1] = model->vertices[3 * T(i).vindices[1] + 1] -
			model->vertices[3 * T(i).vindices[0] + 1];
		u[2] = model->vertices[3 * T(i).vindices[1] + 2] -
			model->vertices[3 * T(i).vindices[0] + 2];

		v[0] = model->vertices[3 * T(i).vindices[2] + 0] -
			model->vertices[3 * T(i).vindices[0] + 0];
		v[1] = model->vertices[3 * T(i).vindices[2] + 1] -
			model->vertices[3 * T(i).vindices[0] + 1];
		v[2] = model->vertices[3 * T(i).vindices[2] + 2] -
			model->vertices[3 * T(i).vindices[0] + 2];
		glmCross(u, v, &model->facetnorms[3 * (i+1)]);
		glmNormalize(&model->facetnorms[3 * (i+1)]);
	}
}


/* glmVertexNormals: Generates smooth vertex normals for a model.
* First builds a list of all the triangles each vertex is in.   Then
* loops through each vertex in the the list averaging all the facet
* normals of the triangles each vertex is in.   Finally, sets the
* normal index in the triangle for the vertex to the generated smooth
* normal.   If the dot product of a facet normal and the facet normal
* associated with the first triangle in the list of triangles the
* current vertex is in is greater than the cosine of the angle
* parameter to the function, that facet normal is not added into the
* average normal calculation and the corresponding vertex is given
* the facet normal.  This tends to preserve hard edges.  The angle to
* use depends on the model, but 90 degrees is usually a good start.
*
* model - initialized GLMmodel structure
* angle - maximum angle (in degrees) to smooth across
*/
GLvoid OBJHandle::glmVertexNormals(GLMmodel* model, GLfloat angle)
{
	GLMnode*    node;
	GLMnode*    tail;
	GLMnode** members;
	GLfloat*    normals;
	GLuint  numnormals;
	GLfloat average[3];
	GLfloat dot, cos_angle;
	GLuint  i, avg;

	assert(model);
	assert(model->facetnorms);

	/* calculate the cosine of the angle (in degrees) */
	cos_angle = cos(angle * M_PI / 180.0);

	/* nuke any previous normals */
	if (model->normals)
		free(model->normals);

	/* allocate space for new normals */
	model->numnormals = model->numtriangles * 3; /* 3 normals per triangle */
	model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));

	/* allocate a structure that will hold a linked list of triangle
	indices for each vertex */
	members = (GLMnode**)malloc(sizeof(GLMnode*) * (model->numvertices + 1));
	for (i = 1; i <= model->numvertices; i++)
		members[i] = NULL;

	/* for every triangle, create a node for each vertex in it */
	for (i = 0; i < model->numtriangles; i++) {
		node = (GLMnode*)malloc(sizeof(GLMnode));
		node->index = i;
		node->next  = members[T(i).vindices[0]];
		members[T(i).vindices[0]] = node;

		node = (GLMnode*)malloc(sizeof(GLMnode));
		node->index = i;
		node->next  = members[T(i).vindices[1]];
		members[T(i).vindices[1]] = node;

		node = (GLMnode*)malloc(sizeof(GLMnode));
		node->index = i;
		node->next  = members[T(i).vindices[2]];
		members[T(i).vindices[2]] = node;
	}

	/* calculate the average normal for each vertex */
	numnormals = 1;
	for (i = 1; i <= model->numvertices; i++) {
		/* calculate an average normal for this vertex by averaging the
		facet normal of every triangle this vertex is in */
		node = members[i];
		if (!node)
			fprintf(stderr, "glmVertexNormals(): vertex w/o a triangle\n");
		average[0] = 0.0; average[1] = 0.0; average[2] = 0.0;
		avg = 0;
		while (node) {
			/* only average if the dot product of the angle between the two
			facet normals is greater than the cosine of the threshold
			angle -- or, said another way, the angle between the two
			facet normals is less than (or equal to) the threshold angle */
			dot = glmDot(&model->facetnorms[3 * T(node->index).findex],
				&model->facetnorms[3 * T(members[i]->index).findex]);
			if (dot > cos_angle) {
				node->averaged = GL_TRUE;
				average[0] += model->facetnorms[3 * T(node->index).findex + 0];
				average[1] += model->facetnorms[3 * T(node->index).findex + 1];
				average[2] += model->facetnorms[3 * T(node->index).findex + 2];
				avg = 1;            /* we averaged at least one normal! */
			} else {
				node->averaged = GL_FALSE;
			}
			node = node->next;
		}

		if (avg) {
			/* normalize the averaged normal */
			glmNormalize(average);

			/* add the normal to the vertex normals list */
			model->normals[3 * numnormals + 0] = average[0];
			model->normals[3 * numnormals + 1] = average[1];
			model->normals[3 * numnormals + 2] = average[2];
			avg = numnormals;
			numnormals++;
		}

		/* set the normal of this vertex in each triangle it is in */
		node = members[i];
		while (node) {
			if (node->averaged) {
				/* if this node was averaged, use the average normal */
				if (T(node->index).vindices[0] == i)
					T(node->index).nindices[0] = avg;
				else if (T(node->index).vindices[1] == i)
					T(node->index).nindices[1] = avg;
				else if (T(node->index).vindices[2] == i)
					T(node->index).nindices[2] = avg;
			} else {
				/* if this node wasn't averaged, use the facet normal */
				model->normals[3 * numnormals + 0] = 
					model->facetnorms[3 * T(node->index).findex + 0];
				model->normals[3 * numnormals + 1] = 
					model->facetnorms[3 * T(node->index).findex + 1];
				model->normals[3 * numnormals + 2] = 
					model->facetnorms[3 * T(node->index).findex + 2];
				if (T(node->index).vindices[0] == i)
					T(node->index).nindices[0] = numnormals;
				else if (T(node->index).vindices[1] == i)
					T(node->index).nindices[1] = numnormals;
				else if (T(node->index).vindices[2] == i)
					T(node->index).nindices[2] = numnormals;
				numnormals++;
			}
			node = node->next;
		}
	}

	model->numnormals = numnormals - 1;

	/* free the member information */
	for (i = 1; i <= model->numvertices; i++) {
		node = members[i];
		while (node) {
			tail = node;
			node = node->next;
			free(tail);
		}
	}
	free(members);

	/* pack the normals array (we previously allocated the maximum
	number of normals that could possibly be created (numtriangles *
	3), so get rid of some of them (usually alot unless none of the
	facet normals were averaged)) */
	normals = model->normals;
	model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
	for (i = 1; i <= model->numnormals; i++) {
		model->normals[3 * i + 0] = normals[3 * i + 0];
		model->normals[3 * i + 1] = normals[3 * i + 1];
		model->normals[3 * i + 2] = normals[3 * i + 2];
	}
	free(normals);
}


/* glmDelete: Deletes a GLMmodel structure.
*
* model - initialized GLMmodel structure
*/
GLvoid OBJHandle::glmDelete(GLMmodel* model)
{
	GLuint i;

	assert(model);

	if (model->pathname)     free(model->pathname);
	if (model->mtllibname) free(model->mtllibname);
	if (model->vertices)     free(model->vertices);
	if (model->normals)  free(model->normals);
	if (model->texcoords)  free(model->texcoords);
	if (model->facetnorms) free(model->facetnorms);
	if (model->triangles)  free(model->triangles);
	if (model->materials) {
		for (i = 0; i < model->nummaterials; i++)
			free(model->materials[i].name);
	}
	free(model->materials);
	free(model);
}

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
* Returns a pointer to the created object which should be free'd with
* glmDelete().
*
* filename - name of the file containing the Wavefront .OBJ format data.  
*/
GLvoid OBJHandle::glmReadOBJ(char* filename,KW_Mesh& mesh,bool bScale,bool bCenter)
{
	GLMmodel* model;
	FILE*   file;

	/* open the file */
	file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "glmReadOBJ() failed: can't open data file \"%s\".\n",
			filename);
		exit(1);
	}

	/* allocate a new model */
	model = (GLMmodel*)malloc(sizeof(GLMmodel));
	model->pathname    = strdup(filename);
	model->mtllibname    = NULL;
	model->numvertices   = 0;
	model->vertices    = NULL;
	model->numnormals    = 0;
	model->normals     = NULL;
	model->numtexcoords  = 0;
	model->texcoords       = NULL;
	model->numfacetnorms = 0;
	model->facetnorms    = NULL;
	model->numtriangles  = 0;
	model->triangles       = NULL;
	model->nummaterials  = 0;
	model->materials       = NULL;
	model->position[0]   = 0.0;
	model->position[1]   = 0.0;
	model->position[2]   = 0.0;

	/* make a first pass through the file to get a count of the number
	of vertices, normals, texcoords & triangles */
	if (!glmFirstPass(model, file))
	{
		model=NULL;
		return;
//		return model;
	}

	/* allocate memory */
	model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
		3 * (model->numvertices + 1));
	model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
		model->numtriangles);
	if (model->numnormals) {
		model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
			3 * (model->numnormals + 1));
	}
	if (model->numtexcoords) {
		model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
			2 * (model->numtexcoords + 1));
	}

	/* rewind to beginning of file and read in the data this pass */
	rewind(file);

	glmSecondPass(model, file);

	/* close the file */
	fclose(file);

	ConvertToCGALPolyhedron(model,mesh);

	UnitizeCGALPolyhedron(mesh,bScale,bCenter);
	
	mesh.SetRenderInfo(true,true,true,true,true);

	delete model;
	model=NULL;
//	return model;
}

GLvoid OBJHandle::glmReadOBJNew(char* filename,KW_Mesh& mesh,bool bScale,bool bCenter,bool bSetRenderInfo/* =true */)
{
	cout<<"Reading file...\n";
	KW_Polyhedron model;
	FILE*   file;
	/* open the file */
	file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "glmReadOBJ() failed: can't open data file \"%s\".\n",
			filename);
		exit(1);
	}

	glmSecondPass(model, file);

	/* close the file */
	fclose(file);

	cout<<"Building mesh...\n";

	ConvertToCGALPolyhedronNew(model,mesh);

	UnitizeCGALPolyhedron(mesh,bScale,bCenter);
	//UnitizeCGALPolyhedron(mesh,false,false);
	//GeometryAlgorithm::SetUniformMeshColor(mesh,vecDefaultColor);

	if (bSetRenderInfo)
	{
		cout<<"Set render info...\n";
		mesh.SetRenderInfo(true,true,true,true,true);
	}

	cout<<"mesh loaded...\n";
}

/* glmWriteOBJ: Writes a model description in Wavefront .OBJ format to
* a file.
*
* model - initialized GLMmodel structure
* filename - name of the file to write the Wavefront .OBJ format data to
* mode  - a bitwise or of values describing what is written to the file
*             GLM_NONE     -  render with only vertices
*             GLM_FLAT     -  render with facet normals
*             GLM_SMOOTH   -  render with vertex normals
*             GLM_TEXTURE  -  render with texture coords
*             GLM_COLOR    -  render with colors (color material)
*             GLM_MATERIAL -  render with materials
*             GLM_COLOR and GLM_MATERIAL should not both be specified.  
*             GLM_FLAT and GLM_SMOOTH should not both be specified.  
*/
GLvoid OBJHandle::glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode)
{
	GLuint  i;
	FILE*   file;

	assert(model);

	/* do a bit of warning */
	if (mode & GLM_FLAT && !model->facetnorms) {
		printf("glmWriteOBJ() warning: flat normal output requested "
			"with no facet normals defined.\n");
		mode &= ~GLM_FLAT;
	}
	if (mode & GLM_SMOOTH && !model->normals) {
		printf("glmWriteOBJ() warning: smooth normal output requested "
			"with no normals defined.\n");
		mode &= ~GLM_SMOOTH;
	}
	if (mode & GLM_TEXTURE && !model->texcoords) {
		printf("glmWriteOBJ() warning: texture coordinate output requested "
			"with no texture coordinates defined.\n");
		mode &= ~GLM_TEXTURE;
	}
	if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
		printf("glmWriteOBJ() warning: flat normal output requested "
			"and smooth normal output requested (using smooth).\n");
		mode &= ~GLM_FLAT;
	}
	if (mode & GLM_COLOR && !model->materials) {
		printf("glmWriteOBJ() warning: color output requested "
			"with no colors (materials) defined.\n");
		mode &= ~GLM_COLOR;
	}
	if (mode & GLM_MATERIAL && !model->materials) {
		printf("glmWriteOBJ() warning: material output requested "
			"with no materials defined.\n");
		mode &= ~GLM_MATERIAL;
	}
	if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
		printf("glmWriteOBJ() warning: color and material output requested "
			"outputting only materials.\n");
		mode &= ~GLM_COLOR;
	}


	/* open the file */
	file = fopen(filename, "w");
	if (!file) {
		fprintf(stderr, "glmWriteOBJ() failed: can't open file \"%s\" to write.\n",
			filename);
		exit(1);
	}

	/* spit out a header */
	fprintf(file, "#  \n");
	fprintf(file, "#  Wavefront OBJ generated by KW\n");
	fprintf(file, "#  \n");

	if (mode & GLM_MATERIAL && model->mtllibname) {
		fprintf(file, "\nmtllib %s\n\n", model->mtllibname);
		glmWriteMTL(model, filename, model->mtllibname);
	}

	/* spit out the vertices */
	fprintf(file, "\n");
	fprintf(file, "# %d vertices\n", model->numvertices);
	for (i = 1; i <= model->numvertices; i++) {
		fprintf(file, "v %f %f %f\n", 
			model->vertices[3 * i + 0],
			model->vertices[3 * i + 1],
			model->vertices[3 * i + 2]);
	}

	/* spit out the smooth/flat normals */
	if (mode & GLM_SMOOTH) {
		fprintf(file, "\n");
		fprintf(file, "# %d normals\n", model->numnormals);
		for (i = 1; i <= model->numnormals; i++) {
			fprintf(file, "vn %f %f %f\n", 
				model->normals[3 * i + 0],
				model->normals[3 * i + 1],
				model->normals[3 * i + 2]);
		}
	} else if (mode & GLM_FLAT) {
		fprintf(file, "\n");
		fprintf(file, "# %d normals\n", model->numfacetnorms);
		for (i = 1; i <= model->numnormals; i++) {
			fprintf(file, "vn %f %f %f\n", 
				model->facetnorms[3 * i + 0],
				model->facetnorms[3 * i + 1],
				model->facetnorms[3 * i + 2]);
		}
	}

	/* spit out the texture coordinates */
	if (mode & GLM_TEXTURE) {
		fprintf(file, "\n");
//		fprintf(file, "# %d texcoords\n", model->texcoords);
		for (i = 1; i <= model->numtexcoords; i++) {
			fprintf(file, "vt %f %f\n", 
				model->texcoords[2 * i + 0],
				model->texcoords[2 * i + 1]);
		}
	}

	fprintf(file, "\n");
	fprintf(file, "# %d faces (triangles)\n", model->numtriangles);
	fprintf(file, "\n");

	for (i = 0; i < model->numtriangles; i++) 
	{
		if (mode & GLM_SMOOTH && mode & GLM_TEXTURE) {
			fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
				T(i).vindices[0], 
				T(i).nindices[0], 
				T(i).tindices[0],
				T(i).vindices[1],
				T(i).nindices[1],
				T(i).tindices[1],
				T(i).vindices[2],
				T(i).nindices[2],
				T(i).tindices[2]);
		} else if (mode & GLM_FLAT && mode & GLM_TEXTURE) {
			fprintf(file, "f %d/%d %d/%d %d/%d\n",
				T(i).vindices[0],
				T(i).findex,
				T(i).vindices[1],
				T(i).findex,
				T(i).vindices[2],
				T(i).findex);
		} else if (mode & GLM_TEXTURE) {
			fprintf(file, "f %d/%d %d/%d %d/%d\n",
				T(i).vindices[0],
				T(i).tindices[0],
				T(i).vindices[1],
				T(i).tindices[1],
				T(i).vindices[2],
				T(i).tindices[2]);
		} else if (mode & GLM_SMOOTH) {
			fprintf(file, "f %d//%d %d//%d %d//%d\n",
				T(i).vindices[0],
				T(i).nindices[0],
				T(i).vindices[1],
				T(i).nindices[1],
				T(i).vindices[2], 
				T(i).nindices[2]);
		} else if (mode & GLM_FLAT) {
			fprintf(file, "f %d//%d %d//%d %d//%d\n",
				T(i).vindices[0], 
				T(i).findex,
				T(i).vindices[1],
				T(i).findex,
				T(i).vindices[2],
				T(i).findex);
		} else {
			fprintf(file, "f %d %d %d\n",
				T(i).vindices[0],
				T(i).vindices[1],
				T(i).vindices[2]);
		}
	}
	fprintf(file, "\n");

	fclose(file);
}


/* glmDraw: Renders the model to the current OpenGL context using the
* mode specified.
*
* model - initialized GLMmodel structure
* mode  - a bitwise OR of values describing what is to be rendered.
*             GLM_NONE     -  render with only vertices
*             GLM_FLAT     -  render with facet normals
*             GLM_SMOOTH   -  render with vertex normals
*             GLM_TEXTURE  -  render with texture coords
*             GLM_COLOR    -  render with colors (color material)
*             GLM_MATERIAL -  render with materials
*             GLM_COLOR and GLM_MATERIAL should not both be specified.  
*             GLM_FLAT and GLM_SMOOTH should not both be specified.  
*/
GLvoid OBJHandle::glmDraw(GLMmodel* model, GLuint mode)
{
	static GLuint i;
	static GLMtriangle* triangle;
	static GLMmaterial* material;

	assert(model);
	assert(model->vertices);

	/* do a bit of warning */
	if (mode & GLM_FLAT && !model->facetnorms) {
		printf("glmDraw() warning: flat render mode requested "
			"with no facet normals defined.\n");
		mode &= ~GLM_FLAT;
	}
	if (mode & GLM_SMOOTH && !model->normals) {
		printf("glmDraw() warning: smooth render mode requested "
			"with no normals defined.\n");
		mode &= ~GLM_SMOOTH;
	}
	if (mode & GLM_TEXTURE && !model->texcoords) {
		printf("glmDraw() warning: texture render mode requested "
			"with no texture coordinates defined.\n");
		mode &= ~GLM_TEXTURE;
	}
	if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
		printf("glmDraw() warning: flat render mode requested "
			"and smooth render mode requested (using smooth).\n");
		mode &= ~GLM_FLAT;
	}
	if (mode & GLM_COLOR && !model->materials) {
		printf("glmDraw() warning: color render mode requested "
			"with no materials defined.\n");
		mode &= ~GLM_COLOR;
	}
	if (mode & GLM_MATERIAL && !model->materials) {
		printf("glmDraw() warning: material render mode requested "
			"with no materials defined.\n");
		mode &= ~GLM_MATERIAL;
	}
	if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
		printf("glmDraw() warning: color and material render mode requested "
			"using only material mode.\n");
		mode &= ~GLM_COLOR;
	}
	if (mode & GLM_COLOR)
		glEnable(GL_COLOR_MATERIAL);
	else if (mode & GLM_MATERIAL)
		glDisable(GL_COLOR_MATERIAL);

	/* perhaps this loop should be unrolled into material, color, flat,
	smooth, etc. loops?  since most cpu's have good branch prediction
	schemes (and these branches will always go one way), probably
	wouldn't gain too much?  */

		if (mode & GLM_COLOR) {
			glColor3fv(material->diffuse);
		}

	for (i = 0; i < model->numtriangles; i++) 
	{
		glBegin(GL_TRIANGLES);
			triangle = &T(i);

			if (mode & GLM_FLAT)
				glNormal3fv(&model->facetnorms[3 * triangle->findex]);

			if (mode & GLM_SMOOTH)
				glNormal3fv(&model->normals[3 * triangle->nindices[0]]);
			if (mode & GLM_TEXTURE)
				glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
			glVertex3fv(&model->vertices[3 * triangle->vindices[0]]);

			if (mode & GLM_SMOOTH)
				glNormal3fv(&model->normals[3 * triangle->nindices[1]]);
			if (mode & GLM_TEXTURE)
				glTexCoord2fv(&model->texcoords[2 * triangle->tindices[1]]);
			glVertex3fv(&model->vertices[3 * triangle->vindices[1]]);

			if (mode & GLM_SMOOTH)
				glNormal3fv(&model->normals[3 * triangle->nindices[2]]);
			if (mode & GLM_TEXTURE)
				glTexCoord2fv(&model->texcoords[2 * triangle->tindices[2]]);
			glVertex3fv(&model->vertices[3 * triangle->vindices[2]]);

		glEnd();
	}
}

/* glmList: Generates and returns a display list for the model using
* the mode specified.
*
* model - initialized GLMmodel structure
* mode  - a bitwise OR of values describing what is to be rendered.
*             GLM_NONE     -  render with only vertices
*             GLM_FLAT     -  render with facet normals
*             GLM_SMOOTH   -  render with vertex normals
*             GLM_TEXTURE  -  render with texture coords
*             GLM_COLOR    -  render with colors (color material)
*             GLM_MATERIAL -  render with materials
*             GLM_COLOR and GLM_MATERIAL should not both be specified.  
* GLM_FLAT and GLM_SMOOTH should not both be specified.  
*/
GLuint OBJHandle::glmList(GLMmodel* model, GLuint mode)
{
	GLuint list;

	list = glGenLists(1);
	glNewList(list, GL_COMPILE);
	glmDraw(model, mode);
	glEndList();

	return list;
}

/* glmWeld: eliminate (weld) vectors that are within an epsilon of
* each other.
*
* model   - initialized GLMmodel structure
* epsilon     - maximum difference between vertices
*               ( 0.00001 is a good start for a unitized model)
*
*/
GLvoid OBJHandle::glmWeld(GLMmodel* model, GLfloat epsilon)
{
	GLfloat* vectors;
	GLfloat* copies;
	GLuint   numvectors;
	GLuint   i;

	/* vertices */
	numvectors = model->numvertices;
	vectors  = model->vertices;
	copies = glmWeldVectors(vectors, &numvectors, epsilon);

#if 0
	printf("glmWeld(): %d redundant vertices.\n", 
		model->numvertices - numvectors - 1);
#endif

	for (i = 0; i < model->numtriangles; i++) {
		T(i).vindices[0] = (GLuint)vectors[3 * T(i).vindices[0] + 0];
		T(i).vindices[1] = (GLuint)vectors[3 * T(i).vindices[1] + 0];
		T(i).vindices[2] = (GLuint)vectors[3 * T(i).vindices[2] + 0];
	}

	/* free space for old vertices */
	free(vectors);

	/* allocate space for the new vertices */
	model->numvertices = numvectors;
	model->vertices = (GLfloat*)malloc(sizeof(GLfloat) * 
		3 * (model->numvertices + 1));

	/* copy the optimized vertices into the actual vertex list */
	for (i = 1; i <= model->numvertices; i++) {
		model->vertices[3 * i + 0] = copies[3 * i + 0];
		model->vertices[3 * i + 1] = copies[3 * i + 1];
		model->vertices[3 * i + 2] = copies[3 * i + 2];
	}

	free(copies);
}


/* glmReadPPM: read a PPM raw (type P6) file.  The PPM file has a header
* that should look something like:
*
*    P6
*    # comment
*    width height max_value
*    rgbrgbrgb...
*
* where "P6" is the magic cookie which identifies the file type and
* should be the only characters on the first line followed by a
* carriage return.  Any line starting with a # mark will be treated
* as a comment and discarded.   After the magic cookie, three integer
* values are expected: width, height of the image and the maximum
* value for a pixel (max_value must be < 256 for PPM raw files).  The
* data section consists of width*height rgb triplets (one byte each)
* in binary format (i.e., such as that written with fwrite() or
* equivalent).
*
* The rgb data is returned as an array of unsigned chars (packed
* rgb).  The malloc()'d memory should be free()'d by the caller.  If
* an error occurs, an error message is sent to stderr and NULL is
* returned.
*
* filename   - name of the .ppm file.
* width      - will contain the width of the image on return.
* height     - will contain the height of the image on return.
*
*/
GLubyte*  OBJHandle::glmReadPPM(const char* filename, int* width, int* height)
{
	FILE* fp;
	int i, w, h, d;
	unsigned char* image;
	char head[70];          /* max line <= 70 in PPM (per spec). */

	fp = fopen(filename, "rb");
	if (!fp) {
		perror(filename);
		return NULL;
	}

	/* grab first two chars of the file and make sure that it has the
	correct magic cookie for a raw PPM file. */
	fgets(head, 70, fp);
	if (strncmp(head, "P6", 2)) {
		fprintf(stderr, "%s: Not a raw PPM file\n", filename);
		return NULL;
	}

	/* grab the three elements in the header (width, height, maxval). */
	i = 0;
	while(i < 3) {
		fgets(head, 70, fp);
		if (head[0] == '#')     /* skip comments. */
			continue;
		if (i == 0)
			i += sscanf(head, "%d %d %d", &w, &h, &d);
		else if (i == 1)
			i += sscanf(head, "%d %d", &h, &d);
		else if (i == 2)
			i += sscanf(head, "%d", &d);
	}

	/* grab all the image data in one fell swoop. */
	image = (unsigned char*)malloc(sizeof(unsigned char)*w*h*3);
	fread(image, sizeof(unsigned char), w*h*3, fp);
	fclose(fp);

	*width = w;
	*height = h;
	return image;
}

#if 0
/* normals */
if (model->numnormals) {
	numvectors = model->numnormals;
	vectors  = model->normals;
	copies = glmOptimizeVectors(vectors, &numvectors);

	printf("glmOptimize(): %d redundant normals.\n", 
		model->numnormals - numvectors);

	for (i = 0; i < model->numtriangles; i++) {
		T(i).nindices[0] = (GLuint)vectors[3 * T(i).nindices[0] + 0];
		T(i).nindices[1] = (GLuint)vectors[3 * T(i).nindices[1] + 0];
		T(i).nindices[2] = (GLuint)vectors[3 * T(i).nindices[2] + 0];
	}

	/* free space for old normals */
	free(vectors);

	/* allocate space for the new normals */
	model->numnormals = numvectors;
	model->normals = (GLfloat*)malloc(sizeof(GLfloat) * 
		3 * (model->numnormals + 1));

	/* copy the optimized vertices into the actual vertex list */
	for (i = 1; i <= model->numnormals; i++) {
		model->normals[3 * i + 0] = copies[3 * i + 0];
		model->normals[3 * i + 1] = copies[3 * i + 1];
		model->normals[3 * i + 2] = copies[3 * i + 2];
	}

	free(copies);
}

/* texcoords */
if (model->numtexcoords) {
	numvectors = model->numtexcoords;
	vectors  = model->texcoords;
	copies = glmOptimizeVectors(vectors, &numvectors);

	printf("glmOptimize(): %d redundant texcoords.\n", 
		model->numtexcoords - numvectors);

	for (i = 0; i < model->numtriangles; i++) {
		for (j = 0; j < 3; j++) {
			T(i).tindices[j] = (GLuint)vectors[3 * T(i).tindices[j] + 0];
		}
	}

	/* free space for old texcoords */
	free(vectors);

	/* allocate space for the new texcoords */
	model->numtexcoords = numvectors;
	model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) * 
		2 * (model->numtexcoords + 1));

	/* copy the optimized vertices into the actual vertex list */
	for (i = 1; i <= model->numtexcoords; i++) {
		model->texcoords[2 * i + 0] = copies[2 * i + 0];
		model->texcoords[2 * i + 1] = copies[2 * i + 1];
	}

	free(copies);
}
#endif

#if 0
/* look for unused vertices */
/* look for unused normals */
/* look for unused texcoords */
for (i = 1; i <= model->numvertices; i++) {
	for (j = 0; j < model->numtriangles; i++) {
		if (T(j).vindices[0] == i || 
			T(j).vindices[1] == i || 
			T(j).vindices[1] == i)
			break;
	}
}
#endif


















/*private functions*/
/* glmMax: returns the maximum of two floats */
GLfloat	OBJHandle::glmMax(GLfloat a, GLfloat b)
{
	if (b > a)
		return b;
	return a;
}

/* glmAbs: returns the absolute value of a float */
GLfloat OBJHandle::glmAbs(GLfloat f)
{
	if (f < 0)
		return -f;
	return f;
}
/* glmDot: compute the dot product of two vectors
*
* u - array of 3 GLfloats (GLfloat u[3])
* v - array of 3 GLfloats (GLfloat v[3])
*/
GLfloat OBJHandle::glmDot(GLfloat* u, GLfloat* v)
{
	assert(u); assert(v);

	return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

/* glmCross: compute the cross product of two vectors
*
* u - array of 3 GLfloats (GLfloat u[3])
* v - array of 3 GLfloats (GLfloat v[3])
* n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
*/
GLvoid OBJHandle::glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
	assert(u); assert(v); assert(n);

	n[0] = u[1]*v[2] - u[2]*v[1];
	n[1] = u[2]*v[0] - u[0]*v[2];
	n[2] = u[0]*v[1] - u[1]*v[0];
}

/* glmNormalize: normalize a vector
*
* v - array of 3 GLfloats (GLfloat v[3]) to be normalized
*/
GLvoid OBJHandle::glmNormalize(GLfloat* v)
{
	GLfloat l;

	assert(v);

	l = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= l;
	v[1] /= l;
	v[2] /= l;
}

/* glmEqual: compares two vectors and returns GL_TRUE if they are
* equal (within a certain threshold) or GL_FALSE if not. An epsilon
* that works fairly well is 0.000001.
*
* u - array of 3 GLfloats (GLfloat u[3])
* v - array of 3 GLfloats (GLfloat v[3]) 
*/
GLboolean OBJHandle::glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon)
{
	if (glmAbs(u[0] - v[0]) < epsilon &&
		glmAbs(u[1] - v[1]) < epsilon &&
		glmAbs(u[2] - v[2]) < epsilon) 
	{
		return GL_TRUE;
	}
	return GL_FALSE;
}

/* glmWeldVectors: eliminate (weld) vectors that are within an
* epsilon of each other.
*
* vectors     - array of GLfloat[3]'s to be welded
* numvectors - number of GLfloat[3]'s in vectors
* epsilon     - maximum difference between vectors 
*
*/
GLfloat* OBJHandle::glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon)
{
	GLfloat* copies;
	GLuint   copied;
	GLuint   i, j;

	copies = (GLfloat*)malloc(sizeof(GLfloat) * 3 * (*numvectors + 1));
	memcpy(copies, vectors, (sizeof(GLfloat) * 3 * (*numvectors + 1)));

	copied = 1;
	for (i = 1; i <= *numvectors; i++) {
		for (j = 1; j <= copied; j++) {
			if (glmEqual(&vectors[3 * i], &copies[3 * j], epsilon)) {
				goto duplicate;
			}
		}

		/* must not be any duplicates -- add to the copies array */
		copies[3 * copied + 0] = vectors[3 * i + 0];
		copies[3 * copied + 1] = vectors[3 * i + 1];
		copies[3 * copied + 2] = vectors[3 * i + 2];
		j = copied;             /* pass this along for below */
		copied++;

duplicate:
		/* set the first component of this vector to point at the correct
		index into the new copies array */
		vectors[3 * i + 0] = (GLfloat)j;
	}

	*numvectors = copied-1;
	return copies;
}



/* glmFindGroup: Find a material in the model */
GLuint OBJHandle::glmFindMaterial(GLMmodel* model, char* name)
{
	GLuint i;

	/* XXX doing a linear search on a string key'd list is pretty lame,
	but it works and is fast enough for now. */
	for (i = 0; i < model->nummaterials; i++) {
		if (!strcmp(model->materials[i].name, name))
			goto found;
	}

	/* didn't find the name, so print a warning and return the default
	material (0). */
	printf("glmFindMaterial():  can't find material \"%s\".\n", name);
	i = 0;

found:
	return i;
}

/* glmDirName: return the directory given a path
*
* path - filesystem path
*
* NOTE: the return value should be free'd.
*/
char* OBJHandle::glmDirName(char* path)
{
	char* dir;
	char* s;

	dir = strdup(path);

	s = strrchr(dir, '/');

	/* added by neo6 for handling Windows paths */
#ifdef WIN32 
	if (!s) s = strrchr(dir,'\\');
#endif

	if (s)
		s[1] = '\0';
	else
		dir[0] = '\0';

	return dir;
}


/* glmReadMTL: read a wavefront material library file
*
* model - properly initialized GLMmodel structure
* name  - name of the material library
*/
GLboolean OBJHandle::glmReadMTL(GLMmodel* model, char* name)
{
	FILE* file;
	char* dir;
	char* filename;
	char    buf[128];
	GLuint nummaterials, i;

	dir = glmDirName(model->pathname);
	filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(name) + 1));
	strcpy(filename, dir);
	strcat(filename, name);
	free(dir);

	file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "glmReadMTL() failed: can't open material file \"%s\".\n",
			filename);
//		exit(1);
		QMessageBox msgBox;
		msgBox.setText("can't open material file");
		msgBox.exec();
		return false;
	}
	free(filename);

	/* count the number of materials in the file */
	nummaterials = 1;
	while(fscanf(file, "%s", buf) != EOF) {
		switch(buf[0]) {
		case '#':               /* comment */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'n':               /* newmtl */
			fgets(buf, sizeof(buf), file);
			nummaterials++;
			sscanf(buf, "%s %s", buf, buf);
			break;
		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		}
	}

	rewind(file);

	model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * nummaterials);
	model->nummaterials = nummaterials;

	/* set the default material */
	for (i = 0; i < nummaterials; i++) {
		model->materials[i].name = NULL;
		model->materials[i].shininess = 65.0;
		model->materials[i].diffuse[0] = 0.8;
		model->materials[i].diffuse[1] = 0.8;
		model->materials[i].diffuse[2] = 0.8;
		model->materials[i].diffuse[3] = 1.0;
		model->materials[i].ambient[0] = 0.2;
		model->materials[i].ambient[1] = 0.2;
		model->materials[i].ambient[2] = 0.2;
		model->materials[i].ambient[3] = 1.0;
		model->materials[i].specular[0] = 0.0;
		model->materials[i].specular[1] = 0.0;
		model->materials[i].specular[2] = 0.0;
		model->materials[i].specular[3] = 1.0;
	}
	model->materials[0].name = strdup("default");

	/* now, read in the data */
	nummaterials = 0;
	while(fscanf(file, "%s", buf) != EOF) {
		switch(buf[0]) {
		case '#':               /* comment */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'n':               /* newmtl */
			fgets(buf, sizeof(buf), file);
			sscanf(buf, "%s %s", buf, buf);
			nummaterials++;
			model->materials[nummaterials].name = strdup(buf);
			break;
		case 'N':
			fscanf(file, "%f", &model->materials[nummaterials].shininess);
			/* wavefront shininess is from [0, 1000], so scale for OpenGL */
			model->materials[nummaterials].shininess /= 1000.0;
			model->materials[nummaterials].shininess *= 128.0;
			break;
		case 'K':
			switch(buf[1]) {
		case 'd':
			fscanf(file, "%f %f %f",
				&model->materials[nummaterials].diffuse[0],
				&model->materials[nummaterials].diffuse[1],
				&model->materials[nummaterials].diffuse[2]);
			break;
		case 's':
			fscanf(file, "%f %f %f",
				&model->materials[nummaterials].specular[0],
				&model->materials[nummaterials].specular[1],
				&model->materials[nummaterials].specular[2]);
			break;
		case 'a':
			fscanf(file, "%f %f %f",
				&model->materials[nummaterials].ambient[0],
				&model->materials[nummaterials].ambient[1],
				&model->materials[nummaterials].ambient[2]);
			break;
		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
			}
			break;
		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		}
	}

	return true;
}

/* glmWriteMTL: write a wavefront material library file
*
* model   - properly initialized GLMmodel structure
* modelpath  - pathname of the model being written
* mtllibname - name of the material library to be written
*/
GLvoid OBJHandle::glmWriteMTL(GLMmodel* model, char* modelpath, char* mtllibname)
{
	FILE* file;
	char* dir;
	char* filename;
	GLMmaterial* material;
	GLuint i;

	dir = glmDirName(modelpath);
	filename = (char*)malloc(sizeof(char) * (strlen(dir)+strlen(mtllibname)));
	strcpy(filename, dir);
	strcat(filename, mtllibname);
	free(dir);

	/* open the file */
	file = fopen(filename, "w");
	if (!file) {
		fprintf(stderr, "glmWriteMTL() failed: can't open file \"%s\".\n",
			filename);
		exit(1);
	}
	free(filename);

	/* spit out a header */
	fprintf(file, "#  \n");
	fprintf(file, "#  Wavefront MTL generated by GLM library\n");
	fprintf(file, "#  \n");
	fprintf(file, "#  GLM library\n");
	fprintf(file, "#  Nate Robins\n");
	fprintf(file, "#  ndr@pobox.com\n");
	fprintf(file, "#  http://www.pobox.com/~ndr\n");
	fprintf(file, "#  \n\n");

	for (i = 0; i < model->nummaterials; i++) {
		material = &model->materials[i];
		fprintf(file, "newmtl %s\n", material->name);
		fprintf(file, "Ka %f %f %f\n", 
			material->ambient[0], material->ambient[1], material->ambient[2]);
		fprintf(file, "Kd %f %f %f\n", 
			material->diffuse[0], material->diffuse[1], material->diffuse[2]);
		fprintf(file, "Ks %f %f %f\n", 
			material->specular[0],material->specular[1],material->specular[2]);
		fprintf(file, "Ns %f\n", material->shininess / 128.0 * 1000.0);
		fprintf(file, "\n");
	}
}


/* glmFirstPass: first pass at a Wavefront OBJ file that gets all the
* statistics of the model (such as #vertices, #normals, etc)
*
* model - properly initialized GLMmodel structure
* file  - (fopen'd) file descriptor 
*/
GLboolean OBJHandle::glmFirstPass(GLMmodel* model, FILE* file) 
{
	GLuint  numvertices;        /* number of vertices in model */
	GLuint  numnormals;         /* number of normals in model */
	GLuint  numtexcoords;       /* number of texcoords in model */
	GLuint  numtriangles;       /* number of triangles in model */
	unsigned    v, n, t;
	char        buf[128];

	numvertices = numnormals = numtexcoords = numtriangles = 0;
	while(fscanf(file, "%s", buf) != EOF) {
		switch(buf[0]) {
		case '#':               /* comment */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'v':               /* v, vn, vt */
			switch(buf[1]) {
			case '\0':          /* vertex */
				/* eat up rest of line */
				fgets(buf, sizeof(buf), file);
				numvertices++;
				break;
			case 'n':           /* normal */
				/* eat up rest of line */
				fgets(buf, sizeof(buf), file);
				numnormals++;
				break;
			case 't':           /* texcoord */
				/* eat up rest of line */
				fgets(buf, sizeof(buf), file);
				numtexcoords++;
				break;
			default:
				printf("glmFirstPass(): Unknown token \"%s\".\n", buf);
				//exit(1);
				return false;
				break;
			}
			break;
		case 'm':
			fgets(buf, sizeof(buf), file);
			sscanf(buf, "%s %s", buf, buf);
			model->mtllibname = strdup(buf);
			if (!glmReadMTL(model, buf))
			{
				return false;
			} 
			break;
		case 'u':
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'g':               /* group */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'f':               /* face */
			v = n = t = 0;
			fscanf(file, "%s", buf);
			/* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
			if (strstr(buf, "//")) {
				/* v//n */
				sscanf(buf, "%d//%d", &v, &n);
				fscanf(file, "%d//%d", &v, &n);
				fscanf(file, "%d//%d", &v, &n);
				numtriangles++;
				while(fscanf(file, "%d//%d", &v, &n) > 0) {
					numtriangles++;
				}
			} else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
				/* v/t/n */
				fscanf(file, "%d/%d/%d", &v, &t, &n);
				fscanf(file, "%d/%d/%d", &v, &t, &n);
				numtriangles++;
				while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
					numtriangles++;
				}
			} else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
				/* v/t */
				fscanf(file, "%d/%d", &v, &t);
				fscanf(file, "%d/%d", &v, &t);
				numtriangles++;
				while(fscanf(file, "%d/%d", &v, &t) > 0) {
					numtriangles++;
				}
			} else {
				/* v */
				fscanf(file, "%d", &v);
				fscanf(file, "%d", &v);
				numtriangles++;
				while(fscanf(file, "%d", &v) > 0) {
					numtriangles++;
				}
			}
			break;

		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		}
	}

	/* set the stats in the model structure */
	model->numvertices  = numvertices;
	model->numnormals   = numnormals;
	model->numtexcoords = numtexcoords;
	model->numtriangles = numtriangles;

	return true;
}

/* glmSecondPass: second pass at a Wavefront OBJ file that gets all
* the data.
*
* model - properly initialized GLMmodel structure
* file  - (fopen'd) file descriptor 
*/
GLvoid OBJHandle::glmSecondPass(GLMmodel* model, FILE* file) 
{
	GLuint  numvertices;        /* number of vertices in model */
	GLuint  numnormals;         /* number of normals in model */
	GLuint  numtexcoords;       /* number of texcoords in model */
	GLuint  numtriangles;       /* number of triangles in model */
	GLfloat*    vertices;           /* array of vertices  */
	GLfloat*    normals;            /* array of normals */
	GLfloat*    texcoords;          /* array of texture coordinates */
	GLuint  material;           /* current material */
	GLuint  v, n, t;
	char        buf[128];

	/* set the pointer shortcuts */
	vertices       = model->vertices;
	normals    = model->normals;
	texcoords    = model->texcoords;

	/* on the second pass through the file, read all the data into the
	allocated arrays */
	numvertices = numnormals = numtexcoords = 1;
	numtriangles = 0;
	material = 0;
	while(fscanf(file, "%s", buf) != EOF) {
		switch(buf[0]) {
		case '#':               /* comment */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'v':               /* v, vn, vt */
			switch(buf[1]) {
			case '\0':          /* vertex */
				fscanf(file, "%f %f %f", 
					&vertices[3 * numvertices + 0], 
					&vertices[3 * numvertices + 1], 
					&vertices[3 * numvertices + 2]);
				numvertices++;
				break;
			case 'n':           /* normal */
				fscanf(file, "%f %f %f", 
					&normals[3 * numnormals + 0],
					&normals[3 * numnormals + 1], 
					&normals[3 * numnormals + 2]);
				numnormals++;
				break;
			case 't':           /* texcoord */
				fscanf(file, "%f %f", 
					&texcoords[2 * numtexcoords + 0],
					&texcoords[2 * numtexcoords + 1]);
				numtexcoords++;
				break;
				}
			break;
		case 'u':
			fgets(buf, sizeof(buf), file);
			break;
		case 'g':               /* group */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'f':               /* face */
			v = n = t = 0;
			fscanf(file, "%s", buf);
			/* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
			if (strstr(buf, "//")) {
				/* v//n */
				sscanf(buf, "%d//%d", &v, &n);
				T(numtriangles).vindices[0] = v;
				T(numtriangles).nindices[0] = n;
				fscanf(file, "%d//%d", &v, &n);
				T(numtriangles).vindices[1] = v;
				T(numtriangles).nindices[1] = n;
				fscanf(file, "%d//%d", &v, &n);
				T(numtriangles).vindices[2] = v;
				T(numtriangles).nindices[2] = n;
				numtriangles++;
				while(fscanf(file, "%d//%d", &v, &n) > 0) {
					T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
					T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
					T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
					T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
					T(numtriangles).vindices[2] = v;
					T(numtriangles).nindices[2] = n;
					numtriangles++;
				}
			} else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
				/* v/t/n */
				T(numtriangles).vindices[0] = v;
				T(numtriangles).tindices[0] = t;
				T(numtriangles).nindices[0] = n;
				fscanf(file, "%d/%d/%d", &v, &t, &n);
				T(numtriangles).vindices[1] = v;
				T(numtriangles).tindices[1] = t;
				T(numtriangles).nindices[1] = n;
				fscanf(file, "%d/%d/%d", &v, &t, &n);
				T(numtriangles).vindices[2] = v;
				T(numtriangles).tindices[2] = t;
				T(numtriangles).nindices[2] = n;
				numtriangles++;
				while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
					T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
					T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
					T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
					T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
					T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
					T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
					T(numtriangles).vindices[2] = v;
					T(numtriangles).tindices[2] = t;
					T(numtriangles).nindices[2] = n;
					numtriangles++;
				}
			} else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
				/* v/t */
				T(numtriangles).vindices[0] = v;
				T(numtriangles).tindices[0] = t;
				fscanf(file, "%d/%d", &v, &t);
				T(numtriangles).vindices[1] = v;
				T(numtriangles).tindices[1] = t;
				fscanf(file, "%d/%d", &v, &t);
				T(numtriangles).vindices[2] = v;
				T(numtriangles).tindices[2] = t;
				numtriangles++;
				while(fscanf(file, "%d/%d", &v, &t) > 0) {
					T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
					T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
					T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
					T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
					T(numtriangles).vindices[2] = v;
					T(numtriangles).tindices[2] = t;
					numtriangles++;
				}
			} else {
				/* v */
				sscanf(buf, "%d", &v);
				T(numtriangles).vindices[0] = v;
				fscanf(file, "%d", &v);
				T(numtriangles).vindices[1] = v;
				fscanf(file, "%d", &v);
				T(numtriangles).vindices[2] = v;
				numtriangles++;
				while(fscanf(file, "%d", &v) > 0) {
					T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
					T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
					T(numtriangles).vindices[2] = v;
					numtriangles++;
				}
			}
			break;

		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		}
	}

}


//with new data structure
GLvoid OBJHandle::glmSecondPass(KW_Polyhedron& model, FILE* file)
{
	float fX,fY,fZ;
	int v,n,t;
	vector<int> VerIndex;
	char	buf[128];
	while(fscanf(file, "%s", buf) != EOF) 
	{
		switch(buf[0]) 
		{
		case '#':               /* comment */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		case 'v':               /* v, vn, vt */
			switch(buf[1]) 
			{
				case '\0':          /* vertex */
					fscanf(file, "%f %f %f", 
						&fX, 
						&fY, 
						&fZ);
					model.vecVertex.push_back(Point_3(fX,fY,fZ));
					break;
				case 'n':           /* normal */
					fscanf(file, "%f %f %f", 
						&fX, 
						&fY, 
						&fZ);
					break;
				case 't':           /* texcoord */
					fscanf(file, "%f %f %f", 
						&fX, 
						&fY, 
						&fZ);
					break;
			}
			break;
		case 'f':               /* face */
			VerIndex.clear();
			fscanf(file, "%s", buf);
			/* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
			if (strstr(buf, "//")) {
				/* v//n */
				sscanf(buf, "%d//%d", &v, &n);
				VerIndex.push_back(v);
				while(fscanf(file, "%d//%d", &v, &n) > 0) {
					VerIndex.push_back(v);
				}
			} else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
				/* v/t/n */
				VerIndex.push_back(v);
				while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
					VerIndex.push_back(v);
				}
			} else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
				/* v/t */
				VerIndex.push_back(v);
				while(fscanf(file, "%d/%d", &v, &t) > 0) {
					VerIndex.push_back(v);
				}
			} else {
				/* v */
				sscanf(buf, "%d", &v);
				VerIndex.push_back(v);
				while(fscanf(file, "%d", &v) > 0) {
					VerIndex.push_back(v);
				}
			}
			model.vecFacet.push_back(VerIndex);
			break;
		case 'u':
			fgets(buf, sizeof(buf), file);
			break;
		case 'g':               /* group */
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		default:
			/* eat up rest of line */
			fgets(buf, sizeof(buf), file);
			break;
		}
	}
}




GLvoid OBJHandle::ConvertGLMmodeltoCPPGLMmodel(GLMmodel * model,CPPGLMmodel & CPPmodel)
{
	CPPmodel.pathname=model->pathname;
	CPPmodel.mtllibname=model->mtllibname;

	CPPmodel.numvertices=model->numvertices;
	for (unsigned int i=1;i<=model->numvertices;i++)
	{
		float a=model->vertices[3*i+0];
		CPPmodel.vertices.push_back(a);//model->vertices[3*i+0]
		CPPmodel.vertices.push_back(model->vertices[3*i+1]);
		CPPmodel.vertices.push_back(model->vertices[3*i+2]);
	}

	CPPmodel.numnormals=model->numnormals;
	for (unsigned int i=1;i<=model->numnormals;i++)
	{
		CPPmodel.normals.push_back(model->normals[3*i+0]);
		CPPmodel.normals.push_back(model->normals[3*i+1]);
		CPPmodel.normals.push_back(model->normals[3*i+2]);
	}

	CPPmodel.numtexcoords=model->numtexcoords;
	for (unsigned int i=1;i<=model->numtexcoords;i++)
	{
		CPPmodel.texcoords.push_back(model->texcoords[2*i+0]);
		CPPmodel.texcoords.push_back(model->texcoords[2*i+1]);
	}

	CPPmodel.numfacetnorms=model->numfacetnorms;
	for (unsigned int i=1;i<=model->numfacetnorms;i++)
	{
		CPPmodel.facetnorms.push_back(model->facetnorms[3*i+0]);
		CPPmodel.facetnorms.push_back(model->facetnorms[3*i+1]);
		CPPmodel.facetnorms.push_back(model->facetnorms[3*i+2]);
	}

	CPPmodel.numtriangles=model->numtriangles;
	for (unsigned int i=0;i<model->numtriangles;i++)
	{
		CPPGLMtriangle temp;
		for (int j=0;j<3;j++)
		{
			temp.vindices[j]=model->triangles[i].vindices[j]-1;
			temp.nindices[j]=model->triangles[i].nindices[j]-1;
			temp.tindices[j]=model->triangles[i].tindices[j]-1;
		}
		temp.findex=model->triangles[i].findex-1;
		CPPmodel.triangles.push_back(temp);
	}

	CPPmodel.nummaterials=model->nummaterials;
	for (unsigned int i=0;i<model->nummaterials;i++)
	{
		CPPGLMmaterial temp;
		temp.name=model->materials[i].name;
		temp.shininess=model->materials[i].shininess;
		for (int j=0;j<4;j++)
		{
			temp.diffuse[j]=model->materials[i].diffuse[j];
			temp.ambient[j]=model->materials[i].ambient[j];
			temp.specular[j]=model->materials[i].specular[j];
			temp.emmissive[j]=model->materials[i].emmissive[j];
		}
		CPPmodel.materials.push_back(temp);
	}
	
	for (int i=0;i<3;i++)
	{
		CPPmodel.position[i]=model->position[i];
	}

}

//glmDelete must be called before this function
GLvoid OBJHandle::ConvertCPPGLMmodeltoGLMmodel(CPPGLMmodel CPPmodel,GLMmodel * model)
{
//	model = (GLMmodel*)malloc(sizeof(GLMmodel));
	
	model->pathname=CPPmodel.pathname;
	model->mtllibname=CPPmodel.mtllibname;

	model->numvertices=CPPmodel.numvertices;
	model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
		3 * (model->numvertices + 1));
	for (unsigned int i=1;i<=model->numvertices;i++)
	{
		model->vertices[3*i+0]=CPPmodel.vertices.at(3*(i-1)+0);
		model->vertices[3*i+1]=CPPmodel.vertices.at(3*(i-1)+1);
		model->vertices[3*i+2]=CPPmodel.vertices.at(3*(i-1)+2);
	}

	//leave the numnormals blank 'cos the glmVertexNormals will compute 
	//it and allocate space for normals automatically
	model->facetnorms=NULL;
	model->numfacetnorms=0;
	
	model->numtexcoords=CPPmodel.numtexcoords;
	model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
		2 * (model->numtexcoords + 1));
	for (unsigned int i=1;i<=model->numtexcoords;i++)
	{
		model->texcoords[2*i+0]=CPPmodel.texcoords.at(2*(i-1)+0);
		model->texcoords[2*i+1]=CPPmodel.texcoords.at(2*(i-1)+1);
	}

	//leave the numfacetnorms blank 'cos the glmFacetNormals will compute 
	//it and allocate space for facetnorms automatically
	model->normals=NULL;
	model->numnormals=0;

	model->numtriangles=CPPmodel.numtriangles;
	model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
		model->numtriangles);
	for (unsigned int i=0;i<model->numtriangles;i++)
	{
		for (int j=0;j<3;j++)
		{
			model->triangles[i].vindices[j]=CPPmodel.triangles.at(i).vindices[j]+1;
			model->triangles[i].nindices[j]=CPPmodel.triangles.at(i).nindices[j]+1;
			model->triangles[i].tindices[j]=CPPmodel.triangles.at(i).tindices[j]+1;
		}
		model->triangles[i].findex=CPPmodel.triangles.at(i).findex+1;
	}

	model->nummaterials = CPPmodel.nummaterials;
	model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * model->nummaterials);
	if (model->nummaterials==0)
	{
		model->materials=0x00;
	}
	else
	{
		for (unsigned int i=0;i<model->nummaterials;i++)
		{
			model->materials[i].name=CPPmodel.materials.at(i).name;
			model->materials[i].shininess=CPPmodel.materials.at(i).shininess;
			for (int j=0;j<4;j++)
			{
				model->materials[i].diffuse[j]=CPPmodel.materials.at(i).diffuse[j];
				model->materials[i].ambient[j]=CPPmodel.materials.at(i).ambient[j];
				model->materials[i].specular[j]=CPPmodel.materials.at(i).specular[j];
				model->materials[i].emmissive[j]=CPPmodel.materials.at(i).emmissive[j];
			}
		}

	}

	for (int i=0;i<3;i++)
	{
		model->position[i]=CPPmodel.position[i];
	}
}

GLvoid OBJHandle::GetCurrentModelVerticePos(CPPGLMmodel& cppmodel,GLdouble * ModelViewMatrix)
{
	for (unsigned int i=0;i<cppmodel.numvertices;i++)
	{
		double dOldCoordinate[4]={0.0,0.0,0.0,1.0};
		double dNewCoordinate[4]={0.0,0.0,0.0,1.0};

		dOldCoordinate[0]=cppmodel.vertices.at(3*i+0);
		dOldCoordinate[1]=cppmodel.vertices.at(3*i+1);
		dOldCoordinate[2]=cppmodel.vertices.at(3*i+2);

		dNewCoordinate[0]=dOldCoordinate[0]*ModelViewMatrix[0]+dOldCoordinate[1]*ModelViewMatrix[4]
						+dOldCoordinate[2]*ModelViewMatrix[8]+dOldCoordinate[3]*ModelViewMatrix[12];
		dNewCoordinate[1]=dOldCoordinate[0]*ModelViewMatrix[1]+dOldCoordinate[1]*ModelViewMatrix[5]
						+dOldCoordinate[2]*ModelViewMatrix[9]+dOldCoordinate[3]*ModelViewMatrix[13];
		dNewCoordinate[2]=dOldCoordinate[0]*ModelViewMatrix[2]+dOldCoordinate[1]*ModelViewMatrix[6]
						+dOldCoordinate[2]*ModelViewMatrix[10]+dOldCoordinate[3]*ModelViewMatrix[14];

		cppmodel.vertices.at(3*i+0)=dNewCoordinate[0];
		cppmodel.vertices.at(3*i+1)=dNewCoordinate[1];
		cppmodel.vertices.at(3*i+2)=dNewCoordinate[2];
	}
}
GLvoid OBJHandle::GetCurrentModelVerticePos(GLMmodel * model,GLdouble * ModelViewMatrix)
{
	for (unsigned int i=1;i<=model->numvertices;i++)
	{
		double dOldCoordinate[4]={0.0,0.0,0.0,1.0};
		double dNewCoordinate[4]={0.0,0.0,0.0,1.0};

		dOldCoordinate[0]=model->vertices[3*i+0];
		dOldCoordinate[1]=model->vertices[3*i+1];
		dOldCoordinate[2]=model->vertices[3*i+2];

		dNewCoordinate[0]=dOldCoordinate[0]*ModelViewMatrix[0]+dOldCoordinate[1]*ModelViewMatrix[4]
		+dOldCoordinate[2]*ModelViewMatrix[8]+dOldCoordinate[3]*ModelViewMatrix[12];
		dNewCoordinate[1]=dOldCoordinate[0]*ModelViewMatrix[1]+dOldCoordinate[1]*ModelViewMatrix[5]
		+dOldCoordinate[2]*ModelViewMatrix[9]+dOldCoordinate[3]*ModelViewMatrix[13];
		dNewCoordinate[2]=dOldCoordinate[0]*ModelViewMatrix[2]+dOldCoordinate[1]*ModelViewMatrix[6]
		+dOldCoordinate[2]*ModelViewMatrix[10]+dOldCoordinate[3]*ModelViewMatrix[14];

		model->vertices[3*i+0]=dNewCoordinate[0];
		model->vertices[3*i+1]=dNewCoordinate[1];
		model->vertices[3*i+2]=dNewCoordinate[2];
	}
}

//GLvoid OBJHandle::UpDateTopoInfo(const GLMmodel * model)
//{
//	modelEdgeInfo.clear();
//	modelVertexInfo.clear();
//	
//	GLMtriangle triangle=model->triangles[0];
//	for (int i=0;i<3;i++)
//	{
//		int iStart=triangle.vindices[(i+1)%3]<triangle.vindices[(i+2)%3]?triangle.vindices[(i+1)%3]:triangle.vindices[(i+2)%3];
//		int iEnd=triangle.vindices[(i+1)%3]>triangle.vindices[(i+2)%3]?triangle.vindices[(i+1)%3]:triangle.vindices[(i+2)%3];
//		vector<int> face;
//		face.push_back(0);
//		modelEdgeInfo.insert(Edge_Tri_Pair(Int_Int_Pair(iStart,iEnd),face));
//
//		map<int,vector<int>>::iterator FindVertex;
//		FindVertex=modelVertexInfo.find(iStart);
//		if(FindVertex==modelVertexInfo.end())
//		{
//			vector<int> NeighborVertexIndex;
//			NeighborVertexIndex.push_back(iEnd);
//			modelVertexInfo.insert(Int_Vector_Pair(iStart,NeighborVertexIndex));
//		}
//		else
//		{
//			vector<int>::iterator findvertex=find((FindVertex->second).begin(),
//				(FindVertex->second).end(),
//				iEnd);
//			if(findvertex==(FindVertex->second).end())
//				(FindVertex->second).push_back(iEnd);
//		}
//
//		FindVertex=modelVertexInfo.find(iEnd);
//		if(FindVertex==modelVertexInfo.end())
//		{
//			vector<int> NeighborVertexIndex;
//			NeighborVertexIndex.push_back(iStart);
//			modelVertexInfo.insert(Int_Vector_Pair(iEnd,NeighborVertexIndex));
//		}
//		else
//		{
//			vector<int>::iterator findvertex=find((FindVertex->second).begin(),
//				(FindVertex->second).end(),
//				iStart);
//			if(findvertex==(FindVertex->second).end())
//				(FindVertex->second).push_back(iStart);
//		}
//	}
//
//	for (unsigned int i=1;i<model->numtriangles;i++)
//	{
//		GLMtriangle triangle=model->triangles[i];
//		for (int j=0;j<3;j++)
//		{
//			int iStart=triangle.vindices[(j+1)%3]<triangle.vindices[(j+2)%3]?triangle.vindices[(j+1)%3]:triangle.vindices[(j+2)%3];
//			int iEnd=triangle.vindices[(j+1)%3]>triangle.vindices[(j+2)%3]?triangle.vindices[(j+1)%3]:triangle.vindices[(j+2)%3];
//
//			EdgeStruct::iterator FindEdge;
//			FindEdge=modelEdgeInfo.find(Int_Int_Pair(iStart,iEnd));
//			if(FindEdge==modelEdgeInfo.end())
//			{
//				vector<int> face;
//				face.push_back(i);
//				modelEdgeInfo.insert(Edge_Tri_Pair(Int_Int_Pair(iStart,iEnd),face));
//			}
//			else
//			{
//				vector<int>::iterator findface=find((FindEdge->second).begin(),
//													(FindEdge->second).end(),
//													i);
//				if(findface==(FindEdge->second).end())
//					(FindEdge->second).push_back(i);
//			}
//
//
//
//			map<int,vector<int>>::iterator FindVertex;
//			FindVertex=modelVertexInfo.find(iStart);
//			if(FindVertex==modelVertexInfo.end())
//			{
//				vector<int> NeighborVertexIndex;
//				NeighborVertexIndex.push_back(iEnd);
//				modelVertexInfo.insert(Int_Vector_Pair(iStart,NeighborVertexIndex));
//			}
//			else
//			{
//				vector<int>::iterator findvertex=find((FindVertex->second).begin(),
//														(FindVertex->second).end(),
//														iEnd);
//				if(findvertex==(FindVertex->second).end())
//					(FindVertex->second).push_back(iEnd);
//			}
//
//			FindVertex=modelVertexInfo.find(iEnd);
//			if(FindVertex==modelVertexInfo.end())
//			{
//				vector<int> NeighborVertexIndex;
//				NeighborVertexIndex.push_back(iStart);
//				modelVertexInfo.insert(Int_Vector_Pair(iEnd,NeighborVertexIndex));
//			}
//			else
//			{
//				vector<int>::iterator findvertex=find((FindVertex->second).begin(),
//					(FindVertex->second).end(),
//					iStart);
//				if(findvertex==(FindVertex->second).end())
//					(FindVertex->second).push_back(iStart);
//			}
//		}
//	}
//	assert(modelEdgeInfo.size()==(model->numtriangles)*3/2);
//	assert(modelVertexInfo.size()==model->numvertices);
//
//	CPPmodelEdgeInfo.clear();
//	CPPmodelVertexInfo.clear();
//	EdgeStruct::iterator CFindEdge=modelEdgeInfo.begin();
//	while(CFindEdge!=modelEdgeInfo.end())
//	{
//		vector<int> CPPFace;
//		for (unsigned int i=0;i<CFindEdge->second.size();i++)
//		{
//			CPPFace.push_back(CFindEdge->second.at(i));
//		}
//		CPPmodelEdgeInfo.insert(Edge_Tri_Pair(Int_Int_Pair(CFindEdge->first.first-1,CFindEdge->first.second-1),
//								CPPFace));
//		CFindEdge++;
//	}
//
//	map<int,vector<int>>::iterator CFindVer=modelVertexInfo.begin();
//	while(CFindVer!=modelVertexInfo.end())
//	{
//		vector<int> CPPVer;
//		for (unsigned int i=0;i<CFindVer->second.size();i++)
//		{
//			CPPVer.push_back(CFindVer->second.at(i)-1);
//		}
//		CPPmodelVertexInfo.insert(Int_Vector_Pair(CFindVer->first-1,CPPVer));
//		CFindVer++;
//	}
//	assert(modelEdgeInfo.size()==CPPmodelEdgeInfo.size());
//	assert(modelVertexInfo.size()==CPPmodelVertexInfo.size());
//}

//int OBJHandle::GetAnotherVertexIndex(const CPPGLMmodel CPPmodel,int iTriIndex,int iKnownVer0,int iKnownVer1)
//{
//	EdgeStruct::iterator FindEdge;
//	FindEdge=CPPmodelEdgeInfo.find(Int_Int_Pair(iKnownVer0,iKnownVer1));
//	assert(FindEdge!=CPPmodelEdgeInfo.end());
//	int iOtherTriIndex=iTriIndex==FindEdge->second.at(0)?FindEdge->second.at(1):FindEdge->second.at(0);
//
//	CPPGLMtriangle triangle=CPPmodel.triangles.at(iOtherTriIndex);
//	for (int i=0;i<3;i++)
//	{
//		if (triangle.vindices[i]!=iKnownVer0&&triangle.vindices[i]!=iKnownVer1)
//		{
//			return triangle.vindices[i];
//		}
//	}
//	return 99999;
//}

//double OBJHandle::Alpha(int iNeighborNum)
//{
//	double tmp = 3.0 + 2.0 * (double)cos(2.0 * CGAL_PI/(double)iNeighborNum);
//	double beta = 5.0/8.0 - (tmp*tmp)/64.0;
//	return (iNeighborNum*(1-beta)/beta);
//}

//void OBJHandle::LoopSubdivision(CPPGLMmodel & CPPmodel)
//{
//	CPPGLMmodel newCppmodel=CPPmodel;
//
//	//first,compute new vertex for each edge
//	//for each triangle
//	//maybe for each edge is better?
//	
//	//Int_Int_Pair: edge   int: new mid point index on this edge
//	map<Int_Int_Pair,int> EdgeMidPointIndex;
//	for (unsigned int i=0;i<CPPmodel.numtriangles;i++)
//	{
//		CPPGLMtriangle triangle=CPPmodel.triangles.at(i);
//		//for each edge
//		for (int j=0;j<3;j++)
//		{
//			int iStart=triangle.vindices[(j+1)%3]<triangle.vindices[(j+2)%3]?triangle.vindices[(j+1)%3]:triangle.vindices[(j+2)%3];
//			int iEnd=triangle.vindices[(j+1)%3]>triangle.vindices[(j+2)%3]?triangle.vindices[(j+1)%3]:triangle.vindices[(j+2)%3];
//			
//			//check if the edge has been handled
//			map<Int_Int_Pair,int>::iterator FindEdgeMidPoint;
//			FindEdgeMidPoint=EdgeMidPointIndex.find(Int_Int_Pair(iStart,iEnd));
//			if (FindEdgeMidPoint!=EdgeMidPointIndex.end())
//			{
//				continue;
//			}
//			//calculate the mid point of the edge
//			//two point on edge,weight: 3/8
//			Point_3 point0(CPPmodel.vertices.at(3*iStart+0),CPPmodel.vertices.at(3*iStart+1),CPPmodel.vertices.at(3*iStart+2));
//			Point_3 point1(CPPmodel.vertices.at(3*iEnd+0),CPPmodel.vertices.at(3*iEnd+1),CPPmodel.vertices.at(3*iEnd+2));
//			//other two points on neighbor triangles,weight: 1/8
//			int iVer0=triangle.vindices[j%3];
//			Point_3 point2(CPPmodel.vertices.at(3*iVer0+0),CPPmodel.vertices.at(3*iVer0+1),CPPmodel.vertices.at(3*iVer0+2));
//			int iVer1=GetAnotherVertexIndex(CPPmodel,i,iStart,iEnd);
//			assert(iVer1!=99999);
//			Point_3 point3(CPPmodel.vertices.at(3*iVer1+0),CPPmodel.vertices.at(3*iVer1+1),CPPmodel.vertices.at(3*iVer1+2));
//
//			Point_3 midPoint(point0.x()*3/8+point1.x()*3/8+point2.x()*1/8+point3.x()*1/8,
//							point0.y()*3/8+point1.y()*3/8+point2.y()*1/8+point3.y()*1/8,
//							point0.z()*3/8+point1.z()*3/8+point2.z()*1/8+point3.z()*1/8);
//
//			//insert new vertices into new model
//			EdgeMidPointIndex.insert(Edge_Int_Pair(Int_Int_Pair(iStart,iEnd),newCppmodel.numvertices));
//			newCppmodel.vertices.push_back(midPoint.x());
//			newCppmodel.vertices.push_back(midPoint.y());
//			newCppmodel.vertices.push_back(midPoint.z());
//			newCppmodel.numvertices++;
//			
//		}
//	}
//	assert(EdgeMidPointIndex.size()==CPPmodelEdgeInfo.size());
//	assert(newCppmodel.numvertices==CPPmodel.numvertices+EdgeMidPointIndex.size());
//	
//	//second,compute the new positions for the old vertices
//	for (unsigned int i=0;i<CPPmodel.numvertices;i++)
//	{
//		map<int,vector<int>>::iterator FindNeighborPoint;
//		FindNeighborPoint=CPPmodelVertexInfo.find(i);
//		assert(FindNeighborPoint!=CPPmodelVertexInfo.end());
//
//		int iNeighborNum=FindNeighborPoint->second.size();
//		double alpha=Alpha(iNeighborNum);
//		double dSumX=alpha*CPPmodel.vertices.at(3*i+0);
//		double dSumY=alpha*CPPmodel.vertices.at(3*i+1);
//		double dSumZ=alpha*CPPmodel.vertices.at(3*i+2);
//
//		for (unsigned int j=0;j<FindNeighborPoint->second.size();j++)
//		{
//			dSumX+=CPPmodel.vertices.at(3*FindNeighborPoint->second.at(j)+0);
//			dSumY+=CPPmodel.vertices.at(3*FindNeighborPoint->second.at(j)+1);
//			dSumZ+=CPPmodel.vertices.at(3*FindNeighborPoint->second.at(j)+2);
//		}
//
//		newCppmodel.vertices.at(3*i+0)=dSumX/(alpha+iNeighborNum);
//		newCppmodel.vertices.at(3*i+1)=dSumY/(alpha+iNeighborNum);
//		newCppmodel.vertices.at(3*i+2)=dSumZ/(alpha+iNeighborNum);
//	}
//
//	//last,construct new faces,delete old faces
//	for (unsigned int i=0;i<CPPmodel.numtriangles;i++)
//	{
//		CPPGLMtriangle triangle=CPPmodel.triangles.at(i);
//		int iVertexIndex[6];
//		for (int j=0;j<3;j++)
//		{
//			iVertexIndex[j]=triangle.vindices[j];
//		}
//
//		for (int j=0;j<3;j++)
//		{
//			int iStart=iVertexIndex[j%3]<iVertexIndex[(j+1)%3]?iVertexIndex[j%3]:iVertexIndex[(j+1)%3];
//			int iEnd=iVertexIndex[j%3]>iVertexIndex[(j+1)%3]?iVertexIndex[j%3]:iVertexIndex[(j+1)%3];
//			map<Int_Int_Pair,int>::iterator FindMidPointIndex;
//			FindMidPointIndex=EdgeMidPointIndex.find(Int_Int_Pair(iStart,iEnd));
//			assert(FindMidPointIndex!=EdgeMidPointIndex.end());
//			iVertexIndex[j+3]=FindMidPointIndex->second;
//		}
//
//		//attention: the order of the vertices must be counter-clock
//		//to follow the right-hand rule,so that the normal points out
//		//0,3,5
//		CPPGLMtriangle newtriangle;
//		newtriangle.vindices[0]=iVertexIndex[0];
//		newtriangle.vindices[1]=iVertexIndex[3];
//		newtriangle.vindices[2]=iVertexIndex[5];
//		newCppmodel.numtriangles++;
//		newCppmodel.triangles.push_back(newtriangle);
//		//3,1,4
//		newtriangle.vindices[0]=iVertexIndex[3];
//		newtriangle.vindices[1]=iVertexIndex[1];
//		newtriangle.vindices[2]=iVertexIndex[4];
//		newCppmodel.numtriangles++;
//		newCppmodel.triangles.push_back(newtriangle);
//		//2,5,4
//		newtriangle.vindices[0]=iVertexIndex[2];
//		newtriangle.vindices[1]=iVertexIndex[5];
//		newtriangle.vindices[2]=iVertexIndex[4];
//		newCppmodel.numtriangles++;
//		newCppmodel.triangles.push_back(newtriangle);
//		//3,4,5
//		newtriangle.vindices[0]=iVertexIndex[3];
//		newtriangle.vindices[1]=iVertexIndex[4];
//		newtriangle.vindices[2]=iVertexIndex[5];
//		newCppmodel.numtriangles++;
//		newCppmodel.triangles.push_back(newtriangle);
//	}
//	//delete old faces
//	newCppmodel.triangles.erase(newCppmodel.triangles.begin(),
//								newCppmodel.triangles.begin()+CPPmodel.triangles.size());
//	newCppmodel.numtriangles-=CPPmodel.numtriangles;
//
//	assert(newCppmodel.numtriangles==newCppmodel.triangles.size());
//	assert(newCppmodel.numtriangles==CPPmodel.numtriangles*4);
//	CPPmodel=newCppmodel;
//}

void OBJHandle::GetModelCopy(GLMmodel * model,GLMmodel * modelCopy)
{
//	assert(model);
//	assert(modelCopy);
////	memcpy(modelCopy,model,sizeof(GLMmodel));
//
//	if (model->pathname)
//	{
//		modelCopy->pathname=new char(strlen(model->pathname)+1);
//		memcpy(modelCopy->pathname,model->pathname,strlen(model->pathname)+1);
//	}
//	else
//	{
//		modelCopy->pathname=NULL;
//	}
//
//	if (model->mtllibname)
//	{
//		modelCopy->mtllibname=new char(strlen(model->mtllibname)+1);
//		memcpy(modelCopy->mtllibname,model->mtllibname,strlen(model->mtllibname)+1);
//	}
//	else
//	{
//		modelCopy->mtllibname=NULL;
//	}
//
//	modelCopy->numvertices=model->numvertices;
//	if (model->vertices)
//	{
//		modelCopy->vertices=(GLfloat *)malloc(sizeof(GLfloat)*3*(modelCopy->numvertices+1));
//	}
//	else
//	{
//		modelCopy->vertices=NULL;
//	}

	//modelCopy->numnormals=model->numnormals;
	//if (model->normals)
	//{
	//	modelCopy->normals=new float(sizeof(float)*3*(modelCopy->numnormals+1));
	//	memcpy(modelCopy->normals,model->normals,sizeof(float)*3*(modelCopy->numnormals+1));
	//}
	//else
	//{
	//	modelCopy->normals=NULL;
	//}

	//modelCopy->numtexcoords=model->numtexcoords;
	//if (model->texcoords)
	//{
	//	modelCopy->texcoords=new float(sizeof(float)*2*(modelCopy->numtexcoords+1));
	//	memcpy(modelCopy->texcoords,model->texcoords,sizeof(float)*2*(modelCopy->numtexcoords+1));
	//}
	//else
	//{
	//	modelCopy->texcoords=NULL;
	//}

	//modelCopy->numfacetnorms=model->numfacetnorms;
	//if (model->facetnorms)
	//{
	//	modelCopy->facetnorms=new float(sizeof(float)*(modelCopy->numfacetnorms+1));
	//	memcpy(modelCopy->facetnorms,model->facetnorms,sizeof(float)*(modelCopy->numtexcoords+1));
	//}
	//else
	//{
	//	modelCopy->facetnorms=NULL;
	//}

	//modelCopy->numtriangles=model->numtriangles;
	//if (model->triangles)
	//{
	//	modelCopy->triangles=(GLMtriangle*)malloc(sizeof(GLMtriangle)*(modelCopy->numtriangles));
	//	memcpy(modelCopy->triangles,model->triangles,sizeof(GLMtriangle)*(modelCopy->numtriangles));
	//	for (int i=0;i<modelCopy->numtriangles;i++)
	//	{
	//		modelCopy->triangles[i].findex=model->triangles[i].findex;
	//		for (int j=0;j<3;j++)
	//		{
	//			modelCopy->triangles[i].nindices[j]=model->triangles[i].nindices[j];
	//			modelCopy->triangles[i].vindices[j]=model->triangles[i].vindices[j];
	//			modelCopy->triangles[i].tindices[j]=model->triangles[i].tindices[j];
	//		}
	//	}
	//}
	//else
	//{
	//	modelCopy->triangles=NULL;
	//}

	//modelCopy->nummaterials=model->nummaterials;
	//if (model->materials)
	//{
	//	modelCopy->materials=(GLMmaterial*)malloc(sizeof(GLMmaterial)*(modelCopy->nummaterials));
	//	memcpy(modelCopy->materials,model->materials,sizeof(GLMmaterial)*(modelCopy->nummaterials));
	//	for (int i=0;i<modelCopy->nummaterials;i++)
	//	{	
	//		if (model->materials[i].name)
	//		{
	//			modelCopy->materials[i].name=new char(strlen(model->materials[i].name)+1);
	//			memcpy(modelCopy->materials[i].name,model->materials[i].name,strlen(model->materials[i].name)+1);
	//		}
	//		modelCopy->materials[i].shininess=modelCopy->materials[i].shininess;
	//		for (int j=0;j<4;j++)
	//		{
	//			modelCopy->materials[i].ambient[j]=model->materials[i].ambient[j];
	//			modelCopy->materials[i].diffuse[j]=model->materials[i].diffuse[j];
	//			modelCopy->materials[i].specular[j]=model->materials[i].specular[j];
	//			modelCopy->materials[i].emmissive[j]=model->materials[i].emmissive[j];
	//		}
	//	}
	//}
	//else
	//{
	//	modelCopy->materials=NULL;
	//}

	//modelCopy->numgroups=model->numgroups;
	//if (model->groups)
	//{
	//	modelCopy->groups=(GLMgroup*)malloc(sizeof(GLMgroup)*(modelCopy->numgroups));
	//	memcpy(modelCopy->groups,model->groups,sizeof(GLMgroup)*(modelCopy->numgroups));
	//	for (int i=0;i<modelCopy->numgroups;i++)
	//	{
	//		if (model->groups[i].name)
	//		{
	//			modelCopy->groups[i].name=new char(strlen(model->groups[i].name)+1);
	//			memcpy(modelCopy->groups[i].name,model->groups[i].name,strlen(model->groups[i].name)+1);
	//		}
	//		else
	//		{
	//			modelCopy->groups[i].name=NULL;
	//		}

	//		modelCopy->groups[i].numtriangles=model->groups[i].numtriangles;
	//		
	//		if (model->groups[i].triangles)
	//		{
	//			modelCopy->groups[i].triangles=new GLuint(sizeof(GLuint)*(modelCopy->groups[i].numtriangles));
	//			memcpy(modelCopy->groups[i].triangles,model->groups[i].triangles,
	//				sizeof(int)*(modelCopy->groups[i].numtriangles));
	//		}
	//		else
	//		{
	//			modelCopy->groups[i].triangles=NULL;
	//		}

	//		modelCopy->groups[i].material=model->groups[i].material;
	//		
	//		if (model->groups[i].next)
	//		{
	//			modelCopy->groups[i].next=new GLMgroup;
	//			memcpy(modelCopy->groups[i].next,model->groups[i].next,sizeof(GLMgroup));
	//		}
	//		else
	//		{
	//			modelCopy->groups[i].next=NULL;
	//		}
	//	}
	//}
	//else
	//{
	//	model->groups=NULL;
	//}

	//for (int i=0;i<3;i++)
	//{
	//	modelCopy->position[i]=model->position[i];
	//}

}


GLboolean OBJHandle::ConvertToCGALPolyhedron(const GLMmodel* model,KW_Mesh& mesh)
{
	mesh.clear();
	Build_triangle<HalfedgeDS> triangle(model);
	mesh.delegate(triangle);
	return true;
}

//with new data structure
GLboolean OBJHandle::ConvertToCGALPolyhedronNew(KW_Polyhedron& model,KW_Mesh& mesh)
{
	mesh.clear();
	Build_polyhedron<HalfedgeDS> poly(model);
	mesh.delegate(poly);
	return true;
}

GLfloat OBJHandle::UnitizeCGALPolyhedron(KW_Mesh& mesh,bool bScale,bool bTranslateToCenter)
{
	GLfloat maxx, minx, maxy, miny, maxz, minz;
	GLfloat cx, cy, cz, w, h, d;
	GLfloat scale;

	assert(!mesh.empty());

	/* get the max/mins */
	maxx = minx = mesh.points_begin()->x();
	maxy = miny = mesh.points_begin()->y();
	maxz = minz = mesh.points_begin()->z();

	for ( Point_iterator  i = mesh.points_begin(); i != mesh.points_end(); i++)
	{
		if (maxx < i->x())
			maxx = i->x();
		if (minx > i->x())
			minx = i->x();

		if (maxy < i->y())
			maxy = i->y();
		if (miny > i->y())
			miny = i->y();

		if (maxz < i->z())
			maxz = i->z();
		if (minz > i->z())
			minz = i->z();
	}

	/* calculate model width, height, and depth */
	/* modified by neo6 */
	w = maxx - minx;
	h = maxy - miny;
	d = maxz - minz;

	/* calculate center of the model */
	cx = (maxx + minx) / 2.0;
	cy = (maxy + miny) / 2.0;
	cz = (maxz + minz) / 2.0;

	/* calculate unitizing scale factor */
	scale = 1.0 / glmMax(glmMax(w, h), d);

	/* translate around center then scale */
	for ( Vertex_iterator i = mesh.vertices_begin(); i != mesh.vertices_end(); i++)
	{
		Point_3 point;
		if (bScale&&bTranslateToCenter)
		{
			point=Point_3((i->point().x()-cx)*scale,(i->point().y()-cy)*scale,(i->point().z()-cz)*scale);
		}
		else if(bTranslateToCenter)
		{
			point=Point_3(i->point().x()-cx,i->point().y()-cy,i->point().z()-cz);
		}
		else
		{
			break;
		}
		i->point()=point;
	}

	//calculate vertex&facet normal
	std::for_each( mesh.facets_begin(),   mesh.facets_end(),   Facet_normal());
	std::for_each( mesh.vertices_begin(), mesh.vertices_end(), Vertex_normal());

	return scale;
}

GLvoid OBJHandle::DrawCGALPolyhedron(KW_Mesh * pmesh, int iViewmode, GLuint color,vector<double> vecDefaultColor, GLenum mode)
{
	assert(!pmesh->empty());

	//const static GLfloat red_color[] = {1.0f, 0.0f, 0.0f, 1.0f};
	//static const GLfloat mat_specular[] = {0.0f, 0.0f, 0.0f, 1.0f};
	//static const GLfloat mat_emission[] = {0.0f, 0.0f, 0.0f, 1.0f};

	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red_color);
	//glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
	//glMaterialfv(GL_FRONT, GL_EMISSION,  mat_emission);
	//glMaterialf (GL_FRONT, GL_SHININESS, 30.0);

	//newly added to get a light spot on the mesh
	//GLfloat mat_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
	//glMaterialf (GL_FRONT, GL_SHININESS, 100.0);
	//newly added to get a light spot on the mesh


	const GLfloat mat_emission[] = {0.0f, 0.0f, 0.0f, 1.0f};//1.0f
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_emission);

	GLfloat mat_amb[] =      { 0.9, 0.9, 0.9, 1.0 };//1.0
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_amb);
	
	GLfloat mat_dif[] =      { 0.5, 0.9, 0.4,1 };//1
	mat_dif[0]=vecDefaultColor.at(0);mat_dif[1]=vecDefaultColor.at(1);
	mat_dif[2]=vecDefaultColor.at(2);mat_dif[3]=vecDefaultColor.at(3);//

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_DOUBLE, 0, &(pmesh->vecRenderVerPos[0]));
	glNormalPointer(GL_DOUBLE, 0, &(pmesh->vecRenderNorm[0]));

	if (iViewmode==POINTS_VIEW)//points
	{
		if (mode==GL_SELECT)
		{
			glPushName(MODEL_NAME);
		}
		glPointSize(3);
		//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
		if (color==COLOR_SELECTED_OPAQUE)
		{
			glColor4fv(OPAQUE_SELECTED_COLOR);
		}
		else if (color==COLOR_SELECTED_TRANSPARENT)
		{
			glColor4fv(TRANSPARENT_SELECTED_COLOR);
		}
		else
		{
			glColor4fv(mat_dif);
		}
		for (unsigned int i=0;i<pmesh->vecRenderFaceType.size();i++)
		{
			glDrawElements(GL_POINTS, pmesh->vecvecRenderFaceID.at(i).size(), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[0]));
		}
		glPointSize(1);
		if (mode==GL_SELECT)
		{
			glPopName();
		}

	}
	else if (iViewmode==SMOOTH_VIEW||iViewmode==HYBRID_VIEW)//1 smooth, 2 hybrid or transparent mode
	{
//		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
		if (color==COLOR_TRANSPARENT || color==COLOR_SELECTED_TRANSPARENT)
		{
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			mat_dif[3]=0.3;
		}

		if (color==COLOR_SELECTED_OPAQUE)
		{
			glColor4fv(OPAQUE_SELECTED_COLOR);
		}
		else if (color==COLOR_SELECTED_TRANSPARENT)
		{
			glColor4fv(TRANSPARENT_SELECTED_COLOR);
		}
		else if (color==COLOR_ORIGINAL||color==COLOR_TRANSPARENT)
		{
			glColor4fv(mat_dif);
		} 
		else if (color==COLOR_DEFORMATION_MATERIAL)
		{
			glEnableClientState(GL_COLOR_ARRAY);
			glColorPointer(4, GL_DOUBLE, 0, &(pmesh->vecRenderVerColor[0]));
		}

		if (mode==GL_SELECT)
		{
			glPushName(MODEL_NAME);
		}
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);

		for (unsigned int i=0;i<pmesh->vecRenderFaceType.size();i++)
		{
			if (pmesh->vecRenderFaceType.at(i)==3)
			{
				glDrawElements(GL_TRIANGLES, pmesh->vecvecRenderFaceID.at(i).size(), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[0]));
			}
			else if (pmesh->vecRenderFaceType.at(i)==4)
			{
				glDrawElements(GL_QUADS, pmesh->vecvecRenderFaceID.at(i).size(), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[0]));
			}
			else
			{
				int iPolyNum=pmesh->vecvecRenderFaceID.at(i).size()/pmesh->vecRenderFaceType.at(i);
				for (int j=0;j<iPolyNum;j++)
				{
					glDrawElements(GL_POLYGON, pmesh->vecRenderFaceType.at(i), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[j*pmesh->vecRenderFaceType.at(i)]));
				}
			}
		}
		if (mode==GL_SELECT)
		{
			glPopName();
		}

		if (color==COLOR_TRANSPARENT)
		{
			glDisable(GL_CULL_FACE);
		}
		if (color==COLOR_DEFORMATION_MATERIAL)
		{
			glDisableClientState(GL_COLOR_ARRAY); 
		}

		glDisable(GL_POLYGON_OFFSET_FILL);

	}
	if (iViewmode==WIREFRAME_VIEW||iViewmode==HYBRID_VIEW)//0 wireframe, 2 hybrid
	{
		glLineWidth(1.0);
		GLfloat mat_amb[] =      { 0, 0, 0, 1 };
		//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_amb);
		GLfloat mat_dif_red[] =      { 1, 0, 0, 1 };
		//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
		glColor4fv(mat_dif_red);

		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	
		glEnable(GL_CULL_FACE);

		if ((mode==GL_SELECT)&&(iViewmode==WIREFRAME_VIEW))
		{
			glPushName(MODEL_NAME);
		}

		if ((mode==GL_RENDER)||(mode==GL_SELECT&&iViewmode!=HYBRID_VIEW))
		{
			for (unsigned int i=0;i<pmesh->vecRenderFaceType.size();i++)
			{
				if (pmesh->vecRenderFaceType.at(i)==3)
				{
					glDrawElements(GL_TRIANGLES, pmesh->vecvecRenderFaceID.at(i).size(), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[0]));
				}
				else if (pmesh->vecRenderFaceType.at(i)==4)
				{
					glDrawElements(GL_QUADS, pmesh->vecvecRenderFaceID.at(i).size(), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[0]));
				}
				else
				{
					int iPolyNum=pmesh->vecvecRenderFaceID.at(i).size()/pmesh->vecRenderFaceType.at(i);
					for (int j=0;j<iPolyNum;j++)
					{
						glDrawElements(GL_POLYGON, pmesh->vecRenderFaceType.at(i), GL_UNSIGNED_INT, &(pmesh->vecvecRenderFaceID.at(i)[j*pmesh->vecRenderFaceType.at(i)]));
					}
				}
			}
		}

		if ((mode==GL_SELECT)&&(iViewmode==WIREFRAME_VIEW))
		{
			glPopName();
		}

		glDisable(GL_CULL_FACE);
		if (iViewmode==HYBRID_VIEW)
		{
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	
		}
		glLineWidth(5.0);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glDisable(GL_COLOR_MATERIAL);

	//if (iViewmode==3)
	//{
	//	glPointSize(3);
	//	glBegin(GL_POINTS);
	//	for (Vertex_iterator i=pmesh->vertices_begin(); i!=pmesh->vertices_end(); i++)
	//	{
	//		if ((color==COLOR_MEAN_CURVATURE)||(color==COLOR_GAUSSIAN_CURVATURE))
	//		{
	//			mat_dif[0]=i->GetColor()[0];
	//			mat_dif[1]=i->GetColor()[1];
	//			mat_dif[2]=i->GetColor()[2];
	//		}
	//		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
	//		//if (mode & GLM_SMOOTH)
	//		glNormal3f(i->normal().x(),
	//			i->normal().y(),
	//			i->normal().z());
	//		glVertex3f(i->point().x(),
	//			i->point().y(),
	//			i->point().z());

	//	}
	//	glEnd();
	//	glPointSize(1);
	//}

	//if (iViewmode==1||iViewmode==2)
	//{
	//	for ( Facet_iterator i = pmesh->facets_begin(); i != pmesh->facets_end(); i++)
	//	{
	//		if (i->facet_degree()==3)
	//		{
	//			glBegin(GL_TRIANGLES);
	//		}
	//		else
	//		{
	//			glBegin(GL_POLYGON);
	//		}
	//		//if (mode & GLM_FLAT)
	//		//	glNormal3f(i->normal().x(),
	//		//	i->normal().y(),
	//		//	i->normal().z());
	//		Halfedge_around_facet_circulator j = i->facet_begin();
	//		do 
	//		{
	//			//			glColor4f(0.0,0.0,1.0,0.5);
	//			if ((color==COLOR_MEAN_CURVATURE)||(color==COLOR_GAUSSIAN_CURVATURE))
	//			{
	//				mat_dif[0]=j->vertex()->GetColor()[0];
	//				mat_dif[1]=j->vertex()->GetColor()[1];
	//				mat_dif[2]=j->vertex()->GetColor()[2];
	//			}
	//			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
	//			//if (mode & GLM_SMOOTH)
	//			glNormal3f(j->vertex()->normal().x(),
	//				j->vertex()->normal().y(),
	//				j->vertex()->normal().z());
	//			glVertex3f(j->vertex()->point().x(),
	//				j->vertex()->point().y(),
	//				j->vertex()->point().z());
	//		} while(++j != i->facet_begin());
	//		glEnd();
	//	}
	//}

	//if (iViewmode==0||iViewmode==2)
	//{
	//	glLineWidth(1.0);
	//	GLfloat mat_amb[] =      { 0, 0, 0, 1 };
	//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_amb);
	//	GLfloat mat_dif[] =      { 1, 0, 0, 1 };

	//	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示

	//	for ( Facet_iterator i = pmesh->facets_begin(); i != pmesh->facets_end(); i++)
	//	{
	//		if (i->facet_degree()==3)
	//		{
	//			glBegin(GL_TRIANGLES);
	//		}
	//		else
	//		{
	//			glBegin(GL_POLYGON);
	//		}
	//		//if (mode & GLM_FLAT)
	//		//	glNormal3f(i->normal().x(),
	//		//	i->normal().y(),
	//		//	i->normal().z());
	//		Halfedge_around_facet_circulator j = i->facet_begin();
	//		do 
	//		{
	//			//			glColor4f(0.0,0.0,1.0,0.5);
	//			if ((color==COLOR_MEAN_CURVATURE)||(color==COLOR_GAUSSIAN_CURVATURE))
	//			{
	//				mat_dif[0]=j->vertex()->GetColor()[0];
	//				mat_dif[1]=j->vertex()->GetColor()[1];
	//				mat_dif[2]=j->vertex()->GetColor()[2];
	//			}
	//			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_dif);
	//			//if (mode & GLM_SMOOTH)
	//			glNormal3f(j->vertex()->normal().x(),
	//				j->vertex()->normal().y(),
	//				j->vertex()->normal().z());
	//			glVertex3f(j->vertex()->point().x(),
	//				j->vertex()->point().y(),
	//				j->vertex()->point().z());
	//		} while(++j != i->facet_begin());
	//		glEnd();
	//	}
	//	if (iViewmode==2)
	//	{
	//		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
	//	}
	//	glLineWidth(5.0);
	//}
}

GLvoid OBJHandle::LoopSubDivision(KW_Mesh& mesh)
{
	CGAL::Subdivision_method_3::Loop_subdivision(mesh,1);
	UnitizeCGALPolyhedron(mesh,false,false);
	
	mesh.SetRenderInfo(true,true,true,true,true);
}

GLvoid OBJHandle::CCSubDivision(KW_Mesh& mesh)
{
	CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh,1);
	UnitizeCGALPolyhedron(mesh,false,false);

	mesh.SetRenderInfo(true,true,true,true,true);
}
