#ifndef PRE_DEF_H
#define PRE_DEF_H

#pragma warning(disable: 4996)
#pragma warning(disable: 4244)
#pragma warning(disable: 4305)
#pragma warning(disable: 4267)
#pragma warning(disable: 4291 4503)

//#include <gl\gl.h>			// Header File For The OpenGL32 Library
//#include <gl\glu.h>			// Header File For The GLu32 Library
#include <gl\glaux.h>		// Header File For The Glaux Library
#pragma comment(lib,"opengl32.lib")
#pragma comment(lib,"glu32.lib")
#pragma comment(lib,"glaux.lib")

#include <assert.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
//#include <sys/timeb.h>
#include <time.h>
#include "SymbolsDef.h"
//#include "GeometryAlgorithm.h"
//#include "Math/Math.h"

using namespace std;



#endif
