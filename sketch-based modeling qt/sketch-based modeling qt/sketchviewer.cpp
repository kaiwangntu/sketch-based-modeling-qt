#include "sketchviewer.h"


SketchViewer::SketchViewer(QWidget *parent)
	: QGLWidget(parent)
{
	startTimer( 11 ); //64-65fps
}

SketchViewer::~SketchViewer()
{

}

void SketchViewer::initializeGL()
{
	//initialization of OpenGL
	glClearColor(1.0f, 1.0f, 1.0f, 0.f);
	//resizeGL( 400 , 300 );
	glShadeModel( GL_SMOOTH );
	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
	glEnable( GL_TEXTURE_2D );
	glEnable( GL_CULL_FACE );
	glEnable( GL_DEPTH_TEST );
}

void SketchViewer::paintGL()
{
	//draw scene here
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslated(0.0, 0.0, -1.0);

	glColor3f(0,1,0);
	glPointSize(5);
	glBegin(GL_POINTS);
	glVertex3d(0,0,0);
	glEnd();
}

void SketchViewer::resizeGL(int width, int height)
{
	//proces resize keep good aspect ratio for 3D scene

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);

	// glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)width/(GLfloat)height, 0.01f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
}

void SketchViewer::mousePressEvent(QMouseEvent *event)
{
	//proces mouse events for rotate/move inside 3D scene
}

void SketchViewer::mouseMoveEvent(QMouseEvent *event)
{
	//proces keyboard events
}

void SketchViewer::timerEvent(QTimerEvent *event)
{
	updateGL();
}