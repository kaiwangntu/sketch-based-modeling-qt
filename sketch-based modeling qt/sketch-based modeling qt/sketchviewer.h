#ifndef SKETCHVIEWER_H
#define SKETCHVIEWER_H

#include <QGLWidget>

#include "ArcBall.h"
//#include "GeometryAlgorithm.h"

class SketchViewer : public QGLWidget
{
	Q_OBJECT

public:
	SketchViewer(QWidget *parent);
	~SketchViewer();

	void resizeGL(int width, int height);

	//if initilize of the view or not
	void Reset(bool bInitial);

	//snapshot of current view
	void saveSceneImage(const char* filename);

	GLfloat* GetArcballTrans() {return this->Transform.M;}
	void SetArcballTrans (GLfloat* Input) {memcpy_s(this->Transform.M,sizeof(this->Transform.M),Input,sizeof(GLfloat)*16);}

	void GetTranslatePara(float& OutPutX,float& OutPutY,float& OutPutZ) {OutPutX=g_fTransX;OutPutY=g_fTransY;OutPutZ=g_fZoom;}
	void SetTranslatePara(float InPutX,float InPutY,float InPutZ){g_fTransX=InPutX;g_fTransY=InPutY;g_fZoom=InPutZ;}

	GLint* GetViewport() {return this->viewport;}
	void SetViewport(GLint* Input) {memcpy_s(this->viewport,sizeof(this->viewport),Input,sizeof(GLint)*4);}

	GLdouble* GetModelview() {return this->modelview;}
	void SetModelview(GLdouble* Input) {memcpy_s(this->modelview,sizeof(this->modelview),Input,sizeof(GLdouble)*16);}

	GLdouble* GetProjection() {return this->projection;}
	void SetProjection(GLdouble* Input) {memcpy_s(this->projection,sizeof(this->projection),Input,sizeof(GLdouble)*16);}

	Matrix3fT& GetViewRotMat(){return this->ThisRot;}
	Matrix4fT& GetViewTransMat(){return this->Transform;}




protected:
	//Arcball
	ArcBallT ArcBall;
	Matrix4fT Transform;
	Matrix3fT LastRot;
	Matrix3fT ThisRot;
	int g_iLastPosX,g_iLastPosY;

	float g_fZoom,g_fTransX,g_fTransY;

	GLint    viewport[4]; 
	GLdouble modelview[16]; 
	GLdouble projection[16]; 


	QCursor* qCursor_Rotate;
	QCursor* qCursor_Move;
	QCursor* qCursor_Zoom;
	QCursor* qCursor_PaintROI;
	QCursor* qCursor_Smooth;
	QCursor* qCursor_Pencil;
	


	void initializeGL();
	void paintGL();
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void timerEvent(QTimerEvent *event);

	//total render function
	void Render(GLenum mode);

	//process the hits of left/right button in selection mode
	void ProcessMouseHit(QPoint point,int iButton);
	//get the name of the selected object(with smallest near z value or biggest number)
	int GetSelectName(GLint hits, GLuint buffer[]);




private:
	
};

#endif // SKETCHVIEWER_H
