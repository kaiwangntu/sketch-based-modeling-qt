#include "sketchviewer.h"
#include "ArcBall.h"
#include "sketchinterface.h"
#include "OBJHandle.h"

#include <QMouseEvent>

SketchViewer::SketchViewer(QWidget *parent)
	: QGLWidget(parent)
{
	startTimer( 11 ); //64-65fps
	//arcball
	Transform.M[0]=Transform.M[5]=Transform.M[10]=Transform.M[15]=1.0;
	Transform.M[1]=Transform.M[2]=Transform.M[3]=Transform.M[4]
	=Transform.M[6]=Transform.M[7]=Transform.M[8]=Transform.M[9]
	=Transform.M[11]=Transform.M[12]=Transform.M[13]=Transform.M[14]=0.0;
	Matrix3fSetIdentity(&LastRot);	// Reset Rotation
	Matrix3fSetIdentity(&ThisRot);	// Reset Rotation

	g_fZoom = 0.0f;g_fTransX = 0.0f;g_fTransY = 0.0f;


	Qt::CursorShape MoveCur = Qt::OpenHandCursor;
	this->qCursor_Move=new QCursor(MoveCur);
	QPixmap PixRotate("Resources//rotate.gif");
	this->qCursor_Rotate= new QCursor(PixRotate);
	QPixmap PixZoom("Resources//zoom.gif");
	this->qCursor_Zoom= new QCursor(PixZoom);
	QPixmap PixpaintROI("Resources//paintROI.gif");
	this->qCursor_PaintROI= new QCursor(PixpaintROI);
	QPixmap Pixsmooth("Resources//smooth.gif");
	this->qCursor_Smooth= new QCursor(Pixsmooth);
	QPixmap Pixpencil("Resources//pencil.gif");
	this->qCursor_Pencil= new QCursor(Pixpencil);

	g_iLastPosX=0;g_iLastPosY=0;

	this->pGrandParent=((SketchInterface*)(parent->parentWidget()));
}

SketchViewer::~SketchViewer()
{
	delete this->qCursor_Move;
	delete this->qCursor_Rotate;
	delete this->qCursor_Zoom;
	delete this->qCursor_PaintROI;
	delete this->qCursor_Smooth;
	delete this->qCursor_Pencil;
}

void SketchViewer::initializeGL()
{
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glShadeModel(GL_SMOOTH);//GL_FLAT for patch view
	//	glEnable(GL_CULL_FACE);//enable this will make the polygon show only front face
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	SketchDoc* pDoc=pGrandParent->GetDoc();
	//	float pos[4] = { 0, 0, 1, 0};
	float* pos=pDoc->GetLightPos();
	glLightfv(GL_LIGHT0,GL_POSITION,pos);
	//old settings
	//	float diffuse[4] = { 1, 1, 1, 1};
	float diffuse[4] = {0.8f, 0.8f, 0.8f, 1.0f};
	//	float ambient[4] = { 1, 1, 1, 0.5};
	float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f};
	//	float specular[4] = { 1.0f, 0.0f, 0.0f, 1.0f};
	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	//	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
}

void SketchViewer::resizeGL(int width, int height)
{
	if (height<=0)
	{
		return;
	}

	this->ArcBall.setBounds(width,height);

	updateGL();

	QSize size(width,height);
	double aspect;
	aspect = (height == 0) ? (double)size.width() : (double)size.width()/(double)size.height();

	glViewport(0,0,size.width(),size.height());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,aspect,0.01,5000.0f);//
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	wglMakeCurrent(NULL,NULL);

	updateGL();
}

void SketchViewer::paintGL()
{
	Render(GL_RENDER);
}

void SketchViewer::Reset()
{
	//Arcball
	Matrix3fSetIdentity(&LastRot);	// Reset Rotation
	Matrix3fSetIdentity(&ThisRot);	// Reset Rotation
	Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);// Reset Rotation
	Transform.M[0]=Transform.M[5]=Transform.M[10]=Transform.M[15]=1;
	Transform.M[1]=Transform.M[2]=Transform.M[3]
	=Transform.M[4]=Transform.M[6]=Transform.M[7]
	=Transform.M[8]=Transform.M[9]=Transform.M[11]
	=Transform.M[12]=Transform.M[13]=Transform.M[14]=0;
	g_fZoom = 0.0f;g_fTransX = 0.0f;g_fTransY = 0.0f;
	//stop plane rotation
	//KillTimer(TIMER_PLANE_ROTATE);
	//KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);
}


void SketchViewer::mousePressEvent(QMouseEvent *event)
{
	//KillTimer(TIMER_PLANE_ROTATE);
	//KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);

	SketchDoc* pDoc=this->pGrandParent->GetDoc();

	QPoint point=event->pos();
	Qt::MouseButton PressedButton=event->button();

	g_iLastPosX  = point.x();
	g_iLastPosY  = point.y();

	if (PressedButton==Qt::LeftButton)
	{
		if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
		{
			ProcessMouseHit(point,MOUSE_LEFT_BUTTON_HIT);
			if ((pDoc->GetRBSelName()==NONE_SELECTED)||(pDoc->GetLBSelName()!=pDoc->GetRBSelName())
				|| (pDoc->GetRBSelName()==MODEL_NAME && pDoc->GetEditMode()==CREATION_MODE))
			{
				Point2fT    MousePt;// NEW: Current Mouse Point
				LastRot=ThisRot;// Set Last Static Rotation To Last Dynamic One
				MousePt.s.X=point.x();
				MousePt.s.Y=point.y();
				ArcBall.click(&MousePt);// Update Start Vector And Prepare For Dragging
				this->setCursor(*qCursor_Rotate);
			}
			else if (pDoc->GetLBSelName()==pDoc->GetRBSelName())
			{
			}
		}
		else if (pDoc->GetManipMode()==SKETCH_MODE)
		{
			if ((pDoc->GetEditMode()==CREATION_MODE)&&(pDoc->GetMeshCreation().GetDrawingPlane()!=NONE_SELECTED))
			{
				if (pDoc->GetMeshCreation().CheckPlaneState())
				{
					pDoc->GetMeshCreation().Input2DProfilePoint(point);
				}
			}
			else if (pDoc->GetEditMode()==EDITING_MODE)
			{
				pDoc->GetMeshCreation().Input2DProfilePoint(point);
			}
			else if (pDoc->GetEditMode()==DEFORMATION_MODE)
			{
				if ((GetKeyState(0x52)<0))////R key pressed,curve to circle the ROI
				{
					if ((!pDoc->GetMeshDeformation().GetHandleNbVertex().empty()))
					{
						pDoc->GetMeshDeformation().SetDrawingCurveType(DEFORMATION_GESTURE_CIRCLE_ROI);
						pDoc->GetMeshDeformation().InputCurvePoint2D(point);
					}
				}
				else if (GetKeyState(0x50)<0)//P key pressed,points to paint the ROI
				{
					this->setCursor(*qCursor_PaintROI);
					if ((!pDoc->GetMeshDeformation().GetHandleNbVertex().empty()))
					{
						pDoc->GetMeshDeformation().SetDrawingCurveType(DEFORMATION_GESTURE_PAINT_ROI);
						pDoc->GetMeshDeformation().InputCurvePoint2D(point);
						pDoc->GetMeshDeformation().PaintROIVertices(pDoc->GetMesh(),this->modelview,this->projection,this->viewport);
					}
				}
				else 
				{
					pDoc->GetMeshDeformation().InputCurvePoint2D(point);
				}
			}
			//else if (pDoc->GetEditMode()==CUTTING_MODE)
			//{
			//	pDoc->GetMeshCutting().InputCurvePoint2D(point);
			//}
			//else if (pDoc->GetEditMode()==EXTRUSION_MODE)
			//{
			//	pDoc->GetMeshExtrusion().InputCurvePoint2D(point);
			//}
			//else if (pDoc->GetEditMode()==SMOOTHING_MODE)
			//{
			//	SetCursor(hCursor_Smooth);
			//	pDoc->GetMeshSmoothing().Init(pDoc);
			//	pDoc->GetMeshSmoothing().InputCurvePoint2D(point);
			//	pDoc->GetMeshSmoothing().PaintROIVertices(pDoc->GetMesh(),this->modelview,this->projection,this->viewport);
			//}
		}
	}
	else if (PressedButton==Qt::RightButton)
	{
		if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
		{
			ProcessMouseHit(point,MOUSE_RIGHT_BUTTON_HIT);
			if (pDoc->GetRBSelName()==NONE_SELECTED)
			{
				this->setCursor(*qCursor_Move);
				if (pDoc->GetEditMode()==CREATION_MODE)
				{
					pDoc->GetMeshCreation().SetDrawingPlane();
				}
				else if (pDoc->GetEditMode()==DEFORMATION_MODE)
				{
					pDoc->GetMeshDeformation().SetSelectedItem();
				}
		//		else if (pDoc->GetEditMode()==EXTRUSION_MODE)
		//		{
		//			pDoc->GetMeshExtrusion().SetSelectedItem();
		//		}
			}
			else
			{
				if (pDoc->GetEditMode()==CREATION_MODE)
				{
					pDoc->GetMeshCreation().SetDrawingPlane();
				}
				else if (pDoc->GetEditMode()==DEFORMATION_MODE)
				{
					pDoc->GetMeshDeformation().SetSelectedItem();
				}
		//		else if (pDoc->GetEditMode()==EXTRUSION_MODE)
		//		{
		//			pDoc->GetMeshExtrusion().SetSelectedItem();
		//		}
			}
		}
		else if (pDoc->GetManipMode()==SKETCH_MODE)
		{
			//do nothing
		}
	}

	updateGL();
}

void SketchViewer::mouseReleaseEvent(QMouseEvent *event)
{
	SketchDoc* pDoc=this->pGrandParent->GetDoc();

	Qt::MouseButton PressedButton=event->button();
	if (PressedButton==Qt::LeftButton)
	{
		if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
		{
			if (pDoc->GetRBSelName()==NONE_SELECTED)
			{
				//do nothing
			}
			else if (pDoc->GetLBSelName()==pDoc->GetRBSelName())
			{
				if (pDoc->GetEditMode()==CREATION_MODE)//stop translation of the reference plane in creation
				{
					pDoc->GetMeshCreation().StopTranslateDrawingPlane();
				}
			}
		}
		else if (pDoc->GetManipMode()==SKETCH_MODE)
		{
			if ((pDoc->GetEditMode()==CREATION_MODE)&&(pDoc->GetMeshCreation().GetDrawingPlane()!=NONE_SELECTED))
			{
				pDoc->GetMeshCreation().Convert2DProfileTo3D();
			}
			else if (pDoc->GetEditMode()==EDITING_MODE)
			{
				pDoc->GetMeshEditing().Convert2DProfileTo3D();
			}
			else if (pDoc->GetEditMode()==DEFORMATION_MODE)
			{
				pDoc->GetMeshDeformation().Conver2DCurveTo3D(pDoc->GetMesh());
			}
		//	else if (pDoc->GetEditMode()==CUTTING_MODE)
		//	{
		//		pDoc->GetMeshCutting().Conver2DCurveTo3D(pDoc->GetMesh());
		//	}
		//	else if (pDoc->GetEditMode()==EXTRUSION_MODE)
		//	{
		//		pDoc->GetMeshExtrusion().Conver2DCurveTo3D(pDoc->GetMesh());
		//	}
		//	else if(pDoc->GetEditMode()==SMOOTHING_MODE)
		//	{
		//		pDoc->GetMeshSmoothing().ClearROI();
		//	}
		}
	}
	else if (PressedButton==Qt::RightButton)
	{
		//do nothing
	}

	if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
	{
		SetViewSelectCursor();
	}

	updateGL();
}

void SketchViewer::mouseMoveEvent(QMouseEvent *event)
{
	// TODO: Add your message handler code here and/or call default
	SketchDoc* pDoc=this->pGrandParent->GetDoc();


	QPoint point=event->pos();
	int g_iStepX = point.x()-g_iLastPosX;
	int g_iStepY = point.y()-g_iLastPosY;

	if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
	{
		if (((pDoc->GetRBSelName()==NONE_SELECTED))||(pDoc->GetLBSelName()!=pDoc->GetRBSelName())
			|| (pDoc->GetRBSelName()==MODEL_NAME && pDoc->GetEditMode()==CREATION_MODE))
		{
			if (event->buttons()&Qt::LeftButton)
			{
				this->setCursor(*qCursor_Rotate);
				Quat4fT ThisQuat;
				Point2fT    MousePt;// NEW: Current Mouse Point
				MousePt.s.X=point.x();
				MousePt.s.Y=point.y();
				ArcBall.drag(&MousePt,&ThisQuat);// Update End Vector And Get Rotation As Quaternion
				Matrix3fSetRotationFromQuat4f(&ThisRot, &ThisQuat);// Convert Quaternion Into Matrix3fT
				//each time the mouse is dragged,the rotation angle from the rest state is re-calculated   
				//LastRot is only computed when the mouse is pressed        --by KW
				Matrix3fMulMatrix3f(&ThisRot, &LastRot);// Accumulate Last Rotation Into This One
				Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);
			}
			else if (event->buttons()&Qt::RightButton)
			{
				this->setCursor(*qCursor_Move);
				g_fTransX  += 0.01f*g_iStepX;
				g_fTransY  -= 0.01f*g_iStepY;
			}
		}
		else if (pDoc->GetLBSelName()==pDoc->GetRBSelName())
		{
			if ((pDoc->GetEditMode()==CREATION_MODE)&&((event->buttons()&Qt::LeftButton)))//translate the reference plane in creation
			{
				this->setCursor(*qCursor_Move);
				pDoc->GetMeshCreation().TranslateDrawingPlane(0.01f*g_iStepX);
			}
			else if (pDoc->GetEditMode()==DEFORMATION_MODE && (event->buttons()&Qt::LeftButton))
			{
				this->setCursor(*qCursor_Move);
				pDoc->GetMeshDeformation().ManipSelItem(g_iStepX,g_iStepY);
			}
	//		else if (pDoc->GetEditMode()==EXTRUSION_MODE && (nFlags & MK_LBUTTON))
	//		{
	//			SetCursor(hCursor_Rotate);
	//			pDoc->GetMeshExtrusion().ManipSelItem(g_iStepX,g_iStepY);
	//		}
		}
	}
	else if (pDoc->GetManipMode()==SKETCH_MODE)
	{
		if ((pDoc->GetEditMode()==CREATION_MODE)&&(pDoc->GetMeshCreation().GetDrawingPlane()!=NONE_SELECTED)
			&&(event->buttons()&Qt::LeftButton))
		{
			pDoc->GetMeshCreation().Input2DProfilePoint(point);
		}
		else if (pDoc->GetEditMode()==EDITING_MODE && (event->buttons()&Qt::LeftButton))
		{
			pDoc->GetMeshEditing().Input2DProfilePoint(point);
		}
		else if (pDoc->GetEditMode()==DEFORMATION_MODE && (event->buttons()&Qt::LeftButton))
		{
			if ((GetKeyState(0x52)<0))//R key pressed,curve to circle the ROI
			{
				if ((!pDoc->GetMeshDeformation().GetHandleNbVertex().empty()))
				{
					pDoc->GetMeshDeformation().InputCurvePoint2D(point);
				}
			}
			else if (GetKeyState(0x50)<0)//P key pressed,points to paint the ROI
			{
				this->setCursor(*qCursor_PaintROI);
				if ((!pDoc->GetMeshDeformation().GetHandleNbVertex().empty()))
				{
					pDoc->GetMeshDeformation().InputCurvePoint2D(point);
					pDoc->GetMeshDeformation().PaintROIVertices(pDoc->GetMesh(),this->modelview,this->projection,this->viewport);
				}
			}
			else
			{
				pDoc->GetMeshDeformation().InputCurvePoint2D(point);
			}

		}
	//	else if (pDoc->GetEditMode()==CUTTING_MODE && (nFlags & MK_LBUTTON))
	//	{
	//		pDoc->GetMeshCutting().InputCurvePoint2D(point);
	//	}
	//	else if (pDoc->GetEditMode()==EXTRUSION_MODE && (nFlags & MK_LBUTTON))
	//	{
	//		pDoc->GetMeshExtrusion().InputCurvePoint2D(point);
	//	}
	//	else if (pDoc->GetEditMode()==SMOOTHING_MODE && (nFlags & MK_LBUTTON))
	//	{
	//		//if (GetKeyState(0x46)<0)//f key pressed,scratch to smooth
	//		//{
	//		SetCursor(hCursor_Smooth);
	//		pDoc->GetMeshSmoothing().InputCurvePoint2D(point);
	//		pDoc->GetMeshSmoothing().PaintROIVertices(pDoc->GetMesh(),this->modelview,this->projection,this->viewport);
			//}
	//	}
	}

	g_iLastPosX  = point.x();
	g_iLastPosY  = point.y();

	updateGL();
}

void SketchViewer::wheelEvent(QWheelEvent *event)
{
	// TODO: Add your message handler code here and/or call default
	//KillTimer(TIMER_PLANE_ROTATE);
	//KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);

	SketchDoc* pDoc=this->pGrandParent->GetDoc();

	if (pDoc->GetManipMode()==VIEW_SELECTION_MODE)
	{
		if (pDoc->GetRBSelName()==NONE_SELECTED || (pDoc->GetRBSelName()==MODEL_NAME && pDoc->GetEditMode()==CREATION_MODE))
		{
			// middle mouse button
			this->setCursor(*qCursor_Zoom);
			if(event->delta()>0)
			{
				g_fZoom+=0.15;
			}
			else
			{
				g_fZoom-=0.15;
			}
		}
		else if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().AdjustPlaneBoundary(event->delta());
		}
		else if (pDoc->GetEditMode()==DEFORMATION_MODE)
		{
			pDoc->GetMeshDeformation().AdjustPlaneBoundary(event->delta());
		}
	//	else if (pDoc->GetEditMode()==EXTRUSION_MODE)
	//	{
	//		pDoc->GetMeshExtrusion().AdjustPlaneBoundary(zDelta);
	//	}
	//}
	else if (pDoc->GetManipMode()==SKETCH_MODE)
	{
		//do nothing
	}
	}

	updateGL();
}

void SketchViewer::keyPressEvent(QKeyEvent *event)
{
	//keyboard messages
	if (event->count()>1)
	{
		return;
	}

	//KillTimer(TIMER_PLANE_ROTATE);
	//KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);
	SketchDoc* pDoc=this->pGrandParent->GetDoc();
	if(event->key()==Qt::Key_Enter)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			if (pDoc->GetMeshCreation().FitLastPlaneCurves())
			{
				pDoc->GetMeshCreation().GenerateMesh(pDoc->GetMesh(),pDoc->GetDefaultColor());
			}
		}
	}
	else if (event->key()==Qt::Key_Escape)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().CancelLastInput();
		}
	}
	else if (event->key()==Qt::Key_Delete)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().DeleteSpecifiedCS();
		}
	}
	else if (event->key()==Qt::Key_F1)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().CopyCNFromLastParaPlane(0);
		}
	}
	else if (event->key()==Qt::Key_F2)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().CopyCNFromLastParaPlane(1);
		}
	}
	else if (event->key()==Qt::Key_F3)
	{
		if (pDoc->GetEditMode()==CREATION_MODE)
		{
			pDoc->GetMeshCreation().CopyCNFromLastParaPlane(2);
		}
	}
	updateGL();
}

void SketchViewer::ProcessMouseHit(QPoint point,int iButton)
{
	GLuint selectBuf[512];
	GLint iHits;

	glSelectBuffer(512, selectBuf);
	glRenderMode (GL_SELECT);
	glInitNames();

	//RECT rect;
	//GetWindowRect(&rect);
	//int cx=rect.right-rect.left;
	int cx=this->rect().right()-this->rect().left();
	//int cy=rect.bottom-rect.top;
	int cy=this->rect().bottom()-this->rect().top();
	double aspect;
	aspect = (cy == 0) ? (double)cx : (double)cx/(double)cy;
	glMatrixMode (GL_PROJECTION);
	glPushMatrix ();
	glLoadIdentity ();
	//get a 5x5 pixel region around the mouse position
	gluPickMatrix ((GLdouble) point.x(), (GLdouble) (this->viewport[3] - point.y()), 
		5.0, 5.0, this->viewport);
	gluPerspective(45,aspect,0.01,5000.0f);//same as in OnSize function
	glMatrixMode(GL_MODELVIEW);

	Render(GL_SELECT);

	glMatrixMode (GL_PROJECTION);
	glPopMatrix ();

	iHits=glRenderMode(GL_RENDER);

	SketchDoc* pDoc=this->pGrandParent->GetDoc();
	if (iButton==MOUSE_LEFT_BUTTON_HIT)
	{
		pDoc->SetLBSelName(GetSelectName(iHits,selectBuf));
	}
	else if (iButton==MOUSE_RIGHT_BUTTON_HIT)
	{
		pDoc->SetRBSelName(GetSelectName(iHits,selectBuf));
	}
}

int SketchViewer::GetSelectName(GLint hits, GLuint buffer[])
{
	GLuint iNumName, *ptr;
	vector<vector<GLuint>> names;//reserved for further use
	//priority: first consider min nearZ value; for similar nearZ values,consider max name
	float fMinNearZ=1000.0;
	const float fErrThre=0.0003;
	int iMaxName=NONE_SELECTED;

	cout<<"hits = "<<hits<<endl;;
	ptr = (GLuint *) buffer;
	for (int i = 0; i < hits; i++) 
	{
		iNumName = *ptr;
		float fCurrentNearZ=0.0;
		//DBWindowWrite(" number of names for this hit = %d\n", iNumName); 
		ptr++;
		//DBWindowWrite("  z1 is %g;", (float) *ptr/0x7fffffff); 
		fCurrentNearZ=(float) *ptr/0x7fffffff;
		ptr++;
		//DBWindowWrite(" z2 is %g\n", (float) *ptr/0x7fffffff); 
		ptr++;
		//DBWindowWrite("   names are ");
		vector<GLuint> name;
		if (fCurrentNearZ-fMinNearZ<fErrThre)
		{
			for (unsigned int j = 0; j < iNumName; j++) 
			{ 
				name.push_back(*ptr);
				if (iMaxName<(int)*ptr)
				{
					iMaxName=*ptr;
					fMinNearZ=fCurrentNearZ;
				}
				//DBWindowWrite("%d ", *ptr);
				ptr++;
			}
		}
		else
		{
			for (unsigned int j = 0; j < iNumName; j++) 
			{ 
				name.push_back(*ptr);
				//DBWindowWrite("%d ", *ptr);
				ptr++;
			}
		}
		if (!name.empty())
		{
			names.push_back(name);
		}
		//DBWindowWrite("\n");
	}
	cout<<"select name: "<<iMaxName<<endl;
	return iMaxName;
}

void SketchViewer::saveSceneImage(QString filename)
{
	QPixmap pixmap = QPixmap::grabWidget(this);
	pixmap.save(filename,"bmp");
}

void SketchViewer::Render(GLenum mode)
{

	SketchDoc* pDoc=this->pGrandParent->GetDoc();

	//has been set when the sketch radio button is checked
	//if (pDoc->GetManipMode()==SKETCH_MODE)
	//{
	//	this->setCursor(*qCursor_Pencil);
	//}

	// Do not call CView::OnPaint() for painting messages
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	float* pos=pDoc->GetLightPos();
	glLightfv(GL_LIGHT0,GL_POSITION,pos);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glDepthFunc(GL_LESS);		
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	//render objects(GL_RENDER)
	glLoadIdentity();
	gluLookAt(0,1,5,0,0,0,0,1,0);
	glPushMatrix();
	glTranslatef(g_fTransX,g_fTransY,g_fZoom);
	glMultMatrixf(Transform.M);


	if (mode==GL_RENDER)
	{
		glGetIntegerv(GL_VIEWPORT, viewport); 
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 
		glGetDoublev(GL_PROJECTION_MATRIX, projection); 
	}

	if (pDoc->GetViewStyle())
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	} 
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	}

	glDisable(GL_LIGHTING);
	if ((pDoc->IsAxisOn())&&(mode==GL_RENDER))
	{
		glLineWidth(2.0);
		glBegin(GL_LINES);
		glColor3f(1,0,0);
		glVertex3f(0,0,0);
		glVertex3f(1,0,0);
		glColor3f(0,1,0);
		glVertex3f(0,0,0);
		glVertex3f(0,1,0);
		glColor3f(0,0,1);
		glVertex3f(0,0,0);
		glVertex3f(0,0,1);
		glEnd();
		glLineWidth(1.0);
	} 
	glEnable(GL_LIGHTING);


	//if (pDoc->IsPrimalMeshShown())
	if (pDoc->GetRenderPreMesh()==MESH_EXIST_VIEW && !pDoc->GetMesh().empty())
	{
		//the mesh is selected
		if (pDoc->GetRBSelName()==MODEL_NAME)
		{
			OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_SELECTED_OPAQUE,
				pDoc->GetDefaultColor(),mode);
		}
		else
		{
			OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),pDoc->GetColorMode(),
				pDoc->GetDefaultColor(),mode);
		}
	}


	glColor3f(1,0,0);
	switch(pDoc->GetEditMode())
	{
	case CREATION_MODE:
		if (mode==GL_RENDER)
		{
			this->renderText(5,25,QString("Creation Mode"),QFont("arial",static_cast<int>(1*20)));
		}

		pDoc->GetMeshCreation().Render(pDoc->GetViewStyle(),mode,this->modelview,this->projection,this->viewport);

		break;
	case EDITING_MODE:
		if (mode==GL_RENDER)
		{
			this->renderText(5,25,QString("Editing Mode"),QFont("arial",static_cast<int>(1*20)));
		}

		pDoc->GetMeshEditing().Render(mode);

		break;
	case DEFORMATION_MODE:
		if (mode==GL_RENDER)
		{
			this->renderText(5,25,QString("Deformation Mode"),QFont("arial",static_cast<int>(1*20)));
		}
		pDoc->GetMeshDeformation().Render(pDoc->GetViewStyle(),mode,pDoc->IsDualMeshShown());
		break;
	case EXTRUSION_MODE:
		if (mode==GL_RENDER)
		{
			this->renderText(5,25,QString("Extrusion Mode"),QFont("arial",static_cast<int>(1*20)));
		}
		//pDoc->GetMeshExtrusion().Render(pDoc->GetViewStyle(),mode);
		break;
	case CUTTING_MODE:
		this->renderText(5,25,QString("Cutting Mode"),QFont("arial",static_cast<int>(1*20)));
		//pDoc->GetMeshCutting().Render(pDoc->GetViewStyle(),mode);
		break;
	case SMOOTHING_MODE:
		this->renderText(5,25,QString("Smoothing Mode"),QFont("arial",static_cast<int>(1*20)));
		//pDoc->GetMeshSmoothing().Render(pDoc->GetViewStyle(),
		//	this->modelview,this->projection,this->viewport,mode);
		break;
	case TEST_MODE:
		this->renderText(5,25,QString("Test Mode"),QFont("arial",static_cast<int>(1*20)));
		pDoc->GetTest().Render(pDoc->GetViewStyle(),
			this->modelview,this->projection,this->viewport);
		break;
	default:
		break;
	}


	if (pDoc->GetRenderPreMesh()==MESH_PREVIEW && !pDoc->GetMesh().empty())
	{
		//the mesh is selected
		if (pDoc->GetRBSelName()==MODEL_NAME)
		{
			OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_SELECTED_TRANSPARENT,
				pDoc->GetDefaultColor(),mode);
		}
		else
		{
			OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_TRANSPARENT,//pDoc->GetColorMode(),
				pDoc->GetDefaultColor(),mode);
		}
	}


	glPopMatrix();

}



void SketchViewer::timerEvent(QTimerEvent *event)
{
	//// TODO: Add your message handler code here and/or call default
	//CKWResearchWorkDoc* pDoc=GetDocument();
	//switch(nIDEvent)
	//{
	//case TIMER_PLANE_WAIT_TO_ROTATE:
	//	DBWindowWrite("Waiting to rotate...\n");
	//	if (pDoc->GetEditMode()==CREATION_MODE)
	//	{
	//		pDoc->GetMeshCreation().WaitAutoPlaneRotate();
	//	}
	//	break;
	//case TIMER_PLANE_ROTATE:
	//	DBWindowWrite("Rotating...\n");
	//	if (pDoc->GetEditMode()==CREATION_MODE)
	//	{
	//		pDoc->GetMeshCreation().RotateSelectedPlane();
	//	}
	//	break;
	//default:
	//	break;
	//}
	//RedrawWindow();
	//CView::OnTimer(nIDEvent);
	updateGL();
}

