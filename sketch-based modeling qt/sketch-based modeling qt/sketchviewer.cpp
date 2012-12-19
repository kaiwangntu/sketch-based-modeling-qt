#include "sketchviewer.h"
#include "ArcBall.h"


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

	//CKWResearchWorkDoc* pDoc=GetDocument();
	////	float pos[4] = { 0, 0, 1, 0};
	//float* pos=pDoc->GetLightPos();
	//glLightfv(GL_LIGHT0,GL_POSITION,pos);
	////old settings
	////	float diffuse[4] = { 1, 1, 1, 1};
	//float diffuse[4] = {0.8f, 0.8f, 0.8f, 1.0f};
	////	float ambient[4] = { 1, 1, 1, 0.5};
	//float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f};
	////	float specular[4] = { 1.0f, 0.0f, 0.0f, 1.0f};
	//glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	//glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	////	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
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

void SketchViewer::Reset(bool bInitial)
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
	////stop plane rotation
	//KillTimer(TIMER_PLANE_ROTATE);
	//KillTimer(TIMER_PLANE_WAIT_TO_ROTATE);
}


void SketchViewer::mousePressEvent(QMouseEvent *event)
{
}

void SketchViewer::mouseReleaseEvent(QMouseEvent *event)
{

}

void SketchViewer::mouseMoveEvent(QMouseEvent *event)
{
}

void SketchViewer::wheelEvent(QWheelEvent *event)
{
}


void SketchViewer::ProcessMouseHit(QPoint point,int iButton)
{
	//GLuint selectBuf[512];
	//GLint iHits;

	//glSelectBuffer(512, selectBuf);
	//glRenderMode (GL_SELECT);
	//glInitNames();

	//RECT rect;
	//GetWindowRect(&rect);
	//int cx=rect.right-rect.left;
	//int cy=rect.bottom-rect.top;
	//double aspect;
	//aspect = (cy == 0) ? (double)cx : (double)cx/(double)cy;

	//glMatrixMode (GL_PROJECTION);
	//glPushMatrix ();
	//glLoadIdentity ();
	////  在鼠标位置生成5X5像素区域
	//gluPickMatrix ((GLdouble) point.x, (GLdouble) (this->viewport[3] - point.y), 
	//	5.0, 5.0, this->viewport);
	//gluPerspective(45,aspect,0.01,5000.0f);//same as in OnSize function
	//glMatrixMode(GL_MODELVIEW);

	//Render(GL_SELECT);

	//glMatrixMode (GL_PROJECTION);
	//glPopMatrix ();

	//iHits=glRenderMode(GL_RENDER);

	//CKWResearchWorkDoc* pDoc=GetDocument();
	//if (iButton==MOUSE_LEFT_BUTTON_HIT)
	//{
	//	pDoc->SetLBSelName(GetSelectName(iHits,selectBuf));
	//}
	//else if (iButton==MOUSE_RIGHT_BUTTON_HIT)
	//{
	//	pDoc->SetRBSelName(GetSelectName(iHits,selectBuf));
	//}
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

void SketchViewer::saveSceneImage(const char* filename)
{
	//wglMakeCurrent(g_pBLDC->m_hDC,m_hGLContext);

	//GLint viewport[4];
	//glGetIntegerv( GL_VIEWPORT, viewport );
	//int width  = viewport[2];
	//int height = viewport[3];
	//width -= width%4;
	//int x=0;
	//int y=0;

	//GLubyte * bmpBuffer = NULL;
	//bmpBuffer = new GLubyte[width*height*3*sizeof(GLubyte)];
	//if (!bmpBuffer)
	//	return;// FALSE;
	//glReadPixels((GLint)x, (GLint)y, (GLint)width, (GLint)height,
	//	GL_BGR_EXT, GL_UNSIGNED_BYTE, bmpBuffer);
	//wglMakeCurrent(g_pBLDC->m_hDC,NULL);


	//FILE *filePtr;
	//fopen_s(&filePtr, filename, "wb");
	//if (!filePtr)
	//	return;// FALSE;

	//BITMAPFILEHEADER bitmapFileHeader;
	//BITMAPINFOHEADER bitmapInfoHeader;

	//bitmapFileHeader.bfType = 0x4D42; //"BM"
	//bitmapFileHeader.bfSize = width*height*3;
	//bitmapFileHeader.bfReserved1 = 0;
	//bitmapFileHeader.bfReserved2 = 0;
	//bitmapFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	//bitmapInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	//bitmapInfoHeader.biWidth = width;
	//bitmapInfoHeader.biHeight = height;
	//bitmapInfoHeader.biPlanes = 1;
	//bitmapInfoHeader.biBitCount = 24;
	//bitmapInfoHeader.biCompression = BI_RGB;
	//bitmapInfoHeader.biSizeImage = 0;
	//bitmapInfoHeader.biXPelsPerMeter = 0;
	//bitmapInfoHeader.biYPelsPerMeter = 0;
	//bitmapInfoHeader.biClrUsed = 0;
	//bitmapInfoHeader.biClrImportant = 0;

	//fwrite(&bitmapFileHeader, sizeof(bitmapFileHeader), 1, filePtr);
	//fwrite(&bitmapInfoHeader, sizeof(bitmapInfoHeader), 1, filePtr);
	//fwrite(bmpBuffer, width*height*3, 1, filePtr);
	//fclose(filePtr);
	//delete [] bmpBuffer;

	//return;// TRUE; 
}

void SketchViewer::Render(GLenum mode)
{

	//CKWResearchWorkDoc* pDoc=GetDocument();

	//if (pDoc->GetManipMode()==SKETCH_MODE)
	//{
	//	SetCursor(hCursor_Pencil);
	//}

	// Do not call CView::OnPaint() for painting messages
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	//float* pos=pDoc->GetLightPos();
	//glLightfv(GL_LIGHT0,GL_POSITION,pos);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glDepthFunc(GL_LESS);		//设置深度测试函数
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	//render objects(GL_RENDER)
	glLoadIdentity();
	gluLookAt(0,1,5,0,0,0,0,1,0);
	glPushMatrix();
	glTranslatef(g_fTransX,g_fTransY,g_fZoom);
	glMultMatrixf(Transform.M);//旋转


	if (mode==GL_RENDER)
	{
		glGetIntegerv(GL_VIEWPORT, viewport); 
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 
		glGetDoublev(GL_PROJECTION_MATRIX, projection); 
	}

	//if (pDoc->GetViewStyle())
	//{
	//	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);	//设置多边形显示模式为双面填充显示
	//} 
	//else
	//{
	//	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	//设置多边形显示模式为双面线显示
	//}

	glDisable(GL_LIGHTING);
	//if ((pDoc->IsAxisOn())&&(mode==GL_RENDER))
	//{
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
	//} 
	glEnable(GL_LIGHTING);


	////if (pDoc->IsPrimalMeshShown())
	//if (pDoc->GetRenderPreMesh()==MESH_EXIST_VIEW && !pDoc->GetMesh().empty())
	//{
	//	//the mesh is selected
	//	if (pDoc->GetRBSelName()==MODEL_NAME)
	//	{
	//		OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_SELECTED_OPAQUE,
	//			pDoc->GetDefaultColor(),mode);
	//	}
	//	else
	//	{
	//		OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),pDoc->GetColorMode(),
	//			pDoc->GetDefaultColor(),mode);
	//	}
	//}

	glColor3f(0,1,1);
	this->renderText(10,10,QString("Test Mode0 Test Mode1 Test Mode2 Test Mode3"),QFont("Arial",static_cast<int>(1*20)));

	//if (mode==GL_RENDER)
	//{
	//	this->RenderText.SetTextPos(this->viewport);
	//}
	//switch(pDoc->GetEditMode())
	//{
	//case CREATION_MODE:
	//	if (mode==GL_RENDER)
	//	{
	//		this->RenderText.glPrint("Creation Mode");
	//	}

	//	pDoc->GetMeshCreation().Render(pDoc->GetViewStyle(),mode,this->modelview,this->projection,this->viewport);

	//	break;
	//case EDITING_MODE:
	//	if (mode==GL_RENDER)
	//	{
	//		this->RenderText.glPrint("Editing Mode");
	//	}

	//	pDoc->GetMeshEditing().Render(mode);

	//	break;
	//case DEFORMATION_MODE:
	//	if (mode==GL_RENDER)
	//	{
	//		this->RenderText.glPrint("Deformation Mode");
	//	}
	//	pDoc->GetMeshDeformation().Render(pDoc->GetViewStyle(),mode,pDoc->IsDualMeshShown());
	//	break;
	//case EXTRUSION_MODE:
	//	if (mode==GL_RENDER)
	//	{
	//		this->RenderText.glPrint("Extrusion Mode");
	//	}
	//	pDoc->GetMeshExtrusion().Render(pDoc->GetViewStyle(),mode);
	//	break;
	//case CUTTING_MODE:
	//	this->RenderText.glPrint("Cutting Mode");
	//	pDoc->GetMeshCutting().Render(pDoc->GetViewStyle(),mode);
	//	break;
	//case SMOOTHING_MODE:
	//	this->RenderText.glPrint("Smoothing Mode");
	//	pDoc->GetMeshSmoothing().Render(pDoc->GetViewStyle(),
	//		this->modelview,this->projection,this->viewport,mode);
	//	break;
	//	//case TEST_MODE:
	//	//	this->RenderText.glPrint("Test Mode");
	//	//	pDoc->GetTest().Render(pDoc->GetViewStyle(),
	//	//		this->modelview,this->projection,this->viewport);
	//	//	break;
	//default:
	//	break;
	//}


	//if (pDoc->GetRenderPreMesh()==MESH_PREVIEW && !pDoc->GetMesh().empty())
	//{
	//	//the mesh is selected
	//	if (pDoc->GetRBSelName()==MODEL_NAME)
	//	{
	//		OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_SELECTED_TRANSPARENT,
	//			pDoc->GetDefaultColor(),mode);
	//	}
	//	else
	//	{
	//		OBJHandle::DrawCGALPolyhedron(&(pDoc->GetMesh()),pDoc->GetViewStyle(),COLOR_TRANSPARENT,//pDoc->GetColorMode(),
	//			pDoc->GetDefaultColor(),mode);
	//	}
	//}


	glPopMatrix();

	//QCursor* qCursor_Rotate;
	//QCursor* qCursor_Move;
	//QCursor* qCursor_Zoom;
	//QCursor* qCursor_PaintROI;
	//QCursor* qCursor_Smooth;
	//QCursor* qCursor_Pencil;

//	this->setCursor(*qCursor_Pencil);
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

