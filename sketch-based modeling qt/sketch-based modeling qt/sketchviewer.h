#ifndef SKETCHVIEWER_H
#define SKETCHVIEWER_H

#include <QGLWidget>

#include "GeometryAlgorithm.h"

class SketchViewer : public QGLWidget
{
	Q_OBJECT

public:
	SketchViewer(QWidget *parent);
	~SketchViewer();

	void resizeGL(int width, int height);

protected:
	void initializeGL();
	void paintGL();
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void timerEvent(QTimerEvent *event);


private:
	
};

#endif // SKETCHVIEWER_H
