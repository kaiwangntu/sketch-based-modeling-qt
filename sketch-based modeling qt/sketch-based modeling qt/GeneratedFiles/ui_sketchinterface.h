/********************************************************************************
** Form generated from reading UI file 'sketchinterface.ui'
**
** Created: Tue Dec 18 11:26:05 2012
**      by: Qt User Interface Compiler version 4.7.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SKETCHINTERFACE_H
#define UI_SKETCHINTERFACE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QToolBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "sketchviewer.h"

QT_BEGIN_NAMESPACE

class Ui_SketchInterfaceClass
{
public:
    QWidget *centralWidget;
    SketchViewer *widget;
    QToolBox *toolBox;
    QWidget *Basics;
    QGroupBox *file;
    QWidget *layoutWidget_17;
    QVBoxLayout *verticalLayout_18;
    QPushButton *open;
    QPushButton *save;
    QPushButton *statistics;
    QPushButton *reset;
    QPushButton *cancelButton;
    QGroupBox *groupBox_3;
    QWidget *layoutWidget_18;
    QVBoxLayout *verticalLayout_22;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_16;
    QSlider *red;
    QHBoxLayout *horizontalLayout_20;
    QLabel *label_17;
    QSlider *green;
    QHBoxLayout *horizontalLayout_21;
    QLabel *label_18;
    QSlider *blue;
    QGroupBox *shading_3;
    QWidget *layoutWidget_19;
    QVBoxLayout *verticalLayout_23;
    QVBoxLayout *verticalLayout_24;
    QRadioButton *smooth;
    QRadioButton *flat;
    QVBoxLayout *verticalLayout_25;
    QCheckBox *solid;
    QCheckBox *wireframe;
    QCheckBox *curvature;
    QCheckBox *pca;
    QCheckBox *pcapoly;
    QGroupBox *shading_4;
    QWidget *layoutWidget_20;
    QVBoxLayout *verticalLayout_26;
    QCheckBox *FPSCheckBox;
    QCheckBox *GridCheckBox;
    QCheckBox *AxisCheckBox;
    QWidget *layoutWidget_21;
    QVBoxLayout *verticalLayout_27;
    QRadioButton *perspective;
    QRadioButton *Orthographic;
    QWidget *Creation;
    QGroupBox *ROI_2;
    QWidget *layoutWidget_22;
    QVBoxLayout *verticalLayout_28;
    QHBoxLayout *horizontalLayout_7;
    QSpinBox *radius;
    QLabel *label_19;
    QRadioButton *BFSvertex;
    QRadioButton *lasso3d;
    QRadioButton *lasso2d;
    QRadioButton *rectangle;
    QRadioButton *brush;
    QGroupBox *groupBox_6;
    QWidget *layoutWidget_23;
    QVBoxLayout *verticalLayout_29;
    QRadioButton *setface;
    QRadioButton *setvertex;
    QRadioButton *setedge;
    QGroupBox *groupBox_7;
    QWidget *layoutWidget_24;
    QVBoxLayout *verticalLayout_30;
    QRadioButton *nobollean;
    QRadioButton *ROI_UNION2;
    QRadioButton *difference;
    QRadioButton *intersection;
    QGroupBox *morphology_2;
    QWidget *layoutWidget_25;
    QVBoxLayout *verticalLayout_31;
    QHBoxLayout *horizontalLayout_8;
    QSpinBox *morraidus;
    QLabel *label_20;
    QVBoxLayout *verticalLayout_32;
    QPushButton *dialtion;
    QPushButton *Erosion;
    QGroupBox *others_2;
    QWidget *layoutWidget_26;
    QVBoxLayout *verticalLayout_33;
    QPushButton *selectall;
    QPushButton *selectreverse;
    QPushButton *cancelselect;
    QWidget *Edit;
    QGroupBox *globaloperation_2;
    QWidget *layoutWidget_27;
    QVBoxLayout *verticalLayout_34;
    QLabel *label_21;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_22;
    QSlider *horizontalSlider;
    QLabel *label_23;
    QPushButton *decimate;
    QComboBox *component;
    QHBoxLayout *horizontalLayout_10;
    QSpinBox *iteration_4;
    QLabel *label_24;
    QHBoxLayout *horizontalLayout_12;
    QSpinBox *continuity;
    QLabel *label_25;
    QPushButton *globalsmooth;
    QComboBox *lsoweights;
    QLabel *anchorpoints_3;
    QHBoxLayout *horizontalLayout_22;
    QLabel *label_26;
    QSlider *anchorpoints_4;
    QLabel *label_27;
    QPushButton *lsosmooth;
    QPushButton *Regularization;
    QGroupBox *roioperation_2;
    QWidget *layoutWidget_28;
    QVBoxLayout *verticalLayout_35;
    QComboBox *smoothtype;
    QPushButton *localsmooth;
    QVBoxLayout *verticalLayout_36;
    QPushButton *removevertex;
    QPushButton *removeface;
    QGroupBox *singleedge_2;
    QWidget *layoutWidget_29;
    QVBoxLayout *verticalLayout_37;
    QPushButton *collapseedge;
    QPushButton *splitedge;
    QPushButton *flipedge;
    QPushButton *removeedge;
    QWidget *Test;
    QGroupBox *selection_2;
    QWidget *layoutWidget_30;
    QVBoxLayout *verticalLayout_38;
    QPushButton *anchorpts;
    QPushButton *controlpts;
    QPushButton *done;
    QPushButton *cancelall;
    QGroupBox *rotation_2;
    QWidget *layoutWidget_31;
    QVBoxLayout *verticalLayout_39;
    QHBoxLayout *horizontalLayout_23;
    QLabel *label_28;
    QSlider *xrotation;
    QHBoxLayout *horizontalLayout_24;
    QLabel *label_29;
    QSlider *yrotation;
    QHBoxLayout *horizontalLayout_25;
    QLabel *label_30;
    QSlider *zrotation;
    QGroupBox *groupBox_8;
    QWidget *layoutWidget_32;
    QVBoxLayout *verticalLayout_40;
    QLabel *iteration_5;
    QSpinBox *iteration_6;
    QWidget *layoutWidget_3;
    QHBoxLayout *horizontalLayout_13;
    QRadioButton *ViewSelect;
    QRadioButton *Sketch;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SketchInterfaceClass)
    {
        if (SketchInterfaceClass->objectName().isEmpty())
            SketchInterfaceClass->setObjectName(QString::fromUtf8("SketchInterfaceClass"));
        SketchInterfaceClass->resize(1265, 920);
        centralWidget = new QWidget(SketchInterfaceClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        widget = new SketchViewer(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(0, 50, 1101, 811));
        toolBox = new QToolBox(centralWidget);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        toolBox->setGeometry(QRect(1110, 10, 151, 851));
        Basics = new QWidget();
        Basics->setObjectName(QString::fromUtf8("Basics"));
        Basics->setGeometry(QRect(0, 0, 98, 28));
        file = new QGroupBox(Basics);
        file->setObjectName(QString::fromUtf8("file"));
        file->setGeometry(QRect(1, 11, 149, 201));
        layoutWidget_17 = new QWidget(file);
        layoutWidget_17->setObjectName(QString::fromUtf8("layoutWidget_17"));
        layoutWidget_17->setGeometry(QRect(30, 20, 79, 171));
        verticalLayout_18 = new QVBoxLayout(layoutWidget_17);
        verticalLayout_18->setSpacing(6);
        verticalLayout_18->setContentsMargins(11, 11, 11, 11);
        verticalLayout_18->setObjectName(QString::fromUtf8("verticalLayout_18"));
        verticalLayout_18->setContentsMargins(0, 0, 0, 0);
        open = new QPushButton(layoutWidget_17);
        open->setObjectName(QString::fromUtf8("open"));

        verticalLayout_18->addWidget(open);

        save = new QPushButton(layoutWidget_17);
        save->setObjectName(QString::fromUtf8("save"));

        verticalLayout_18->addWidget(save);

        statistics = new QPushButton(layoutWidget_17);
        statistics->setObjectName(QString::fromUtf8("statistics"));

        verticalLayout_18->addWidget(statistics);

        reset = new QPushButton(layoutWidget_17);
        reset->setObjectName(QString::fromUtf8("reset"));

        verticalLayout_18->addWidget(reset);

        cancelButton = new QPushButton(layoutWidget_17);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        verticalLayout_18->addWidget(cancelButton);

        groupBox_3 = new QGroupBox(Basics);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(1, 223, 149, 111));
        layoutWidget_18 = new QWidget(groupBox_3);
        layoutWidget_18->setObjectName(QString::fromUtf8("layoutWidget_18"));
        layoutWidget_18->setGeometry(QRect(20, 30, 111, 77));
        verticalLayout_22 = new QVBoxLayout(layoutWidget_18);
        verticalLayout_22->setSpacing(6);
        verticalLayout_22->setContentsMargins(11, 11, 11, 11);
        verticalLayout_22->setObjectName(QString::fromUtf8("verticalLayout_22"));
        verticalLayout_22->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_16 = new QLabel(layoutWidget_18);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout_19->addWidget(label_16);

        red = new QSlider(layoutWidget_18);
        red->setObjectName(QString::fromUtf8("red"));
        red->setMaximum(99);
        red->setSingleStep(1);
        red->setValue(80);
        red->setOrientation(Qt::Horizontal);

        horizontalLayout_19->addWidget(red);


        verticalLayout_22->addLayout(horizontalLayout_19);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        label_17 = new QLabel(layoutWidget_18);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_20->addWidget(label_17);

        green = new QSlider(layoutWidget_18);
        green->setObjectName(QString::fromUtf8("green"));
        green->setMaximum(99);
        green->setValue(99);
        green->setOrientation(Qt::Horizontal);

        horizontalLayout_20->addWidget(green);


        verticalLayout_22->addLayout(horizontalLayout_20);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        label_18 = new QLabel(layoutWidget_18);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_21->addWidget(label_18);

        blue = new QSlider(layoutWidget_18);
        blue->setObjectName(QString::fromUtf8("blue"));
        blue->setValue(99);
        blue->setOrientation(Qt::Horizontal);

        horizontalLayout_21->addWidget(blue);


        verticalLayout_22->addLayout(horizontalLayout_21);

        shading_3 = new QGroupBox(Basics);
        shading_3->setObjectName(QString::fromUtf8("shading_3"));
        shading_3->setGeometry(QRect(1, 340, 149, 221));
        layoutWidget_19 = new QWidget(shading_3);
        layoutWidget_19->setObjectName(QString::fromUtf8("layoutWidget_19"));
        layoutWidget_19->setGeometry(QRect(20, 20, 116, 196));
        verticalLayout_23 = new QVBoxLayout(layoutWidget_19);
        verticalLayout_23->setSpacing(6);
        verticalLayout_23->setContentsMargins(11, 11, 11, 11);
        verticalLayout_23->setObjectName(QString::fromUtf8("verticalLayout_23"));
        verticalLayout_23->setContentsMargins(0, 0, 0, 0);
        verticalLayout_24 = new QVBoxLayout();
        verticalLayout_24->setSpacing(6);
        verticalLayout_24->setObjectName(QString::fromUtf8("verticalLayout_24"));
        smooth = new QRadioButton(layoutWidget_19);
        smooth->setObjectName(QString::fromUtf8("smooth"));
        smooth->setChecked(true);

        verticalLayout_24->addWidget(smooth);

        flat = new QRadioButton(layoutWidget_19);
        flat->setObjectName(QString::fromUtf8("flat"));

        verticalLayout_24->addWidget(flat);


        verticalLayout_23->addLayout(verticalLayout_24);

        verticalLayout_25 = new QVBoxLayout();
        verticalLayout_25->setSpacing(6);
        verticalLayout_25->setObjectName(QString::fromUtf8("verticalLayout_25"));
        solid = new QCheckBox(layoutWidget_19);
        solid->setObjectName(QString::fromUtf8("solid"));
        solid->setChecked(true);

        verticalLayout_25->addWidget(solid);

        wireframe = new QCheckBox(layoutWidget_19);
        wireframe->setObjectName(QString::fromUtf8("wireframe"));

        verticalLayout_25->addWidget(wireframe);

        curvature = new QCheckBox(layoutWidget_19);
        curvature->setObjectName(QString::fromUtf8("curvature"));

        verticalLayout_25->addWidget(curvature);

        pca = new QCheckBox(layoutWidget_19);
        pca->setObjectName(QString::fromUtf8("pca"));

        verticalLayout_25->addWidget(pca);

        pcapoly = new QCheckBox(layoutWidget_19);
        pcapoly->setObjectName(QString::fromUtf8("pcapoly"));

        verticalLayout_25->addWidget(pcapoly);


        verticalLayout_23->addLayout(verticalLayout_25);

        shading_4 = new QGroupBox(Basics);
        shading_4->setObjectName(QString::fromUtf8("shading_4"));
        shading_4->setGeometry(QRect(0, 570, 149, 171));
        layoutWidget_20 = new QWidget(shading_4);
        layoutWidget_20->setObjectName(QString::fromUtf8("layoutWidget_20"));
        layoutWidget_20->setGeometry(QRect(50, 20, 53, 84));
        verticalLayout_26 = new QVBoxLayout(layoutWidget_20);
        verticalLayout_26->setSpacing(6);
        verticalLayout_26->setContentsMargins(11, 11, 11, 11);
        verticalLayout_26->setObjectName(QString::fromUtf8("verticalLayout_26"));
        verticalLayout_26->setContentsMargins(0, 0, 0, 0);
        FPSCheckBox = new QCheckBox(layoutWidget_20);
        FPSCheckBox->setObjectName(QString::fromUtf8("FPSCheckBox"));

        verticalLayout_26->addWidget(FPSCheckBox);

        GridCheckBox = new QCheckBox(layoutWidget_20);
        GridCheckBox->setObjectName(QString::fromUtf8("GridCheckBox"));

        verticalLayout_26->addWidget(GridCheckBox);

        AxisCheckBox = new QCheckBox(layoutWidget_20);
        AxisCheckBox->setObjectName(QString::fromUtf8("AxisCheckBox"));

        verticalLayout_26->addWidget(AxisCheckBox);

        layoutWidget_21 = new QWidget(shading_4);
        layoutWidget_21->setObjectName(QString::fromUtf8("layoutWidget_21"));
        layoutWidget_21->setGeometry(QRect(30, 110, 99, 52));
        verticalLayout_27 = new QVBoxLayout(layoutWidget_21);
        verticalLayout_27->setSpacing(6);
        verticalLayout_27->setContentsMargins(11, 11, 11, 11);
        verticalLayout_27->setObjectName(QString::fromUtf8("verticalLayout_27"));
        verticalLayout_27->setContentsMargins(0, 0, 0, 0);
        perspective = new QRadioButton(layoutWidget_21);
        perspective->setObjectName(QString::fromUtf8("perspective"));
        perspective->setChecked(true);

        verticalLayout_27->addWidget(perspective);

        Orthographic = new QRadioButton(layoutWidget_21);
        Orthographic->setObjectName(QString::fromUtf8("Orthographic"));
        Orthographic->setChecked(false);

        verticalLayout_27->addWidget(Orthographic);

        toolBox->addItem(Basics, QString::fromUtf8("Basics"));
        Creation = new QWidget();
        Creation->setObjectName(QString::fromUtf8("Creation"));
        Creation->setGeometry(QRect(0, 0, 98, 28));
        ROI_2 = new QGroupBox(Creation);
        ROI_2->setObjectName(QString::fromUtf8("ROI_2"));
        ROI_2->setGeometry(QRect(1, 10, 149, 191));
        layoutWidget_22 = new QWidget(ROI_2);
        layoutWidget_22->setObjectName(QString::fromUtf8("layoutWidget_22"));
        layoutWidget_22->setGeometry(QRect(20, 20, 98, 167));
        verticalLayout_28 = new QVBoxLayout(layoutWidget_22);
        verticalLayout_28->setSpacing(6);
        verticalLayout_28->setContentsMargins(11, 11, 11, 11);
        verticalLayout_28->setObjectName(QString::fromUtf8("verticalLayout_28"));
        verticalLayout_28->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        radius = new QSpinBox(layoutWidget_22);
        radius->setObjectName(QString::fromUtf8("radius"));
        radius->setMinimum(0);
        radius->setValue(10);

        horizontalLayout_7->addWidget(radius);

        label_19 = new QLabel(layoutWidget_22);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_7->addWidget(label_19);


        verticalLayout_28->addLayout(horizontalLayout_7);

        BFSvertex = new QRadioButton(layoutWidget_22);
        BFSvertex->setObjectName(QString::fromUtf8("BFSvertex"));
        BFSvertex->setChecked(false);

        verticalLayout_28->addWidget(BFSvertex);

        lasso3d = new QRadioButton(layoutWidget_22);
        lasso3d->setObjectName(QString::fromUtf8("lasso3d"));

        verticalLayout_28->addWidget(lasso3d);

        lasso2d = new QRadioButton(layoutWidget_22);
        lasso2d->setObjectName(QString::fromUtf8("lasso2d"));
        lasso2d->setChecked(true);

        verticalLayout_28->addWidget(lasso2d);

        rectangle = new QRadioButton(layoutWidget_22);
        rectangle->setObjectName(QString::fromUtf8("rectangle"));

        verticalLayout_28->addWidget(rectangle);

        brush = new QRadioButton(layoutWidget_22);
        brush->setObjectName(QString::fromUtf8("brush"));

        verticalLayout_28->addWidget(brush);

        groupBox_6 = new QGroupBox(Creation);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(1, 210, 149, 101));
        layoutWidget_23 = new QWidget(groupBox_6);
        layoutWidget_23->setObjectName(QString::fromUtf8("layoutWidget_23"));
        layoutWidget_23->setGeometry(QRect(40, 20, 61, 80));
        verticalLayout_29 = new QVBoxLayout(layoutWidget_23);
        verticalLayout_29->setSpacing(6);
        verticalLayout_29->setContentsMargins(11, 11, 11, 11);
        verticalLayout_29->setObjectName(QString::fromUtf8("verticalLayout_29"));
        verticalLayout_29->setContentsMargins(0, 0, 0, 0);
        setface = new QRadioButton(layoutWidget_23);
        setface->setObjectName(QString::fromUtf8("setface"));
        setface->setChecked(true);

        verticalLayout_29->addWidget(setface);

        setvertex = new QRadioButton(layoutWidget_23);
        setvertex->setObjectName(QString::fromUtf8("setvertex"));

        verticalLayout_29->addWidget(setvertex);

        setedge = new QRadioButton(layoutWidget_23);
        setedge->setObjectName(QString::fromUtf8("setedge"));

        verticalLayout_29->addWidget(setedge);

        groupBox_7 = new QGroupBox(Creation);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        groupBox_7->setGeometry(QRect(1, 320, 149, 141));
        layoutWidget_24 = new QWidget(groupBox_7);
        layoutWidget_24->setObjectName(QString::fromUtf8("layoutWidget_24"));
        layoutWidget_24->setGeometry(QRect(20, 30, 93, 108));
        verticalLayout_30 = new QVBoxLayout(layoutWidget_24);
        verticalLayout_30->setSpacing(6);
        verticalLayout_30->setContentsMargins(11, 11, 11, 11);
        verticalLayout_30->setObjectName(QString::fromUtf8("verticalLayout_30"));
        verticalLayout_30->setContentsMargins(0, 0, 0, 0);
        nobollean = new QRadioButton(layoutWidget_24);
        nobollean->setObjectName(QString::fromUtf8("nobollean"));
        nobollean->setChecked(true);

        verticalLayout_30->addWidget(nobollean);

        ROI_UNION2 = new QRadioButton(layoutWidget_24);
        ROI_UNION2->setObjectName(QString::fromUtf8("ROI_UNION2"));

        verticalLayout_30->addWidget(ROI_UNION2);

        difference = new QRadioButton(layoutWidget_24);
        difference->setObjectName(QString::fromUtf8("difference"));

        verticalLayout_30->addWidget(difference);

        intersection = new QRadioButton(layoutWidget_24);
        intersection->setObjectName(QString::fromUtf8("intersection"));

        verticalLayout_30->addWidget(intersection);

        morphology_2 = new QGroupBox(Creation);
        morphology_2->setObjectName(QString::fromUtf8("morphology_2"));
        morphology_2->setGeometry(QRect(1, 470, 149, 121));
        layoutWidget_25 = new QWidget(morphology_2);
        layoutWidget_25->setObjectName(QString::fromUtf8("layoutWidget_25"));
        layoutWidget_25->setGeometry(QRect(20, 20, 98, 99));
        verticalLayout_31 = new QVBoxLayout(layoutWidget_25);
        verticalLayout_31->setSpacing(6);
        verticalLayout_31->setContentsMargins(11, 11, 11, 11);
        verticalLayout_31->setObjectName(QString::fromUtf8("verticalLayout_31"));
        verticalLayout_31->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        morraidus = new QSpinBox(layoutWidget_25);
        morraidus->setObjectName(QString::fromUtf8("morraidus"));
        morraidus->setMinimum(1);
        morraidus->setMaximum(50);

        horizontalLayout_8->addWidget(morraidus);

        label_20 = new QLabel(layoutWidget_25);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        horizontalLayout_8->addWidget(label_20);


        verticalLayout_31->addLayout(horizontalLayout_8);

        verticalLayout_32 = new QVBoxLayout();
        verticalLayout_32->setSpacing(6);
        verticalLayout_32->setObjectName(QString::fromUtf8("verticalLayout_32"));
        dialtion = new QPushButton(layoutWidget_25);
        dialtion->setObjectName(QString::fromUtf8("dialtion"));

        verticalLayout_32->addWidget(dialtion);

        Erosion = new QPushButton(layoutWidget_25);
        Erosion->setObjectName(QString::fromUtf8("Erosion"));

        verticalLayout_32->addWidget(Erosion);


        verticalLayout_31->addLayout(verticalLayout_32);

        others_2 = new QGroupBox(Creation);
        others_2->setObjectName(QString::fromUtf8("others_2"));
        others_2->setGeometry(QRect(0, 600, 149, 141));
        layoutWidget_26 = new QWidget(others_2);
        layoutWidget_26->setObjectName(QString::fromUtf8("layoutWidget_26"));
        layoutWidget_26->setGeometry(QRect(30, 30, 79, 101));
        verticalLayout_33 = new QVBoxLayout(layoutWidget_26);
        verticalLayout_33->setSpacing(6);
        verticalLayout_33->setContentsMargins(11, 11, 11, 11);
        verticalLayout_33->setObjectName(QString::fromUtf8("verticalLayout_33"));
        verticalLayout_33->setContentsMargins(0, 0, 0, 0);
        selectall = new QPushButton(layoutWidget_26);
        selectall->setObjectName(QString::fromUtf8("selectall"));

        verticalLayout_33->addWidget(selectall);

        selectreverse = new QPushButton(layoutWidget_26);
        selectreverse->setObjectName(QString::fromUtf8("selectreverse"));

        verticalLayout_33->addWidget(selectreverse);

        cancelselect = new QPushButton(layoutWidget_26);
        cancelselect->setObjectName(QString::fromUtf8("cancelselect"));

        verticalLayout_33->addWidget(cancelselect);

        toolBox->addItem(Creation, QString::fromUtf8("Creation"));
        Edit = new QWidget();
        Edit->setObjectName(QString::fromUtf8("Edit"));
        Edit->setGeometry(QRect(0, 0, 151, 743));
        globaloperation_2 = new QGroupBox(Edit);
        globaloperation_2->setObjectName(QString::fromUtf8("globaloperation_2"));
        globaloperation_2->setGeometry(QRect(1, 2, 149, 381));
        layoutWidget_27 = new QWidget(globaloperation_2);
        layoutWidget_27->setObjectName(QString::fromUtf8("layoutWidget_27"));
        layoutWidget_27->setGeometry(QRect(10, 20, 131, 352));
        verticalLayout_34 = new QVBoxLayout(layoutWidget_27);
        verticalLayout_34->setSpacing(6);
        verticalLayout_34->setContentsMargins(11, 11, 11, 11);
        verticalLayout_34->setObjectName(QString::fromUtf8("verticalLayout_34"));
        verticalLayout_34->setContentsMargins(0, 0, 0, 0);
        label_21 = new QLabel(layoutWidget_27);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        verticalLayout_34->addWidget(label_21);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_22 = new QLabel(layoutWidget_27);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        horizontalLayout_9->addWidget(label_22);

        horizontalSlider = new QSlider(layoutWidget_27);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setMinimum(1);
        horizontalSlider->setMaximum(50);
        horizontalSlider->setValue(10);
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_9->addWidget(horizontalSlider);

        label_23 = new QLabel(layoutWidget_27);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        horizontalLayout_9->addWidget(label_23);


        verticalLayout_34->addLayout(horizontalLayout_9);

        decimate = new QPushButton(layoutWidget_27);
        decimate->setObjectName(QString::fromUtf8("decimate"));

        verticalLayout_34->addWidget(decimate);

        component = new QComboBox(layoutWidget_27);
        component->setObjectName(QString::fromUtf8("component"));

        verticalLayout_34->addWidget(component);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        iteration_4 = new QSpinBox(layoutWidget_27);
        iteration_4->setObjectName(QString::fromUtf8("iteration_4"));
        iteration_4->setMinimum(1);
        iteration_4->setMaximum(999);
        iteration_4->setValue(10);

        horizontalLayout_10->addWidget(iteration_4);

        label_24 = new QLabel(layoutWidget_27);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        horizontalLayout_10->addWidget(label_24);


        verticalLayout_34->addLayout(horizontalLayout_10);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        continuity = new QSpinBox(layoutWidget_27);
        continuity->setObjectName(QString::fromUtf8("continuity"));
        continuity->setMaximum(2);
        continuity->setValue(1);

        horizontalLayout_12->addWidget(continuity);

        label_25 = new QLabel(layoutWidget_27);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        horizontalLayout_12->addWidget(label_25);


        verticalLayout_34->addLayout(horizontalLayout_12);

        globalsmooth = new QPushButton(layoutWidget_27);
        globalsmooth->setObjectName(QString::fromUtf8("globalsmooth"));

        verticalLayout_34->addWidget(globalsmooth);

        lsoweights = new QComboBox(layoutWidget_27);
        lsoweights->setObjectName(QString::fromUtf8("lsoweights"));

        verticalLayout_34->addWidget(lsoweights);

        anchorpoints_3 = new QLabel(layoutWidget_27);
        anchorpoints_3->setObjectName(QString::fromUtf8("anchorpoints_3"));

        verticalLayout_34->addWidget(anchorpoints_3);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        label_26 = new QLabel(layoutWidget_27);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        horizontalLayout_22->addWidget(label_26);

        anchorpoints_4 = new QSlider(layoutWidget_27);
        anchorpoints_4->setObjectName(QString::fromUtf8("anchorpoints_4"));
        anchorpoints_4->setMinimum(1);
        anchorpoints_4->setMaximum(100);
        anchorpoints_4->setValue(10);
        anchorpoints_4->setOrientation(Qt::Horizontal);

        horizontalLayout_22->addWidget(anchorpoints_4);

        label_27 = new QLabel(layoutWidget_27);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        horizontalLayout_22->addWidget(label_27);


        verticalLayout_34->addLayout(horizontalLayout_22);

        lsosmooth = new QPushButton(layoutWidget_27);
        lsosmooth->setObjectName(QString::fromUtf8("lsosmooth"));

        verticalLayout_34->addWidget(lsosmooth);

        Regularization = new QPushButton(layoutWidget_27);
        Regularization->setObjectName(QString::fromUtf8("Regularization"));

        verticalLayout_34->addWidget(Regularization);

        roioperation_2 = new QGroupBox(Edit);
        roioperation_2->setObjectName(QString::fromUtf8("roioperation_2"));
        roioperation_2->setGeometry(QRect(1, 390, 149, 161));
        layoutWidget_28 = new QWidget(roioperation_2);
        layoutWidget_28->setObjectName(QString::fromUtf8("layoutWidget_28"));
        layoutWidget_28->setGeometry(QRect(10, 20, 131, 132));
        verticalLayout_35 = new QVBoxLayout(layoutWidget_28);
        verticalLayout_35->setSpacing(6);
        verticalLayout_35->setContentsMargins(11, 11, 11, 11);
        verticalLayout_35->setObjectName(QString::fromUtf8("verticalLayout_35"));
        verticalLayout_35->setContentsMargins(0, 0, 0, 0);
        smoothtype = new QComboBox(layoutWidget_28);
        smoothtype->setObjectName(QString::fromUtf8("smoothtype"));

        verticalLayout_35->addWidget(smoothtype);

        localsmooth = new QPushButton(layoutWidget_28);
        localsmooth->setObjectName(QString::fromUtf8("localsmooth"));

        verticalLayout_35->addWidget(localsmooth);

        verticalLayout_36 = new QVBoxLayout();
        verticalLayout_36->setSpacing(6);
        verticalLayout_36->setObjectName(QString::fromUtf8("verticalLayout_36"));
        removevertex = new QPushButton(layoutWidget_28);
        removevertex->setObjectName(QString::fromUtf8("removevertex"));

        verticalLayout_36->addWidget(removevertex);

        removeface = new QPushButton(layoutWidget_28);
        removeface->setObjectName(QString::fromUtf8("removeface"));

        verticalLayout_36->addWidget(removeface);


        verticalLayout_35->addLayout(verticalLayout_36);

        singleedge_2 = new QGroupBox(Edit);
        singleedge_2->setObjectName(QString::fromUtf8("singleedge_2"));
        singleedge_2->setGeometry(QRect(1, 570, 149, 171));
        layoutWidget_29 = new QWidget(singleedge_2);
        layoutWidget_29->setObjectName(QString::fromUtf8("layoutWidget_29"));
        layoutWidget_29->setGeometry(QRect(30, 20, 79, 136));
        verticalLayout_37 = new QVBoxLayout(layoutWidget_29);
        verticalLayout_37->setSpacing(6);
        verticalLayout_37->setContentsMargins(11, 11, 11, 11);
        verticalLayout_37->setObjectName(QString::fromUtf8("verticalLayout_37"));
        verticalLayout_37->setContentsMargins(0, 0, 0, 0);
        collapseedge = new QPushButton(layoutWidget_29);
        collapseedge->setObjectName(QString::fromUtf8("collapseedge"));

        verticalLayout_37->addWidget(collapseedge);

        splitedge = new QPushButton(layoutWidget_29);
        splitedge->setObjectName(QString::fromUtf8("splitedge"));

        verticalLayout_37->addWidget(splitedge);

        flipedge = new QPushButton(layoutWidget_29);
        flipedge->setObjectName(QString::fromUtf8("flipedge"));

        verticalLayout_37->addWidget(flipedge);

        removeedge = new QPushButton(layoutWidget_29);
        removeedge->setObjectName(QString::fromUtf8("removeedge"));

        verticalLayout_37->addWidget(removeedge);

        toolBox->addItem(Edit, QString::fromUtf8("Edit"));
        Test = new QWidget();
        Test->setObjectName(QString::fromUtf8("Test"));
        Test->setGeometry(QRect(0, 0, 98, 28));
        selection_2 = new QGroupBox(Test);
        selection_2->setObjectName(QString::fromUtf8("selection_2"));
        selection_2->setGeometry(QRect(10, 10, 131, 181));
        layoutWidget_30 = new QWidget(selection_2);
        layoutWidget_30->setObjectName(QString::fromUtf8("layoutWidget_30"));
        layoutWidget_30->setGeometry(QRect(20, 20, 91, 151));
        verticalLayout_38 = new QVBoxLayout(layoutWidget_30);
        verticalLayout_38->setSpacing(6);
        verticalLayout_38->setContentsMargins(11, 11, 11, 11);
        verticalLayout_38->setObjectName(QString::fromUtf8("verticalLayout_38"));
        verticalLayout_38->setContentsMargins(0, 0, 0, 0);
        anchorpts = new QPushButton(layoutWidget_30);
        anchorpts->setObjectName(QString::fromUtf8("anchorpts"));

        verticalLayout_38->addWidget(anchorpts);

        controlpts = new QPushButton(layoutWidget_30);
        controlpts->setObjectName(QString::fromUtf8("controlpts"));

        verticalLayout_38->addWidget(controlpts);

        done = new QPushButton(layoutWidget_30);
        done->setObjectName(QString::fromUtf8("done"));

        verticalLayout_38->addWidget(done);

        cancelall = new QPushButton(layoutWidget_30);
        cancelall->setObjectName(QString::fromUtf8("cancelall"));

        verticalLayout_38->addWidget(cancelall);

        rotation_2 = new QGroupBox(Test);
        rotation_2->setObjectName(QString::fromUtf8("rotation_2"));
        rotation_2->setGeometry(QRect(10, 200, 131, 131));
        layoutWidget_31 = new QWidget(rotation_2);
        layoutWidget_31->setObjectName(QString::fromUtf8("layoutWidget_31"));
        layoutWidget_31->setGeometry(QRect(11, 35, 111, 77));
        verticalLayout_39 = new QVBoxLayout(layoutWidget_31);
        verticalLayout_39->setSpacing(6);
        verticalLayout_39->setContentsMargins(11, 11, 11, 11);
        verticalLayout_39->setObjectName(QString::fromUtf8("verticalLayout_39"));
        verticalLayout_39->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_23 = new QHBoxLayout();
        horizontalLayout_23->setSpacing(6);
        horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
        label_28 = new QLabel(layoutWidget_31);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        horizontalLayout_23->addWidget(label_28);

        xrotation = new QSlider(layoutWidget_31);
        xrotation->setObjectName(QString::fromUtf8("xrotation"));
        xrotation->setMaximum(360);
        xrotation->setSingleStep(10);
        xrotation->setOrientation(Qt::Horizontal);

        horizontalLayout_23->addWidget(xrotation);


        verticalLayout_39->addLayout(horizontalLayout_23);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        label_29 = new QLabel(layoutWidget_31);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        horizontalLayout_24->addWidget(label_29);

        yrotation = new QSlider(layoutWidget_31);
        yrotation->setObjectName(QString::fromUtf8("yrotation"));
        yrotation->setMaximum(360);
        yrotation->setSingleStep(10);
        yrotation->setOrientation(Qt::Horizontal);

        horizontalLayout_24->addWidget(yrotation);


        verticalLayout_39->addLayout(horizontalLayout_24);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        label_30 = new QLabel(layoutWidget_31);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        horizontalLayout_25->addWidget(label_30);

        zrotation = new QSlider(layoutWidget_31);
        zrotation->setObjectName(QString::fromUtf8("zrotation"));
        zrotation->setMaximum(360);
        zrotation->setSingleStep(10);
        zrotation->setOrientation(Qt::Horizontal);

        horizontalLayout_25->addWidget(zrotation);


        verticalLayout_39->addLayout(horizontalLayout_25);

        groupBox_8 = new QGroupBox(Test);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        groupBox_8->setGeometry(QRect(10, 350, 131, 101));
        layoutWidget_32 = new QWidget(groupBox_8);
        layoutWidget_32->setObjectName(QString::fromUtf8("layoutWidget_32"));
        layoutWidget_32->setGeometry(QRect(11, 30, 109, 48));
        verticalLayout_40 = new QVBoxLayout(layoutWidget_32);
        verticalLayout_40->setSpacing(6);
        verticalLayout_40->setContentsMargins(11, 11, 11, 11);
        verticalLayout_40->setObjectName(QString::fromUtf8("verticalLayout_40"));
        verticalLayout_40->setContentsMargins(0, 0, 0, 0);
        iteration_5 = new QLabel(layoutWidget_32);
        iteration_5->setObjectName(QString::fromUtf8("iteration_5"));

        verticalLayout_40->addWidget(iteration_5);

        iteration_6 = new QSpinBox(layoutWidget_32);
        iteration_6->setObjectName(QString::fromUtf8("iteration_6"));
        iteration_6->setMinimum(0);
        iteration_6->setMaximum(1000);
        iteration_6->setValue(0);

        verticalLayout_40->addWidget(iteration_6);

        toolBox->addItem(Test, QString::fromUtf8("Test"));
        layoutWidget_3 = new QWidget(centralWidget);
        layoutWidget_3->setObjectName(QString::fromUtf8("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(10, 10, 221, 31));
        horizontalLayout_13 = new QHBoxLayout(layoutWidget_3);
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        horizontalLayout_13->setContentsMargins(0, 0, 0, 0);
        ViewSelect = new QRadioButton(layoutWidget_3);
        ViewSelect->setObjectName(QString::fromUtf8("ViewSelect"));
        ViewSelect->setLayoutDirection(Qt::LeftToRight);
        ViewSelect->setChecked(true);

        horizontalLayout_13->addWidget(ViewSelect);

        Sketch = new QRadioButton(layoutWidget_3);
        Sketch->setObjectName(QString::fromUtf8("Sketch"));

        horizontalLayout_13->addWidget(Sketch);

        SketchInterfaceClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SketchInterfaceClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1265, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        SketchInterfaceClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(SketchInterfaceClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        SketchInterfaceClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(SketchInterfaceClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        SketchInterfaceClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());

        retranslateUi(SketchInterfaceClass);

        toolBox->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(SketchInterfaceClass);
    } // setupUi

    void retranslateUi(QMainWindow *SketchInterfaceClass)
    {
        SketchInterfaceClass->setWindowTitle(QApplication::translate("SketchInterfaceClass", "SketchInterface", 0, QApplication::UnicodeUTF8));
        file->setTitle(QApplication::translate("SketchInterfaceClass", "File", 0, QApplication::UnicodeUTF8));
        open->setText(QApplication::translate("SketchInterfaceClass", "Open", 0, QApplication::UnicodeUTF8));
        save->setText(QApplication::translate("SketchInterfaceClass", "Save", 0, QApplication::UnicodeUTF8));
        statistics->setText(QApplication::translate("SketchInterfaceClass", "Statistics", 0, QApplication::UnicodeUTF8));
        reset->setText(QApplication::translate("SketchInterfaceClass", "Reset", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("SketchInterfaceClass", "Quit", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SketchInterfaceClass", "Color-RGB", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("SketchInterfaceClass", "R", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("SketchInterfaceClass", "G", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("SketchInterfaceClass", "B", 0, QApplication::UnicodeUTF8));
        shading_3->setTitle(QApplication::translate("SketchInterfaceClass", "Rendering", 0, QApplication::UnicodeUTF8));
        smooth->setText(QApplication::translate("SketchInterfaceClass", "Smooth", 0, QApplication::UnicodeUTF8));
        flat->setText(QApplication::translate("SketchInterfaceClass", "Flat", 0, QApplication::UnicodeUTF8));
        solid->setText(QApplication::translate("SketchInterfaceClass", "Solid", 0, QApplication::UnicodeUTF8));
        wireframe->setText(QApplication::translate("SketchInterfaceClass", "Wireframe", 0, QApplication::UnicodeUTF8));
        curvature->setText(QApplication::translate("SketchInterfaceClass", "Curvature", 0, QApplication::UnicodeUTF8));
        pca->setText(QApplication::translate("SketchInterfaceClass", "PCA OBB", 0, QApplication::UnicodeUTF8));
        pcapoly->setText(QApplication::translate("SketchInterfaceClass", "Local Fitting", 0, QApplication::UnicodeUTF8));
        shading_4->setTitle(QApplication::translate("SketchInterfaceClass", "Others", 0, QApplication::UnicodeUTF8));
        FPSCheckBox->setText(QApplication::translate("SketchInterfaceClass", "FPS", 0, QApplication::UnicodeUTF8));
        GridCheckBox->setText(QApplication::translate("SketchInterfaceClass", "Grid", 0, QApplication::UnicodeUTF8));
        AxisCheckBox->setText(QApplication::translate("SketchInterfaceClass", "Axis", 0, QApplication::UnicodeUTF8));
        perspective->setText(QApplication::translate("SketchInterfaceClass", "Perspective", 0, QApplication::UnicodeUTF8));
        Orthographic->setText(QApplication::translate("SketchInterfaceClass", "Orthographic", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(Basics), QApplication::translate("SketchInterfaceClass", "Basics", 0, QApplication::UnicodeUTF8));
        ROI_2->setTitle(QApplication::translate("SketchInterfaceClass", "Methods", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("SketchInterfaceClass", "Radius", 0, QApplication::UnicodeUTF8));
        BFSvertex->setText(QApplication::translate("SketchInterfaceClass", "Single click", 0, QApplication::UnicodeUTF8));
        lasso3d->setText(QApplication::translate("SketchInterfaceClass", "Lasso 3D", 0, QApplication::UnicodeUTF8));
        lasso2d->setText(QApplication::translate("SketchInterfaceClass", "Lasso 2D", 0, QApplication::UnicodeUTF8));
        rectangle->setText(QApplication::translate("SketchInterfaceClass", "Rectangle", 0, QApplication::UnicodeUTF8));
        brush->setText(QApplication::translate("SketchInterfaceClass", "Brush", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("SketchInterfaceClass", "Elements", 0, QApplication::UnicodeUTF8));
        setface->setText(QApplication::translate("SketchInterfaceClass", "Face", 0, QApplication::UnicodeUTF8));
        setvertex->setText(QApplication::translate("SketchInterfaceClass", "Vertex", 0, QApplication::UnicodeUTF8));
        setedge->setText(QApplication::translate("SketchInterfaceClass", "Edge", 0, QApplication::UnicodeUTF8));
        groupBox_7->setTitle(QApplication::translate("SketchInterfaceClass", "Boolean", 0, QApplication::UnicodeUTF8));
        nobollean->setText(QApplication::translate("SketchInterfaceClass", "No boolean", 0, QApplication::UnicodeUTF8));
        ROI_UNION2->setText(QApplication::translate("SketchInterfaceClass", "Union", 0, QApplication::UnicodeUTF8));
        difference->setText(QApplication::translate("SketchInterfaceClass", "Difference", 0, QApplication::UnicodeUTF8));
        intersection->setText(QApplication::translate("SketchInterfaceClass", "Intersection", 0, QApplication::UnicodeUTF8));
        morphology_2->setTitle(QApplication::translate("SketchInterfaceClass", "Morphology", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("SketchInterfaceClass", "Radius", 0, QApplication::UnicodeUTF8));
        dialtion->setText(QApplication::translate("SketchInterfaceClass", "Dilation", 0, QApplication::UnicodeUTF8));
        Erosion->setText(QApplication::translate("SketchInterfaceClass", "Erosion", 0, QApplication::UnicodeUTF8));
        others_2->setTitle(QApplication::translate("SketchInterfaceClass", "Others", 0, QApplication::UnicodeUTF8));
        selectall->setText(QApplication::translate("SketchInterfaceClass", "All", 0, QApplication::UnicodeUTF8));
        selectreverse->setText(QApplication::translate("SketchInterfaceClass", "Reverse", 0, QApplication::UnicodeUTF8));
        cancelselect->setText(QApplication::translate("SketchInterfaceClass", "Cancel", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(Creation), QApplication::translate("SketchInterfaceClass", "Creation", 0, QApplication::UnicodeUTF8));
        globaloperation_2->setTitle(QApplication::translate("SketchInterfaceClass", "Global", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("SketchInterfaceClass", "Decimate percent", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("SketchInterfaceClass", "1", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("SketchInterfaceClass", "50", 0, QApplication::UnicodeUTF8));
        decimate->setText(QApplication::translate("SketchInterfaceClass", "Decimate", 0, QApplication::UnicodeUTF8));
        component->clear();
        component->insertItems(0, QStringList()
         << QApplication::translate("SketchInterfaceClass", "Tangential", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Normal", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Both", 0, QApplication::UnicodeUTF8)
        );
        label_24->setText(QApplication::translate("SketchInterfaceClass", "Iteration", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("SketchInterfaceClass", "Continuity", 0, QApplication::UnicodeUTF8));
        globalsmooth->setText(QApplication::translate("SketchInterfaceClass", "OpenMesh Smooth", 0, QApplication::UnicodeUTF8));
        lsoweights->clear();
        lsoweights->insertItems(0, QStringList()
         << QApplication::translate("SketchInterfaceClass", "Uniform weights", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Cotangent weights", 0, QApplication::UnicodeUTF8)
        );
        anchorpoints_3->setText(QApplication::translate("SketchInterfaceClass", "Anchor pts percent", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("SketchInterfaceClass", "1", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("SketchInterfaceClass", "100", 0, QApplication::UnicodeUTF8));
        lsosmooth->setText(QApplication::translate("SketchInterfaceClass", "LSO Smooth", 0, QApplication::UnicodeUTF8));
        Regularization->setText(QApplication::translate("SketchInterfaceClass", "Regularization", 0, QApplication::UnicodeUTF8));
        roioperation_2->setTitle(QApplication::translate("SketchInterfaceClass", "ROI", 0, QApplication::UnicodeUTF8));
        smoothtype->clear();
        smoothtype->insertItems(0, QStringList()
         << QApplication::translate("SketchInterfaceClass", "Uniform direction", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Cotangent direction", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Taubin smoothing", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SketchInterfaceClass", "Tangential", 0, QApplication::UnicodeUTF8)
        );
        localsmooth->setText(QApplication::translate("SketchInterfaceClass", "Local smooth", 0, QApplication::UnicodeUTF8));
        removevertex->setText(QApplication::translate("SketchInterfaceClass", "Remove faces", 0, QApplication::UnicodeUTF8));
        removeface->setText(QApplication::translate("SketchInterfaceClass", "Remove vertices", 0, QApplication::UnicodeUTF8));
        singleedge_2->setTitle(QApplication::translate("SketchInterfaceClass", "Single Edge", 0, QApplication::UnicodeUTF8));
        collapseedge->setText(QApplication::translate("SketchInterfaceClass", "Collapse", 0, QApplication::UnicodeUTF8));
        splitedge->setText(QApplication::translate("SketchInterfaceClass", "Split", 0, QApplication::UnicodeUTF8));
        flipedge->setText(QApplication::translate("SketchInterfaceClass", "Flip", 0, QApplication::UnicodeUTF8));
        removeedge->setText(QApplication::translate("SketchInterfaceClass", "Remove", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(Edit), QApplication::translate("SketchInterfaceClass", "Edit", 0, QApplication::UnicodeUTF8));
        selection_2->setTitle(QApplication::translate("SketchInterfaceClass", "Preprocess", 0, QApplication::UnicodeUTF8));
        anchorpts->setText(QApplication::translate("SketchInterfaceClass", "Anchor Pts", 0, QApplication::UnicodeUTF8));
        controlpts->setText(QApplication::translate("SketchInterfaceClass", "Control Pts", 0, QApplication::UnicodeUTF8));
        done->setText(QApplication::translate("SketchInterfaceClass", "Done", 0, QApplication::UnicodeUTF8));
        cancelall->setText(QApplication::translate("SketchInterfaceClass", "Cancel All", 0, QApplication::UnicodeUTF8));
        rotation_2->setTitle(QApplication::translate("SketchInterfaceClass", "Rotation", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("SketchInterfaceClass", "R", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("SketchInterfaceClass", "G", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("SketchInterfaceClass", "B", 0, QApplication::UnicodeUTF8));
        groupBox_8->setTitle(QApplication::translate("SketchInterfaceClass", "ARAP Deform", 0, QApplication::UnicodeUTF8));
        iteration_5->setText(QApplication::translate("SketchInterfaceClass", "Iteration ", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(Test), QApplication::translate("SketchInterfaceClass", "Test", 0, QApplication::UnicodeUTF8));
        ViewSelect->setText(QApplication::translate("SketchInterfaceClass", "View && Select", 0, QApplication::UnicodeUTF8));
        Sketch->setText(QApplication::translate("SketchInterfaceClass", "Sketch", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SketchInterfaceClass", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SketchInterfaceClass: public Ui_SketchInterfaceClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SKETCHINTERFACE_H
