#pragma once
#ifndef SYMBOLS_DEF_H
#define SYMBOLS_DEF_H

////type define
//typedef std::pair <int,int> Int_Int_Pair;


#define VIEW_SELECTION_MODE 0
#define SKETCH_MODE 1

#define NONE_SELECTED -1

#define CREATION_MODE 0
#define EDITING_MODE 1
#define DEFORMATION_MODE 2
#define EXTRUSION_MODE 3
#define CUTTING_MODE 4
#define SMOOTHING_MODE 5
#define TEST_MODE 6

#define COLOR_ORIGINAL 0x01
#define COLOR_MEAN_CURVATURE 0x02
#define COLOR_GAUSSIAN_CURVATURE 0x03
#define COLOR_DEFORMATION_MATERIAL 0x03
#define COLOR_TRANSPARENT 0x04
#define COLOR_SELECTED_TRANSPARENT 0x05
#define COLOR_SELECTED_OPAQUE 0x06

#define WIREFRAME_VIEW 0
#define SMOOTH_VIEW 1
#define HYBRID_VIEW 2
#define POINTS_VIEW 3

const GLfloat OPAQUE_SELECTED_COLOR[] = {0.54f, 0.0f, 1.0f, 1.0f};
const GLfloat TRANSPARENT_SELECTED_COLOR[] = {0.54f, 0.0f, 1.0f, 0.2f};
const GLfloat OPAQUE_CROSS_SECTION_COLOR[] = {1.0f, 1.0f, 0.0f, 1.0f};
const GLfloat TRANSPARENT_CROSS_SECTION_COLOR[] = {1.0f, 1.0f, 0.0f, 0.3f};

#define MESH_NOT_PREVIEW 0
#define MESH_PREVIEW 1
#define MESH_EXIST_VIEW 2

#define MOUSE_LEFT_BUTTON_HIT 0
#define MOUSE_RIGHT_BUTTON_HIT 1

const bool CREATION_PLANE_READY_FOR_DRAWING=true;
const bool CREATION_PLANE_NOT_READY_FOR_DRAWING=false;

#define MODEL_NAME 1000
#define CREATION_XOY_PLANE 0
#define CREATION_XOZ_PLANE 1
#define CREATION_YOZ_PLANE 2
#define CREATION_PLANE_NAME_BEGIN 0
#define CREATION_PLANE_NAME_END 2
#define CREATION_SKETCH_CURVE_NAME_BEGIN 2000
#define CREATION_SKETCH_CURVE_NAME_END 2099
#define CREATION_COMPUTE_CURVE_NAME_BEGIN 2100
#define CREATION_COMPUTE_CURVE_NAME_END 2199
#define CREATION_CURVE_NAME_BEGIN 2000
#define CREATION_CURVE_NAME_END 2199

#define DEFORMATION_NORM_PLANE 10
#define DEFORMATION_TAN_PLANE 11
#define DEFORMATION_BINORM_PLANE 12
#define DEFORMATION_PLANE_NAME_BEGIN 10
#define DEFORMATION_PLANE_NAME_END 12
#define DEFORMATION_HANDLE_CURVE 3000
#define DEFORMATION_DEFORM_CURVE 3001
#define DEFORMATION_DEFORM_PROJ_CURVE 3002
#define DEFORMATION_CURVE_NAME_BEGIN 3000
#define DEFORMATION_CURVE_NAME_END 3002
#define DEFORMATION_GESTURE_CIRCLE_ROI 3010//just a name, can't be selected,so >DEFORMATION_CURVE_NAME_END
#define DEFORMATION_GESTURE_PAINT_ROI 3011//just a name, can't be selected,so >DEFORMATION_CURVE_NAME_END

#define CUTTING_ACROSS_CURVE 4000
#define CUTTING_BITE_CURVE 4001

#define EXTRUSION_REF_PLANE 13
#define EXTRUSION_CLOSED_CURVE 5000
#define EXTRUSION_SILH_CURVE 5001

#define TIMER_PLANE_WAIT_TO_ROTATE 1 
#define TIMER_PLANE_ROTATE 2
//time interval between user action and auto plane rotation
#define INTERVAL_BEFORE_ROTATE 1000
//interval between each plane rotation
#define INTERVAL_BETWEEN_ROTATE 10

const int CREATION_PLANE_BOUND_SIZE=2;

//Tao Ju's surface reconstruction algorithm
const int TaoJuSurfReconstAlgo=0;
//our progressive surface reconstruction algorithm
const int ProgSurfReconstAlgo=1;

//render cross section cylinder in our progressive surface reconstruction algorithm
const int CR_RENDER_ALL_CYLINDER=99999; 
const int CR_RENDER_NONE_CYLINDER=-1; 

#endif