//
//  systemStruct.h
//  MedianFlow
//
//  Created by 陈裕昕 on 14/11/8.
//  Copyright (c) 2014年 陈裕昕. All rights reserved.
//

#ifndef MedianFlow_systemStruct_h
#define MedianFlow_systemStruct_h

#include <opencv2/opencv.hpp>

using namespace cv;

typedef float TYPE_OF_COORD;
typedef TYPE_OF_COORD TYPE_MF_COORD;

typedef Point_<TYPE_OF_COORD> TYPE_OF_PT;
typedef TYPE_OF_PT TYPE_MF_PT;
typedef Rect_<TYPE_MF_COORD> TYPE_MF_BB;

static const bool OF_USE_OPENCV = 1;

static const TYPE_OF_PT PT_ERROR = TYPE_OF_PT(-1, -1);
static const TYPE_MF_BB BB_ERROR = TYPE_MF_BB(PT_ERROR, PT_ERROR);

static const int MF_HALF_PATCH_SIZE = 4;
static const int MF_NPTS = 10;
static const int MF_ERROR_DIST = 20;

static const int MF_TRACK_SUCCESS = 0;
static const int MF_TRACK_F_PTS = -1; // number of points after filtering is too little
static const int MF_TRACK_F_BOX = -2; // result box is out of bounds
static const int MF_TRACK_F_CONFUSION = -3; // tracking result is disordered
static const int MF_TRACK_F_BOX_SMALL = -4; // input box is too small

static const int MF_REJECT_OFERROR = 1 << 0;
static const int MF_REJECT_NCC = 1 << 1;
static const int MF_REJECT_FB = 1 << 2;

static const bool NCC_USE_OPENCV = 0; // 1(lower speed): use matchTemplate(), 0(faster)

#endif
