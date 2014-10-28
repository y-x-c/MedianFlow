//
//  VideoController.cpp
//  MedianFlow
//
//  Created by 陈裕昕 on 10/16/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#include "VideoController.h"

VideoController::~VideoController()
{
    delete videoCapture;
}

VideoController::VideoController(string &filename):
    curr(0), frame(0)
{
    videoCapture = new VideoCapture(filename);
    
    // check if the video file is opened
}

Mat VideoController::getCurrFrame()
{
    return frames[curr];
}

Mat VideoController::getPrevFrame()
{
    return frames[curr ^ 1];
}

bool VideoController::readNextFrame()
{
    curr ^= 1;
    frame++;
    return videoCapture -> read(frames[curr]);
}

Size VideoController::frameSize()
{
    return Size(videoCapture->get(CV_CAP_PROP_FRAME_WIDTH), videoCapture->get(CV_CAP_PROP_FRAME_HEIGHT));
}

int VideoController::frameNumber()
{
    return frame;
}