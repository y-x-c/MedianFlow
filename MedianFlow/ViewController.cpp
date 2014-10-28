//
//  ViewController.cpp
//  MedianFlow
//
//  Created by 陈裕昕 on 10/16/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#include "ViewController.h"


ViewController::~ViewController()
{
    
}

ViewController::ViewController(VideoController *videoController):
    retWindowName("result")
{
    this->videoController = videoController;
    
    trajectory = Mat(videoController->frameSize(), CV_8UC3, Scalar::all(0));
}

void ViewController::drawLines(vector<Point2f> firstPts, vector<Point2f> secondPts, Scalar color)
{
    for(int i = 0; i < firstPts.size(); i++)
    {
        line(trajectory, firstPts[i], secondPts[i], color);
    }
}

void ViewController::drawCircles(vector<Point2f> pts, Scalar color, int radius)
{
    for(int i = 0; i < pts.size(); i++)
    {
        circle(trajectory, pts[i], radius, color);
    }
}

void ViewController::showCurrFrame()
{
    Mat ret;
    
    addWeighted(videoController->getCurrFrame(), 1, trajectory, 1, 0, ret);
    
    namedWindow(retWindowName, WINDOW_AUTOSIZE);
    
    imshow(retWindowName, ret);
    
    waitKey(1);
}