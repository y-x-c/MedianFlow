//
//  ViewController.h
//  MedianFlow
//
//  Created by 陈裕昕 on 10/16/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#ifndef __MedianFlow__ViewController__
#define __MedianFlow__ViewController__

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "VideoController.h"

using namespace std;
using namespace cv;

class ViewController
{
private:
    VideoController *videoController;
    
    Mat trajectory;
    
    string retWindowName;
    
public:
    
    ViewController();
    
    ViewController(VideoController *videoController);
    
    ~ViewController();
    
    void drawCircles(vector<Point2f> pts, Scalar color = Scalar(30, 30, 230), int radius = 1);
    
    void drawLines(vector<Point2f> firstPts, vector<Point2f> secondPts, Scalar color = Scalar(30, 30, 230));
    
    void showCurrFrame();
};

#endif /* defined(__MedianFlow__ViewController__) */
