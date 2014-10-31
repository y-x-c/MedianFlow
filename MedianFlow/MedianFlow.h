//
//  MedianFlow.h
//  MedianFlow
//
//  Created by 陈裕昕 on 10/29/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#ifndef __MedianFlow__MedianFlow__
#define __MedianFlow__MedianFlow__

//debug
#include "ViewController.h"
// end debug
#include "OpticalFlow.h"
#include <iostream>

using namespace std;
using namespace cv;

class MedianFlow
{
private:
    Mat prevImg, nextImg;
    
    OpticalFlow *opticalFlow;
    ViewController *viewController;
    
    vector<Point2f> generatePts(const Rect_<float> &box);
    
    double calcNCC(const Mat &img0, const Mat &img1);
    
    void filterFB(const vector<Point2f> &initialPts, const vector<Point2f> &FBPts, vector<bool> &rejected);
    void filterNCC(const vector<Point2f> &initialPts, const vector<Point2f> &FPts, vector<bool> &rejected);
    
    Rect_<float> calcRect(const Rect_<float> &rect, const vector<Point2f> &pts, const vector<Point2f> &FPts, const vector<bool> &rejected, bool &valid);
    
public:
    
    MedianFlow();
    MedianFlow(const Mat &prevImg, const Mat &nextImg);
    MedianFlow(const Mat &prevImg, const Mat &nextImg, ViewController *_viewController);
    
    ~MedianFlow();
    
    static bool compare(const pair<float, int> &a, const pair<float, int> &b);
    
    Rect_<float> trackBox(const Rect_<float> &inputBox);
};

#endif /* defined(__MedianFlow__MedianFlow__) */
