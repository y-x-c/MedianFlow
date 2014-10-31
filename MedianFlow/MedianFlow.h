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
    
    OpticalFlow *opticalFlow, *opticalFlowSwap;
    ViewController *viewController;
    
    vector<Point2i> generatePts(const Rect_<int> &box);
    
    double calcNCC(const Mat &img0, const Mat &img1);
    
    void filterFB(const vector<Point2i> &initialPts, const vector<Point2i> &FBPts, vector<bool> &rejected);
    void filterNCC(const vector<Point2i> &initialPts, const vector<Point2i> &FPts, vector<bool> &rejected);
    
    Rect_<int> calcRect(const Rect_<int> &rect, const vector<Point2i> &pts, const vector<Point2i> &FPts, const vector<bool> &rejected, bool &valid);
    
public:
    
    MedianFlow();
    MedianFlow(const Mat &prevImg, const Mat &nextImg);
    MedianFlow(const Mat &prevImg, const Mat &nextImg, ViewController *_viewController);
    
    ~MedianFlow();
    
    static bool compare(const pair<float, int> &a, const pair<float, int> &b);
    
    static const int MEDIANFLOW_TRACK_SUCCESS = 0;
    static const int MEDIANFLOW_TRACK_FAILURE = -1;
    Rect_<int> trackBox(const Rect_<int> &inputBox, int &status);
};

#endif /* defined(__MedianFlow__MedianFlow__) */
