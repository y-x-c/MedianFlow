//
//  OpticalFlow.h
//  MedianFlow
//
//  Created by 陈裕昕 on 10/15/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#ifndef __MedianFlow__OpticalFlow__
#define __MedianFlow__OpticalFlow__

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class OpticalFlow
{
private:
    // Size of the window centered by the traced point.
    // The value must be odd.
    const static int neighborSize = 37;
    const static int maxLevel = 0; // 0 means no pyramid
    
    Mat prevImg, nextImg;
    vector<Point2f> prevPts, nextPts;
    vector<Mat> prevImgs, nextImgs, Ixs, Iys, Its;
    
    bool isInside(const Point2f &pt, int imgWidth, int imgHeight);
    
    void preprocess();
    Point2f calculate(const Point2f &trackPoint, const Mat &Ix, const Mat &Iy, const Mat &It);
    Point2f calculatePyr(const Point2f &trackPoint);
    
protected:
//debug
public:
//end debug
    virtual vector<Point2f> generateNeighborPts(const Point2f &pt, int imgWidth, int imgHeight);
    
public:
    
    OpticalFlow();
    
    OpticalFlow(const Mat &prevImg, const Mat &nextImg);
    
    ~OpticalFlow();
    
    void trackPts(vector<Point2f> &pts, vector<Point2f> &retPts);
};

#endif /* defined(__MedianFlow__OpticalFlow__) */
