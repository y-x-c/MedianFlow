//
//  OpticalFlow.cpp
//  MedianFlow
//
//  Created by 陈裕昕 on 10/15/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#include "OpticalFlow.h"

OpticalFlow::OpticalFlow()
{

}

OpticalFlow::OpticalFlow(const Mat &prevImg, const Mat &nextImg)
{
    //check image size
    
    cvtColor(prevImg, this->prevImg, CV_BGR2GRAY);
    cvtColor(nextImg, this->nextImg, CV_BGR2GRAY);
    
    this->prevImg.convertTo(this->prevImg, CV_32F);
    this->nextImg.convertTo(this->nextImg, CV_32F);

    preprocess();
}

OpticalFlow::~OpticalFlow()
{

}

void OpticalFlow::preprocess()
{
    buildPyramid(prevImg, prevImgs, maxLevel);
    buildPyramid(nextImg, nextImgs, maxLevel);
    
    Ixs.resize(maxLevel + 1);
    Iys.resize(maxLevel + 1);
    Its.resize(maxLevel + 1);
    
    for(int i = 0; i <= maxLevel; i++)
    {

        //Sobel(prevImgs[i], Ixs[i], -1, 1, 0, 3);
        //Sobel(prevImgs[i], Iys[i], -1, 0, 1, 3);
        Scharr(prevImgs[i], Ixs[i], -1, 1, 0);
        Scharr(prevImgs[i], Iys[i], -1, 0, 1);
        
        //cout << "prevImg:\n" << prevImgs[i] << endl;
        //cout << "Ixs:\n" << Ixs[i] << endl;
        
        Its[i] = nextImgs[i] - prevImgs[i];
    
        Ixs[i] /= 32; Iys[i] /= 32;
    }
    
    //cout << "Ix:\n" << Ix << endl;
    //cout << "It:\n" << It << endl;
}

vector<Point2f> OpticalFlow::generateNeighborPts(const Point2f &pt, int imgWidth, int imgHeight)
{
    int l, r, u, d;
    int halfSize = (neighborSize >> 1);
    
    l = pt.x - halfSize;
    r = pt.x + halfSize + 1; // [l, r)
    u = pt.y - halfSize;
    d = pt.y + halfSize + 1;
    
    if(l < 0) l = 0;
    if(r > imgWidth) r = imgWidth;
    if(u < 0) u = 0;
    if(d > imgHeight) d = imgHeight;
    
    vector<Point2f> result;
    
    for(int x = l; x < r; x++)
    {
        for(int y = u; y < d; y++)
        {
            result.push_back(Point2f(x, y));
        }
    }
    
    return result;
}

bool OpticalFlow::isInside(const Point2f &pt, int imgWidth, int imgHeight)
{
    return pt.x < imgWidth && pt.y < imgHeight && pt.x >= 0 && pt.y >= 0;
}

Point2f OpticalFlow::calculate(const Point2f &trackPoint, const Mat &Ix, const Mat &Iy, const Mat &It)
{
    const int imgWidth = Ix.cols;
    const int imgHeight = Ix.rows;
    
    if(!isInside(trackPoint, imgWidth, imgHeight))
    {
        return Point2f(-1, -1);
    }
    
    vector<Point2f> pts = generateNeighborPts(trackPoint, imgWidth, imgHeight);
    
    Mat A(int(pts.size()), 2, CV_32F);
    Mat b(int(pts.size()), 1, CV_32F);
    
    for(int i = 0; i < pts.size(); i++)
    {
        // x 对应于列， y 对应于行
        A.at<float>(i, 0) = Ix.at<float>(pts[i].y, pts[i].x);
        A.at<float>(i, 1) = Iy.at<float>(pts[i].y, pts[i].x);

        b.at<float>(i) =  - It.at<float>(pts[i].y, pts[i].x);
    }
    
    Mat d;
    solve(A, b, d, DECOMP_QR);
    
    //cout << "A:\n" << A << endl;
    //cout << "b:\n" << b << endl;
    cout << "d:\n" << d << endl;
    
    float dx = d.at<float>(0), dy = d.at<float>(1);
    
    return Point2f(dx + trackPoint.x, dy + trackPoint.y);
}

Point2f OpticalFlow::calculatePyr(const Point2f &trackPoint)
{
    Point2f resPoint = trackPoint;
    
    resPoint.x /= pow(2, maxLevel);
    resPoint.y /= pow(2, maxLevel);

    for(int l = maxLevel; l >= 0; l--)
    {
        resPoint = calculate(resPoint, Ixs[l], Iys[l], Its[l]);
        if(l)
        {
            resPoint.x *= 2.0;
            resPoint.y *= 2.0;
        }
    }
    return resPoint;
}

void OpticalFlow::trackPts(vector<Point2f> &pts, vector<Point2f> &retPts)
{
    retPts.clear();
    for(vector<Point2f>::iterator it = pts.begin(); it != pts.end(); it++)
    {
        Point2f pt = calculatePyr(*it);
        
        retPts.push_back(pt);
    }
}

