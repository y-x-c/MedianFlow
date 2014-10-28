//
//  main.cpp
//  MedianFlow
//
//  Created by 陈裕昕 on 10/7/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "VideoController.h"
#include "ViewController.h"
#include "OpticalFlow.h"

using namespace std;
using namespace cv;

void test()
{
    string filename("/Users/Orthocenter/Developments/MedianFlow/4.m4v");
    //string filename("/Users/Orthocenter/Developments/MedianFlow/2.mpg");
    VideoController videoController(filename);
    ViewController viewController(&videoController);
    
    videoController.readNextFrame();
    
    int curr = 0;
    vector<Point2f> pts[2];
    //
    vector<Point2f> groundTruth_pts[2];
                pts[0].push_back(Point2f(104, 103)); // 33 33
    for(int i = 0; i < 0; i++)
    {
        pts[0].push_back(Point2f(rand() % 320, rand() % 240));
    }
    
    //
    groundTruth_pts[0] = pts[0];
    
    while(videoController.readNextFrame())
    {
        Mat prevFrame = videoController.getPrevFrame(), currFrame = videoController.getCurrFrame();
        
        // debug
        // generate a 2 * 2 block
        Mat newFrame = genTestFrame(50, 50);
        
        // end debug
        
        
        
        OpticalFlow *opticalFlow = new OpticalFlow(prevFrame, currFrame);
        
        // debug
        //vector<Point2f> trackPts = opticalFlow->generateNeighborPts(pts[0][0]);
        //viewController.drawCircles(trackPts);
        // end debug
        
        opticalFlow->trackPts(pts[curr], pts[curr ^ 1]);
        
        delete opticalFlow;
        
        //
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(prevFrame, currFrame, groundTruth_pts[curr], groundTruth_pts[curr ^ 1], status, err, Size(15, 15), 1);
        
        Point2f d = pts[curr ^1][0] - pts[curr][0];
        Point2f gd = groundTruth_pts[curr ^ 1][0] - groundTruth_pts[curr][0];
        cout << "groudTruth_d=\n" << gd << endl;
        cout << "gdx / dx = " << gd.x / d.x << endl;
        
        viewController.drawLines(pts[curr], pts[curr ^ 1]);
        
        // debug
        //viewController.drawLines(groundTruth_pts[curr], groundTruth_pts[curr ^ 1], Scalar(255, 255, 255));
        // end debug
        
        viewController.showCurrFrame();
        
        cout << "frame #" << videoController.frameNumber() << endl;
        cout << "pts = " << endl << pts[curr ^ 1] << endl;
        cout << "groundTruth_pts = " << endl << groundTruth_pts[curr ^ 1] << endl;
        
        waitKey(0);
        
        curr ^= 1;
    }
}

int main(int argc, const char * argv[])
{
    srand(time(0));
    
    test();
    
    return 0;
}
