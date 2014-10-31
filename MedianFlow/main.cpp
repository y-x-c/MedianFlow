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
#include "MedianFlow.h"

using namespace std;
using namespace cv;

Mat genTestFrame(int x, int y)
{
    // this function will generate a block at position (x, y)
    // the background color of the frame this function generated is black
    // and the block is white
    // size of the block is 2 * 2
    // frame size is 320 * 240
    
    Mat frame(240, 320, CV_8U, Scalar::all(0));
    frame.at<char>(y, x) = 255;
    //frame.at<char>(y + 1, x + 1) = 255;
    //frame.at<char>(y, x + 1) = 255;
    //frame.at<char>(y + 1, x) = 255;
    
    cvtColor(frame, frame, CV_GRAY2BGR);
    
    return frame;
}

void test()
{
    //string filename("/Users/Orthocenter/Developments/MedianFlow/5.m4v");
    string filename("/Users/Orthocenter/Developments/MedianFlow/mid.avi");
    VideoController videoController(filename);
    ViewController viewController(&videoController);
    
    videoController.readNextFrame();
    
    int curr = 0;
    vector<Point2i> pts[2];
    //
    vector<Point2i> groundTruth_pts[2];
                pts[0].push_back(Point2f(40, 40)); // 33 33
    for(int i = 0; i < 0; i++)
    {
        pts[0].push_back(Point2f(rand() % 320, rand() % 240));
    }
    
    //
    groundTruth_pts[0] = pts[0];
    
    int count = 0;
    
    while(videoController.readNextFrame())
    {
        //Mat prevFrame = videoController.getPrevFrame(), currFrame = videoController.getCurrFrame();
        
        // debug
        // generate a 1 * 1 block
        Mat prevFrame = genTestFrame(40, 40 + count * 2);
        Mat currFrame = genTestFrame(40, 40 + (++count) * 2);
        // end debug
        
        imshow("prev", prevFrame);
        imshow("curr", currFrame);
        waitKey(1);
        
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
        
        viewController.showCache();
        
        cout << "frame #" << videoController.frameNumber() << endl;
        cout << "pts = " << endl << pts[curr ^ 1] << endl;
        cout << "groundTruth_pts = " << endl << groundTruth_pts[curr ^ 1] << endl;
        
        waitKey(0);
        
        curr ^= 1;
    }
}

void testMF()
{
    string filename("/Users/Orthocenter/Developments/MedianFlow/car.mpg");
    //VideoController videoController(0);
    VideoController videoController(filename);
    ViewController viewController(&videoController);
    
    videoController.jumpToFrameNum(0);
    
    videoController.readNextFrame();
    Rect_<int> box = viewController.getRect();
    
    viewController.refreshCache();
    viewController.drawRect(box);
    viewController.showCache();
    
    while(videoController.readNextFrame())
    {
        cout << "Frame #" << videoController.frameNumber() << endl;
        viewController.refreshCache();
        
        MedianFlow &medianFlow = *(new MedianFlow(videoController.getPrevFrame(), videoController.getCurrFrame(), &viewController));
        
        int status;
        
        box = medianFlow.trackBox(box, status);
        
        if(status == MedianFlow::MEDIANFLOW_TRACK_FAILURE)
        {
            videoController.readNextFrame();
            viewController.refreshCache();
            viewController.showCache();
            while(waitKey() == 'n')
            {
                videoController.readNextFrame();
                viewController.refreshCache();
                viewController.showCache();
            }
            box = viewController.getRect();
        }
        else
        {
            viewController.drawRect(box);
            viewController.showCache();
        }
        
        delete &medianFlow;
    }
}

int main(int argc, const char * argv[])
{
    //test();
    
    testMF();
    
    return 0;
}
