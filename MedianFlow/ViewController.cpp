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
    retWindowName("Median Flow")
{
    this->videoController = videoController;
}

void ViewController::refreshCache()
{
    if(videoController->cameraMode)
        videoController->readNextFrame();
    videoController->getCurrFrame().copyTo(cache);
}

void ViewController::drawLines(const vector<Point2f> &firstPts, const vector<Point2f> &secondPts, Scalar color)
{
    for(int i = 0; i < firstPts.size(); i++)
    {
        if(secondPts[i] == Point2f(-1, -1)) continue;
        line(cache, firstPts[i], secondPts[i], color);
    }
}

void ViewController::drawCircles(const vector<Point2f> &pts, Scalar color, int radius)
{
    for(int i = 0; i <
        pts.size(); i++)
    {
        circle(cache, pts[i], radius, color);
    }
}

void ViewController::showCurrFrame(const string &winName)
{
    if(winName.size())
        imshow(winName, cache);
    else
        imshow(retWindowName, cache);
    
    waitKey(1);
}

void ViewController::drawRect(const Rect &rect)
{
    rectangle(cache, rect, Scalar(255, 255, 255));
}

void ViewController::onMouse(int event, int x, int y, int flags, void* param)
{
    pair<pair<void*, void*>, bool*> pp = *(pair<pair<void*, void*>, bool*>*)(param);
    pair<void*, void*> p = pp.first;
    
    bool &selectDone = *((bool*)pp.second);
    Rect &rect = *((Rect*)p.first);
    ViewController &viewController = *((ViewController*)p.second);
    
    if(event == CV_EVENT_LBUTTONDOWN)
    {
        rect = Rect(Point2d(x, y), rect.br());
        cout << rect << endl;
    }
    
    if(event == CV_EVENT_LBUTTONUP || (flags == CV_EVENT_FLAG_LBUTTON && event != CV_EVENT_LBUTTONDOWN))
    {
        rect = Rect(rect.tl(), Point2d(x, y));
        viewController.refreshCache();
        viewController.drawRect(rect);
        viewController.showCurrFrame(string("Median Flow"));
        
        if(event == CV_EVENT_LBUTTONUP){
            selectDone = true;
        }
    }
}

Rect ViewController::getRect()
{
    namedWindow("Median Flow", CV_WINDOW_AUTOSIZE);
    
    imshow("Median Flow", videoController->getCurrFrame());
    
    int width = videoController->getCurrFrame().cols;
    int height = videoController->getCurrFrame().rows;
    
    Rect rect(Point2f(), Point2f(width + 1, height + 1));
    pair<void*, void*> p(&rect, this);
    
    bool selectDone = false;
    
    pair<pair<void*, void*>, bool*> pp(p, &selectDone);
    
    bool flag = false;
    
    setMouseCallback("Median Flow", ViewController::onMouse, &pp);
    
    while(!flag)
    {
        waitKey(1);
        
        if(videoController->cameraMode)
        {
            refreshCache();
            drawRect(rect);
            showCurrFrame(string("Median Flow"));
        }
        //static int count = 0;
        //cout << ++count << " " << selectDone << endl;
        if(selectDone && rect.width >= 10 + 4 * 2 && rect.height >= 10 + 4 * 2 && rect.width <= width && rect.height <= height) flag = true;
    }
    
    destroyWindow("Median Flow");
    
    return rect;
}