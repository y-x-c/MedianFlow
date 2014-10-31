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

void ViewController::showCache(const string &winName)
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
    int width = viewController.videoController->getCurrFrame().cols;
    int height = viewController.videoController->getCurrFrame().rows;
    
    
    if(event == CV_EVENT_LBUTTONDOWN)
    {
        selectDone = false;
        rect = Rect(Point2d(x, y), rect.br());
    }
    
    if(flags == CV_EVENT_FLAG_LBUTTON && event != CV_EVENT_LBUTTONDOWN)
    {
        rect = Rect(rect.tl(), Point2d(x, y));
        viewController.refreshCache();
        viewController.drawRect(rect);
        viewController.showCache(string("Median Flow"));
        
        if(rect.width >= 10 + 4 * 2 && rect.height >= 10 + 4 * 2 && rect.width <= width && rect.height <= height)
        {
            selectDone = true;
        }
    }
    
    if(event == CV_EVENT_LBUTTONUP)
    {
        if(rect.width >= 10 + 4 * 2 && rect.height >= 10 + 4 * 2 && rect.width <= width && rect.height <= height)
        {
            selectDone = true;
        }
        else
        {
            rect = Rect(Point2f(-1, -1), Point2f(width + 1, height + 1));
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
    
    setMouseCallback("Median Flow", ViewController::onMouse, &pp);
    
    if(videoController->cameraMode)
    {
        while(!(selectDone && waitKey(10) != -1))
        {
            refreshCache();
            drawRect(rect);
            showCache(string("Median Flow"));
        }
    }
    else
    {
        while(!selectDone)
        {
            waitKey();
        }
    }
    
    destroyWindow("Median Flow");
    
    return rect;
}