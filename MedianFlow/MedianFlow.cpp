 //
//  MedianFlow.cpp
//  MedianFlow
//
//  Created by 陈裕昕 on 10/29/14.
//  Copyright (c) 2014 陈裕昕. All rights reserved.
//

#include <cmath>
#include "MedianFlow.h"

MedianFlow::MedianFlow()
{

}

MedianFlow::MedianFlow(const Mat &prevImg, const Mat &nextImg)
{
    if(prevImg.channels() == 3)
        cvtColor(prevImg, this->prevImg, CV_BGR2GRAY);
    else
        prevImg.copyTo(this->prevImg);
    
    if(prevImg.channels() == 3)
        cvtColor(nextImg, this->nextImg, CV_BGR2GRAY);
    else
        nextImg.copyTo(this->nextImg);
    
    this->prevImg.convertTo(this->prevImg, CV_32F);
    this->nextImg.convertTo(this->nextImg, CV_32F);
    
    opticalFlow = new OpticalFlow(this->prevImg, this->nextImg);
}

MedianFlow::MedianFlow(const Mat &prevImg, const Mat &nextImg, ViewController *_viewController)
{
    if(prevImg.channels() == 3)
        cvtColor(prevImg, this->prevImg, CV_BGR2GRAY);
    else
        prevImg.copyTo(this->prevImg);
    
    if(prevImg.channels() == 3)
        cvtColor(nextImg, this->nextImg, CV_BGR2GRAY);
    else
        nextImg.copyTo(this->nextImg);
    
    this->prevImg.convertTo(this->prevImg, CV_32F);
    this->nextImg.convertTo(this->nextImg, CV_32F);
    
    opticalFlow = new OpticalFlow(this->prevImg, this->nextImg, OpticalFlow::USEOPENCV);
    
    viewController = _viewController;
}

MedianFlow::~MedianFlow()
{
    delete opticalFlow;
    opticalFlow = NULL;
}

vector<Point2f> MedianFlow::generatePts(const Rect_<float> &_box)
{
    Point2f tl(max(0.0f, _box.tl().x), max(0.0f, _box.tl().y));
    Point2f br(min(float(prevImg.cols), _box.br().x), min(float(prevImg.rows), _box.br().y));
    Rect_<float> box(tl, br);
    
    float stepX = (box.width - 2 * 4) / 9;
    float stepY = (box.height - 2 * 4) / 9;
    int x0 = box.x + 4;
    int y0 = box.y + 4;
    
    vector<Point2f> ret;
    
    for(int fx = 0; fx < 10; fx++)
    {
        for(int fy = 0; fy < 10; fy++)
        {
            ret.push_back(Point2f(x0 + fx * stepX, y0 + fy * stepY));
        }
    }
    
    return ret;
}

bool MedianFlow::compare(const pair<float, int> &a, const pair<float, int> &b)
// caution : prefix static can only be specified inside the class definition
{
    return a.first < b.first;
}

void MedianFlow::filterFB(const vector<Point2f> &initialPts, const vector<Point2f> &FBPts, vector<bool> &rejected)
{
    int size = int(initialPts.size());
    vector<pair<float, int> > V;
    
    for(int i = 0; i < size; i++)
    {
        if(initialPts[i] == Point2f(-1, -1) || FBPts[i] == Point2f(-1, -1)){
            rejected[i] = true;
            continue;
        }
        float dist = norm(Mat(initialPts[i]), Mat(FBPts[i]));
        V.push_back(make_pair(dist, i));
    }
    
    sort(V.begin(), V.end(), compare);
    
    int count = 0;
    
    for(int i = (int)V.size() / 2; i < V.size(); i++)
    {
        if(!rejected[V[i].second]) count++;
        rejected[V[i].second] = true;
    }
    
    cout << "FB filter out " << count << endl;
}

double MedianFlow::calcNCC(const cv::Mat &img0, const cv::Mat &img1)
{
    Mat vImg0, vImg1; // convert image to 1 dimension vector
    
    vImg0 = img0.clone();
    vImg1 = img1.clone();
    
    vImg0.reshape(0, vImg0.cols * vImg0.rows);
    vImg1.reshape(0, vImg1.cols * vImg1.rows);
    
    Mat v01 = vImg0.t() * vImg1;
    
    double norm0, norm1;
    
    norm0 = norm(vImg0);
    norm1 = norm(vImg1);
    
    return abs(v01.at<float>(0)) / norm0 / norm1;
}

void MedianFlow::filterNCC(const vector<Point2f> &initialPts, const vector<Point2f> &FPts, vector<bool> &rejected)
{
    int size = int(initialPts.size());
    vector<pair<float, int> > V;
    
    for(int i = 0; i < size; i++)
    {
        if(initialPts[i] == Point2f(-1, -1) || FPts[i] == Point2f(-1, -1)) continue;
        
        if(initialPts[i].x < 4 || initialPts[i].y < 4 || initialPts[i].x + 4 > prevImg.cols || initialPts[i].y + 4 > prevImg.rows) continue;
        if(FPts[i].x < 4 || FPts[i].y < 4 || FPts[i].x + 4 > prevImg.cols || FPts[i].y + 4 > prevImg.rows) continue;
        
        Point2f win(4.0, 4.0);
        Rect rect0(initialPts[i] - win, initialPts[i] + win);
        Rect rect1(FPts[i] - win, FPts[i] + win);
        
        double ncc = calcNCC(this->prevImg(rect0), this->nextImg(rect1));
        
        V.push_back(make_pair(ncc, i));
    }
    
    sort(V.begin(), V.end(), compare);
    
    int count = 0;
    
    for(int i = int(V.size()) / 2; i < V.size(); i++)
    {
        if(!rejected[V[i].second]) count++;
        rejected[V[i].second] = true;
        
        // debug
        //int ii = V[i].second;
        
        //Point2f win(4.0, 4.0);
        //Rect rect0(initialPts[ii] - win, initialPts[ii] + win);
        //Rect rect1(FPts[ii] - win, FPts[ii] + win);
        
        //imshow("imgPatch-prev", prevImg(rect0));
        //imshow("imgPatch-next", nextImg(rect1));
        //cout << ii << " " << V[i].first << endl;
        //waitKey();
        // end debug
        
    }
    
    cout << "NCC filter out " << count << endl;
}

Rect_<float> MedianFlow::calcRect(const Rect_<float> &rect, const vector<Point2f> &pts, const vector<Point2f> &FPts, const vector<bool> &rejected, bool &valid)
{
    const int size = int(pts.size());
    
    vector<float> dxs, dys;
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i]) continue;
        
        dxs.push_back(FPts[i].x - pts[i].x);
        dys.push_back(FPts[i].y - pts[i].y);
    }
    
    if(dxs.size() ==0)
    {
        valid = false;
        return Rect_<float>();
    }
    
    sort(dxs.begin(), dxs.end());
    sort(dys.begin(), dys.end());
    
    float dx = dxs[dxs.size() / 2];
    float dy = dys[dys.size() / 2];
    Point2f delta(dx, dy);
    
    vector<float> ratios;
    vector<float> absDist;
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i]) continue;
        
        for(int j = i + 1; j < size; j++)
        {
            if(rejected[j]) continue;
            
            float dist0 = norm(Mat(pts[i]), Mat(pts[j]));
            float dist1 = norm(Mat(FPts[i]), Mat(FPts[j]));
            float ratio = dist1 / dist0;
            
            ratios.push_back(ratio);
            absDist.push_back(abs(dist1- dist0));
        }
    }
    
    sort(ratios.begin(), ratios.end());
    float ratio = ratios[ratios.size() / 2];

    Rect_<float> ret(delta + rect.tl(), delta + rect.br());
    
    Point2f center((ret.tl().x + ret.br().x) / 2, (ret.tl().y + ret.br().y) / 2);
    Point2d tl(center.x - ret.width / 2 * ratio, center.y - ret.height / 2 * ratio);
    Point2d br(center.x + ret.width / 2 * ratio, center.y + ret.height / 2 * ratio);
    
    ret = Rect_<float>(tl, br);
    
    //ret = Rect_<float>(tl, Size2f(rect.width * ratio, rect.height * ratio));
    
    sort(absDist.begin(), absDist.end());
    if(absDist[absDist.size() / 2] > 10)
        valid = false;
    else
        valid = true;
    
    return ret;
}

Rect_<float> MedianFlow::trackBox(const Rect_<float> &inputBox)
{
    // size of the inputBox is assumed to be larger than (10 + 4 * 2) * (10 + 4 * 2)
    //assert(inputBox.width >= 10 + 4 * 2 && inputBox.height >= 10 + 4 * 2);
    //if(inputBox.width >= 10 + 4 * 2 && inputBox.height >= 10 + 4 * 2) return inputBox;
    
    vector<Point2f> pts = generatePts(inputBox);
    
    vector<Point2f> retF, retFB;
    
    opticalFlow->trackPts(pts, retF);
    
    // debug
    //viewController->drawCircles(pts, Scalar(233, 33, 233), 2);
    //viewController->drawLines(pts, retF);
    //viewController->showCurrFrame();
    //waitKey();
    // end debug
    
    opticalFlow->swapImg();
    opticalFlow->trackPts(retF, retFB);

    // debug
    //viewController->drawLines(pts, retF, Scalar(239, 123, 34));
    //viewController->showCurrFrame();
    // end debug
    
    vector<bool> rejected(10 * 10);
    
    filterFB(pts, retFB, rejected);
    filterNCC(pts, retF, rejected);
    
    // debug
    vector<Point2f> rPts, rPts2, rPts3;
    for(int i = 0; i < pts.size(); i++)
    {
        if(rejected[i]) continue;
        rPts.push_back(pts[i]);
        rPts2.push_back(retF[i]);
        rPts3.push_back(retFB[i]);
    }
    viewController->drawCircles(rPts, Scalar(255, 255, 0));
    viewController->drawCircles(rPts2);
    //viewController->drawCircles(rPts3, Scalar(23, 45, 214));
    viewController->drawLines(rPts, rPts2);
    
    cout << "after filter:" << rPts.size() << endl;
    // end debug
    
    Rect_<float> ret;
    
    bool valid;
    ret = calcRect(inputBox, pts, retF, rejected, valid);
    
    if(!valid)
    {
        ret = viewController->getRect();
        cout << "get Rect " << ret << endl;
    }
    
    return ret;
}