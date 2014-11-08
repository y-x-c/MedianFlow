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

MedianFlow::MedianFlow(const Mat &prevImg, const Mat &nextImg, ViewController *_viewController)
{
    assert(prevImg.type() == CV_8U || prevImg.type() == CV_8UC3);
    assert(nextImg.type() == CV_8U || nextImg.type() == CV_8UC3);
    
    if(prevImg.channels() == 3)
        // for invoking MedianFlow directly
        cvtColor(prevImg, this->prevImg, CV_BGR2GRAY);
    else
        this->prevImg = prevImg; // prevImg will be saved in class TLD
        //prevImg.copyTo(this->prevImg);
    
    if(prevImg.channels() == 3)
        cvtColor(nextImg, this->nextImg, CV_BGR2GRAY);
    else
        this->nextImg = nextImg;
        //nextImg.copyTo(this->nextImg);
    
    //
    //this->prevImg.convertTo(this->prevImg, CV_32F);
    //this->nextImg.convertTo(this->nextImg, CV_32F);
    
    opticalFlow = new OpticalFlow(this->prevImg, this->nextImg, OF_USE_OPENCV);
    opticalFlowSwap = new OpticalFlow(this->nextImg, this->prevImg, OF_USE_OPENCV);
    
    viewController = _viewController;
}

MedianFlow::~MedianFlow()
{
    delete opticalFlow;
    opticalFlow = NULL;
    
    delete opticalFlowSwap;
    opticalFlowSwap = NULL;
}

void MedianFlow::generatePts(const TYPE_MF_BB &_box, vector<TYPE_MF_PT> &ret)
{
    TYPE_MF_PT tl(max(0.f, _box.tl().x), max(0.f, _box.tl().y));
    TYPE_MF_PT br(min((float)prevImg.cols, _box.br().x), min((float)prevImg.rows, _box.br().y));
    TYPE_MF_BB box(tl, br);
    
    float stepX = (float)(box.width - 2 * MF_HALF_PATCH_SIZE) / (MF_NPTS - 1);
    float stepY = (float)(box.height - 2 * MF_HALF_PATCH_SIZE) / (MF_NPTS - 1);
    int x0 = box.x + MF_HALF_PATCH_SIZE;
    int y0 = box.y + MF_HALF_PATCH_SIZE;
    
    if(!ret.empty()) ret.clear();
    
    for(int fx = 0; fx < MF_NPTS; fx++)
    {
        for(int fy = 0; fy < MF_NPTS; fy++)
        {
            ret.push_back(TYPE_MF_PT(x0 + fx * stepX, y0 + fy * stepY));
        }
    }
}

bool MedianFlow::compare(const pair<float, int> &a, const pair<float, int> &b)
// caution : prefix static can only be specified inside the class definition
{
    return a.first < b.first;
}

bool MedianFlow::isPointInside(const TYPE_MF_PT &pt, const TYPE_MF_COORD alpha)
{
    int width = prevImg.cols, height = prevImg.rows;
    return (pt.x >= 0 + alpha) && (pt.y >= 0 + alpha) && (pt.x <= width - alpha) && (pt.y <= height - alpha);
}

bool MedianFlow::isBoxUsable(const TYPE_MF_BB &rect)
{
    bool insideBr = isPointInside(rect.br()), insideTl = isPointInside(rect.tl());
    int width = prevImg.cols, height = prevImg.rows;
    
    if(rect.br().x < 0 || rect.br().y < 0 || rect.tl().x > width || rect.tl().y > height)
        return false;
    
    TYPE_MF_BB _rect(rect);
    
    if(!insideTl) _rect.tl() = TYPE_MF_PT(0, 0);
    if(!insideBr) _rect.br() = TYPE_MF_PT(width, height);
    
    if(_rect.width < MF_NPTS || _rect.height < MF_NPTS)
        return false;
    
    if(_rect.width > width || _rect.height > height) return false;
    
    return true;
}

void MedianFlow::filterOFError(const vector<TYPE_MF_PT> &pts, const vector<uchar> &status, vector<int> &rejected)
{
    for(int i = 0; i < pts.size(); i++)
    {
        if(status[i] == 0) rejected[i] |= MF_REJECT_OFERROR;
    }
}

void MedianFlow::filterFB(const vector<TYPE_MF_PT> &initialPts, const vector<TYPE_MF_PT> &FBPts, vector<int> &rejected)
{
    int size = int(initialPts.size());
    vector<pair<float, int> > V;
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i] & MF_REJECT_OFERROR) continue;
        
        float dist = norm(Mat(initialPts[i]), Mat(FBPts[i]));
        V.push_back(make_pair(dist, i));
    }
    
    sort(V.begin(), V.end(), compare);
    
    for(int i = (int)V.size() / 2; i < V.size(); i++)
    {
        rejected[V[i].second] |= MF_REJECT_FB;
    }
}

float MedianFlow::calcNCC(const cv::Mat &img0, const cv::Mat &img1)
{
    //    Mat vImg0, vImg1; // convert image to 1 dimension vector
    //
    //    vImg0 = img0.clone();
    //    vImg1 = img1.clone();
    //
    //    vImg0.reshape(0, vImg0.cols * vImg0.rows);
    //    vImg1.reshape(0, vImg1.cols * vImg1.rows);
    //
    //    Mat v01 = vImg0.t() * vImg1;
    //
    //    float norm0, norm1;
    //
    //    norm0 = norm(vImg0);
    //    norm1 = norm(vImg1);
    //
    //    return abs(v01.at<float>(0)) / norm0 / norm1;
    
    Mat nccMat;
    matchTemplate(img0, img1, nccMat, CV_TM_CCORR_NORMED);
    
    return nccMat.at<float>(0);
}

void MedianFlow::filterNCC(const vector<TYPE_MF_PT> &initialPts, const vector<TYPE_MF_PT> &FPts, vector<int> &rejected)
{
    int size = int(initialPts.size());
    vector<pair<float, int> > V;
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i] & MF_REJECT_OFERROR) continue;
        
        if(!isPointInside(initialPts[i], MF_HALF_PATCH_SIZE)) continue;
        if(!isPointInside(FPts[i], MF_HALF_PATCH_SIZE)) continue;
        
        Point2d win(MF_HALF_PATCH_SIZE, MF_HALF_PATCH_SIZE);
        Point2d pt1(initialPts[i].x, initialPts[i].y);
        Point2d pt2(FPts[i].x, FPts[i].y);
        
        // must be int
        Rect_<int> rect0(pt1 - win, pt1 + win);
        Rect_<int> rect1(pt2 - win, pt2 + win);
        
        float ncc = calcNCC(this->prevImg(rect0), this->nextImg(rect1));
        
        V.push_back(make_pair(ncc, i));
    }
    
    sort(V.begin(), V.end(), compare);
    
    for(int i = int(V.size()) / 2; i < V.size(); i++)
    {
        rejected[V[i].second] |= MF_REJECT_NCC;
    }
}

TYPE_MF_BB MedianFlow::calcRect(const TYPE_MF_BB &rect, const vector<TYPE_MF_PT> &pts, const vector<TYPE_MF_PT> &FPts, const vector<int> &rejected, int &status)
{
    const int size = int(pts.size());
    
    vector<TYPE_MF_COORD> dxs, dys;
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i]) continue;
        
        dxs.push_back(FPts[i].x - pts[i].x);
        dys.push_back(FPts[i].y - pts[i].y);
    }
    
    if(dxs.size() <= 1)
    {
        status = MF_TRACK_F_PTS;
        return BB_ERROR;
    }
    
    sort(dxs.begin(), dxs.end());
    sort(dys.begin(), dys.end());
    
    TYPE_MF_COORD dx = dxs[dxs.size() / 2];
    TYPE_MF_COORD dy = dys[dys.size() / 2];
    TYPE_MF_PT delta(dx, dy);
    
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
        }
    }
    
    sort(ratios.begin(), ratios.end());
    float ratio = ratios[ratios.size() / 2];
    
    TYPE_MF_BB ret(delta + rect.tl(), delta + rect.br());
    
    TYPE_MF_PT center((float)(ret.tl().x + ret.br().x) / 2, (float)(ret.tl().y + ret.br().y) / 2);
    TYPE_MF_PT tl(center.x - ret.width / 2 * ratio, center.y - ret.height / 2 * ratio);
    TYPE_MF_PT br(center.x + ret.width / 2 * ratio, center.y + ret.height / 2 * ratio);
    
    ret = TYPE_MF_BB(tl, br);

    // debug
    //cout << ret << endl;
    //
    
    for(int i = 0; i < size; i++)
    {
        if(rejected[i]) continue;
        
        float dist = norm(Mat(pts[i]), Mat(FPts[i]));
        
        absDist.push_back(dist);
    }

    sort(absDist.begin(), absDist.end());
    
    float medianAbsDist = absDist[(int)absDist.size() / 2];
    
    for(auto &i : absDist)
        //caution : must add '&'
    {
        i = abs(i - medianAbsDist);
    }
    
    sort(absDist.begin(), absDist.end());
    if(absDist[(int)absDist.size() / 2] > MF_ERROR_DIST)
    {
        status = MF_TRACK_F_CONFUSION;
        return BB_ERROR;
    }
    
    if(!isBoxUsable(ret))
    {
        status = MF_TRACK_F_BOX;
        return BB_ERROR;
    }
    
    status = MF_TRACK_SUCCESS;
    return ret;
}

TYPE_MF_BB MedianFlow::trackBox(const TYPE_MF_BB &inputBox, int &status)
{
    // width and height of the inputBox should be larger than (MF_NPTS + MF_HALF_PATCH_SIZE * 2)

    if(inputBox.width < MF_NPTS + MF_HALF_PATCH_SIZE * 2 || inputBox.height < MF_NPTS + MF_HALF_PATCH_SIZE * 2)
    {
        status = MF_TRACK_F_BOX_SMALL;
        return BB_ERROR;
    }
    
    vector<TYPE_MF_PT> pts;
    generatePts(inputBox, pts);
    
    vector<TYPE_MF_PT> retF, retFB;
    vector<uchar> statusF, statusFB;
    
    opticalFlow->trackPts(pts, retF, statusF);
    opticalFlowSwap->trackPts(retF, retFB, statusFB);

    vector<int> rejected(MF_NPTS * MF_NPTS);
    
    filterOFError(retF, statusF, rejected);
    filterOFError(retFB, statusFB, rejected);
    
    filterFB(pts, retFB, rejected);
    filterNCC(pts, retF, rejected);
    
    TYPE_MF_BB ret;
    
    ret = calcRect(inputBox, pts, retF, rejected, status);
    
    // show result
    if(viewController)
    {
        vector<TYPE_MF_PT> rPts, rPts2, rPts3;
        
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
        
        cout << "number of points after filtering:" << rPts.size() << endl;
        cout << ret << endl;
    }
    //
    
    if(status != MF_TRACK_SUCCESS)
    {
        cout << "Tracking failed, error code : " << status << endl;
        
        return BB_ERROR;
    }
    
    return ret;
}