//
//  Homography.hpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//
#pragma once
#include <ofMain.h>
#include <ofxCv.h>
#include <vector>

#include "Const.hpp"

class homographyClass{
    public :
    
    vector<cv::Point2f> srcPoints,warpedPoints;     //ホモグラフィの基準となる4点(×2)
    cv::Mat homographyMat;     //ホモグラフィ行列(変換行列)
    cv::Point2f* curPoint;      //一度選択した座標を変更する際に使うポインタ
    bool movingPoint = false;
    
    /* 諸々のフラグ */
    bool ready = false;
    bool first = true;
    
    /* 基準点選択用 */
    bool movePoint(vector<cv::Point2f>& points, cv::Point2f point);
    void drawPoints(vector<cv::Point2f>& points);
    
    /* 基準点変更時にどの点を変更するのか調べるための関数(どの点が近いのか距離を測定する) */
    double distance(cv::Point2f a, cv::Point2f b);
    
    /* 座標に対しホモグラフィ行列を適用する関数 */
    void executeTransform(ofVec3f &input);
    // 変換行列をかけるベクトルは(x,y,1)の形で、3項目は(省略されているが)1をかけている
};
