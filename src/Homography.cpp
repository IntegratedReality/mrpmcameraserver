//
//  Homography.cpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//

#include "Homography.hpp"

/* 基準点変更時にどの点を変更するのか調べるための関数(どの点が近いのか距離を測定する) */
double homographyClass::distance(cv::Point2f a, cv::Point2f b){
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

/* 座標に対しホモグラフィ行列を適用する関数 */
void homographyClass::executeTransform(ofVec3f &input){
    // 変換行列をかけるベクトルは(x,y,1)の形で、3項目は(省略されているが)1をかけている
    double x = input.x; //変数を退避
    double y = input.y;
    double scale = homographyMat.at<double>(2,0) * x + homographyMat.at<double>(2,1) * y + 1;
    input.x = (homographyMat.at<double>(0,0) * x + homographyMat.at<double>(0,1) * y + homographyMat.at<double>(0,2))/scale;
    input.y = (homographyMat.at<double>(1,0) * x + homographyMat.at<double>(1,1) * y + homographyMat.at<double>(1,2))/scale;
}

void homographyClass::drawPoints(vector<cv::Point2f>& points) {
    ofNoFill();
    ofPushStyle();
    ofSetColor(200, 10, 10);
    for(int i = 0; i < points.size(); i++) {
        ofDrawCircle(points[i].x,points[i].y, 10);
        ofDrawCircle(points[i].x,points[i].y, 1);
    }
    ofPopStyle();
}

bool homographyClass::movePoint(vector<cv::Point2f>& points, cv::Point2f point) {
    point.x -= cam_margin;
    point.y -= cam_margin;
    for(int i = 0; i < points.size(); i++) {
        if(distance(points[i],point) < 60) {
            movingPoint = true;
            curPoint = &points[i];
            return true;
        }
    }
    return false;
}
