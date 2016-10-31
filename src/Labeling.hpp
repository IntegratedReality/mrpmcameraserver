//
//  Labeling.hpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//
#pragma once

#include "Const.hpp"
#include <ofMain.h>
#include <ofxCv.h>
#include <vector>


class labelingClass{
    public :
    
    labelingClass();
    
    std::vector<int> parents;
    
    void drawRegions(ofVec3f* point, int nums);
    
    inline bool isIn(int w,int h,int x,int y);
    
    inline unsigned int getAt(const cv::Mat& img,int x,int y);
    
    inline int compress(std::vector<int>& parents,int a);
    
    //aの属すグループとbの属すグループを併合（併合後の代表を返す）
    inline int link(std::vector<int>& parents,int a,int b);
    //番号とびとびなラベルを0,1,2,...に貼り替え
    inline int relabel(std::vector<int>& parents);
    int labeling(const cv::Mat& image,cv::Mat_<int>& label);
    
};
