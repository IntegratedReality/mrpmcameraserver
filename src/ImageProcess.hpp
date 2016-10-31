//
//  ImageProcess.hpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//

#pragma once
#include <ofMain.h>

#include "Const.hpp"

class imageProcess{
    public :
    
    /* for camera input */
    ofImage bin;
    
    /* pixel datas */
    
    unsigned char *pixels_origin;
    unsigned char *pixels_gray;
    unsigned char *pixels_bin;
    
    unsigned char red,green,blue,gray;
    
    /* for labeling */
    
    int num = 0;
    int previous_num = region;  //一個前のラベル数を保存
    int num_of_light = 0;   //min_region以上の領域の個数
    int filter_intensity = 1;   //medianフィルターの強度
    cv::Mat_<int> labels = cv::Mat_<int>::zeros(camheight,camwidth);
    cv::Mat bin_mat;    //ofとCVの変換(ラッパー)用
    ofVec3f center_point[region];       //ラベリングされた領域の重心を保管する (x座標,y座標,領域の大きさ)
    ofVec2f usingArea[2];   //使用する範囲を指定(左上と右下)
    ofVec2f areaSize;
    void writePoints();
    
    ofTexture camTexture;
    ofTexture binTexture;
    ofFbo camFbo;
    ofFbo binFbo;
    ofFbo stringFbo;
    
    ofVec3f homographyCorner[4];
    
    /* strings */
    string filter_info;
    string fpsString;
    string selectedMarker;
    string normalizedArea1;
    string normalizedArea2;
    string camFpsString;
    
    /* flags */
    bool isNewframe = false;
    bool showImage = true;
    bool setCoord = false;  //使用する範囲を指定して座標を確定
    bool setCoordToggle = false;    //指定する点の切り替え用
    bool usingAreaDecided = false;
    bool usingAreaConfig = false;
    
    imageProcess(){
        homographyCorner[0] = ofVec3f(0,0,0);
        homographyCorner[1] = ofVec3f(camwidth,0,0);
        homographyCorner[2] = ofVec3f(camwidth,camheight,0);
        homographyCorner[3] = ofVec3f(0,camheight,0);
    }
};
