//
//  ImageProcess.cpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//

#include "ImageProcess.hpp"

void imageProcess::writePoints(){      //ラベリング後に重心を求め、それを表示する
    string position;
    int limit;
    int counter = 0;
    if (num > 15){
        limit = 15;
    }
    else {
        limit = num-1;
    }
    
    int i = 1;
    while (counter < limit && i < region-10){
        if (center_point[i].z != 0){
            position = ofToString(i) + " : ";
            position += ofToString(center_point[i]);
            ofDrawBitmapString(position,30 , 15 * counter + 20 );
            counter++;
        }
        i++;
    }
    ofSetColor(255);
}

