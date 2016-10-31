//
//  Simulator.hpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//
#pragma once
#include <ofMain.h>
#include "Const.hpp"

class simulatorClass{
    public :
    
    ofImage simulationImg;
    unsigned char *pixels_simulation;
    void markerGen(ofVec2f center);
    
    ofVec2f markerPos[3];
    int interval = 0;
    bool turn = false;
    
    inline void drawCube(ofVec2f point1);
    void init();
    void movementManager();
};