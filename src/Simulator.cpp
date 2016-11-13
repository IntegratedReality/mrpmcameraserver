//
//  Simulator.cpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//

#include "Simulator.hpp"

void simulatorClass::drawCube(ofVec2f point1){
    for (int i = point1.x; i <= point1.x + 3; i++){
        for (int j = point1.y; j <= point1.y + 3; j++){
            pixels_simulation[j * camwidth + i ] = 255;
        }
    }
}

void simulatorClass::init(){
    for (int i = 0; i < camheight; i++){
        for (int j = 0; j < camwidth; j++){
            pixels_simulation[i * camwidth + j] = 0;
        }
    }
}
void simulatorClass::movementManager(){
    if (interval == 250){
        turn = !turn;
        interval = 0;
    }
    if (!turn){
        markerPos[0].x += 1;
        
        markerPos[1].x -= 1;
        
        markerPos[2].y += 1;
    }
    else {
        markerPos[0].x -= 1;
        
        markerPos[1].x += 1;
        
        markerPos[2].y -= 1;
        
    }
    interval++;
}


void simulatorClass::markerGen(ofVec2f center){
    drawCube(ofVec2f(center.x + 5, center.y));
    drawCube(ofVec2f(center.x - 10, center.y + 5));
    drawCube(ofVec2f(center.x - 10, center.y - 5));
}

