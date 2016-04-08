#pragma once

#include "ofxOsc.h"
#include <chrono>

class CameraSender {
	public:
		void init(const char *_host, int _port);
		void sendData(int _id, int _time, double _x, double _y, double _theta);
    
        /* for time stamp */
        chrono::time_point<chrono::system_clock> start;
        chrono::time_point<chrono::system_clock> currentTime;
        std::chrono::duration<double> elapsedTime;
        uint32_t timeStamp;
	private:
		ofxOscSender sender;
};
