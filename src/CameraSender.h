#pragma once

#include "ofxOsc.h"

class CameraSender {
	public:
		void init(const char *_host, int _port);
		void sendData(int _id, int _time, double _x, double _y, double _theta);
	private:
		ofxOscSender sender;
};
