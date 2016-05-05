#include "CameraSender.h"

void CameraSender::init(const char *_host, int _port) {
	sender.setup(_host, _port);
}

void CameraSender::sendData(int _id, int _time, double _x, double _y, double _theta) {
	ofxOscMessage m;
	m.setAddress("/camera/position");
	m.addIntArg(_id);
	m.addIntArg(_time);
	m.addDoubleArg(_x);
	m.addDoubleArg(_y);
	m.addDoubleArg(_theta);
	sender.sendMessage(m);
}
