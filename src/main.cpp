#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
	ofSetupOpenGL(ofwindow_width,ofwindow_height,OF_WINDOW);			// <-------- setup the GL context
    //ofSetFullscreen(true);

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
