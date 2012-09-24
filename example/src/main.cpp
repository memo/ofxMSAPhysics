#include "ofMain.h"
#include "testApp.h"


//========================================================================
int main( ){
	ofSetupOpenGL(1280, 720, OF_WINDOW);			// <-------- setup the GL context
	ofRunApp(new testApp());
}
