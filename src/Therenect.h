/*
 Therenect - A virtual Theremin for the Kinect 
 Copyright (c) 2010 Martin Kaltenbrunner <martin@tuio.org>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _THERENECT
#define _THERENECT

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofAddons.h"
#include "ofxCvKalman.h"
#include "ofxMidi.h"
#include "ofxControlPanel.h"

class Therenect : public ofBaseApp
{
	public:
		void setup();
		void update();
		void draw();
        void exit();

		void drawPointCloud();
	
		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		void audioRequested(float * input, int bufferSize, int nChannels);
		int sampleRate, bufferSize;
		float *sound_data;
		
		float range;
		float amplitude, frequency;
		float amplset, freqset;
		float rotation;
		int oscmode;
	
		ofPoint volumePoint, pitchPoint;
		ofPoint vReferencePoint, pReferencePoint;
		ofPoint vControlPoint, pControlPoint;

		ofxKinect kinect;
		ofxCvGrayscaleImage	controlImage;
		ofxCvGrayscaleImage	depthImage;
		
		//ofImage testImage;
		bool drawing;
	
		ofxMidiOut midi;
		unsigned char midi_note, midi_channel;
		bool midi_on;
	
		int position;
		int tiltAngle;	
	
		float rotX, rotY;
		bool paused;
		bool manually;
		int scale;
	
		ofxControlPanel gui;
		void eventsIn(guiCallbackData & data);
};

#endif
