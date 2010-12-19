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


#include "Therenect.h"


ofxCvKalman *vPointSmoothed[3];
ofxCvKalman *pPointSmoothed[3];

//--------------------------------------------------------------
void Therenect::setup()
{
	ofSetWindowTitle("Therenect 0.9.1");
	ofSetVerticalSync(true);
	
	kinect.init();
	kinect.setVerbose(true);
	kinect.enableDepthNearValueWhite(true);
	kinect.open();
	
	paused = false;

	//testImage.loadImage("test2.png");
	//testImage.setImageType(OF_IMAGE_GRAYSCALE);
	
	depthImage.allocate(kinect.width, kinect.height);
	controlImage.allocate(kinect.width, kinect.height);

	position  = 165;
	tiltAngle = 15;
	kinect.setCameraTiltAngle(tiltAngle);
	
	rotX = -5;
	rotY = -30;

	sampleRate = 44100;
	bufferSize = 400;
	
	frequency	= 0.0f;
	amplitude	= 0.0f;
	
	amplset		= amplitude;
	freqset		= frequency;
	range		= 75;

	volumePoint.x = kinect.width/4;
	volumePoint.y = 4*kinect.height/5;
	volumePoint.z = position;
	
	pitchPoint.x = 2*kinect.width/3;
	pitchPoint.y = kinect.height/2;
	pitchPoint.z = position;
	
	for (int i=0;i<3;i++) vPointSmoothed[i] = NULL;
	for (int i=0;i<3;i++) pPointSmoothed[i] = NULL;
	
	ofSoundStreamSetup(1,0,this, sampleRate, bufferSize, 2);
	sound_data = new float[bufferSize];
	manually = false;
	scale = 0;
	
	midi_note = 0;
	midi_on = false;
	midi_channel = 1;
	midi.listPorts();
	
	oscmode = 0;
	drawing = false;
	
	ofxControlPanel::setBackgroundColor(simpleColor(30, 30, 30, 200));
	gui.setup("Settings", 840, 15, 265, 545);
	gui.loadSettings("TherenectSettings.xml");
	
	gui.addPanel("", 4, false);
	gui.setWhichPanel(0);
	
	vector <string> wave_names;
	wave_names.push_back("Theremin");
	wave_names.push_back("Sinewave");
	wave_names.push_back("Sawtooth");
	wave_names.push_back("Squarewave");
	gui.addTextDropDown("Waveform", "WAVE", 0, wave_names);
	
	
	gui.addSlider("Frequency Range", "FREQ_RANGE",(range-50)*2, 0.0, 100.0, false);	

	
	vector <string> scale_names;
	scale_names.push_back("Continuous");
	scale_names.push_back("Chromatic");
	scale_names.push_back("Ionian/Major");
	scale_names.push_back("Pentatonic");
	gui.addTextDropDown("Musical Scale", "SCALE", 0, scale_names);
	
	gui.addSlider("Antenna distance", "ANTENNA_DISTANCE", 255-position, 0.0, 255.0, true);	
	gui.addSlider("Kinect angle", "KINECT_ANGLE", 15.0, -32.0, 32.0, true);	
	
	if (midi.portNames.size()>0) {
		gui.addToggle("MIDI enabled", "MIDI_ENABLED", midi_on);		
		gui.addTextDropDown("MIDI device", "MIDI_DEVICE", 0, midi.portNames);
		gui.addSlider("MIDI channel", "MIDI_CHANNEL", 1.0, 1.0, 16.0, true);
	} else midi_on = false;
	
	gui.setupEvents();
	gui.enableEvents();
	ofAddListener(gui.guiEvent, this, &Therenect::eventsIn);
	
	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void Therenect::update()
{
	kinect.update();

	unsigned short *depth = kinect.getRawDepthPixels();
	//controlImage.setFromPixels(testImage.getPixels(), kinect.width, kinect.height);
	controlImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	controlImage.mirror(false, true);
	unsigned char *pix = controlImage.getPixels();
	int numPixels = controlImage.getWidth() * controlImage.getHeight()-1;
	depthImage.setFromPixels(pix, kinect.width, kinect.height);
	depthImage.flagImageChanged();
	


	vReferencePoint.set(-100000, -100000, -100000);
	pReferencePoint.set(-100000, -100000, -100000);
	
	float closestPitch=100000.0f;
	float closestVolume=100000.0f;

	long psum, vsum;
	double pxsum,pysum,pzsum;
	double vxsum,vysum,vzsum;
	psum = pxsum = pysum = pzsum = 0;
	vsum = vxsum = vysum = vzsum = 0;
	
	for(int i = numPixels; i >= 0 ; i--){
				
		if (pix[i]<32) {
			pix[i]=0;
			continue;
		}
		
		int y = i/kinect.width;
		int x = i-y*kinect.width;
		int index = y*kinect.width+(kinect.width-x)-1;
		float dpt = 255.0f-((depth[index]-200)/920.0f)*255.0f;
		if (depth[index]==2047) dpt=0.0f;
		
		float dx = (pitchPoint.x - x)/(float)kinect.width;
		float dy = (pitchPoint.y - y)/(float)kinect.width;
		float dz = (pitchPoint.z - dpt)/255.0f;
		float pitchDistance = dx*dx + dy*dy + dz*dz;

		dx = (volumePoint.x - x)/(float)kinect.width;
		dy = (volumePoint.y - y)/(float)kinect.height;
		dz = (volumePoint.z - dpt)/255.0f;
		float volumeDistance = dx*dx + dy*dy + dz*dz;
		
		if (pitchDistance<0.064f) {
			pix[i] = 255-(unsigned char)floor(pitchDistance*4000.0f);
			double weight = (pix[i]*pix[i])/255.0f;

			psum+=weight;
			pxsum+=x*weight;
			pysum+=y*weight;
			pzsum+=dpt*weight;

			if (pitchDistance<closestPitch) {
				pReferencePoint.set(x,y,dpt);
				closestPitch=pitchDistance;
			}
		} else if (volumeDistance<0.064f) {
			pix[i] = 255-(unsigned char)floor(volumeDistance*4000.0f);
			double weight = (pix[i]*pix[i])/127.0f;

			vsum+=weight;
			vxsum+=x*weight;
			vysum+=y*weight;
			vzsum+=dpt*weight;

			if (volumeDistance<=closestVolume) {
				vReferencePoint.set(x,y,dpt);
				closestVolume=volumeDistance;
			}
		} else pix[i]=0;
		
	}
	
	
	
	
	
	controlImage.flagImageChanged();
	
	vsum+=16384;
	vxsum+=vReferencePoint.x*16384;
	vysum+=vReferencePoint.y*16384;
	vzsum+=vReferencePoint.z*16384;

	psum+=16384;
	pxsum+=pReferencePoint.x*16384;
	pysum+=pReferencePoint.y*16384;
	pzsum+=pReferencePoint.z*16384;
		
	if (psum>0) pReferencePoint.set(pxsum/psum, pysum/psum, pzsum/psum);
	if (vsum>0) vReferencePoint.set(vxsum/vsum, vysum/vsum, vzsum/vsum);

	

	if (vControlPoint.z>127)  vControlPoint=vReferencePoint;	
	else if (vReferencePoint.x<0) {
		for (int i=0;i<3;i++) {
			if (vPointSmoothed[i]!=NULL) {
				delete vPointSmoothed[i];
				vPointSmoothed[i] = NULL;
			}
		}
		vControlPoint = vReferencePoint;			
	} else if (vPointSmoothed[0] == NULL) {
		vPointSmoothed[0] = new ofxCvKalman(vReferencePoint.x);
		vPointSmoothed[1] = new ofxCvKalman(vReferencePoint.y);
		vPointSmoothed[2] = new ofxCvKalman(vReferencePoint.z);
		vControlPoint = vReferencePoint;
	} else {
		vControlPoint.x = vPointSmoothed[0]->correct(vReferencePoint.x);
		vControlPoint.y = vPointSmoothed[1]->correct(vReferencePoint.y);
		vControlPoint.z = vPointSmoothed[2]->correct(vReferencePoint.z);
	}


	if (pControlPoint.z>127) pControlPoint=pReferencePoint;
	else if (pReferencePoint.x<0) {
		for (int i=0;i<3;i++) {
			if (pPointSmoothed[i]!=NULL) {
				delete pPointSmoothed[i];
				pPointSmoothed[i] = NULL;
			}
		}
		pControlPoint = pReferencePoint;
	} else if (pPointSmoothed[0] == NULL) {
		pPointSmoothed[0] = new ofxCvKalman(pReferencePoint.x);
		pPointSmoothed[1] = new ofxCvKalman(pReferencePoint.y);
		pPointSmoothed[2] = new ofxCvKalman(pReferencePoint.z);
		pControlPoint = pReferencePoint;
	} else {
		pControlPoint.x = pPointSmoothed[0]->correct(pReferencePoint.x);
		pControlPoint.y = pPointSmoothed[1]->correct(pReferencePoint.y);
		pControlPoint.z = pPointSmoothed[2]->correct(pReferencePoint.z);
	}


	
	
	
	float dx = (pControlPoint.x- pitchPoint.x)/(float)kinect.width;
	float dy = (pControlPoint.y- pitchPoint.y)/(float)kinect.width;
	float dz = (pControlPoint.z- pitchPoint.z)/255.0f;
	
	if (!manually) {
		double pitch_dist = dx*dx + dy*dy + dz*dz;
		double pitch = ((range/10.0f)-1)-(range*pitch_dist);
		freqset = 8.175*pow(2,pitch);
		if (freqset<8.175) freqset=1;
	}
		
	dx = (vControlPoint.x- volumePoint.x)/(float)kinect.width;
	dy = (vControlPoint.y- volumePoint.y)/(float)kinect.width;
	dz = (vControlPoint.z- volumePoint.z)/127.0f;
		
	if (!manually) {
		double volume_dist = dx*dx + dy*dy + dz*dz;
		double volume = volume_dist*16.0f;
		amplset = volume;
		if (amplset>0.5f) amplset=0.5f;
		else if (amplset<0.0f) amplset=0.0f;
		if (freqset==1) amplset=0.0f;
	}
	
	gui.update();
}

//--------------------------------------------------------------
void Therenect::draw()
{
	ofBackground(128, 128, 128);
	if (paused) return;
	
	ofSetColor(0, 0, 0);
	ofRect(425, 15, 400, 300);
	ofPushMatrix();
	ofTranslate(425, 15);
	ofSetColor(255, 255, 255);
	drawPointCloud();
	ofPopMatrix();
	ofSetColor(128, 128, 128);
	ofRect(415, 15, 10, 300);
	ofRect(825, 15, 200, 300);
	
	ofSetColor(255, 255, 255);
	depthImage.draw(15, 15, 400, 300);
	controlImage.draw(15, 325, 400, 300);
	ofSetColor(200, 0, 200);
	ofCircle(15+pControlPoint.x/(float)kinect.width*400,325+pControlPoint.y/(float)kinect.height*300,pControlPoint.z/64+4);
	ofCircle(15+vControlPoint.x/(float)kinect.width*400,325+vControlPoint.y/(float)kinect.height*300,vControlPoint.z/64+4);

	ofSetColor(0, 0, 0);
	ofRect(425, 325, 400, 300);

	drawing = true;
	ofSetColor(255, 255, 255);
	float start = sound_data[0];
	
	for (int i = 1; i < bufferSize; i++){
		float end = sound_data[i];
		float xpos = 425 + (float)i/(bufferSize)*400;
		ofLine(xpos, 475+start*300, xpos+1, 475+end*300);
		start=end;
	}
	drawing = false;
	
	if (midi_note) {
		char midiStr[8];
		sprintf(midiStr, "%d", midi_note);
		ofDrawBitmapString(midiStr, 430, 340);
	}
	
	if (pControlPoint.z < pitchPoint.z) {
		ofSetColor(200, 0, 200);
		ofCircle(425+pControlPoint.x/(float)kinect.width*400,325+pControlPoint.y/(float)kinect.height*300,pControlPoint.z/16+4);
		ofSetColor(0, 0, 200);
		ofCircle(425+pitchPoint.x/(float)kinect.width*400,325+pitchPoint.y/(float)kinect.height*300,volumePoint.z/16+4);
	} else {
		ofSetColor(0, 0, 200);
		ofCircle(425+pitchPoint.x/(float)kinect.width*400,325+pitchPoint.y/(float)kinect.height*300,volumePoint.z/16+4);
		ofSetColor(200, 0, 200);
		ofCircle(425+pControlPoint.x/(float)kinect.width*400,325+pControlPoint.y/(float)kinect.height*300,pControlPoint.z/16+4);
	}
	
	if (vControlPoint.z < volumePoint.z) {
		ofSetColor(200, 0, 200);
		ofCircle(425+vControlPoint.x/(float)kinect.width*400,325+vControlPoint.y/(float)kinect.height*300,vControlPoint.z/16+4);
		ofSetColor(0, 0, 200);
		ofCircle(425+volumePoint.x/(float)kinect.width*400,325+volumePoint.y/(float)kinect.height*300,volumePoint.z/16+4);
	} else {
		ofSetColor(0, 0, 200);
		ofCircle(425+volumePoint.x/(float)kinect.width*400,325+volumePoint.y/(float)kinect.height*300,volumePoint.z/16+4);
		ofSetColor(200, 0, 200);
		ofCircle(425+vControlPoint.x/(float)kinect.width*400,325+vControlPoint.y/(float)kinect.height*300,vControlPoint.z/16+4);
	}

	ofSetColor(255, 255, 255);
	ofDrawBitmapString("(c) 2010 Martin Kaltenbrunner\nInterface Culture Lab\nKunstuniversitaet Linz, Austria", 850, 595);
		
	gui.draw();
}

//--------------------------------------------------------------

void Therenect::drawPointCloud() {
	
	int step = 8;
	ofTranslate(380, 120,-740);
	ofRotateX(rotX);
	ofRotateY(rotY);
	ofTranslate(380,120,-740);
	
	unsigned char *pix = depthImage.getPixels();
	for (int j = 0; j < kinect.height; j+=step) {
		for (int i = 0; i < kinect.width; i+=step) {
			float distance = pix[j*kinect.width+i];
			if (distance==0) continue;
			
			ofPushMatrix();
			ofTranslate(i - kinect.width, j - kinect.height, distance*5 );
			ofRotateX(-rotX);
			ofRotateY(-rotY);
			ofSetColor(distance,distance,distance);
			ofCircle(0, 0, step);
			//ofLine(0, 0, 1, 1);
			ofPopMatrix();
		}
	}
	
	ofSetColor(0,0,200);
	ofPushMatrix();
	ofTranslate(pitchPoint.x - kinect.width, pitchPoint.y - kinect.height, pitchPoint.z*5 );
	ofRotateX(-rotX);
	ofRotateY(-rotY);
	ofCircle(0, 0, step);
	ofPopMatrix();
	ofSetColor(0,0,200);
	ofPushMatrix();
	ofTranslate(volumePoint.x - kinect.width, volumePoint.y - kinect.height, volumePoint.z*5 );
	ofRotateX(-rotX);
	ofRotateY(-rotY);
	ofCircle(0, 0, step);
	ofPopMatrix();
	
	ofSetColor(200,0,200);
	ofPushMatrix();
	ofTranslate(pReferencePoint.x - kinect.width, pReferencePoint.y - kinect.height, pReferencePoint.z*5 );
	ofRotateX(-rotX);
	ofRotateY(-rotY);
	ofCircle(0, 0, step);
	ofPopMatrix();
	ofSetColor(200,0,200);
	ofPushMatrix();
	ofTranslate(vReferencePoint.x - kinect.width, vReferencePoint.y - kinect.height, vReferencePoint.z*5 );
	ofRotateX(-rotX);
	ofRotateY(-rotY);
	ofCircle(0, 0, step);
	ofPopMatrix();
	
}

//--------------------------------------------------------------
void Therenect::audioRequested(float *output, int bufferSize, int nChannels){	
	
	if (amplset>0.5f) amplset=0.5f;
	
	int ionian_table[12]     = {0,0,2,2,4,5,5,7,7,9,9,11};
//								C C D D E F F G G A A H
	int pentatonic_table[12] = {0,0,2,2,4,4,7,7,7,9,9,9 };
//								C C D D E E G G G A A A
	
	
	if (scale) {

		int new_midi_note;
		if (scale==1) {
			new_midi_note = 69 + round(12.0f * log2(freqset/440.0f));
			frequency = 440.0f * pow(2, (new_midi_note-69)/12.0f);
			//printf("%d\n",midi_note);
			//printf("%d %f\n",midi_note, frequency);
		} else if (scale==2) {
			new_midi_note = 69 + round(12 * log2(freqset/440.0f));
			int note = new_midi_note%12;
			int base_note = new_midi_note - note;
			new_midi_note = base_note + ionian_table[note];
			frequency = 440.0f * pow(2, (new_midi_note-69)/12.0f);
			//printf("%d\n",midi_note);
			//printf("%d %f\n",midi_note, frequency);
		} else if (scale==3) {
			
			new_midi_note = 69 + round(12 * log2(freqset/440.0f));
			int note = new_midi_note%12;
			int base_note = new_midi_note - note;
			new_midi_note = base_note + pentatonic_table[note];
			frequency = 440.0f * pow(2, (new_midi_note-69)/12.0f);
			
			//printf("%d\n",midi_note);
			//printf("%d %f\n",midi_note, frequency);
		}
		
		if (midi_on) {
			if (frequency<32.7) new_midi_note=0;
			int velocity = floor((amplset/0.5f)*127);
			midi.sendControlChange(midi_channel, 7, velocity);
			if (new_midi_note!=midi_note) {
				//printf("%f %d %d\n",freqset,midi_note, new_midi_note);
				midi.sendNoteOff(midi_channel, midi_note, 0);
				midi.sendNoteOn(midi_channel, new_midi_note, 127);
				midi_note = new_midi_note;
			}
			
			for (int i = 0; i < bufferSize; i++){
				output[i] = 0.0f;
				sound_data[i] = 0.0f;
			}
			return;
		}
		
	}  

	
	float sample = 0;
	float phase = 0;
	
	float step = pow(2,fabs(frequency-freqset)/(sampleRate/6))-0.96f;
	//if (fabs(frequency-freqset)>0) printf("%f %f %f\n",frequency, freqset, step);
	
	for (int i = 0; i < bufferSize; i++){
		
		if (frequency!=freqset) {
			float freqdiff = freqset-frequency;
			
			if (fabs(freqdiff)<0.04f) frequency = freqset;
			else if (freqdiff>step) frequency+=step;
			else frequency-=step;
		}
		
		if (amplitude!=amplset) {
			float ampldiff = amplset-amplitude;
			if (fabs(ampldiff)<0.00005) amplitude = amplset;
			else if (ampldiff>0) amplitude+=0.00005;
			else amplitude-=0.00005;
		}
		
		switch (oscmode) {
			case 0:
				rotation += (((frequency/2.0f) / sampleRate) * TWO_PI);
				while (rotation > TWO_PI) rotation -= TWO_PI;
				sample = (fabs(sin(rotation))-0.5f)*amplitude*2.0f;
				break;
			case 1:
				rotation += (frequency / sampleRate) * TWO_PI;
				while (rotation > TWO_PI) rotation -= TWO_PI;
				sample = sin(rotation)*amplitude;
				break;
			case 2:
				rotation += (frequency / sampleRate) * TWO_PI;
				while (rotation > TWO_PI) rotation -= TWO_PI;
				sample = (rotation/PI-1.0f)*amplitude;
				break;
			case 3:
				rotation += (frequency / sampleRate) * TWO_PI;
				while (rotation > TWO_PI) rotation -= TWO_PI;
				sample = (floor(rotation/PI)-0.5f)*amplitude;
				break;

		}
		
		output[i] = sample;
		if (!drawing) sound_data[i] = sample;
	}
}

//--------------------------------------------------------------
void Therenect::exit(){
	kinect.close();
	delete[] sound_data;
}

//--------------------------------------------------------------
void Therenect::keyPressed (int key)
{
	switch (key)
	{	
		case '<':
		case ',':
			position ++;
			if (position > 255) position = 255;
			volumePoint.z = position;
			pitchPoint.z = position;
			gui.setValueI("ANTENNA_DISTANCE", 255-position, 0);
			break;
		case '>':		
		case '.':		
			position --;
			if (position < 0) position = 0;
			volumePoint.z = position;
			pitchPoint.z = position;
			gui.setValueI("ANTENNA_DISTANCE", 255-position, 0);
			break;
			
		case '+':
		case '=':
			tiltAngle ++;
			if (tiltAngle > 30) tiltAngle = 30;
			kinect.setCameraTiltAngle(tiltAngle);
			gui.setValueI("KINECT_ANGLE", tiltAngle, 0);
			break;
		case '-':		
			tiltAngle --;
			if (tiltAngle < -30) tiltAngle = -30;
			kinect.setCameraTiltAngle(tiltAngle);
			gui.setValueI("KINECT_ANGLE", tiltAngle, 0);
			break;
		case 'd':
			if (paused) {
				paused = false;
				ofSetWindowTitle("Therenect");
			} else {
				paused = true;
				ofSetWindowTitle("Therenect - display off");
			}
			break;
		case '0':
			oscmode = 0;
			gui.setValueI("WAVE", oscmode, 0);
			break;
		case '1':
			oscmode = 1;
			gui.setValueI("WAVE", oscmode, 0);
			break;
		case '2':
			oscmode = 2;
			gui.setValueI("WAVE", oscmode, 0);
			break;
		case '3':
			oscmode = 3;
			gui.setValueI("WAVE", oscmode, 0);
			break;
		case 'f':
			scale = 0;
			gui.setValueI("SCALE", scale, 0);
			break;
		case 'c':
			scale = 1;
			gui.setValueI("SCALE", scale, 0);
			break;
		case 'i':
			scale = 2;
			gui.setValueI("SCALE", scale, 0);
			break;
		case 'p':
			scale = 3;
			gui.setValueI("SCALE", scale, 0);
			break;
		case 'm':
			if (!midi_on) {
				midi.openPort();
				midi_note = 0;
				midi_on = true;
				gui.setValueB("MIDI_ENABLED", 1, 0);
				if (!scale) { 
					scale=1;
					gui.setValueI("SCALE", scale, 0);
				}
			} else {
				midi_on = false;
				midi.closePort();
				midi_note = 0;
				gui.setValueB("MIDI_ENABLED", 0, 0);
				scale=0;
				gui.setValueI("SCALE", scale, 0);
			}
			break;
	}
}

//--------------------------------------------------------------
void Therenect::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void Therenect::mouseDragged(int x, int y, int button)
{
	if ((x>425) && (x<825) && (y>15) && (y<315)) {
		rotY = 70 - (x-425)/400.0f*150;
	} else if ((x>425) && (x<825) && (y>325) && (y<725)) {
		amplset = (1-(float)(y-325)/300.0f)*.5f;
		freqset = 32.7*pow(2,6.0f*((x-425)/400.0f));
	}
	
	gui.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void Therenect::mousePressed(int x, int y, int button)
{
	if ((x>425) && (x<825) && (y>325) && (y<725)) {
		manually=true;
	}
	
	gui.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void Therenect::mouseReleased(int x, int y, int button)
{
	if ((x>425) && (x<825) && (y>325) && (y<725)) {
		manually = false;
	}
	
	gui.mouseReleased();
}

//--------------------------------------------------------------
void Therenect::windowResized(int w, int h)
{}

//--------------------------------------------------------------
void Therenect::eventsIn(guiCallbackData & data){
	
	string eventName = data.getXmlName();
	printf("event name is %s\n", eventName.c_str());
	
	if (eventName=="ANTENNA_DISTANCE") {
		position = 255-data.getInt(0);
		volumePoint.z = position;
		pitchPoint.z = position;
	} else if (eventName=="KINECT_ANGLE") {
		tiltAngle = data.getInt(0);
	} else if (eventName=="MIDI_ENABLED") {
		midi_on = data.getInt(0);		
		if (midi_on) {
			midi.openPort(gui.getValueI("MIDI_DEVICE", 0));
			if (!scale) { 
				scale=1;
				gui.setValueI("SCALE", scale, 0);
			}
		} else {
			midi.closePort();
			scale = 0;
			gui.setValueI("SCALE", scale, 0);
		}
	} else if (eventName=="MIDI_DEVICE") {
		if (midi_on) {
			midi.closePort();
			midi.openPort(gui.getValueI("MIDI_DEVICE", 0));
		}
	} else if (eventName=="MIDI_CHANNEL") {
		midi_channel = data.getInt(0);
	} else if (eventName=="WAVE") {
		oscmode = data.getInt(0);
	} else if (eventName=="SCALE") {
		scale = data.getInt(0);
	} else if (eventName=="FREQ_RANGE") {
		range = 50.0f+data.getFloat(0)/2.0f;
	}
	
	
}

