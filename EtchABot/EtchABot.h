/*
 * EtchABot.h - Library for controlling the EtchABot.
 * Created by Debra Ansell (geekmomprojects), October 22, 2015.
 * www.geekmomprojects.com
 */
#ifndef EtchABot_h
#define EtchABot_h

#include "Arduino.h"
#include <Stepper.h>  // You MUST include the stepper library in your sketch
					  // as well because of Arduino IDE quirkiness in library includes
//#include <Time.h>

#define POCKET_SIZE 1 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:use for scope
#define TRAVEL_SIZE 2

// Actual backlash varies with each Etch-a-Sketch.  Need to run
// calibration to find the optimal values
#define DEFAULT_HORZ_BACKLASH 0 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:for scope testing, changed from default of 120 to 0.
#define DEFAULT_VERT_BACKLASH 0 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:for scope testing, changed from default of 120 to 0.

#define MOTOR_SPEED 1000  		// units are rev/min -- org:40 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:appears to be UNUSED
#define STEPS_PER_ROT 2048 		// when running steppers in 4-step mode
								// which is what <Stepper.h> uses

// Size of drawing area in units of stepper motor steps
#define MAX_X_TRAVEL 6500
#define MAX_Y_TRAVEL 4600
#define MAX_X_POCKET 30000 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:modified for scope. Much more travel than the etch-a-sketch.
#define MAX_Y_POCKET 10000 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:modified for scope. Much more travel than the etch-a-sketch.

// Give the pen time to get to the right position
#define STEP_DELAY 3 // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RN/DK:modified for scope. Was: 3

class EtchABot
{
  private:
    int _hBacklash = DEFAULT_HORZ_BACKLASH;
	int _vBacklash = DEFAULT_VERT_BACKLASH;
	int _etchType;
	int _xMin=0, _yMin = 0, _xMax, _yMax;
	int _currentX = 0, _currentY = 0;  //Assume we're starting at the upper left corner always
	uint8_t _prevHorzDir = 0, _prevVertDir = 0;
	uint8_t _motorPins[8];   //List of motor pins used (in order horz, vert);
	Stepper _stepperHorz, _stepperVert;
	
	boolean horzDirChange(uint8_t dir);
	boolean vertDirChange(uint8_t dir);
	
  public:
    EtchABot(int type, int h1, int h2, int h3, int h4, int v1, int v2, int v3, int v4);
	EtchABot(int type);		
	
    void drawLine(int targetX, int targetY, boolean motorShutOff=false);
	//void drawArc(int xCenter, int yCenter, int nSegs, float degrees);
	void turnOffMotors();
	void turnOffHorzMotor();
	void turnOffVertMotor();
	
	// Get/set functions
	int getType() {return _etchType;}
	int getX() {return _currentX;}
	int getY() {return _currentY;}
	int getMaxX() {return _xMax;}
	int getMaxY() {return _yMax;}
	int getHBacklash() {return _hBacklash;}
	int getVBacklash() {return _vBacklash;}
	void setHBacklash(int b) {_hBacklash = b;}
	void setVBacklash(int b) {_vBacklash = b;}
	
};

#endif