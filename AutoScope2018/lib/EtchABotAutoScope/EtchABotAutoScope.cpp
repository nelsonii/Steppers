/* EtchABotAutoScope.cpp - library for EtchABot motor controls.
 * Created by Debra Ansell, October 22, 2015
 * www.geekmomprojects.com
 * 
 * RN:
 * 
 * Extended and modified to support automatic microscope project
 * 
 * Stepper X is on Mega Digital Pins 4,5,6,7
 * Stepper Y is on Mega Digital Pins 8,9,10,11
 * (I shifted these around to free up pins 2 and 3 for interrupt use)
 * 
 */

#include "Arduino.h"
#include "EtchABotAutoScope.h"

// Constructor definition for arbitrary wiring of motors to Arduino pins.  The step order 
// is different for the horizontal motor because the positive rotation direction is 
// clockwise for the horizontal motor, but counter-clockwise for the vertical and erase motors.
EtchABotAutoScope::EtchABotAutoScope(int type, int h1=7, int h2=6, int h3=5, int h4=4,   //Motor pin connections
				  int v1=11, int v2=10, int v3=9, int v4=8): _stepperHorz(STEPS_PER_ROT, h4, h2, h3, h1),
			      _stepperVert(STEPS_PER_ROT, v3, v1, v4, v2)
{
	if (type == TRAVEL_SIZE) {
		_xMax = MAX_X_TRAVEL;
		_yMax = MAX_Y_TRAVEL;
	} else {
		_xMax = MAX_X_POCKET;
		_yMax = MAX_Y_POCKET;
	}
	
	_etchType = type;
	_motorPins[0] = h1;
	_motorPins[1] = h2;
	_motorPins[2] = h3;
	_motorPins[4] = h4;
	_motorPins[5] = v1;
	_motorPins[6] = v2;
	_motorPins[7] = v3;
	_motorPins[8] = v4;
	
	pinMode(h1, OUTPUT);
	pinMode(h2, OUTPUT);
	pinMode(h3, OUTPUT);
	pinMode(h4, OUTPUT);
	pinMode(v1, OUTPUT);
	pinMode(v2, OUTPUT);
	pinMode(v3, OUTPUT);
	pinMode(v4, OUTPUT);

}

// Constructor definition using default wiring values.  The step order is different for the
// horizontal motor because the positive rotation direction is clockwise for the horizontal motor, 
// but counter-clockwise for the vertical and erase motors.
EtchABotAutoScope::EtchABotAutoScope(int type) : _stepperHorz(STEPS_PER_ROT, 4, 6, 5, 7),
			                                           _stepperVert(STEPS_PER_ROT, 9, 11, 8, 10)
{
	if (type == TRAVEL_SIZE) {
		_xMax = MAX_X_TRAVEL;
		_yMax = MAX_Y_TRAVEL;
	} else {
		_xMax = MAX_X_POCKET;
		_yMax = MAX_Y_POCKET;
	}
	_etchType = type;
	_motorPins[0] = 7;
	_motorPins[1] = 5;
	_motorPins[2] = 4;
	_motorPins[4] = 6;
	_motorPins[5] = 8;
	_motorPins[6] = 10;
	_motorPins[7] = 9;
	_motorPins[8] = 11;

	
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(11, OUTPUT);

}

 
// True if we've changed directions horizontally
boolean EtchABotAutoScope::horzDirChange(uint8_t dir) {
  if (_prevHorzDir && dir != _prevHorzDir) {
    _prevHorzDir = dir;
    return true;
  } else if (!_prevHorzDir) {
    _prevHorzDir = dir;
  }
  return false;
}

// True if we've changed directions vertically
boolean EtchABotAutoScope::vertDirChange(uint8_t dir) {
  if (_prevVertDir && dir != _prevVertDir) {
    _prevVertDir = dir;
    return true;
  } else if (!_prevVertDir) {
      _prevVertDir = dir;
  }
  return false;
}

// Draws a line from the current position to the end position.  If we are at the end position, returns 0.
// uses Bresenham's line algorithm to compute the next step.  Implementation of Bresenham taken from:
// http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C.2B.2B
void EtchABotAutoScope::drawLine(int targetX, int targetY, boolean motorShutOff) {
 
  // Boundary check
  if (targetX < _xMin) targetX = _xMin;
  if (targetX > _xMax) targetX = _xMax;
  if (targetY < _yMin) targetY = _yMin;
  if (targetY > _yMax) targetY = _yMax;
  
  int dx = abs(targetX - _currentX);
  int sx = _currentX < targetX ? 1 : -1;
  int dy = abs(targetY - _currentY);
  int sy = _currentY < targetY ? 1 : -1;
  int err = (dx > dy ? dx : -dy)/2;
  int e2;
    
  // Deal with backlash (if any) before stepping.  Step the motor by the backlash
  // amount without incrementing position.  
  if (dx && horzDirChange(sx)) {
    //Serial.println("horizontal direction change");
    for (int i = 0; i < _hBacklash; i++) {
      _stepperHorz.setSpeed(700);
      _stepperHorz.step(sx);
      delay(STEP_DELAY);
    }
  }

  if (dy && vertDirChange(sy)) {
    //Serial.println("vertical direction change");
    for (int i = 0; i < _vBacklash; i++) {
      _stepperVert.setSpeed(700);
      _stepperVert.step(sy);
      delay(STEP_DELAY);
    }
  }
  
  // Bresenham's algorithm implemented below
  boolean thereYet = false;
  while(!thereYet) {
    
    e2 = err;
    if (e2 > -dx) { 
      err -= dy; 
      _stepperHorz.setSpeed(700);
      _stepperHorz.step(sx); 
      delay(STEP_DELAY);
      _currentX += sx;
    }
    if (e2 < dy) { 
      err += dx; 
      _stepperVert.setSpeed(700);
      _stepperVert.step(sy); 
      delay(STEP_DELAY);
      _currentY += sy;
    }
    if(_currentX == targetX && _currentY == targetY) thereYet = true;
  }

  if (motorShutOff) {
	turnOffHorzMotor();
	turnOffVertMotor();
  }
}



void EtchABotAutoScope::turnOffHorzMotor() {
	for (int i = 0; i < 4; i++) {
		digitalWrite(_motorPins[i], LOW);
	}
}

void EtchABotAutoScope::turnOffVertMotor() {
	for (int i = 4; i < 8; i++) {
		digitalWrite(_motorPins[i], LOW);
	}
}

void EtchABotAutoScope::turnOffMotors() {
	turnOffHorzMotor();
	turnOffVertMotor();
}
