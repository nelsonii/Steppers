/*
 * Example of working 2 Thumbsticks (PS2 Thumbstick for Arduino) with Calibration
 * Connect first thumbstick X to A1, Y to A0
 *         second thumbstick X to A3,Y to A2
 */
//including a library
#include <Gamepad.h>

//initialize a centers of axises for calibration
int leftXcenter = 500;
int leftYcenter = 500;
double multiplierLX = 0.254;
double multiplierLY = 0.254;

//Initializing a Gamepad
Gamepad gp;
void setup() {
  //initializing inputs
  pinMode(A9, INPUT);
  pinMode(A8, INPUT);
  pinMode(7,  INPUT_PULLUP);
  calibrate();
}

void loop() {
  
  int lx, ly;
  lx = analogRead(A9);
  ly = analogRead(A8);
  
  //we need to convert a 0-1000 to -127 - 127
  lx = floor((lx - leftXcenter) * multiplierLX);
  ly = floor((ly - leftYcenter) * multiplierLY);
  if(lx > 127) lx = 127;
  if(ly > 127) ly = 127;
  
  gp.setLeftXaxis(lx);
  //because i have placed a thumbstick in breadboard, i must invert a Y axis and swap X and Y axises
  gp.setLeftYaxis(ly);
  
  int leftStickButton;
  leftStickButton = digitalRead(7);
  
  if(leftStickButton == LOW)
	  gp.setButtonState(10, true);
  else
	  gp.setButtonState(10, false);

  delay(20);
}

void calibrate()
{
  int lx, ly;
  int i = 0;
  while(i < 8)
  {
    lx = analogRead(A9);
    ly = analogRead(A8);
    bool validLX = lx > (leftXcenter - 100) && lx < (leftXcenter + 100);
    bool validLY = ly > (leftYcenter - 100) && ly < (leftYcenter + 100);
    if(validLX && validLY)
    {
      i++;
      //nothing to do here!
    }
    else i = 0;
    delay(20);
  }
  leftXcenter = lx;
  leftYcenter = ly;
  multiplierLX = (double)127 / (double)lx;
  multiplierLY = (double)127 / (double)ly;
}
