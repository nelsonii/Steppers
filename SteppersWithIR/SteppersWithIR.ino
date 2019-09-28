#include <Stepper.h>
#include "EtchABot.h"
#include "Adafruit_NECremote.h"
 
//=================================================================================

// How much movement per action. A multiplier.
#define MAX_DISTANCE 100

// Connect a 38KHz remote control sensor to the pin below
#define IRpin       11

//=================================================================================

// Define a stepper and the pins it will use
EtchABot etch(POCKET_SIZE);

// remote object (for reading IR)
Adafruit_NECremote remote(IRpin);


//=================================================================================

void setup() {


  //For debugging / console feedback
  Serial.begin(9600);

} //setup

//=================================================================================

  int lastcode = -1;
  int xNorm = 0;
  int yNorm = 0;


//=================================================================================

void loop() {

  // ----------------------------------------------------

        // Move Home
        //Serial.println("Moving to home position (0,0)...");
        //etch.drawLine(0, 0, true); //go home (0,0) -- this will eventually use stop switches

  // ----------------------------------------------------

  int IRcode = remote.listen(); 

  if (IRcode >= 0) {
    lastcode = IRcode;
  }
  
  switch (lastcode) {
    //5=Arrow Up
    case 5:
      xNorm = 0;
      yNorm = 1;
      break;
    //13=Arrow Down
    case 13:
      xNorm = 0;
      yNorm = -1;
      break;
    //8=Arrow Left
    case 8:
      xNorm = 1;
      yNorm = 0;
      break;
    //10=Arrow Right
    case 10:
      xNorm = -1;
      yNorm = 0;
      break;
    default:
      xNorm = 0;
      yNorm = 0;
      break;
  }
  
  // Figure out what the Target X and Y will be.
  int targetX = (int) round(etch.getX() + MAX_DISTANCE*xNorm);
  int targetY = (int) round(etch.getY() + MAX_DISTANCE*yNorm);

  // Limit movement to values between zero and MaxX/MaxY
  // (defined in EtchABot code -- uses the _SIZE when class instantiated)
  targetX = constrain(targetX, 0, etch.getMaxX());
  targetY = constrain(targetY, 0, etch.getMaxY());

  // For debugging
  /*
  Serial.print(" (");
  Serial.print("tX:");
  Serial.print(targetX);
  Serial.print(", tY:");
  Serial.print(targetY);
  Serial.println(") ");
  */
  

  //Move to targeted location
  //TargetX, TargetY, MotorShutOff 
  etch.drawLine(targetX, targetY, true);

  // ----------------------------------------------------

} //loop
