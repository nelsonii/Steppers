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

  int xNorm = 0;
  int yNorm = 0;
  boolean running = false;

//=================================================================================

void loop() {

  int IRcode = remote.listen(0.25); 

  if (IRcode >= 0) {
    switch (IRcode) {
      case 5:
        //5=Arrow Up
        xNorm = 0;
        yNorm = 1;
        running = true;
        break;
      case 13:
        //13=Arrow Down
        xNorm = 0;
        yNorm = -1;
        running = true;
        break;
      case 8:
        //8=Arrow Left
        xNorm = 1;
        yNorm = 0;
        running = true;
        break;
      case 10:
        //10=Arrow Right
        xNorm = -1;
        yNorm = 0;
        running = true;
        break;
      default:
        xNorm = 0;
        yNorm = 0;
        running = false;
        break;
    }//switch
  }//if(IRCode)
  
  if (running) {
    // Figure out what the Target X and Y will be.
    int targetX = (int) round(etch.getX() + MAX_DISTANCE*xNorm);
    int targetY = (int) round(etch.getY() + MAX_DISTANCE*yNorm);
  
    // Limit movement to values between zero and MaxX/MaxY
    // (defined in EtchABot code -- uses the _SIZE when class instantiated)
    // Note that this is "belt and suspenders" code -- the etch class also keeps track of limits
    targetX = constrain(targetX, 0, etch.getMaxX());
    targetY = constrain(targetY, 0, etch.getMaxY());
  
    //Move to targeted location
    //TargetX, TargetY, MotorShutOff 
    etch.drawLine(targetX, targetY, true);
  }//if(running)
  
  // ----------------------------------------------------

} //loop
