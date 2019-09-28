//AutoScope

#include <Stepper.h>
#include "EtchABot.h"
#include <IRremote.h>

// Note: There are multiple versions of the IRremote class.
//       Originally, I used the newest development version from github
//       but I found a slimmed down version on Brainy Bits, and used that.
//       The slimmed down version does everything we need and brought the
//       program memory usage down 6%. Every bit helps.
 
//=================================================================================

// Constants 

// Code version. Used for developer reference.
const long APP_VER = 2016050401;

// Turns on/off Serial Output debugging
// Turning off debugging saves a considerable amount of memory.
const boolean DEBUG = 0;
const short BAUD = 9600;

// How much movement per action. A multiplier.
// For the microscope gears, 50 steps is roughly a mm.
const int MAX_DISTANCE = 50;

// Connect a 38KHz remote control sensor to the pin below
const int IRpin = 12;

// Mapping of actions to IR codes.
// These codes depend on the remote you are using.
// Test the remote responses to get the codes and
// then update this list as necessary.
const long MOVE_UP    =   16621663;  // Up Arrow
const long MOVE_DOWN  =   16625743;  // Down Arrow
const long MOVE_LEFT  =   16584943;  // Left Arrow
const long MOVE_RIGHT =   16601263;  // Right Arrow
const long MOVE_HOME  =   16609423;  // "Return" / Back
const long MOVE_STOP  =   16605343;  // Stop/Mode
const long KEY_SETUP  =   16589023;  // Enter/Exit setup (direct coordinate) mode
const long KEY_ENTER  =   16617583;  // Move to entered coordinates (setup mode)
const long KEY_0 = 16593103;
const long KEY_1 = 16582903;
const long KEY_2 = 16615543;
const long KEY_3 = 16599223;
const long KEY_4 = 16591063;
const long KEY_5 = 16623703;
const long KEY_6 = 16607383;
const long KEY_7 = 16586983;
const long KEY_8 = 16619623;
const long KEY_9 = 16603303;

//=================================================================================

// Define a stepper and the pins it will use
EtchABot etch(POCKET_SIZE);

// For reading IR -- this library is non-blocking (unlike the simplier ones)
IRrecv irRec(IRpin);
// For reading IR -- a decode data type used by the IRrecv library/class.
decode_results irResults;

//=================================================================================

void setup() {

  if (DEBUG) {
    Serial.begin(BAUD);
    Serial.print("AutoScope ");
    Serial.println(APP_VER);
  }

  // Start the IR receiver. The IRrecv library is extremely powerful, but 
  // takes a lot of memory. I may swap out with a simplified version. The
  // primary reason I use it is because it's NON-BLOCKING -- other things 
  // can go on while it looks for IR activity. Very important for the 
  // continious running of the motors (see "running" flag below). 
  irRec.enableIRIn(); // Start the IR receiver

} //setup

//=================================================================================

  // Global Variables
  // Define and set to default value

  //Normalized movement of X and Y. Normalization is needed for joystick control.
  //For the IR control, it's always going to be a whole number (-1, 0, 1)
  int xNorm = 0;
  int yNorm = 0;
  
  // The "running" flag is used with the IR control in mind. It allows a single
  // press of the remote to keep running/looping until another button (or limit)
  // is pressed (reached). This means you don't have to hold down a button.
  // Just press it once and it runs. The joystick only runs when moved.
  boolean running = false;

//=================================================================================

void loop() {


   if (irRec.decode(&irResults)) {

    long irCode = (long) irResults.value;

    if (DEBUG) {
      Serial.print("IR Code:");
      Serial.println(irCode);
    }
    
    switch (irCode) {
      case KEY_SETUP:
        if (DEBUG) Serial.println("Enter Setup Mode, for direct entry of coordinates.");

        //TODO
      
      case KEY_ENTER:
        if (DEBUG) Serial.println("Move to entered coordinates (only if in Setup Mode).");

        //TODO
      
      case MOVE_UP:
        if (DEBUG) Serial.println("Slew Up Until Stopped...");
        xNorm = 0;
        yNorm = -1;
        running = true;
        break;
      case MOVE_DOWN:
        if (DEBUG) Serial.println("Slew Down Until Stopped...");
        xNorm = 0;
        yNorm = 1;
        running = true;
        break;
      case MOVE_LEFT:
        if (DEBUG) Serial.println("Slew Left Until Stopped...");
        xNorm = -1;
        yNorm = 0;
        running = true;
        break;
      case MOVE_RIGHT:
        if (DEBUG) Serial.println("Slew Right Until Stopped...");
        xNorm = 1;
        yNorm = 0;
        running = true;
        break;
      case MOVE_HOME:
        if (DEBUG) Serial.println("Move to Home and Stop.");
        xNorm = -1;
        yNorm = -1;
        running = true;
        break;
      case MOVE_STOP:
        if (DEBUG) Serial.println("Stopping.");
        xNorm = 0;
        yNorm = 0;
        running = false;
        break;
    }//switch
    // Receive the next value
    irRec.resume(); 
  }//irDecode

  //Only activate the motor code if the "running" flag is True
  if (running) {
    
    // Figure out what the Target X and Y will be.
    int targetX = (int) round(etch.getX() + MAX_DISTANCE*xNorm);
    int targetY = (int) round(etch.getY() + MAX_DISTANCE*yNorm);
  
    // Limit movement to values between zero and MaxX/MaxY
    // (defined in EtchABot code -- uses the _SIZE when class instantiated)
    // Note that this is "belt and suspenders" code -- the etch class also keeps track of limits
    targetX = constrain(targetX, 0, etch.getMaxX());
    targetY = constrain(targetY, 0, etch.getMaxY());
      
    if (DEBUG) {
      Serial.print(targetX);
      Serial.print(",");
      Serial.println(targetY);
    }
    
    //Move to targeted location
    //TargetX, TargetY, MotorShutOff 
    etch.drawLine(targetX, targetY, true);

    // If both Target X and Y are Zero ("not"), turn off running. Needed for Home command.
    if (!targetX && !targetY) {running = false;}

    // TODO: If limits hit, stop. How to handle is one limit is set, but other not?

  }//running
    
} //loop
