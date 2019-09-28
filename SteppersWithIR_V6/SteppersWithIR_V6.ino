// AutoScope
//
// A project by Dr. Dan Kott, Ron Nelson, and the Central Texas Makers team.
// Based on a concept by Dr. Shane Battye of PathOBin.com
// Using code from Geek Mom Projects, Brainy Bits, and Adafruit.
//
// Written: Summer of 2016
// Contact: dfkott (at) aol.com  -  ron.nelson@CentralTexasMakers.com

//=================================================================================

// Libraries

//For Stepper Motor Control and Movement
#include <Stepper.h>
#include "EtchABot.h"

// Note: Thanks to Geek Mom Projects for giving me a head-start on getting 
//       the stepper motors going. I based much of the motor movement logic
//       off of her well documented EtchABot project. It was extremely helpful
//       as Debra is using the same steppers and controllers as we are.
//       http://www.geekmomprojects.com/etchabot-a-cnc-etch-a-sketch/


//For IR Remote Control (input)
//#include <IRremote.h>
#include "src/IRremote/IRremote.h"

// Note: There are multiple versions of the IRremote class.
//       Originally, I used the newest development version from github
//       but I found a slimmed down version on Brainy Bits, and used that.
//       The slimmed down version does everything we need and brought the
//       program memory usage down 6%. Every bit helps.

// LCD Display over I2C
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

//=================================================================================

// Constants 

// Code version. Used for developer reference.
const long APP_VER = 2016051401;

// Turns on/off Serial Output debugging
// Turning off debugging saves a considerable amount of memory.
const boolean DEBUG = 0;
const short BAUD = 9600;

// How much movement per action. A multiplier.
// For the microscope gears, 50 steps is roughly a mm.
const int MAX_DISTANCE = 50;

// Connect a 38KHz remote control sensor to the pin below
const int IRpin = 12;

// Good old Pin 13 for a status LED. Lights up in certain modes.
const int LEDpin = 13;

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
const long KEY_0      =   16593103;
const long KEY_1      =   16582903;
const long KEY_2      =   16615543;
const long KEY_3      =   16599223;
const long KEY_4      =   16591063;
const long KEY_5      =   16623703;
const long KEY_6      =   16607383;
const long KEY_7      =   16586983;
const long KEY_8      =   16619623;
const long KEY_9      =   16603303;

// Used for the Manual Coordinate selection process
const int USER_ENTERING_X     = 1;
const int USER_ENTERING_Y     = 2;
const int USER_DONE           = 3;

//=================================================================================

// Define a stepper and the pins it will use
EtchABot etch(POCKET_SIZE);

// For reading IR -- this library is non-blocking (unlike the simplier ones)
IRrecv irRec(IRpin);
// For reading IR -- a decode data type used by the IRrecv library/class.
decode_results irResults;

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

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

  // Turn off the status LED
  digitalWrite (LEDpin, LOW);

  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  DisplayMainMenu();

} //setup

//=================================================================================

  // Global Variables
  // Define and set to default value

  // For the IR control, it's always going to be a whole number (-1, 0, 1)
  // For direct coordinate entry, it will be an absolute value / position;
  int xMoveAmount = 0;
  int yMoveAmount = 0;
  
  // The "running" flag is used with the IR control in mind. It allows a single
  // press of the remote to keep running/looping until another button (or limit)
  // is pressed (reached). This means you don't have to hold down a button.
  // Just press it once and it runs forever one incremented step at a time (-1 or 1 * MAX_DISTANCE)
  boolean runMotors = false;

  // The coordinate mode is used when a direct coordinate is entered. 
  // The motors should skew to that coordinate and then stop running.
  // This is unlike the "up/down/left/right" where we run forever one
  // incremented step at a time (-1/1 * MAX_DISTANCE)
  boolean coordinateMode = false;
  int coordinateStatus = USER_DONE;
  String buildX = "";
  String buildY = "";  

  // Used to capture the user's X and Y coordinates
  int coordinateX = 0;
  int coordinateY = 0;

//=================================================================================

void loop() {

  //Check to see if we have results from the IR input. This is non-blocking code.
  if (irRec.decode(&irResults)) {
    //Pass the IR value to a method that does the work
    AutoScopeDecodeIR((long) irResults.value);
    // Resume reading IR values
    irRec.resume(); 
  }

  //Gets the motors running (if enabled)
  AutoScopeRunMotors();
    
} //loop

//=================================================================================

void AutoScopeRunMotors() {

  //Method which handles the movement of the motors
  
  //Only activate the motor code if the "running" flag is True
  if (runMotors) {

    int targetX = 0;
    int targetY = 0;

    // If this is a coordinate mode move (where absolution positions were sent)
    // Then we only want it to run once. So, turn the motor flag off and reset the coordinate mode
    // We also don't determine a target location -- we just use the values passed.
    if (coordinateMode) {
      targetX = xMoveAmount;
      targetY = yMoveAmount;
      runMotors = false;
      coordinateMode = false;
    }else{
      // Figure out what the Target X and Y will be. The value passed is small (-1/1) and by the IR
      targetX = (int) round(etch.getX() + MAX_DISTANCE*xMoveAmount);
      targetY = (int) round(etch.getY() + MAX_DISTANCE*yMoveAmount);
    }

    // Limit movement to values between zero and MaxX/MaxY
    // (defined in EtchABot code -- uses the _SIZE when class instantiated)
    // Note that this is "belt and suspenders" code -- the etch class also keeps track of limits
    targetX = constrain(targetX, 0, etch.getMaxX());
    targetY = constrain(targetY, 0, etch.getMaxY());
      
    //Move to targeted location
    //TargetX, TargetY, MotorShutOff 
    etch.drawLine(targetX, targetY, true);

    // If both Target X and Y are Zero ("not"), turn off running. Needed for Home command.
    if (!targetX && !targetY) {runMotors = false;}

    // TODO: If limits hit, stop. How to handle is one limit is set, but other not?
    // TODO: This will be microswitches or similar which will indicate when min/max X/Y are reached

    if (DEBUG) {
      Serial.print("New Position:");
      Serial.print(etch.getX());
      Serial.print(",");
      Serial.println(etch.getY());
    }
  }//runMotors
  
}//AutoScopeRunMotors

//=================================================================================

void AutoScopeDecodeIR(long irCode) {

  // Method which takes the IR codes and turns them into commands

    if (DEBUG) {
      Serial.print("IR Code (long):");
      Serial.println(irCode);
    }
    
    switch (irCode) {
      case KEY_SETUP:
        switch (coordinateStatus) {
          case USER_ENTERING_X:
            CoordinateSwitchToY();
            break;
          case USER_ENTERING_Y:
            CoordinateSwitchToDone();
            break;
          case USER_DONE:
            CoordinateSwitchToX();
            break;
          default:
            digitalWrite (LEDpin, LOW);
            coordinateStatus = USER_DONE;
            DisplayMainMenu();
            break;
        }
        break;
      case KEY_ENTER:
        if (DEBUG) Serial.println("Moving to entered coordinate. X:" + buildX + " Y:" + buildY);
        //CoordinateMode tells AutoScopeRunMotors that this is a direct entry of final coordinates.
        coordinateMode = true;
        //Pass new coordinates through. Might re-think this, depending on how
        //coordinates are entered by the user. Number of steps? Or mm? or something else?
        xMoveAmount = buildX.toInt();
        yMoveAmount = buildY.toInt();
        runMotors = true;
        break;
      case KEY_0:
        BuildCoordinates(0);
        break;
      case KEY_1:
        BuildCoordinates(1);
        break;
      case KEY_2:
        BuildCoordinates(2);
        break;
      case KEY_3:
        BuildCoordinates(3);
        break;
      case KEY_4:
        BuildCoordinates(4);
        break;
      case KEY_5:
        BuildCoordinates(5);
        break;
      case KEY_6:
        BuildCoordinates(6);
        break;
      case KEY_7:
        BuildCoordinates(7);
        break;
      case KEY_8:
        BuildCoordinates(8);
        break;
      case KEY_9:
        BuildCoordinates(9);
        break;
      case MOVE_UP:
        if (DEBUG) Serial.println("Slew Up Until Stopped...");
        xMoveAmount = 0;
        yMoveAmount = -1;
        runMotors = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Slew Up...");
        lcd.setCursor(1, 1);
        lcd.print("...STOP Cancels");
        break;
      case MOVE_DOWN:
        if (DEBUG) Serial.println("Slew Down Until Stopped...");
        xMoveAmount = 0;
        yMoveAmount = 1;
        runMotors = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Slew Down...");
        lcd.setCursor(1, 1);
        lcd.print("...STOP Cancels");
        break;
      case MOVE_LEFT:
        if (DEBUG) Serial.println("Slew Left Until Stopped...");
        xMoveAmount = -1;
        yMoveAmount = 0;
        runMotors = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Slew Left...");
        lcd.setCursor(1, 1);
        lcd.print("...STOP Cancels");
        break;
      case MOVE_RIGHT:
        if (DEBUG) Serial.println("Slew Right Until Stopped...");
        xMoveAmount = 1;
        yMoveAmount = 0;
        runMotors = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Slew Right...");
        lcd.setCursor(1, 1);
        lcd.print("...STOP Cancels");
        break;
      case MOVE_HOME:
        if (DEBUG) Serial.println("Move to Home and Stop.");
        xMoveAmount = -1;
        yMoveAmount = -1;
        runMotors = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Move Home & Stop.");
        break;
      case MOVE_STOP:
        if (DEBUG) Serial.println("Stopping.");
        xMoveAmount = 0;
        yMoveAmount = 0;
        runMotors = false;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Stopping.");
        break;
    }//switch
}//AutoScopeDecodeIR

//=================================================================================

void CoordinateSwitchToX() {
  if (DEBUG) Serial.println("Manual Entry: Switching from DONE to X.");
  digitalWrite (LEDpin, HIGH);
  coordinateStatus = USER_ENTERING_X;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:_");
  lcd.setCursor(0, 1);
  lcd.print("");
}//CoordinateSwitchToX

void CoordinateSwitchToY() {
  if (DEBUG) Serial.println("Manual Entry: Switching from X to Y.");
  digitalWrite (LEDpin, HIGH);
  coordinateStatus = USER_ENTERING_Y;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:" + buildX);
  lcd.setCursor(0, 1);
  lcd.print("Y:_");
}//CoordinateSwitchToY

void CoordinateSwitchToDone() {
  if (DEBUG) Serial.println("Manual Entry: Switching from Y to DONE.");
  digitalWrite (LEDpin, LOW);
  coordinateStatus = USER_DONE;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Press ENTER to Move.");
}//CoordinateSwitchToDone

//=================================================================================

void BuildCoordinates(int iNumericKey) {

  //Builds the coordinate numbers 
  switch (coordinateStatus) {
    case USER_ENTERING_X:
      buildX = buildX + (String) iNumericKey;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("X:" + buildX + "_");
      lcd.setCursor(0, 1);
      lcd.print("");
      break;
    case USER_ENTERING_Y:
      buildY = buildY + (String) iNumericKey;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("X:" + buildX);
      lcd.setCursor(0, 1);
      lcd.print("Y:" + buildY + "_");
      break;
  }

  if (DEBUG) Serial.println("Manual Coordinates are X:" + buildX + " Y:" + buildY);

}//BuildCoordinates

//=================================================================================

void DisplayMainMenu() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("AutoScope!");
  lcd.setCursor(3,1);
  lcd.print(APP_VER);

}

//=================================================================================





