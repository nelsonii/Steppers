
// AutoScope
//
// A project by Dr. Dan Kott, Ron Nelson, and the Central Texas Makers team.
// Based on a concept by Dr. Shane Battye of PathOBin.com
// Using code from Geek Mom Projects and SunFounder.
//
// Written: Winter 2018
// Contact: dfkott (at) aol.com  -  ron.nelson@CentralTexasMakers.com

//=================================================================================


// Libraries


//For Stepper Motor Control and Movement
#include <Arduino.h>
#include <Stepper.h>
#include "EtchABotAutoScope.h"
// Note: Thanks to Geek Mom Projects for giving me a head-start on getting 
//       the stepper motors going. I based much of the motor movement logic
//       off of her well documented EtchABot project. It was extremely helpful
//       as Debra is using the same steppers and controllers as we are.
//       http://www.geekmomprojects.com/etchabot-a-cnc-etch-a-sketch/


// LCD Display over I2C
#include <Wire.h>
#include "LiquidCrystal_I2C.h"

// VL6180X Laser Distance Sensor
#include "Adafruit_VL6180X.h"


//=================================================================================


//Joystick pins
#define JOY_VCC_PIN     A0    //hack to get VCC and GND to adjoining pins
#define JOY_GND_PIN     A1    //hack to get VCC and GND to adjoining pins
#define JOY_Y_PIN       A2    //flip X/Y depending on joystick orientation
#define JOY_X_PIN       A3    //flip X/Y depending on joystick orientation
#define JOY_B_PIN       A4    //click button on joystick

//Buzzer pins
#define BUZ_VCC_PIN     A5    //hack to get VCC and GND to adjoining pins
#define BUZ_PWM_PIN     A6    //Buzzer "I/O"
#define BUZ_GND_PIN     A7    //hack to get VCC and GND to adjoining pins


//=================================================================================


// Constants 

// Code version. Used for developer reference.
const long APP_VER = 2018120801;

// Turns on/off Serial Output debugging
// Turning off debugging saves a considerable amount of memory.
const boolean DEBUG = 1;
const short BAUD = 9600;

// Good old Pin 13 for a status LED. Lights up in certain modes.
const int LEDpin = 13;

// Used for the Manual Coordinate selection process
const int USER_ENTERING_X     = 1;
const int USER_ENTERING_Y     = 2;
const int USER_DONE           = 3;

//Max values for horz and vert (from joystick)
const int JOY_X_MAX = 1023;
const int JOY_Y_MAX = 1023;


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
  // Just press it once and it runs forever one incremented step at a time (-1 or 1)
  // Since I switched this back to joystick control, not as important, but keeping.
  boolean runMotors = false;

  // The coordinate mode is used when a direct coordinate is entered. 
  // The motors should skew to that coordinate and then stop running.
  boolean coordinateMode = false;
  int coordinateStatus = USER_DONE;
  String buildX = "";
  String buildY = "";  

  // Used to capture the user's X and Y coordinates
  int coordinateX = 0;
  int coordinateY = 0;

  //Joystick variables
  int vert = 0;
  int horz = 0;
  int SEL, prevSEL = HIGH;
  long lastDebounceTime = 0;
  long debounceDelay = 50;  //millis


//=================================================================================


// Library Classes

// Define a stepper and the pins it will use
/* 
 * Stepper X is on Mega Digital Pins 4,5,6,7
 * Stepper Y is on Mega Digital Pins 8,9,10,11
 * (I shifted these around to free up pins 2 and 3 for interrupt use)
 * 
 */
EtchABotAutoScope etch(POCKET_SIZE); // Uses default pins (found in EtchABotAutoScope.cpp)


// Sunfounder LCD 20char x 4 line
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

// Distance Sensor Class
Adafruit_VL6180X distance = Adafruit_VL6180X();


//=================================================================================


void DisplayMainMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("AutoScope ");
  lcd.print(APP_VER);
}


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
      // Figure out what the Target X and Y will be. The value passed is small (-1/1).
      targetX = (int) round(etch.getX() + xMoveAmount);
      targetY = (int) round(etch.getY() + yMoveAmount);
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

    lcd.setCursor(0,2);
    lcd.print("X:");
    lcd.print(etch.getX());
    lcd.print(" ");
    lcd.setCursor(0,3);
    lcd.print("Y:");
    lcd.print(etch.getY());
    lcd.print(" ");

  }//runMotors
  
}//AutoScopeRunMotors


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
//=================================================================================
//=================================================================================


void setup() {

  if (DEBUG) {
    Serial.begin(BAUD); delay(100); Serial.print("AutoScope "); Serial.println(APP_VER);
  }

  //Status LED (onboard)
  pinMode(LEDpin, OUTPUT);
  digitalWrite (LEDpin, LOW);

  //Start Laser Distance Sensor
  distance.begin();

  //Joystick
  //hack to get VCC and GND to adjoining pins -- get power and ground
  pinMode(JOY_GND_PIN, OUTPUT);
  pinMode(JOY_VCC_PIN, OUTPUT);
  digitalWrite(JOY_GND_PIN, LOW);
  digitalWrite(JOY_VCC_PIN, HIGH);

  //Set HORZ and VERT pins (from joystick) to input
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_X_PIN, INPUT);

  //joystick button
  pinMode(JOY_B_PIN, INPUT);
  digitalWrite(JOY_B_PIN, HIGH);  // turn on pullup resistor

  //Buzzer
  //hack to get VCC and GND to adjoining pins -- get power and ground
  pinMode(BUZ_PWM_PIN, OUTPUT);
  pinMode(BUZ_GND_PIN, OUTPUT);
  pinMode(BUZ_VCC_PIN, OUTPUT);
  digitalWrite(BUZ_PWM_PIN, HIGH); // The buzzer I'm using is low level trigger, so High = Off
  digitalWrite(BUZ_GND_PIN, LOW);
  digitalWrite(BUZ_VCC_PIN, HIGH);

  //LCD
  // set up the LCD's number of rows and columns: 
  lcd.init();  //initialize the lcd
  lcd.backlight();  //backlight 
  
  //Buzz on startup
  tone(BUZ_PWM_PIN, 1000); delay(250);
  noTone(BUZ_PWM_PIN);digitalWrite(BUZ_PWM_PIN, HIGH);

  //LCD Main Menu
  DisplayMainMenu();

} //setup


//=================================================================================
//=================================================================================
//=================================================================================

void loop() {

  runMotors = false;

  // Get direction relative to current position
  vert = analogRead(JOY_Y_PIN);
  horz = analogRead(JOY_X_PIN);
  //Serial.print(horz); Serial.print("-"); Serial.println(vert);

  // Get direction and magnitude of joystick vector (normalized to [-1.0,1.0] range
  // Very handy for reading joysticks and getting simple results to work with.
  // The multiply by negative # is used if you want to switch CW/CCW, 
  // depending on how the motors are aligned.
  float xNorm = -2.0*(horz - JOY_X_MAX/2.0)/JOY_X_MAX;
  float yNorm = 2.0*(vert - JOY_Y_MAX/2.0)/JOY_Y_MAX;
  
  // Ignore small fluctuations around the center
  if (abs(xNorm) < .1) xNorm = 0.0;
  if (abs(yNorm) < .1) yNorm = 0.0;

  // Only figure out new coordinates if the joystick was moved.
  if ((xNorm!=0.0) || (yNorm!=0.0)) {
    
    xMoveAmount = round(xNorm);
    yMoveAmount = round(yNorm);

    if (DEBUG) {
      Serial.print(" ("); Serial.print("xMoveAmount:"); Serial.print(xMoveAmount); Serial.print(", yMoveAmount:"); Serial.print(yMoveAmount); Serial.print(") ");
    }

    runMotors = true;
  }//if xNorm/yNorm moved

  //Gets the motors running (if enabled)
  AutoScopeRunMotors();



    // Laser Distance Sensor ----------------------------------------------------

    uint8_t range = distance.readRange();
    uint8_t status = distance.readRangeStatus();

    lcd.setCursor(0,1);
    lcd.print("Range: ");

    if (status == VL6180X_ERROR_NONE) {
      lcd.print(range);lcd.print(" mm      ");
    }
    else{
      lcd.print(range);lcd.print("OUT      ");
    }
  


} //loop

//=================================================================================
