#include <Stepper.h>
#include "EtchABot.h"

//=================================================================================

// Define a stepper and the pins it will use
EtchABot etch(POCKET_SIZE);

//=================================================================================

//Joystick variables
int vert = 0;
int horz = 0;
int SEL, prevSEL = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;  //millis

//Max values for horz and vert (from joystick)
const int VERT_MAX = 1023;
const int HORZ_MAX = 1023;

//=================================================================================

//How much movement per joystick action. A multiplier.
#define MAX_DISTANCE 10

//Joystick pins
#define VCC_PIN     A0    //hack to get VCC and GND to adjoining pins
#define VERT_PIN    A1
#define HORZ_PIN    A2
#define BUTTON_PIN  A3
#define GND_PIN     A4    //hack to get VCC and GND to adjoining pins

//=================================================================================

void setup() {

  //hack to get VCC and GND to adjoining pins -- get power and ground
  pinMode(GND_PIN, OUTPUT);
  pinMode(VCC_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);
  digitalWrite(VCC_PIN, HIGH);

  //Set HORZ and VERT pins (from joystick) to input
  pinMode(VERT_PIN, INPUT);
  pinMode(HORZ_PIN, INPUT);

  //joystick button
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);  // turn on pullup resistor

  //For debugging / console feedback
  Serial.begin(9600);

} //setup

//=================================================================================

void loop() {

  // ----------------------------------------------------

  // If button pressed, move to Home (0,0)
  // Check for button push - and debounce
  int reading = digitalRead(BUTTON_PIN);
  if (reading != prevSEL) {
     // reset timer
     lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != SEL) {
      SEL = reading;
        
      if (SEL == LOW) {  // Goes to ground if pressed (see sparkfun tutorial 272)
        // Move Home
        Serial.println("Moving to home position (0,0)...");
        etch.drawLine(0, 0, true); //go home (0,0) -- this will eventually use stop switches
      }
    }
  }
  prevSEL = reading;

  // ----------------------------------------------------
  
  // Get direction relative to current position
  vert = analogRead(VERT_PIN);
  horz = analogRead(HORZ_PIN);

  // Get direction and magnitude of joystick vector (normalized to [-1.0,1.0] range
  // Very handy for reading joysticks and getting simple results to work with.
  float xNorm = -2.0*(horz - HORZ_MAX/2.0)/HORZ_MAX;
  float yNorm = 2.0*(vert - VERT_MAX/2.0)/VERT_MAX;
  
  // Ignore small fluctuations around the center
  if (abs(xNorm) < 0.1) xNorm = 0.0;
  if (abs(yNorm) < 0.1) yNorm = 0.0;

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
  Serial.print("xNorm:");
  Serial.print(xNorm);
  Serial.print(", yNorm:");
  Serial.print(yNorm);
  Serial.print(") ");
  */
  Serial.print(" (");
  Serial.print("tX:");
  Serial.print(targetX);
  Serial.print(", tY:");
  Serial.print(targetY);
  Serial.println(") ");
  

  //Move to targeted location
  //TargetX, TargetY, MotorShutOff 
  etch.drawLine(targetX, targetY, true);

  // ----------------------------------------------------

} //loop
