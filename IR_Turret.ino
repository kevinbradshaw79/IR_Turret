//////////////////////////////////////////////////
              //  LICENSE  //
//////////////////////////////////////////////////
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (IRTurret Control Code)
  * Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#pragma endregion LICENSE

//////////////////////////////////////////////////
              //  LIBRARIES  //
//////////////////////////////////////////////////
#pragma region LIBRARIES

#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>

#pragma endregion LIBRARIES

//////////////////////////////////////////////////
               //  IR CODES  //
//////////////////////////////////////////////////
#pragma region IR CODES
/*
** if you want to add other remotes (as long as they're on the same protocol above):
** press the desired button and look for a hex code similar to those below (ex: 0x11)
** then add a new line to #define newCmdName 0x11,
** and add a case to the switch statement like case newCmdName: 
** this will let you add new functions to buttons on other remotes!
** the best remotes to try are cheap LED remotes, some TV remotes, and some garage door openers
*/

//defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x18
#define down 0x52
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

#pragma endregion IR CODES

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
#pragma region PINS AND PARAMS
//this is where we store global variables!
Servo yawServo; //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo; //names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal = 90; //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal = 90;

int pitchMoveSpeed = 8; //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90; //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90; //value to stop the roll motor - keep this at 90

int yawPrecision = 150; // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 200; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 150; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 33; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax

// Ultrasonic Sensor Pins
const int trigPin = 7;  // Ultrasonic sensor TRIG pin
const int echoPin = 8;  // Ultrasonic sensor ECHO pin

// Motion Tracking Variables
bool trackingMode = false;  // Toggle for motion tracking mode
int fireDistance = 50;  // Distance in cm to auto-fire (adjust based on your needs)
int detectionThreshold = 15;  // Minimum distance change in cm to consider as motion
int scanSpeed = 50;  // Speed for scanning (lower = slower, higher = faster)
int trackingYawSpeed = 30;  // Speed for tracking targets (slower for accuracy)
int motionConfirmCount = 3;  // Number of consecutive motion detections required to lock on

void shakeHeadYes(int moves = 3); //function prototypes for shakeHeadYes and No for proper compiling
void shakeHeadNo(int moves = 3);
long measureDistance();  // function to measure distance with ultrasonic sensor
void scanAndTrack();  // function to scan for and track motion
#pragma endregion PINS AND PARAMS

//////////////////////////////////////////////////
              //  S E T U P  //
//////////////////////////////////////////////////
#pragma region SETUP
void setup() { //this is our setup function - it runs once on start up, and is basically where we get everything "set up"
    Serial.begin(9600); // initializes the Serial communication between the computer and the microcontroller

    yawServo.attach(10); //attach YAW servo to pin 10
    pitchServo.attach(11); //attach PITCH servo to pin 11
    rollServo.attach(12); //attach ROLL servo to pin 12

    // Initialize ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Just to know which program is running on my microcontroller
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin 9"));

    homeServos(); //set servo motors to home position

    Serial.println(F("Ultrasonic motion tracking enabled - Press # to toggle tracking mode"));
}
#pragma endregion SETUP

//////////////////////////////////////////////////
               //  L O O P  //
//////////////////////////////////////////////////
#pragma region LOOP

void loop() {

    /*
    * Check if received data is available and if yes, try to decode it.
    */
    if (IrReceiver.decode()) {

        /*
        * !!!Important!!! Enable receiving of the next value,
        * since receiving has stopped after the end of the current received data packet.
        */
        IrReceiver.resume(); // Enable receiving of the next value

        // Only process known commands, ignore noise
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            return; // Ignore unknown/noise signals
        }

        /*
        * Finally, check the received data and perform actions according to the received command
        */

        switch(IrReceiver.decodedIRData.command){ //this is where the commands are handled

            case up: //pitch up
              upMove(1);
              break;
            
            case down: //pitch down
              downMove(1);
              break;

            case left: //fast counterclockwise rotation
              leftMove(1);
              break;
            
            case right: //fast clockwise rotation
              rightMove(1);
              break;
            
            case ok: //firing routine 
              fire();
              //Serial.println("FIRE");
              break;
              
            case star:
              fireAll();
              delay(50);
              break;

            case cmd1:
              shakeHeadYes(3);
              break;

            case cmd2:
              shakeHeadNo(3);
              break;

            case hashtag: // Toggle motion tracking mode
              trackingMode = !trackingMode;
              if (trackingMode) {
                Serial.println("TRACKING MODE ENABLED");
                shakeHeadYes(2);
              } else {
                Serial.println("TRACKING MODE DISABLED");
                shakeHeadNo(2);
                homeServos();
              }
              break;

        }
    }

    // If tracking mode is enabled, scan and track motion
    if (trackingMode) {
        scanAndTrack();
    }

    delay(5);
}

#pragma endregion LOOP

//////////////////////////////////////////////////
               // FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS

void leftMove(int moves){ // function to move left
    for (int i = 0; i < moves; i++){
        yawServo.write(yawStopSpeed + yawMoveSpeed); // adding the servo speed = 180 (full counterclockwise rotation speed)
        delay(yawPrecision); // stay rotating for a certain number of milliseconds
        yawServo.write(yawStopSpeed); // stop rotating
        delay(5); //delay for smoothness
        Serial.println("LEFT");
  }

}

void rightMove(int moves){ // function to move right
  for (int i = 0; i < moves; i++){
      yawServo.write(yawStopSpeed - yawMoveSpeed); //subtracting the servo speed = 0 (full clockwise rotation speed)
      delay(yawPrecision);
      yawServo.write(yawStopSpeed);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){ // function to tilt up
  for (int i = 0; i < moves; i++){
        if((pitchServoVal+pitchMoveSpeed) < pitchMax){ //make sure the servo is within rotation limits (less than 150 degrees by default)
        pitchServoVal = pitchServoVal + pitchMoveSpeed;//increment the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){ // function to tilt down
  for (int i = 0; i < moves; i++){
      if((pitchServoVal-pitchMoveSpeed) > pitchMin){//make sure the servo is within rotation limits (greater than 35 degrees by default)
        pitchServoVal = pitchServoVal - pitchMoveSpeed; //decrement the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

void fire() { //function for firing a single dart
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision);//time for approximately 60 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); //delay for smoothness
    Serial.println("FIRING");
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision * 6); //time for 360 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); // delay for smoothness
    Serial.println("FIRING ALL");
}

void homeServos(){ // sends servos to home positions
    yawServo.write(yawStopSpeed); //setup YAW servo to be STOPPED (90)
    delay(20);
    rollServo.write(rollStopSpeed); //setup ROLL servo to be STOPPED (90)
    delay(100);
    pitchServo.write(100); //set PITCH servo to 100 degree position
    delay(100);
    pitchServoVal = 100; // store the pitch servo value
    Serial.println("HOMING");
}

void shakeHeadYes(int moves = 3) { //sets the default number of nods to 3, but you can pass in whatever number of nods you want
      Serial.println("YES");

    if ((pitchMax - pitchServoVal) < 15){
      pitchServoVal = pitchServoVal - 15;
    }else if ((pitchServoVal - pitchMin) < 15){
      pitchServoVal = pitchServoVal + 15;
    }
    pitchServo.write(pitchServoVal);

    int startAngle = pitchServoVal; // Current position of the pitch servo
    int lastAngle = pitchServoVal;
    int nodAngle = startAngle + 15; // Angle for nodding motion

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // Nod up
        for (int angle = startAngle; angle <= nodAngle; angle++) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at nodding position
        // Nod down
        for (int angle = nodAngle; angle >= startAngle; angle--) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at starting position
    }
}

void shakeHeadNo(int moves = 3) {
    Serial.println("NO");

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // rotate right, stop, then rotate left, stop
        yawServo.write(140);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50);
        yawServo.write(40);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50); // Pause at starting position
    }
}

long measureDistance() {
    // Take multiple samples and average them to reduce noise
    const int samples = 3;
    long total = 0;
    int validSamples = 0;

    for (int i = 0; i < samples; i++) {
        // Clear the trigPin
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);

        // Set the trigPin HIGH for 10 microseconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Read the echoPin, returns the sound wave travel time in microseconds
        long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout for max ~5m range

        // Calculate distance in centimeters (speed of sound is 343m/s or 0.0343cm/μs)
        // Distance = (Time × Speed) / 2 (divide by 2 because sound travels to object and back)
        long distance = duration * 0.0343 / 2;

        // Only include valid readings (within range)
        if (distance > 0 && distance <= 400) {
            total += distance;
            validSamples++;
        }

        if (i < samples - 1) {
            delay(10); // Short delay between samples
        }
    }

    // Return 0 if no valid samples
    if (validSamples == 0) {
        return 0;
    }

    // Return average of valid samples
    return total / validSamples;
}

void scanAndTrack() {
    static long baselineDistance = 0;
    static long previousDistance = 0;
    static bool targetLocked = false;
    static unsigned long lastScanTime = 0;
    static unsigned long lastStatusPrint = 0;
    static bool scanningRight = true;
    static bool trackingRight = true;
    static int noMotionCounter = 0;
    static int motionDetections = 0;  // Count consecutive motion detections
    static int scanCounter = 0;
    static int trackMoveCounter = 0;

    // Control loop timing - take measurement every 200ms
    if (millis() - lastScanTime < 200) {
        return;
    }
    lastScanTime = millis();

    // Take distance measurement first
    long currentDistance = measureDistance();

    // If we have a valid reading
    if (currentDistance > 0) {

        // Check if target is within firing range
        if (currentDistance <= fireDistance && targetLocked) {
            Serial.print(">> FIRING at ");
            Serial.print(currentDistance);
            Serial.println("cm");
            fire();
            delay(500); // Brief pause after firing
            baselineDistance = 0;  // Reset baseline after firing
            previousDistance = 0;
            motionDetections = 0;
            return;
        }

        // Motion detection: if distance changed significantly, we detected motion
        if (baselineDistance > 0) {
            long distanceChange = abs(currentDistance - baselineDistance);

            // Debug output
            Serial.print("Dist: ");
            Serial.print(currentDistance);
            Serial.print("cm, Base: ");
            Serial.print(baselineDistance);
            Serial.print("cm, Diff: ");
            Serial.print(distanceChange);
            Serial.print("cm");

            if (distanceChange > detectionThreshold) {
                // Increment motion detection counter (but don't go crazy)
                if (!targetLocked && motionDetections < motionConfirmCount) {
                    motionDetections++;
                    Serial.print(" >> MOTION! (");
                    Serial.print(motionDetections);
                    Serial.print("/");
                    Serial.print(motionConfirmCount);
                    Serial.println(")");
                } else if (targetLocked) {
                    Serial.println(" (tracking)");
                }

                // Only lock on after multiple consecutive detections
                if (motionDetections >= motionConfirmCount && !targetLocked) {
                    targetLocked = true;
                    Serial.print(">> TARGET LOCKED at ");
                    Serial.print(currentDistance);
                    Serial.println("cm");
                }

                noMotionCounter = 0;
            } else {
                Serial.println();
                // Reset motion detection counter if no motion
                motionDetections = 0;
                noMotionCounter++;

                // If no motion detected for a while, go back to scanning
                if (noMotionCounter > 30 && targetLocked) {
                    targetLocked = false;
                    noMotionCounter = 0;
                    baselineDistance = 0;  // Reset baseline
                    Serial.println(">> Target lost");
                }
            }
        } else {
            Serial.print("Setting baseline: ");
            Serial.print(currentDistance);
            Serial.println("cm");
        }

        // Set baseline once and keep it fixed while scanning
        // Don't update it - this prevents "creeping" that follows targets
        if (baselineDistance == 0) {
            baselineDistance = currentDistance;
        }
        // DO NOT update baseline - let it stay fixed to detect real motion

        previousDistance = currentDistance;
    } else {
        Serial.println("No valid reading");
        // No valid reading - might have lost target
        motionDetections = 0;  // Reset motion counter
        noMotionCounter++;
        if (noMotionCounter > 20 && targetLocked) {
            targetLocked = false;
            noMotionCounter = 0;
            baselineDistance = 0;  // Reset baseline
            Serial.println(">> Target lost");
        }
    }

    // NOW perform scanning/tracking motion AFTER taking measurements
    if (!targetLocked) {
        // Reset baseline every few scans to adapt to changing view while sweeping
        if (scanCounter > 0 && scanCounter % 3 == 0) {
            baselineDistance = 0;  // Reset baseline periodically during scan
        }

        // Change direction after sweeping one way - CHECK THIS FIRST
        scanCounter++;
        if (scanCounter > 8) {  // Reduced for faster direction changes
            scanningRight = !scanningRight;
            scanCounter = 0;
            baselineDistance = 0;  // Reset baseline when changing direction
            motionDetections = 0;   // Reset motion counter too
            if (millis() - lastStatusPrint > 3000) {  // Print status every 3 seconds
                Serial.println("Scanning...");
                lastStatusPrint = millis();
            }
        }

        // Scan back and forth - more visible movement
        if (scanningRight) {
            yawServo.write(yawStopSpeed - scanSpeed);
            delay(80);  // Increased for more visible movement
            yawServo.write(yawStopSpeed);
        } else {
            yawServo.write(yawStopSpeed + scanSpeed);
            delay(80);  // Increased for more visible movement
            yawServo.write(yawStopSpeed);
        }
    } else {
        // Active tracking - sweep back and forth to follow the target
        trackMoveCounter++;

        // Continuously sweep in tracking direction
        if (trackingRight) {
            yawServo.write(yawStopSpeed - trackingYawSpeed);
            delay(60);
            yawServo.write(yawStopSpeed);
        } else {
            yawServo.write(yawStopSpeed + trackingYawSpeed);
            delay(60);
            yawServo.write(yawStopSpeed);
        }

        // Reverse direction every few moves
        if (trackMoveCounter > 6) {
            trackingRight = !trackingRight;
            trackMoveCounter = 0;
        }
    }
}

#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
               //  END CODE  //
//////////////////////////////////////////////////

   