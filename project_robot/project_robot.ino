#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_VL53L0X.h"
#include <utility/imumaths.h>
#include <math.h>
/*
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"
*/

#define lightPin A0
#define turnSpeed 115 
#define stopSpeed 95
#define deltaTurnAngle 120
#define forwardSpeed 125
#define stopDistance 100

int addOffset = 90 + offset;
int subOffset = 90 - offset;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo left;
Servo right;

void setup() {
  pinMode(9, OUTPUT); // blue
  pinMode(10, OUTPUT); // green

  left.attach(5);
  right.attach(6);
  Serial.begin(9600);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
   if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    bno.setExtCrystalUse(true);

  // Receive first acknowledgement from the pi to see if it is ready to start
  char start = Serial.read();
  while(start != 'r'){      // r means ready
    start = Serial.read();
  }
}

void turnUntilFound(int angleDegrees){
     int leftWrite, rightWrite;
     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     int current = euler.x();
     if(angleDegrees>0){  
        leftWrite = subOffset;
        rightWrite = addOffset
     }
     else{
        leftWrite = addOffset;
        rightWrite = subOffset;
     }
      while(current != ((float)angleDegrees-0.1){
        left.write(leftWrite);
        right.write(rightWrite);
        delay(16);
        current = euler.x();
      }
      stopRobot();
}

void loop() {
  int light = analogRead(lightPin);
  char instruction; // Instructions to send to the pi
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if(light>200){            // Too bright, hide mode
    digitalWrite(10,LOW);   // Light up the LED blue
    digitalWrite(9,HIGH);
    
    instruction = 't'; // tell raspberry its time to "turn" or find the box
    Serial.print(instruction); 
    
    // Receive an acknowledgment from the pi
    char ack = Serial.read();
    // If the pi sends 'm' this means to move directly forward to the box
    while(ack != 'm'){
      // Supposed to turn, need the code from the pi to turn towards the box
      // This function will be used to turn the robot if the ACK received is not 'm', so if its 't' or something
       turnUntilFound(euler.x());
       ack = Serial.read();
    }
      // Time to move straight to the box
      if (measure.RangeStatus != 4) { 
      if(measure.RangeMilliMeter > stopDistance){   // While we havent hit anything move forward
        left.write(forwardSpeed);
        right.write(forwardSpeed);
      }
      else{     // We are too close to the box, stop moving
        left.write(stopSpeed);
        right.write(stopSpeed);
      }
      }
  }

 
  else{                     // Dark enough, stay still
    left.write(stopSpeed);
    right.write(stopSpeed);
    instruction = 's'; // tell raspberry it is time to "stay" still 
    Serial.print(instruction);
    digitalWrite(9,LOW); // Light up the green LED
    digitalWrite(10,HIGH);
  }

}
