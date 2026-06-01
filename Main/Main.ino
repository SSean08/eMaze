/*
  Author: Sean Andre C. Ancheta
  This close-looped system will have a refresh rate of 250Hz.
  That is, 4mS or 4000uS of period.

  This will implement a wall-hugging algorithm with the left wall with a PID controller to maintain a centered position.
  This system will be using a ToF sensor to measure the wall's distances, particularly, the VL53L0X ToF sensor.
*/

// Libraries
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "Filter.h"


/*
  DEFINE AND INITIALIZE CONSTANTS HERE
*/


const int MAX_REFRESH_RATE = 100; // Hz
const int REFRESH_PERIOD = 1 / MAX_REFRESH_RATE * 1000000; // uS

// VL053L0X ToF Sensor Pins

const int XSHUT_LEFT = 2;
const int INTERRUPT_LEFT = 3;

const int XSHUT_RIGHT = 11;
const int INTERRUPT_RIGHT = 5;

const int XSHUT_FRONT = 6;
const int INTERRUPT_FRONT = 7;


// LM298N Motor Driver Pins
const int MOTOR_LEFT_IN1 = 14;
const int MOTOR_LEFT_IN2 = 15;

const int MOTOR_RIGHT_IN1 = 16;
const int MOTOR_RIGHT_IN2 = 17;

// LM298N Motor Driver PWM Pins
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;

// PID
// const float Kp = 0.25;
// const float Ki = 0.2;
// const float Kd = 0.01;
const float Kp = 0.25f;
const float Ki = 0.0f;
const float Kd = 0.0f;

const int ALPHA = 0.2;

// Default Speed
const int DEFAULT_SPEED = 0.13*255; // 20% of max speed (255)
const float rotationSpeedRelativeToDefault = 0.2*255; // Old default speed
const long rotationCount = 11000; // Number of iterations to turn 90 degrees, this is a hyperparameter that can be tuned based on the robot's turning speed and desired turning angle.

// Far wall threshold
// If the distance to the wall is greater than this threshold, it will be considered as no wall.
const int farWallThreshold = 130; // cm
// If the distance to the wall is less than this threshold, it will be considered as near wall.
const int nearWallThreshold = 15; // cm
// Front near wall threshold
// Lesser so that left wall will be prioritized always.
const int frontNearWallThreshold = 50; // cm







/*
  DEFINE AND INITIALIZE VARIABLES HERE
*/

long beginTime = 0, endTime;
float motorSpeedLeft = 0, motorSpeedRight = 0;

// Integral controller
float integralSum = .0f;
float lastError = .0f;
unsigned long lastTime = 0;

// Derivative controller
float lastDerivativeError = .0f;
float filteredDerivative = .0f;

// Initialize ToF sensors
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

//Instantiate filters for each sensor with alpha = 0.2 (tunable)
ToFFilter filterFront(0.2f);
ToFFilter filterLeft(0.2f);
ToFFilter filterRight(0.2f);

// ToF sensor distance readings
uint16_t distanceFront = 0;
uint16_t distanceLeft = 0;
uint16_t distanceRight = 0;





/*
  USER-DEFINED FUNCTIONS HERE
*/


void stopMotors() {
  // This function will stop the robot by setting motor speeds to 0 and applying brakes.
  motorSpeedLeft = 0;
  motorSpeedRight = 0;

  // Apply brakes by setting both IN1 and IN2 to HIGH for both motors
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);

  // Set PWM to 0 to ensure motors are stopped
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void turn(int direction) {
  // This function will stop the robot and make it turn with calculated rotation.
  // 0 = left, 1 = right, 2 = forward

  for (int i = 0 ; i < rotationCount; i++) {
      // Set motor speeds for turning

    if (direction == 0) {
      // Turn left
      motorSpeedLeft = -rotationSpeedRelativeToDefault;
      motorSpeedRight = rotationSpeedRelativeToDefault;
    } else if (direction == 1) {
      // Turn right
      motorSpeedLeft = rotationSpeedRelativeToDefault;
      motorSpeedRight = -rotationSpeedRelativeToDefault;
    } else {
      // Go forward
      motorSpeedLeft = rotationSpeedRelativeToDefault;
      motorSpeedRight = rotationSpeedRelativeToDefault;
    }

    if (direction == 2) {
      // If going forward, we can set the motor speeds to default speed.
      digitalWrite(MOTOR_LEFT_IN1, HIGH);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
      analogWrite(MOTOR_LEFT_PWM, motorSpeedLeft);
      digitalWrite(MOTOR_RIGHT_IN1, HIGH);
      digitalWrite(MOTOR_RIGHT_IN2, LOW);
      analogWrite(MOTOR_RIGHT_PWM, motorSpeedRight);
      continue; // Skip the rest of the loop and continue going forward.
    }
    

    
    // Set motor directions and speeds for left and right
    if (motorSpeedLeft > 0) {
      digitalWrite(MOTOR_LEFT_IN1, HIGH);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
      analogWrite(MOTOR_LEFT_PWM, motorSpeedLeft);
    } else {
      digitalWrite(MOTOR_LEFT_IN1, LOW);
      digitalWrite(MOTOR_LEFT_IN2, HIGH);
      analogWrite(MOTOR_LEFT_PWM, -motorSpeedLeft);
    }
    if (motorSpeedRight > 0) {
      digitalWrite(MOTOR_RIGHT_IN1, HIGH);
      digitalWrite(MOTOR_RIGHT_IN2, LOW);
      analogWrite(MOTOR_RIGHT_PWM, motorSpeedRight);
    } else {
      digitalWrite(MOTOR_RIGHT_IN1, LOW);
      digitalWrite(MOTOR_RIGHT_IN2, HIGH);
      analogWrite(MOTOR_RIGHT_PWM, -motorSpeedRight); 
    }

  }
  stopMotors(); // Stop the robot after turning for the specified count, this will ensure that the robot will not keep turning indefinitely.
}


void setup() {
  delay(1000);
  // Serial communication bitrate


  Serial.begin(115200);
  Wire.begin();
  Serial.println("I2C Initialized!");


  // Setup ToF sensor pins
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(INTERRUPT_FRONT, INPUT);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(INTERRUPT_LEFT, INPUT);

  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(INTERRUPT_RIGHT, INPUT);

  // Set ToF sensor unique addresses
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  Serial.println("Setting ToF sensor unique addresses...");

  Serial.println("Setting front sensor address...");

  // Front sensor
  digitalWrite(XSHUT_FRONT, HIGH);
  sensorFront.setTimeout(2000);
  delay(100);
  if (!sensorFront.init()) {
    Serial.println("Failed to set ToF front sensor address!");
  } else {
    sensorFront.setAddress(0x30); // Set unique I2C address for front sensor  
    Serial.println("Front sensor address set!");
  }



  Serial.println("Setting left sensor address...");
  // Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  sensorLeft.setTimeout(2000);
  delay(100);
  if (!sensorLeft.init()) {
    Serial.println("Failed to set ToF left sensor address!");
  } else {
    sensorLeft.setAddress(0x40); // Set unique I2C address for left sensor
    Serial.println("Left sensor address set!");
  }


  Serial.println("Setting right sensor address...");
  // Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(100);
  sensorRight.setTimeout(2000);
  if (!sensorRight.init()) {
    Serial.println("Failed to set ToF right sensor address!");
  } else {
    sensorRight.setAddress(0x50); // Set unique I2C address for right sensor
    Serial.println("Right sensor address set!");
  }

  Serial.println("ToF sensors initialized with unique addresses!");
  delay(200);

  Serial.println("Setting sensor speeds...");
  

  sensorLeft.setMeasurementTimingBudget(20000);
  sensorRight.setMeasurementTimingBudget(20000);
  sensorFront.setMeasurementTimingBudget(20000);
  Serial.println("Sensor setup done");
  

  delay(1000);

  

  // Setup motor driver pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Setup motor driver PWM pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Use built-in LED as Debug pin, it is high when system refresh rate < 250Hz
  pinMode(LED_BUILTIN, OUTPUT);

  lastTime = micros();

  
}



void loop() {
  beginTime = micros();

  float dt = (beginTime - lastTime) / 1000000.0; // Convert to seconds

  if (dt <= 0.0) return;
  lastTime = beginTime;

  // // Main control loop code

  // //Read sensors

  // Save previous distances for filtering
  if (firstReadLeft) {
    previousDistanceLeft = distanceLeft;
    firstReadLeft = false;
  } else if (abs(previousDistanceLeft - distanceLeft) > 200) {
    // If the distance suddenly changes by more than 20 cm, we can consider it as noise and use the previous distance instead.
    distanceLeft = previousDistanceLeft;
  } else {
    previousDistanceLeft = distanceLeft;
  }

  if (firstReadRight) {
    previousDistanceRight = distanceRight;
    firstReadRight = false;
  } else if (abs(previousDistanceRight - distanceRight) > 200) {
    // If the distance suddenly changes by more than 20 cm, we can consider it as noise and use the previous distance instead.
    distanceRight = previousDistanceRight;
  } else {
    previousDistanceRight = distanceRight;
  }



  // Serial.print("Front: ");
  // Serial.print(distanceFront);
  // Serial.print(" cm, Left: ");
  // Serial.print(distanceLeft);
  // Serial.print(" cm, Right: ");
  // Serial.print(distanceRight);
  // Serial.println(" cm");

  float centeringError = (distanceLeft - distanceRight);

  integralSum += ((centeringError + lastError) / 2.0) * dt; // Trapezoidal integration

  float rawDerivative = ((centeringError - lastDerivativeError) /dt);
  filteredDerivative = (ALPHA * rawDerivative) + ((1.0 - ALPHA) * filteredDerivative);

  lastDerivativeError = centeringError;

  // Reset integral when centeringError are withing -5 <= centeringError <=5;
    if (centeringError >= -5 && centeringError <= 5) {
      integralSum = 0;
      filteredDerivative = 0;
    }

  // OLD PID CODE
  if (centeringError > 0) {
    // If the left distance is greater than the right distance, it means that the robot is closer to the right wall and needs to turn left to center itself.
    motorSpeedLeft = DEFAULT_SPEED;
    motorSpeedRight = DEFAULT_SPEED + (int) (Kp * centeringError) + (int) (Ki * integralSum) + (int) (Kd * filteredDerivative);
  } else if (centeringError < 0) {
    // Turn left, centeringError is negative
    motorSpeedLeft = DEFAULT_SPEED + (int) (-Kp * centeringError) + (int) (-Ki * integralSum) + (int) (-Kd * filteredDerivative);
    motorSpeedRight = DEFAULT_SPEED;
  } else {
    // Go straight
    motorSpeedLeft = DEFAULT_SPEED;
    motorSpeedRight = DEFAULT_SPEED;
  }

  lastError = centeringError;

  // // Walls scenarios
  // if (distanceLeft > farWallThreshold) {
  //   // No left wall, that means it's open, turn left
  //   delay(1000);
  //   turn(0);
  //   turn(2);
  //   goto END;
  // } else if (distanceFront <= frontNearWallThreshold) {
  //   // Near front wall, stop
  //   motorSpeedLeft = 0;
  //   motorSpeedRight = 0;

  //   if (distanceRight > farWallThreshold) {
  //     // No right wall, that means it's open, turn right
  //     delay(1000);
  //     turn(1);
  //     goto END;
  //   } else {
  //     // Both left and right walls are blocked, go back
  //     // Turn right twice.
  //     delay(1000);
  //     turn(1);
  //     turn(1);
  //     goto END;
  //   }
  // }

  // Set motor speeds

  // Clip integral windup and motor speeds to max PWM value (255)
  motorSpeedLeft = constrain(motorSpeedLeft, 20, 50);
  motorSpeedRight = constrain(motorSpeedRight, 20, 50);

  // Print motorSpeedLeft and right and centeringError
  Serial.println("Motor Speed Left:" + String(motorSpeedLeft) + ",Motor Speed Right:" + String(motorSpeedRight) + ",Centering Error:" + String(centeringError));

  // Set motor directions and speeds
  if (motorSpeedLeft > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_PWM, motorSpeedLeft);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -motorSpeedLeft);
  }

  if (motorSpeedRight > 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, motorSpeedRight);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -motorSpeedRight);
  }

  END:

  if ((micros() - beginTime) > REFRESH_PERIOD) {
    digitalWrite(LED_BUILTIN, 1);
  } else digitalWrite(LED_BUILTIN, 0);
}
