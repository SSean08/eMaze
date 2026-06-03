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
#include "ToFFilter.h"
#include <PID_v1.h>

#define DBG


/*
  DEFINE AND INITIALIZE CONSTANTS HERE
*/


const int MAX_REFRESH_RATE = 100; // Hz
const int REFRESH_PERIOD = (int) (1.0f / MAX_REFRESH_RATE * 1000000); // uS

// VL053L0X ToF Sensor Pins

const int XSHUT_LEFT = 2;
const int INTERRUPT_LEFT = 3;

const int XSHUT_RIGHT = 11;
const int INTERRUPT_RIGHT = 5;


// LM298N Motor Driver Pins
const int MOTOR_LEFT_IN1 = 14;
const int MOTOR_LEFT_IN2 = 15;

const int MOTOR_RIGHT_IN1 = 16;
const int MOTOR_RIGHT_IN2 = 17;

// LM298N Motor Driver PWM Pins
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;

// PID
const double Kp = 0.09f;
const double Ki = 0.0f;
const double Kd = 0.0f;
double setPoint = 0.0f;
double PIDinput = 0.0f;
double PIDoutput = 0.0f;

// Default Speed
const int DEFAULT_SPEED = 0.175*255; // 20% of max speed (255)
const float rotationSpeedRelativeToDefault = 0.2*255; // Old default speed
const long rotationCount = 11000; // Number of iterations to turn 90 degrees, this is a hyperparameter that can be tuned based on the robot's turning speed and desired turning angle.

// Calibration offset for the difference of the left and right ToF sensors.
const int calibrationOffset = 3; // mm, this can be tuned based on the actual readings of the sensors when the robot is centered between the walls.




/*
  DEFINE AND INITIALIZE VARIABLES HERE
*/

long beginTime = 0, endTime;
float motorSpeedLeft = 0, motorSpeedRight = 0;

// Initialize ToF sensors
VL53L0X sensorLeft;
VL53L0X sensorRight;

//Instantiate filters for each sensor with alpha = 0.2 (tunable)
ToFFilter filterLeft(0.2f);
ToFFilter filterRight(0.2f);


// ToF sensor distance readings
int distanceLeft = 0;
int distanceRight = 0;
bool sensorReady = false; // Flag to indicate if sensors are initialized and ready
// PID controller for centering
PID centeringPID(&PIDinput, &PIDoutput, &setPoint, Kp, Ki, Kd, DIRECT);



/*
  USER-DEFINED FUNCTIONS HERE
*/

#include <Wire.h>

void i2c_recovery() {
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, HIGH); // Set clock high
  
  // Pulse the clock 9 times to force slave to release SDA
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  
  // Send a Stop Condition
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  
  // Re-initialize the I2C bus
  Wire.end();
  Wire.begin();
}



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

  i2c_recovery();

  // Use built-in LED as Debug pin, it is high when system refresh rate < 250Hz
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial communication bitrate


  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(1000);
  Serial.println("I2C Initialized!");


  // Setup ToF sensor pins
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(INTERRUPT_LEFT, INPUT);

  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(INTERRUPT_RIGHT, INPUT);

  // Set ToF sensor unique addresses
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  Serial.println("Setting ToF sensor unique addresses...");

  Serial.println("Setting left sensor address...");
  // Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  sensorLeft.setTimeout(2000);
  delay(500);
  if (!sensorLeft.init()) {
    Serial.println("Failed to set ToF left sensor address!");
    sensorReady = false; // Sensors failed to initialize, set flag to false
  } else {
    sensorLeft.setAddress(0x49); // Set unique I2C address for left sensor
    Serial.println("Left sensor address set!");
    sensorReady = true; // Mark sensor as ready
  }


  Serial.println("Setting right sensor address...");
  // Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(500);
  sensorRight.setTimeout(2000);
  if (!sensorRight.init()) {
    Serial.println("Failed to set ToF right sensor address!");
    sensorReady = false; // Sensors failed to initialize, set flag to false
  } else {
    sensorRight.setAddress(0x59); // Set unique I2C address for right sensor
    Serial.println("Right sensor address set!");
    sensorReady = true; // Mark sensor as ready
  }

  Serial.println("ToF sensors initialized with unique addresses!");
  delay(200);

  Serial.println("Setting sensor speeds...");
  

  sensorLeft.setMeasurementTimingBudget(20000);
  sensorRight.setMeasurementTimingBudget(20000);

  if (sensorReady) {
    Serial.println("All sensors initialized successfully! Starting continuous measurement...");
  } else {
    Serial.println("One or more sensors failed to initialize. Please check connections and try again.");
    while (true) {
      // Halt the program if sensors are not ready
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED to indicate error state
      delay(500);
      digitalWrite(LED_BUILTIN, LOW); // Turn on built-in LED to indicate error state
    }
  }

  Serial.println("Starting continous measurement");
  sensorLeft.startContinuous();
  sensorRight.startContinuous();
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

  

  // Configure PID controller
  centeringPID.SetMode(AUTOMATIC); // Enable PID computation
  centeringPID.SetOutputLimits(-180, 180); // Allow negative/positive corrections (expanded to prevent clipping)
  Serial.println("PID configured: AUTOMATIC, limits [-180,180]");
}



void loop() {
  beginTime = micros();

  // Read raw distance values from ToF sensors
  uint16_t rawDistanceLeft = sensorLeft.readRangeContinuousMillimeters();
  uint16_t rawDistanceRight = sensorRight.readRangeContinuousMillimeters();

  // Update filtered distance values using the ToFFilter class
  distanceLeft = filterLeft.update(rawDistanceLeft);
  distanceRight = filterRight.update(rawDistanceRight);

  // PID control for wall following
  PIDinput = (distanceLeft - distanceRight) - calibrationOffset; // Calculate the error
  if (abs(PIDinput) <= 3) {
    PIDinput = 0;
  }
  centeringPID.Compute();
  motorSpeedLeft = DEFAULT_SPEED + PIDoutput;
  motorSpeedRight = DEFAULT_SPEED - PIDoutput;

  // Clamps motor speeds to 33 and 55 to prevent excessive speed.
  motorSpeedLeft = constrain(motorSpeedLeft, 33, 60);
  motorSpeedRight = constrain(motorSpeedRight, 33, 60);

  // Write PWM to motor driver to set speeds and directions
  #ifndef DBG
  if (motorSpeedLeft > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_PWM, motorSpeedLeft);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, motorSpeedLeft);
  }

  if (motorSpeedRight > 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, motorSpeedRight);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, motorSpeedRight); 
  }
  #endif

  #ifdef DBG
  //Print motor speeds and distances for debugging
  Serial.print("L:");
  Serial.print(distanceLeft);
  Serial.print(" R:");
  Serial.print(distanceRight);
  Serial.print(" Err:");
  Serial.print(PIDinput);
  Serial.print(" Out:");
  Serial.print(PIDoutput);
  Serial.print(" ML:");
  Serial.print(motorSpeedLeft);
  Serial.print(" MR:");
  Serial.println(motorSpeedRight);
  #endif

  if ((micros() - beginTime) > REFRESH_PERIOD) {
    digitalWrite(LED_BUILTIN, 1);
  } else digitalWrite(LED_BUILTIN, 0);
}
