/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

// Initialize ToF sensors
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

// ToF sensor distance readings
uint16_t distanceFront = 0;
uint16_t distanceLeft = 0;
uint16_t distanceRight = 0;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY

const int XSHUT_LEFT = 2;
const int INTERRUPT_LEFT = 3;

const int XSHUT_RIGHT = 11;
const int INTERRUPT_RIGHT = 5;

const int XSHUT_FRONT = 6;
const int INTERRUPT_FRONT = 7;


void setup()
{
  delay(1000);
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
  
}

void loop()
{
  Serial.print(sensorLeft.readRangeSingleMillimeters());
  if (sensorLeft.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print(", ");
  Serial.print(sensorRight.readRangeSingleMillimeters());
  if (sensorRight.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print(", ");
  Serial.print(sensorFront.readRangeSingleMillimeters());
  if (sensorFront.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  
}
