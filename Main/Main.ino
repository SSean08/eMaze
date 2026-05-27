/*
  Author: Sean Andre C. Ancheta

  This close-looped system will have a refresh rate of 250Hz.
  That is, 4mS or 4000uS of period.

*/

/*
  DEFINE AND INITIALIZE CONSTANTS HERE
*/

/*
  DEFINE AND INITIALIZE VARIABLES HERE
*/

long beginTime = 0, endTime;



void setup() {
  Serial.begin(115200);

  // Use built-in LED as Debug pin, it is high when system refresh rate < 250Hz
  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {
  beginTime = micros();

  // Main control loop code



  if ((micros() - beginTime) > 4000) {
    digitalWrite(LED_BUILTIN, 1);
  } else digitalWrite(LED_BUILTIN, 0);
}
