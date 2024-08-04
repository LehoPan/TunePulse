#include <Arduino.h>
#include "blocks_lib.h"
#include "bootloaderTools.h"
#include "target.h"

float a = 100.0;
float b = 500.0;
float c = 3;
float d = 0.1;
static MotionPlanTrapezoidal motion0(a, b, c, d);

void setup() {
  SerialUSB.begin();
  
  pinMode(PINOUT::LED_GRN, OUTPUT);

}


void loop() {

  motion0.tick();
  SerialUSB.print(">testVel:");
  SerialUSB.println(motion0.get_current_vel());

  SerialUSB.print(">testPosition:");
  SerialUSB.println(motion0.get_current_pos());

  delay(100);

}
