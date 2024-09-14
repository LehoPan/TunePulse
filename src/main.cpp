#include <Arduino.h>
#include "blocks_lib.h"
#include "bootloaderTools.h"
#include "target.h"

inputTarget test = {100, 0.001, 10, 3, 0};
static MotionPlanTrapezoidal motion0(test);


void setup() {
  SerialUSB.begin();
  pinMode(PINOUT::LED_GRN, OUTPUT);
  
}

void loop() {

  motion0.tick();
  SerialUSB.print(">0testVel:");
  SerialUSB.println(motion0.get_current_vel());

  SerialUSB.print(">0testPosition:");
  SerialUSB.println(motion0.get_current_pos());
  
  // motion1.tick();
  // SerialUSB.print(">1testAccel:");
  // SerialUSB.println(motion1.get_current_accel());

  // SerialUSB.print(">1testVel:");
  // SerialUSB.println(motion1.get_current_vel());

  // SerialUSB.print(">1testPosition:");
  // SerialUSB.println(motion1.get_current_pos());

  delay(1);

}