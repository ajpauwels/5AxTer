#include <Arduino.h>

#include "timers.h"
#include "state_machine.h"
#include "stepper_motors.h"
#include "pin_map.h"
#include "settings.h"
#include "PathPlanner.hpp"

// The index of each stepper in the StepperManager
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define A_INDEX 3
#define B_INDEX 4

volatile StepperMotor SM_X(24, 25);
volatile StepperMotor SM_Y(26, 27);
volatile StepperMotor SM_Z(28, 29);
PathPlanner pp;
bool result;

volatile unsigned int controlLoop_ctr = 0;
volatile bool NEW_X_DIR = HIGH;
volatile bool NEW_Y_DIR = HIGH;
volatile bool NEW_Z_DIR = HIGH;
volatile bool NEW_A_DIR = HIGH;
volatile bool NEW_B_DIR = HIGH;
volatile float newSpeed = 0;

volatile bool FLAG = HIGH;

void controlTimerISR();
Timer controlTimer(CONTROL_TIMER_PIT_CH);

void setup() {
  // Initial printer state
  state = RAMP_UP;

  Serial.begin(9600);

  result = pp.loadFile("OBJ.TXT");

  // Start the control loop - 100Hz
  controlTimer.begin(controlTimerISR, CONTROL_LOOP_FREQ);
  SM_X.setSpeed(0);
  SM_Y.setSpeed(0);
  SM_Z.setSpeed(0);
}

void loop() {
  pp.update();
}

void controlTimerISR() {
  ++controlLoop_ctr;
  switch (state) {
    case RAMP_DOWN: {
      float newSpeed = 3.0 - ((float)controlLoop_ctr / (float)CONTROL_LOOP_FREQ);
      SM_X.setSpeed((NEW_X_DIR ? 1.0 : -1.0) * newSpeed);
      SM_Y.setSpeed((NEW_Y_DIR ? 1.0 : -1.0) * (3.0 - newSpeed));
      SM_Z.setSpeed((NEW_Z_DIR ? 1.0 : -1.0) * newSpeed);
      if (newSpeed <= 0) {
        controlLoop_ctr = 0;
        NEW_X_DIR = !NEW_X_DIR;
        NEW_Z_DIR = !NEW_Z_DIR;
        state = RAMP_UP;
      }
      break;
    }
    case RAMP_UP: {
      float newSpeed = 0.0 + ((float)controlLoop_ctr / (float)CONTROL_LOOP_FREQ);
      SM_X.setSpeed((NEW_X_DIR ? 1.0 : -1.0) * newSpeed);
      SM_Y.setSpeed((NEW_Y_DIR ? 1.0 : -1.0) * (3.0 - newSpeed));
      SM_Z.setSpeed((NEW_Z_DIR ? 1.0 : -1.0) * newSpeed);
      if (newSpeed >= 3.0) {
        controlLoop_ctr = 0;
        NEW_Y_DIR = !NEW_Y_DIR;
        state = RAMP_DOWN;
      }
      break;
    }
  }
}
