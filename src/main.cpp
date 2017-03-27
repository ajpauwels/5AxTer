#include <Arduino.h>

#include "timers.h"
#include "state_machine.h"
#include "stepper_motors.h"
#include "pin_map.h"

// The index of each stepper in the StepperManager
#define X_INDEX 0
#define Y_INDEX 1

unsigned int controlLoop_ctr = 0;
bool NEW_X_DIR = HIGH;
bool NEW_Y_DIR = HIGH;
float newSpeed = 0;

void controlTimerISR();
Timer controlTimer(CONTROL_TIMER_PIT_CH);

void setup() {
  // Printer initializes to the WAIT state
  state = RAMP_UP;

  // Start the control loop - 100Hz
  controlTimer.begin(controlTimerISR, 100);
  StepperManager::begin();
  StepperManager::setSpeed(X_INDEX, 0.0);
  StepperManager::setSpeed(Y_INDEX, 3.0);
  digitalWriteFast(PIN_X_MOTOR_DIR, NEW_X_DIR);
  digitalWriteFast(PIN_Y_MOTOR_DIR, NEW_Y_DIR);
}

void loop() {
  StepperManager::update();
  if (controlTimer.isReady()) {
    ++controlLoop_ctr;
    switch (state) {
      case MAX_SPEED:
        newSpeed = newSpeed + controlLoop_ctr * 0.00001;

        if (newSpeed <= 10) {
          StepperManager::setSpeed(Y_INDEX, newSpeed);
        }
        break;
      case HOLD:
        break;
      case RAMP_DOWN: {
        float newSpeed = 3.0 - controlLoop_ctr * 0.01;
        StepperManager::setSpeed(X_INDEX, newSpeed);
        StepperManager::setSpeed(Y_INDEX, 3.0 - newSpeed);
        if (newSpeed <= 0) {
          controlLoop_ctr = 0;
          NEW_X_DIR = !NEW_X_DIR;
          digitalWriteFast(PIN_X_MOTOR_DIR, NEW_X_DIR);
          state = RAMP_UP;
        }
        break;
      }
      case RAMP_UP: {
        float newSpeed = 0.0 + controlLoop_ctr * 0.01;
        StepperManager::setSpeed(X_INDEX, newSpeed);
        StepperManager::setSpeed(Y_INDEX, 3.0 - newSpeed);
        if (newSpeed >= 3.0) {
          controlLoop_ctr = 0;
          NEW_Y_DIR = !NEW_Y_DIR;
          digitalWriteFast(PIN_Y_MOTOR_DIR, NEW_Y_DIR);
          state = RAMP_DOWN;
        }
        break;
      }
      case WAIT_BEFORE_START:
        if (controlLoop_ctr >= 300000) {
          controlLoop_ctr = 0;
          state = START;
        }
        break;
      case WAIT_BEFORE_STOP:
        if (controlLoop_ctr >= 300000) {
          controlLoop_ctr = 0;
          state = STOP;
        }
        break;
      case START:
        StepperManager::setSpeed(X_INDEX, 1.0);
        state = WAIT_BEFORE_STOP;
        break;
      case STOP:
        StepperManager::setSpeed(Y_INDEX, 0.0);
        state = WAIT_BEFORE_START;
        break;
    }
  }
}

void controlTimerISR() {}
