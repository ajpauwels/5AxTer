#include <Arduino.h>

#include "timers.h"
#include "state_machine.h"
#include "stepper_motors.h"
#include "pin_map.h"
#include "settings.h"

// The index of each stepper in the StepperManager
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define A_INDEX 3
#define B_INDEX 4

unsigned int controlLoop_ctr = 0;
bool NEW_X_DIR = HIGH;
bool NEW_Y_DIR = HIGH;
float newSpeed = 0;

bool FLAG = HIGH;

void controlTimerISR();
Timer controlTimer(CONTROL_TIMER_PIT_CH);

void setup() {
  // Initial printer state
  state = RAMP_UP;

  Serial.begin(9600);

  pinMode(28, OUTPUT);

  // Start the control loop - 100Hz
  controlTimer.begin(controlTimerISR, CONTROL_LOOP_FREQ);
  StepperManager::begin();
  StepperManager::setSpeed(X_INDEX, 0.0);
  StepperManager::setSpeed(Y_INDEX, 3.0);
  digitalWriteFast(PIN_X_MOTOR_DIR, NEW_X_DIR);
  digitalWriteFast(PIN_Y_MOTOR_DIR, NEW_Y_DIR);
}

void loop() {
  StepperManager::update();
  if (controlTimer.isReady()) {
    digitalWriteFast(28, FLAG);
    FLAG = !FLAG;
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
        float newSpeed = 3.0 - ((float)controlLoop_ctr / (float)CONTROL_LOOP_FREQ);
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
        float newSpeed = 0.0 + ((float)controlLoop_ctr / (float)CONTROL_LOOP_FREQ);
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
        if (controlLoop_ctr >= 20000) {
          controlLoop_ctr = 0;
          state = START;
        }
        break;
      case WAIT_BEFORE_STOP:
        if (controlLoop_ctr >= 20000) {
          controlLoop_ctr = 0;
          state = STOP;
        }
        break;
      case START:
        StepperManager::setSpeed(X_INDEX, 1.0);
        state = WAIT_BEFORE_STOP;
        break;
      case STOP:
        StepperManager::setSpeed(X_INDEX, 2.0);
        state = WAIT_BEFORE_START;
        break;
    }
  }
}

void controlTimerISR() {}
