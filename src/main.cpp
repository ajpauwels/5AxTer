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

PathPlanner pp;
bool result;

bool FLAG = true;

unsigned int controlLoop_ctr = 0;
bool NEW_X_DIR = HIGH;
bool NEW_Y_DIR = HIGH;
bool NEW_Z_DIR = HIGH;
bool NEW_A_DIR = HIGH;
bool NEW_B_DIR = HIGH;
float newSpeed = 0;

void controlTimerISR();
void fileReadISR();
Timer controlTimer(CONTROL_TIMER_PIT_CH);
Timer fileReadTimer(3);

void setup() {
  // Initial printer state
  state = RAMP_UP;
  pinMode(33, OUTPUT);

  Serial.begin(9600);

  result = pp.loadFile("OBJ.TXT");

  fileReadTimer.begin(fileReadISR, 5000);

  // Start the control loop - 100Hz
  controlTimer.begin(controlTimerISR, CONTROL_LOOP_FREQ);
  StepperManager::begin();
  StepperManager::setSpeed(X_INDEX, 0.0);
  StepperManager::setSpeed(Y_INDEX, 3.0);
  StepperManager::setSpeed(Z_INDEX, 0.0);
  StepperManager::setSpeed(A_INDEX, 0.0);
}

void loop() {
  StepperManager::update();
  // if (fileReadTimer.isReady()) {
  //   pp.update();
  // }
  if (controlTimer.isReady()) {
    // if (FLAG) digitalWriteFast(33, HIGH);
    // else digitalWriteFast(33, LOW);
    // FLAG = !FLAG;
    ++controlLoop_ctr;
    switch (state) {
      case MAX_SPEED:
        newSpeed = newSpeed + controlLoop_ctr * 0.00001;

        if (newSpeed <= 10) {
          StepperManager::setSpeed(Y_INDEX, newSpeed);
        }
        break;
      case HOLD:
        if (controlLoop_ctr >= 60000) {
          StepperManager::setSpeed(X_INDEX, (NEW_X_DIR ? 1 : -1) * 3.0, 400);
          Serial.println(StepperManager::stepsLeft(X_INDEX));
          NEW_X_DIR = !NEW_X_DIR;
          controlLoop_ctr = 0;
        }
        break;
      case RAMP_DOWN: {
        float newSpeed = 3.0 - ((float)controlLoop_ctr / (float)CONTROL_LOOP_FREQ);
        StepperManager::setSpeed(X_INDEX, (NEW_X_DIR ? 1 : -1) * newSpeed);
        StepperManager::setSpeed(Y_INDEX, (NEW_Y_DIR ? 1 : -1) * (3.0 - newSpeed));
        StepperManager::setSpeed(Z_INDEX, (NEW_Z_DIR ? 1 : -1) * newSpeed);
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
        StepperManager::setSpeed(X_INDEX, (NEW_X_DIR ? 1 : -1) * newSpeed);
        StepperManager::setSpeed(Y_INDEX, (NEW_Y_DIR ? 1 : -1) * (3.0 - newSpeed));
        StepperManager::setSpeed(Z_INDEX, (NEW_Z_DIR ? 1 : -1) * newSpeed);
        if (newSpeed >= 3.0) {
          controlLoop_ctr = 0;
          NEW_Y_DIR = !NEW_Y_DIR;
          state = RAMP_DOWN;
        }
        break;
      }
      case NEXT_INSTRUCTION: {
        // if (gCommandBuff.available()) {
        //   GCommand comm = gCommandBuff.pop();
        //
        //   switch (comm.getType()) {
        //     case 0: {
        //       G0 formattedComm = GCodeReader::toG0(comm);
        //       break;
        //     }
        //   }
        // }
        break;
      }
    }
    pp.update();
  }
}

void controlTimerISR() {}

void fileReadISR() {}
