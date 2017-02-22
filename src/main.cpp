#include <Arduino.h>

#include "timers.h"
#include "state_machine.h"
#include "stepper_motors.h"
#include "pin_map.h"

unsigned int controlLoop_ctr = 0;

void controlTimerISR() {}

StepperMotor xMotor(PIN_X_MOTOR_CLK, PIN_X_MOTOR_DIR);
Timer controlTimer(CONTROL_TIMER_PIT_CH);

void setup() {
  // Printer initializes to the WAIT state
  state = WAIT_BEFORE_START;

  // Start the control loop - 100000Hz
  controlTimer.begin(controlTimerISR, 100000);
  xMotor.setFreq(1, CW);
}

void loop() {
  if (controlTimer.isReady()) {
    runMotors();
    switch (state) {
      case WAIT_BEFORE_START:
        ++controlLoop_ctr;
        if (controlLoop_ctr >= 200000) {
          controlLoop_ctr = 0;
          state = START;
        }
        break;
      case WAIT_BEFORE_STOP:
        ++controlLoop_ctr;
        if (controlLoop_ctr >= 100000) {
          controlLoop_ctr = 0;
          state = STOP;
        }
        break;
      case START:
        // xMotor.setFreq(6, 1);
        state = WAIT_BEFORE_STOP;
        break;
      case STOP:
        // xMotor.stop();
        state = WAIT_BEFORE_START;
        break;
    }
  }
}

void runMotors() {
  xMotor.run();
}
