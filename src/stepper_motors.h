#ifndef STEPPER_MOTORS_H
#define STEPPER_MOTORS_H

#include <Arduino.h>
#include "pin_map.h"
#include "timers.h"

// Stepper timer runs at 100kHz = 10us per timer tick
#define STEPPER_TIMER_FREQ 100000

volatile char flag = 0;

#define MICRO_STEPPING_MULTIPLIER 1
#define STEPS_PER_REV 200 * MICRO_STEPPING_MULTIPLIER
#define ACTIVE_DUTY_CYCLE 20

#define NUM_STEPPERS 5

class StepperMotor {
private:
  unsigned short int PIN_CLK, PIN_DIR;
  uint32_t freq;
  uint32_t stepperSlot;

  static void stepperISR();
  static Timer stepperTimer;
  static uint32_t numActiveSteppers;
  static uint32_t nextAvailableSlot;
  volatile static uint32_t stepperCntrs[NUM_STEPPERS];
  volatile static uint32_t stepperActiveCountingValues[NUM_STEPPERS];
  volatile static uint32_t stepperInactiveCountingValues[NUM_STEPPERS];
  volatile static char activeSteppers[NUM_STEPPERS];
  volatile static char outputStates[NUM_STEPPERS];
  volatile static uint32_t clkPins[NUM_STEPPERS];
  volatile static uint32_t stepError[NUM_STEPPERS];
  volatile static uint32_t accumulatedError[NUM_STEPPERS];

public:
  StepperMotor(unsigned short int pinClk_c, unsigned short int pinDir_c) : PIN_CLK(pinClk_c), PIN_DIR(pinDir_c) {
    // Set all appropriate pins to output
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_MS1, OUTPUT);
    pinMode(PIN_MS2, OUTPUT);

    // Set all pins to inactive state
    digitalWriteFast(PIN_MS1, LOW);
    digitalWriteFast(PIN_MS2, LOW);
    digitalWriteFast(PIN_DIR, HIGH);
    digitalWriteFast(PIN_CLK, HIGH);

    // Start the motor at 0 rotational velocity
    freq = 0;
  }

  /**
   * Sets the motor turning at the desired angular velocity
   *
   * @param vel The desired angular velocity in rad/s
   */
  bool setAngularVel(unsigned int vel, char dir) {
    return setFreq(vel, dir);
  }

  /**
   * Sets the motor turning at the desired frequency. The acceptable frequencies
   * are currently 0.1Hz < f < 6Hz
   *
   * @param freq The desired frequency is turns/sec
   */
   bool setFreq(double desiredFreq, char dir) {
     StepperMotor::stepperTimer.stop();
     if (numActiveSteppers >= NUM_STEPPERS || desiredFreq > 6 || desiredFreq < 0.1) {
       StepperMotor::stepperTimer.begin(stepperISR, STEPPER_TIMER_FREQ);
       return false;
     }

     stepperSlot = nextAvailableSlot;
     outputStates[stepperSlot] = LOW;
     clkPins[stepperSlot] = PIN_CLK;
     accumulatedError[stepperSlot] = 0;
     activeSteppers[stepperSlot] = 1;

     freq = desiredFreq;

     uint32_t stepsPerSec = freq * STEPS_PER_REV;
     // 20,000 = (1,000,000) / (10 * 5) this is to get the number of 10us counts and then the ACTIVE time into 1/5th the INACTIVE time
     stepperActiveCountingValues[stepperSlot] = 20000 / stepsPerSec;
     stepperInactiveCountingValues[stepperSlot] = (20000.0 / (double)stepsPerSec) * 4;
     stepperCntrs[stepperSlot] = stepperActiveCountingValues[stepperSlot];
     stepError[stepperSlot] = (100000000.0 / (double)stepsPerSec) - 1000 * stepperActiveCountingValues[stepperSlot] - 1000 * stepperInactiveCountingValues[stepperSlot];

     ++numActiveSteppers;
     if (numActiveSteppers >= NUM_STEPPERS) {
       nextAvailableSlot = -1;
     } else {
       unsigned int i = 1;
       while (activeSteppers[stepperSlot + i]) {
         ++i;
       }

       nextAvailableSlot = stepperSlot + i;
     }

     digitalWriteFast(PIN_DIR, dir);
     stepperTimer.begin(stepperISR, STEPPER_TIMER_FREQ);
     return true;
   }

   void stop() {
     stepperTimer.stop();
     if (freq > 0) {
       activeSteppers[stepperSlot] = 0;
       freq = 0;
       --numActiveSteppers;
       if (nextAvailableSlot > stepperSlot) {
         nextAvailableSlot = stepperSlot;
       }
       digitalWriteFast(PIN_CLK, HIGH);
    }

    if (numActiveSteppers > 0) stepperTimer.begin(stepperISR, STEPPER_TIMER_FREQ);
   }
};

volatile uint32_t StepperMotor::stepperCntrs[NUM_STEPPERS];
volatile uint32_t StepperMotor::stepperActiveCountingValues[NUM_STEPPERS];
volatile uint32_t StepperMotor::stepperInactiveCountingValues[NUM_STEPPERS];
volatile char StepperMotor::activeSteppers[NUM_STEPPERS];
volatile char StepperMotor::outputStates[NUM_STEPPERS];
volatile uint32_t StepperMotor::clkPins[NUM_STEPPERS];
volatile uint32_t StepperMotor::stepError[NUM_STEPPERS];
volatile uint32_t StepperMotor::accumulatedError[NUM_STEPPERS];

void StepperMotor::stepperISR() {
  for (unsigned int ctr = 0; ctr < NUM_STEPPERS; ++ctr) {
    if (activeSteppers[ctr]) {
      --stepperCntrs[ctr];
      if (stepperCntrs[ctr] == 0) {
        digitalWriteFast(clkPins[ctr], outputStates[ctr]);
        if (!outputStates[ctr]) accumulatedError[ctr] += stepError[ctr];
        outputStates[ctr] = !outputStates[ctr];

        if (outputStates[ctr]) stepperCntrs[ctr] = stepperActiveCountingValues[ctr];
        else stepperCntrs[ctr] = stepperInactiveCountingValues[ctr];

        if (accumulatedError[ctr] / 10000 > 0) {
          stepperCntrs[ctr] += 1;
          accumulatedError[ctr] = accumulatedError[ctr] - 10000;
        }
      }
    }
  }
}

Timer StepperMotor::stepperTimer(STEPPER_TIMER_PIT_CH);
uint32_t StepperMotor::numActiveSteppers = 0;
uint32_t StepperMotor::nextAvailableSlot = 0;

#endif
