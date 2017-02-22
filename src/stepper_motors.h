#ifndef STEPPER_MOTORS_H
#define STEPPER_MOTORS_H

#include <Arduino.h>
#include "pin_map.h"
#include "timers.h"

// Stepper timer runs at 100kHz = 10us per timer tick
#define STEPPER_TIMER_FREQ 100000

// 1 = full step, 2 = half step, 4 = quarter step, 8 = eighth step
#define MICRO_STEPPING_MULTIPLIER 1
#define STEPS_PER_REV 200 * MICRO_STEPPING_MULTIPLIER

// Set to % duty cycle that the ACTIVE clock signal should be (e.g. 10 = 10% duty cycle)
#define ACTIVE_DUTY_CYCLE 20

// Set to the number of steppers this library needs to be able to handle
#define NUM_STEPPERS 5

// Comment this out if your stepper motor is sourcing rather than sinking
#define SINKING

#ifdef SINKING
#define ACTIVE LOW
#define INACTIVE HIGH
#else
#define ACTIVE HIGH
#define INACTIVE LOW
#endif

#define CW ACTIVE
#define CCW INACTIVE

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

    // Set step level
    switch (MICRO_STEPPING_MULTIPLIER) {
      // Full step
      case 1:
        digitalWriteFast(PIN_MS1, ACTIVE);
        digitalWriteFast(PIN_MS2, ACTIVE);
        break;
      // Half step
      case 2:
        digitalWriteFast(PIN_MS1, INACTIVE);
        digitalWriteFast(PIN_MS2, ACTIVE);
        break;
      // Quarter step
      case 4:
        digitalWriteFast(PIN_MS1, ACTIVE);
        digitalWriteFast(PIN_MS2, INACTIVE);
        break;
      // Eighth step
      case 8:
        digitalWriteFast(PIN_MS1, INACTIVE);
        digitalWriteFast(PIN_MS2, INACTIVE);
      // Default to eighth step (slowest and smoothest)
      default:
        digitalWriteFast(PIN_MS1, INACTIVE);
        digitalWriteFast(PIN_MS2, INACTIVE);
    }

    // Pull direction pin LOW (clockwise)
    digitalWriteFast(PIN_DIR, ACTIVE);

    // Pull clock pin HIGH to
    digitalWriteFast(PIN_CLK, HIGH);

    // Start the motor at 0 rotational velocity
    freq = 0;
  }

  void runMotor() {
    
  }

  /**
   * Sets the motor turning at the desired angular velocity
   *
   * @param vel The desired angular velocity in rad/s
   */
  bool setAngularVel(unsigned int vel, char dir) {
    return setFreq(vel / (2 * M_PI), dir);
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
