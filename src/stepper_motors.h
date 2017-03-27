#ifndef STEPPER_MOTORS_H
#define STEPPER_MOTORS_H

#include <Arduino.h>
#include "pin_map.h"
#include "timers.h"

// Stepper timer runs at 100kHz = 10us per timer tick
#define STEPPER_TIMER_FREQ 1000000

// Defines with how much precision error compensation will occur
#define ERROR_COMPENSATION_PRECISION 6

// 1 = full step, 2 = half step, 4 = quarter step, 8 = eighth step
#define MICRO_STEPPING_MULTIPLIER 8
#define STEPS_PER_REV 200 * MICRO_STEPPING_MULTIPLIER

// Set to % duty cycle that the ACTIVE clock signal should be (e.g. 10 = 10% duty cycle)
#define ACTIVE_DUTY_CYCLE 20

// Set to the number of steppers this library needs to be able to handle
#define NUM_STEPPERS 2

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
  volatile unsigned short int PIN_CLK, PIN_DIR, PIN_MS1, PIN_MS2, PIN_ON_OFF;
  volatile uint16_t stepping;
  volatile bool clkFlag;

public:
  StepperMotor(uint16_t pinClk_c, uint16_t pinDir_c) : PIN_CLK(pinClk_c), PIN_DIR(pinDir_c) {
    // Set all appropriate pins to output
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    // Pull clock pin INACTIVE to indicate reduced current mode
    digitalWriteFast(PIN_CLK, INACTIVE);
    clkFlag = INACTIVE;

    // Pull direction pin ACTIVE (CW)
    digitalWriteFast(PIN_DIR, ACTIVE);
  }

  StepperMotor(uint16_t pinClk_c, uint16_t pinDir_c, uint16_t pinMS1_c, uint16_t pinMS2_c, uint16_t pinOnOff_c, uint16_t stepping_c) :
  PIN_CLK(pinClk_c), PIN_DIR(pinDir_c), PIN_MS1(pinMS1_c), PIN_MS2(pinMS2_c), PIN_ON_OFF(pinOnOff_c) {
    // Set all appropriate pins to output
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_MS1, OUTPUT);
    pinMode(PIN_MS2, OUTPUT);
    pinMode(PIN_ON_OFF, OUTPUT);

    // Turn the motor off for now
    digitalWriteFast(PIN_ON_OFF, INACTIVE);

    // Pull clock pin INACTIVE to indicate reduced current mode
    digitalWriteFast(PIN_CLK, INACTIVE);
    clkFlag = INACTIVE;

    // Pull direction pin ACTIVE (CW)
    digitalWriteFast(PIN_DIR, ACTIVE);

    // If the stepping value is invalid, default to eighth step
    if (stepping_c == 1 || stepping_c == 2 || stepping_c == 4 || stepping_c == 8) {
      setStepping(stepping_c);
    } else {
      setStepping(8);
    }
  }

  /**
   * Accepts 1, 2, 4, or 8 to represent full, half, quarter, and eighth
   * stepping and updates motor pins to reflect the change. If any other
   * value is provided, nothing happens.
   *
   * @param stepping The stepping level
   */
  void setStepping(uint16_t stepping) {
    // Set step level
    switch (stepping) {
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
        break;
      // Default returns immediately
      default:
        return;
    }

    this->stepping = stepping;
  }

  /**
   * Set the clock pin to ACTIVE state, indicating a step
   */
  void setClockActive() {
    digitalWriteFast(PIN_CLK, ACTIVE);
  }

  /**
   * Set the clock pin to INACTIVE state, indicating reduced current mode
   */
  void setClockInactive() {
    digitalWriteFast(PIN_CLK, INACTIVE);
  }

  /**
   * Flips the clock pin to ACTIVE if currently INACTIVE and
   * vice-versa.
   *
   * @return The new state of the clock pin
   */
  bool toggleClock() {
    digitalWriteFast(PIN_CLK, clkFlag);
    clkFlag = !clkFlag;

    return clkFlag;
  }

  /**
   * Returns whether the clock pin is ACTIVE or INACTIVE
   *
   * @return ACTIVE if the pin is ACTIVE, INACTIVE otherwise
   */
   bool getClockStatus() {
     return clkFlag;
   }
};

/**
 * StepperManager is essentially a static class/singleton. Instead of using
 * the standard C++ class-definition, which handles static classes messily,
 * the underlying data structures and functions are wrapped in a StepperManager
 * namespace. The private members are stored within in an anonymous namespace
 * which is inaccessible to any other classes
 */
namespace StepperManager {
  namespace {
    // Sorted by index, the steppers are: X, Y, Z, A, B
    StepperMotor steppers[NUM_STEPPERS] { {PIN_X_MOTOR_CLK, PIN_X_MOTOR_DIR}, {PIN_Y_MOTOR_CLK, PIN_Y_MOTOR_DIR} };
    // Frequency each stepper is running at
    uint32_t stepperFreqs[NUM_STEPPERS] { 0, 0 };
    // Number of timer ISR ticks the clock signal should be active
    uint64_t stepperActiveCounts[NUM_STEPPERS] { 0, 0 };
    // Number of timer ISR ticks the clock signal should be inactive
    uint64_t stepperInactiveCounts[NUM_STEPPERS] { 0, 0 };
    // Number of full clock cycles before have to add an error compensation tick to the inactive signal
    uint64_t stepperErrorCounts[NUM_STEPPERS][ERROR_COMPENSATION_PRECISION];
    // When to reset the error counter value to 0
    uint64_t stepperErrorCounterResetValues[NUM_STEPPERS];
    // Counts number of full clock cycles for error compensation
    volatile uint64_t stepperErrorCounters[NUM_STEPPERS];
    // Counts number of timer ISR ticks for motor clocks
    volatile int64_t stepperClockCounters[NUM_STEPPERS] { 0, 0 };
    // The status of each stepper motor
    volatile bool stepperStatuses[NUM_STEPPERS] { INACTIVE, INACTIVE };
    // The timer used to run the steppers
    Timer stepperTimer(STEPPER_TIMER_PIT_CH);

    /**
     * Iterates through each stepper and toggles its clock
     * if its counter has counted-down to zero. Increments
     * the error counter on the rising edge of every active
     * clock cycle.
     */
    void stepperISR() {
      // Loop through each stepper motor
      for (uint32_t i = 0; i < NUM_STEPPERS; ++i) {
        // If the count has reached zero, toggle a step
        if (--stepperClockCounters[i] == 0) {
          StepperMotor* stepper = steppers + i;
          stepperStatuses[i] = stepper->toggleClock();

          // If we've gone through a full cycle, increment the error counter
          if (stepperStatuses[i] == ACTIVE) {
            stepperErrorCounters[i]++;
          }
        }
      }
    }
  }

  /**
   * Starts the stepper timer to begin operations
   */
  void begin() {
    stepperTimer.begin(stepperISR, STEPPER_TIMER_FREQ);
  }

  /**
   * This function must be called regularly by the device's control loop.
   * Regularly means it MUST run, at bare minimum, 2 to 3 times as fast
   * as the stepper timer. Play it safe, run it orders of magnitude faster
   * than the stepper timer to ensure performance.
   * Gathers clock count, error, and status information for each stepper
   * and updates its clock count accordingly. Uses error compensation
   * information to adjust clock counts and keep steps accurate.
   */
  void update() {
    // Update each stepper
    for (uint32_t i = 0; i < NUM_STEPPERS; ++i) {
      // Collect data also used in timer interrupts
      stepperTimer.disableInterrupts();
      int64_t stepperCountVal = stepperClockCounters[i];
      uint64_t stepperErrorVal = stepperErrorCounters[i];
      bool stepperStatusVal = stepperStatuses[i];
      stepperTimer.enableInterrupts();

      // If the clock value has counted down to <= 0, update the stepper with new values
      if (stepperCountVal <= 0) {
        // The base count depends on whether the stepper is now active or inactive
        uint64_t nextCount = stepperStatusVal == ACTIVE ? stepperActiveCounts[i] : stepperInactiveCounts[i];

        // Error compensation only occurs on the inactive portion of the cycle
        if (stepperStatusVal == INACTIVE) {
          // Get the first level of error counting
          uint64_t errorCount = stepperErrorCounts[i][0];

          // Check each level of error counting to see if new ticks should be
          // added to the next clock count, stops when the maximum level of error
          // counting is reached or the next error counting value is 0
          for (unsigned int error_idx = 1; error_idx < ERROR_COMPENSATION_PRECISION && errorCount > 0; ++error_idx) {
            if (stepperErrorVal % errorCount == 0) {
              nextCount++;
            }

            errorCount = stepperErrorCounts[i][error_idx];
          }
        }

        // Update values used in the timer ISR
        stepperTimer.disableInterrupts();
        // The stepperClockCounter is <= 0, any negative value must be added to the nextCount
        // as these are overshoot ticks and therefore reduce the number of ticks in the next cycle
        stepperClockCounters[i] += nextCount;

        // Only reset the error counter on the falling edge of the cycle
        if (stepperStatusVal == INACTIVE && stepperErrorVal == stepperErrorCounterResetValues[i]) {
          stepperErrorCounters[i] = 0;
        }
        stepperTimer.enableInterrupts();
      }
    }
  }

  /**
   * Takes in a value 0 < x < 1 which represents a fraction of an
   * ISR tick. Computes when to add extra counts to the stepper
   * timings to compensate for missed fractional ISRs. Precision
   * of the error correction can be set using symbolic constants at
   * top of file. Precision dictates how many different error correction
   * counters are kept track of at once
   *
   * @param remainder The remainder to compute error correction for
   */
  void computeErrorCalibration(uint32_t stepper_idx, int multiplier, float remainder) {
    unsigned int error_idx = 0;
    unsigned int resetValue = 1;

    for (; error_idx < ERROR_COMPENSATION_PRECISION; ++error_idx) {
      if (remainder <= 0.01) {
        stepperErrorCounts[stepper_idx][error_idx] = 0;
      } else {
        float newCount = (1.0 / remainder);
        int ceilNewCount = (int)ceil(newCount);
        if (error_idx > 0) {
          stepperErrorCounts[stepper_idx][error_idx] = stepperErrorCounts[stepper_idx][error_idx - 1] * ceilNewCount;
        } else {
          stepperErrorCounts[stepper_idx][error_idx] = ceilNewCount;
        }

        resetValue *= ceilNewCount;
        remainder = remainder * ceilNewCount - 1.0;
      }
    }
  }

  /**
   * Set the speed of a stepper motor in rotations per second. Positive
   * values indicate clockwise, negative values counter-clockwise.
   * A value of 0 stops the motor. The general range of acceptable
   * values is 0.1Hz < f < 6Hz.
   *
   * @param motorIndex The index of the motor
   * @param freq The desired speed
   */
  void setSpeed(unsigned int motorIndex, float freq) {
    // Does not refer to a valid motor, ignore
    if (motorIndex >= NUM_STEPPERS) return;

    // Store the requested frequency
    stepperFreqs[motorIndex] = freq;

    float activeCount_f, inactiveCount_f;
    int64_t activeCount_i, inactiveCount_i;
    if (freq != 0.0) {
      // Compute number of steps active and inactive based on timer freq and percent duty cycle
      float activeUnit = ((float)STEPPER_TIMER_FREQ * (float)ACTIVE_DUTY_CYCLE / ((float)STEPS_PER_REV)) / 100.0;
      activeCount_f = activeUnit / freq;
      activeCount_i = floor(activeCount_f);
      inactiveCount_f = activeCount_f * 4.0;
      inactiveCount_i = floor(inactiveCount_f);

      // Add up the remainders of both the active and inactive count
      float totalRemainder_f = (activeCount_f - activeCount_i) + (inactiveCount_f - inactiveCount_i);
      float totalRemainder_i = floor(totalRemainder_f);

      // If the added remainder adds a whole step, account for that step and get the new remainder
      if (totalRemainder_f >= 1) {
        inactiveCount_f += 1;
        inactiveCount_i += 1;
        totalRemainder_f -= 1;
        totalRemainder_i -= 1;
      }

      // Compute error correction based on remainder
      computeErrorCalibration(motorIndex, activeCount_i + inactiveCount_i, totalRemainder_f - totalRemainder_i);
    } else {
      activeCount_i = inactiveCount_i = 0;
    }

    // Disable interrupts and set the new counter values
    stepperTimer.disableInterrupts();
    // Set the stepper in reduced current mode
    steppers[motorIndex].setClockInactive();
    stepperClockCounters[motorIndex] = inactiveCount_i;
    stepperActiveCounts[motorIndex] = activeCount_i;
    stepperInactiveCounts[motorIndex] = inactiveCount_i;
    stepperTimer.enableInterrupts();
  }

  /**
   * Takes in a motor to set the speed of and the speed in
   * mm/s and sets the motor at that speed.
   *
   * @param motorIndex The index of the motor to change
   * @param speed The speed in mm/s
   */
  void setSpeedMM(unsigned int motorIndex, float speed) {
    float inches = speed / 25.4;
    float rotations = inches / 0.1;

    return setSpeed(motorIndex, rotations);
  }
};

#endif
