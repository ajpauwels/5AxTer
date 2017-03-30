#ifndef SETTINGS_H
#define SETTINGS_H

/** STEPPER MOTOR SETTINGS **/
// The frequency to run the stepper timer at (default = 100000 = 100kHz)
#define STEPPER_TIMER_FREQ 100000

// Defines with how much precision error compensation will occur (default = 6)
#define ERROR_COMPENSATION_PRECISION 6

// 1 = full step, 2 = half step, 4 = quarter step, 8 = eighth step
#define MICRO_STEPPING_MULTIPLIER 8
#define BASE_STEPS_PER_REV 200
#define STEPS_PER_REV BASE_STEPS_PER_REV * MICRO_STEPPING_MULTIPLIER

// Set to % duty cycle that the ACTIVE clock signal should be (default = 50 = 50% duty cycle)
#define ACTIVE_DUTY_CYCLE 50

// Set to the number of steppers this library needs to be able to handle
#define NUM_STEPPERS 2

// Comment this out if your stepper motor is sourcing rather than sinking
#define SINKING

/** CONTROL SETTINGS **/
// The frequency to run the control loop at (default = 20000 = 20kHz)
#define CONTROL_LOOP_FREQ 20000

#endif
