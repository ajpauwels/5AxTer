#ifndef SETTINGS_H
#define SETTINGS_H

/** DEBUG SETTINGS **/
// Uncomment to print out debug messages to serial
#define DEBUG

/** STEPPER MOTOR SETTINGS **/
// 1 = full step, 2 = half step, 4 = quarter step, 8 = eighth step
#define MICRO_STEPPING_MULTIPLIER 8
#define BASE_STEPS_PER_REV 200
#define STEPS_PER_REV BASE_STEPS_PER_REV * MICRO_STEPPING_MULTIPLIER

// Comment this out if your stepper motor is sourcing rather than sinking
#define SINKING

/** CONTROL SETTINGS **/
// The frequency to run the control loop at (default = 20000 = 20kHz)
#define CONTROL_LOOP_FREQ 20000

// At each iteration of the control loop, up to how many GCode commands are loaded from SD
#define MAX_LOADED_GCODE_PER_ITERATION 10

/** BUFFER SETTINGS **/
// The number of chars to buffer from file
#define FILE_CHAR_BUFFER_SIZE 2048
// The number of GCode commands to buffer from the SD card
#define GCODE_COMMAND_BUFFER_SIZE 1024
#define PATH_POINTS_BUFFER_SIZE 1024

/** MISCELLANEOUS **/
// The maximum number of characters in a line of GCode (default = 108)
// Default based on:
//  - 4 characters for GCode, e.g. GXXX
//  - 12 characters per parameter, e.g. X12345.67890
//  - 8 parameters, e.g. G0 X Y Z A B E F S
//  - 1 space between each parameter + GCode
#define GCODE_MAX_LINE_WIDTH 108

#endif
