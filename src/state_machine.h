#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum MACHINE_STATE {
  WAIT_BEFORE_START,
  WAIT_BEFORE_STOP,
  START,
  STOP,
  RAMP_UP,
  RAMP_DOWN,
  HOLD,
  MAX_SPEED
} state;

#endif
