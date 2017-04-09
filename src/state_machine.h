#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum MACHINE_STATE {
  RAMP_UP,
  RAMP_DOWN,
  HOLD,
  MAX_SPEED,
  NEXT_INSTRUCTION
} state;

#endif
