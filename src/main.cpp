#include <Arduino.h>
#include <SPI.h>

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

#define X_AXIS 24
#define Y_AXIS 26
#define Z_AXIS 28
#define A_AXIS 33
#define B_AXIS 35
#define MCLK 25000000
#define MULT 0x10000000
SPISettings s(40000000, MSBFIRST, SPI_MODE2);

PathPlanner pp;
bool result;

unsigned int controlLoop_ctr = 0;
bool NEW_X_DIR = HIGH;
bool NEW_Y_DIR = HIGH;
bool NEW_Z_DIR = HIGH;
bool NEW_A_DIR = HIGH;
bool NEW_B_DIR = HIGH;
float newSpeed = 0;

bool FLAG = HIGH;

void controlTimerISR();
void fileReadISR();
void setFreq(uint32_t FSYNC, uint32_t f);
void write(uint16_t d);
Timer controlTimer(CONTROL_TIMER_PIT_CH);
Timer fileReadTimer(3);

void setup() {
  // Initial printer state
  state = RAMP_UP;

  Serial.begin(9600);
  delay(10);

  // Freq gen
  pinMode(X_AXIS, OUTPUT);
  pinMode(Y_AXIS, OUTPUT);
  pinMode(Z_AXIS, OUTPUT);
  pinMode(A_AXIS, OUTPUT);
  pinMode(B_AXIS, OUTPUT);
  Serial.begin(9600);
  digitalWrite(X_AXIS, HIGH);
  digitalWrite(Y_AXIS, HIGH);
  digitalWrite(Z_AXIS, HIGH);
  digitalWrite(A_AXIS, HIGH);
  digitalWrite(B_AXIS, HIGH);
  SPI.begin();
  setFreq(X_AXIS, 0);
  setFreq(Y_AXIS, 0);
  setFreq(Z_AXIS, 0);
  setFreq(A_AXIS, 0);
  setFreq(B_AXIS, 0);

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
  setFreq(X_AXIS, 10000);
  setFreq(Y_AXIS, 20000);
  setFreq(Z_AXIS, 30000);
  setFreq(A_AXIS, 40000);
  setFreq(B_AXIS, 50000);
  delay(2000);
  // StepperManager::update();
  if (controlTimer.isReady()) {
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
  }
}

void setFreq(uint32_t FSYNC, uint32_t f) {
  uint32_t freq_word = ((float)f / (float)MCLK) * (float)MULT;
  for (unsigned int i = 0; i < 1; ++i) {
    digitalWrite(FSYNC, LOW);
    write(0x2168);
    write(0x4000 | (freq_word & 0x3FFF));
    write(0x4000 | ((freq_word >> 14) & 0x3FFF));
    write(0xC000);
    write(0x2068);
    digitalWrite(FSYNC, HIGH);
  }
}

void write(uint16_t d) {
  SPI.beginTransaction(s);
  SPI.transfer(highByte(d)); //Serial.println(highByte(dat));
  SPI.transfer(lowByte(d)); //Serial.println(lowByte(dat));
  SPI.endTransaction();
}


void controlTimerISR() {}

void fileReadISR() {}
