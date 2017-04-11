#ifndef STEPPER_MOTORS_H
#define STEPPER_MOTORS_H

#include <Arduino.h>
#include <SPI.h>
#include "pin_map.h"
#include "timers.h"
#include "settings.h"

#ifdef SINKING
#define ACTIVE LOW
#define INACTIVE HIGH
#else
#define ACTIVE HIGH
#define INACTIVE LOW
#endif

#define CW ACTIVE
#define CCW INACTIVE

#define SINE_WAVE 0
#define SQUARE_WAVE 1
#define TRIANGLE_WAVE 2
#define F_MCLK 25000000
#define F_MULT 0x10000000

class FreqGen {
private:
  uint32_t currFreq;
  int16_t fsyncPin;

  void write16(uint16_t d) volatile {
    SPI.beginTransaction(FreqGen::settings);
    SPI.transfer(highByte(d));
    SPI.transfer(lowByte(d));
    SPI.endTransaction();
  }

public:
  const static SPISettings settings;

  FreqGen(int16_t _fsyncPin) : fsyncPin(_fsyncPin) {
    SPI.begin();
    pinMode(fsyncPin, OUTPUT);
    digitalWriteFast(fsyncPin, HIGH);
    reset();
  }

  void reset() {
    digitalWriteFast(fsyncPin, LOW);
    write16(0x2100);
    digitalWriteFast(fsyncPin, HIGH);
  }

  void set(uint32_t freq, uint32_t waveType = SQUARE_WAVE) volatile  {
    // Serial.println("HERE");
    // Only square waves for now...ONLY SQUARE WAVES YA HEAR
    if (waveType != SQUARE_WAVE) return;

    // Output frequency capped at 12.5MHz
    if (freq > 12500000) return;

    // Only perform for valid pins
    if (fsyncPin < 0) return;

    // Compute the frequency word and send the damn square wave
    uint32_t freq_word = ((float)freq / (float)F_MCLK) * (float)F_MULT;
    digitalWriteFast(fsyncPin, LOW);
    write16(0x4000 | (freq_word & 0x3FFF));
    write16(0x4000 | ((freq_word >> 14) & 0x3FFF));
    write16(0xC000);
    write16(0x2068);
    digitalWriteFast(fsyncPin, HIGH);
  }
};

const SPISettings FreqGen::settings(40000000, MSBFIRST, SPI_MODE2);

class StepperMotor {
private:
  volatile unsigned short int PIN_FSYNC, PIN_DIR, PIN_MS1, PIN_MS2, PIN_ON_OFF;
  volatile FreqGen clk;
  volatile uint16_t stepping;

public:
  StepperMotor(uint16_t pinFsync_c, uint16_t pinDir_c) : PIN_FSYNC(pinFsync_c), PIN_DIR(pinDir_c), clk(PIN_FSYNC) {
    // Set all appropriate pins to output
    pinMode(PIN_DIR, OUTPUT);

    // Pull direction pin ACTIVE (CW)
    digitalWriteFast(PIN_DIR, ACTIVE);

    setStepping(8);
  }

  // StepperMotor(uint16_t pinClk_c, uint16_t pinDir_c, uint16_t pinMS1_c, uint16_t pinMS2_c, uint16_t pinOnOff_c, uint16_t stepping_c) :
  // PIN_CLK(pinClk_c), PIN_DIR(pinDir_c), PIN_MS1(pinMS1_c), PIN_MS2(pinMS2_c), PIN_ON_OFF(pinOnOff_c) {
  //   // Set all appropriate pins to output
  //   pinMode(PIN_CLK, OUTPUT);
  //   pinMode(PIN_DIR, OUTPUT);
  //   pinMode(PIN_MS1, OUTPUT);
  //   pinMode(PIN_MS2, OUTPUT);
  //   pinMode(PIN_ON_OFF, OUTPUT);
  //
  //   // Turn the motor off for now
  //   digitalWriteFast(PIN_ON_OFF, INACTIVE);
  //
  //   // Pull clock pin INACTIVE to indicate reduced current mode
  //   digitalWriteFast(PIN_CLK, INACTIVE);
  //   clkFlag = INACTIVE;
  //
  //   // Pull direction pin ACTIVE (CW)
  //   digitalWriteFast(PIN_DIR, ACTIVE);
  //
  //   // If the stepping value is invalid, default to eighth step
  //   if (stepping_c == 1 || stepping_c == 2 || stepping_c == 4 || stepping_c == 8) {
  //     setStepping(stepping_c);
  //   } else {
  //     setStepping(8);
  //   }
  // }

  /**
   * Accepts 1, 2, 4, or 8 to represent full, half, quarter, and eighth
   * stepping and updates motor pins to reflect the change. If any other
   * value is provided, nothing happens.
   *
   * @param stepping The stepping level
   */
  void setStepping(uint16_t stepping) volatile {
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
   * Sets the direction of the stepper to either clockwise or
   * counter-clockwise
   *
   * @param Use the CW or CCW symbols to define direction
   */
   void setDirection(bool dir) volatile {
     digitalWriteFast(PIN_DIR, dir);
   }

   /**
    * Sets the stepper at the desired rotational frequency.
    * Input is in hz, positive values are CW and negative values
    * CCW
    *
    * @param freq The desired angular velocity
    */
    void setSpeed(float freq) volatile {
      freq = STEPS_PER_REV * freq;
      clk.set(floor(0.5 + abs(freq)));
      if (freq < 0) {
        setDirection(CCW);
      } else {
        setDirection(CW);
      }
    }
};

#endif
