#include <Arduino.h>

#include "timers.h"
#include "state_machine.h"

unsigned int pin = 12;

void setup() {
  // Set the LED pin mode
  pinMode(pin, OUTPUT);

  // Printer initializes to the WAIT state
  state = PIN_HIGH;

  // Start the control loop
  ControlTimer::begin(3000000);
}

void loop() {
  if (ControlTimer::isReady()) {
    switch (state) {
      case WAIT:
        break;
      case START:
        break;
      case STOP:
        break;
      case PIN_HIGH:
        digitalWrite(pin, HIGH);
        state = PIN_LOW;
        break;
      case PIN_LOW:
        digitalWrite(pin, LOW);
        state = PIN_HIGH;
        break;
    }
  }
}
