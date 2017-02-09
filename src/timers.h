#ifndef CONTROL_TIMERS
#define CONTROL_TIMERS

#include <kinetis.h>

class ControlTimer {
public:
  static volatile char ready;

  static void begin(uint32_t micros) {
    // Open the PIT clock gate
    SIM_SCGC6 |= SIM_SCGC6_PIT;

    // Enable PIT
    PIT_MCR = PIT_MCR_FRZ;

    // Setup timer 1
    KINETISK_PIT_CHANNEL_t *channel;
    channel = KINETISK_PIT_CHANNELS + 1;
    channel->LDVAL = (F_BUS / 1000000) * micros - 1;
    channel->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

    // Turn on timer 1
    NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  }

  static void stop() {
    KINETISK_PIT_CHANNEL_t *channel;
    channel = KINETISK_PIT_CHANNELS + 1;

    // Disable timer interrupts and the timer itself
    channel->TCTRL = 0;
  }

  static char isReady() {
    if (ready) {
      ready = 0;
      return 1;
    }
    PIT_TCTRL1 = 3;

    return 0;
  }
};

// The control timer starts un-ready and is set to ready by the first timer interrupt
volatile char ControlTimer::ready = 0;
/**
 * The ISR sets the ready variable to true and resets the interrupt flag
 */
void pit1_isr() {
  PIT_TFLG1 = 1;
  ControlTimer::ready = 1;
}

#endif
