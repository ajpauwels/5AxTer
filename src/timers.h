#ifndef CONTROL_TIMERS
#define CONTROL_TIMERS

#include <kinetis.h>

typedef void (*ISR)();

class Timer {
private:
  unsigned short int channelNr;

public:
  static volatile char READY_VARS[4];
  static volatile ISR PIT_ISRS[4];

  Timer(unsigned short int channelNr_c) : channelNr(channelNr_c) {}

  void begin(ISR timerISR_c, uint32_t hz) {
    // Open the PIT clock gate
    SIM_SCGC6 |= SIM_SCGC6_PIT;

    // Enable PIT
    PIT_MCR = PIT_MCR_FRZ;

    // Setup timer 1
    KINETISK_PIT_CHANNEL_t* channel;
    channel = KINETISK_PIT_CHANNELS + channelNr;
    channel->LDVAL = (F_BUS / hz) - 1;
    channel->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
    PIT_ISRS[channelNr] = timerISR_c;

    // Turn on timer 1
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + channelNr);
  }

  void stop() {
    // Disable timer interrupts and the timer itself
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0 + channelNr);

    KINETISK_PIT_CHANNEL_t *channel;
    channel = KINETISK_PIT_CHANNELS + channelNr;

    channel->TCTRL = 0;
    Timer::READY_VARS[channelNr] = 0;
    Timer::PIT_ISRS[channelNr] = nullptr;
  }

  char isReady() {
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0 + channelNr);
    char ready = Timer::READY_VARS[channelNr];
    if (ready) {
      Timer::READY_VARS[channelNr] = 0;
      NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + channelNr);
      return 1;
    }
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + channelNr);
    return 0;
  }
};

// The control timer starts un-ready and is set to ready by the first timer interrupt
// volatile char ControlTimer::ready = 0;
volatile char Timer::READY_VARS[] = { 0, 0, 0, 0 };
volatile ISR Timer::PIT_ISRS[];

/**
 * The ISR sets the ready variable to true and resets the interrupt flag
 */
 void pit0_isr() {
   PIT_TFLG0 = 1;
   Timer::READY_VARS[0] = 1;
   Timer::PIT_ISRS[0]();
 }

void pit1_isr() {
  PIT_TFLG1 = 1;
  Timer::READY_VARS[1] = 1;
  Timer::PIT_ISRS[1]();
}

void pit2_isr() {
  PIT_TFLG2 = 1;
  Timer::READY_VARS[2] = 1;
  Timer::PIT_ISRS[2]();
}

void pit3_isr() {
  PIT_TFLG1 = 1;
  Timer::READY_VARS[2] = 1;
  Timer::PIT_ISRS[2]();
}

#endif
