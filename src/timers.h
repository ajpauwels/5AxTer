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

  /**
   * Constructor simply records which PIT this timer will unsigned*
   *
   * @param channelNr_c The PIT timer to use (0-3)
   */
  Timer(unsigned short int channelNr_c) : channelNr(channelNr_c) {}

  /**
   * Begins the timer t0 run at the specified frequencly and call
   * the given ISR
   *
   * @param timerISR_c The ISR to run
   * @param hz The frequency to run at in hertz
   */
  void begin(ISR timerISR_c, uint32_t hz) {
    // Open the PIT clock gate
    SIM_SCGC6 |= SIM_SCGC6_PIT;

    // Enable PIT
    PIT_MCR = PIT_MCR_FRZ;

    // Setup the specified PIT by setting its counter and control values
    KINETISK_PIT_CHANNEL_t* channel;
    channel = KINETISK_PIT_CHANNELS + channelNr;
    channel->LDVAL = (F_BUS / hz) - 1;
    channel->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

    // Register the provided ISR in the ISR array
    PIT_ISRS[channelNr] = timerISR_c;

    // Turn on interrupts for the specified timer
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + channelNr);
  }

  /**
   * Disables the timer and its interrupt capabilities. This function clears
   * the timer's registers, interrupts, ISR, and ready status. To resume,
   * begin() must be called with a new frequency and ISR
   */
  void stop() {
    // Disable timer interrupts and the timer itself
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0 + channelNr);

    KINETISK_PIT_CHANNEL_t *channel;
    channel = KINETISK_PIT_CHANNELS + channelNr;

    channel->TCTRL = 0;
    Timer::READY_VARS[channelNr] = 0;
    Timer::PIT_ISRS[channelNr] = nullptr;
  }

  /**
   * Returns true if the timer has gone off since the last time this function
   * was called
   *
   * @return 1 if timer has gone off, false otherwise
   */
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
