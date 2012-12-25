#include <avr/io.h>
#include <avr/interrupt.h>

#include "NexaRX.h"

// // radio:
// #define NEXA_PORT PINC
// #define NEXA_DDR DDRC
// #define NEXA_MASK (1 << 4)
// #define NEXA_PCINT_MSK PCMSK1
// #define NEXA_PCINT PCINT12
// #define NEXA_PCICR_MASK (1 << PCIE1)

// sw1:
// #define NEXA_PORT PIND
// #define NEXA_DDR DDRD
// #define NEXA_MASK (1 << 6)
// #define NEXA_PCINT_MSK PCMSK2
// #define NEXA_PCINT PCINT22
// #define NEXA_PCICR_MASK (1 << PCIE2)

// debug2:
#define NEXA_PORT PIND
#define NEXA_DDR DDRD
#define NEXA_MASK (1 << 5)
#define NEXA_PCINT_MSK PCMSK2
#define NEXA_PCINT PCINT21
#define NEXA_PCICR_MASK (1 << PCIE2)

#define NEXA_READ !!(NEXA_PORT & NEXA_MASK)

#define DEBUG0_PORT PORTB
#define DEBUG0_DDR DDRB
#define DEBUG0_IDX (0)
#define DEBUG0_MASK (1 << DEBUG0_IDX)

#define DEBUG1_PORT PORTD
#define DEBUG1_DDR DDRD
#define DEBUG1_IDX (4)
#define DEBUG1_MASK (1 << DEBUG1_IDX)

// #define DEBUG2_PORT PORTD
// #define DEBUG2_DDR DDRD
// #define DEBUG2_IDX (5)
// #define DEBUG2_MASK (1 << DEBUG2_IDX)

#define DEBUG(x, y) do { DEBUG##x##_PORT = (DEBUG##x##_PORT & ~DEBUG##x##_MASK) | ((y) << DEBUG##x##_IDX); } while (0)

static bool debug_value = 0;

#define DEBUGFLIP(x) DEBUG(x, (debug_value ^= 1))

static const int prescaler = 1;
static const double systemHz = 16000000;
static const double bitLength = 350.0 / 1000000.0;
static const unsigned int finetuning = 47; // fine-tuned with an oscilloscope
static const unsigned int timerPreload = 65536 - systemHz / prescaler / (1 / bitLength) + finetuning;
static const unsigned int timerPreload2 = 65536 - systemHz / prescaler / (1 / (bitLength / 2)) + finetuning;

NexaRXInstance NexaRX;

enum SeqState {
  SS_ANYTHING, // wait for some signal
  SS_OTHER, // wait for the first value to be one
  SS_STOPPING // wait for the stop sequence to finish
};

static SeqState state;
static unsigned long bits;
static unsigned char num_bits;
static unsigned char code_bits; // code bits received, most recent one is bit #0
static unsigned char code_state; // number of elements of code received

static volatile int messagebox1;
static volatile int messagebox2;

static volatile int bits_ever = 0;

extern void debug(unsigned);

static
void
setup_timer()
{
  // initialize timer1 
  cli();
  TCCR1A = 0;
  TCCR1B = 0;

//  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TCCR1B |= (1 << CS10);    // no prescaler 
//  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  sei();
}

static
void
start_timer()
{
  // first sample is retrieved at cycle length/2
  TCNT1 = timerPreload2;
  TIMSK1 |= (1 << TOIE1);
  TIFR1 |= (1 << TOV1);
}

static
void
stop_timer()
{
  TIMSK1 &= ~(1 << TOIE1);
}

static
void
start_capture()
{
  // disable IO interrupt
  NEXA_PCINT_MSK &= ~(1 << NEXA_PCINT);
  PCICR &= ~NEXA_PCICR_MASK;
  state = SS_OTHER;
  start_timer();
}

static
void
next_capture()
{
  stop_timer();
  state = SS_ANYTHING;
  bits = 0;
  num_bits = 0;
  code_state = 0;
  code_bits = 0;

  // enable IO interrupt
  NEXA_PCINT_MSK |= (1 << NEXA_PCINT);
  PCICR |= NEXA_PCICR_MASK;

  DEBUG(0, 0);
}

void
NexaRXInstance::setup()
{
  // input pin
  NEXA_DDR &= ~NEXA_MASK;

  debug(timerPreload);
  debug(timerPreload2);

  cli();
  // enable Port Change interrupt
  PCICR |= NEXA_PCICR_MASK;
  next_capture();
  NEXA_PCINT_MSK |= (1 << NEXA_PCINT);

  DEBUG0_DDR |= DEBUG0_MASK;
  DEBUG1_DDR |= DEBUG1_MASK;
  // DEBUG2_DDR |= DEBUG2_MASK;

  setup_timer();
  sei();
}

ISR(PCINT2_vect)
{
  static int debug = 0;
  //++bits_ever;
  if (NEXA_READ) {
    debug ^= 1;
    messagebox1 = 42;
    start_capture();
    debug ^= 1;
  }
}

void
handle_message(unsigned long)
{
  messagebox1 = 10;
}

ISR(TIMER1_OVF_vect)
{
  cli();
  TCNT1 = timerPreload;
  DEBUGFLIP(0);
  bool value = NEXA_READ;
  DEBUGFLIP(0);
  //++bits_ever;
  switch (state) {
  case SS_ANYTHING: {
    if (!value) { // we expected high value, but didn't get it, so bail out
      messagebox1 = 1;
      next_capture();
    }
    state = SS_OTHER;
  } // fall through
  case SS_OTHER: {
    code_bits = (code_bits << 1) | value;
    ++code_state;
  } break;
  case SS_STOPPING: {
    ++code_state;
    if (value) {
      messagebox1 = 2;
      // something detected during stop
      DEBUGFLIP(0);
      next_capture();
      DEBUGFLIP(0);
    }
    if (code_state == 32) {
      handle_message(bits);
      DEBUGFLIP(0);
      next_capture();
      DEBUGFLIP(0);
    }
  } break;
  }
  if (state == SS_OTHER && code_state == 8) {
    code_state = 0;
    switch (code_bits) {
    case 0210: { // 10 001 000
      bits = (bits << 1) | 0;
      ++num_bits;
    } break;
    case 0356: { // 11 101 110
      // 1: unsupported
      messagebox1 = 3;
      //DEBUGFLIP(0);
      next_capture();
      //DEBUGFLIP(0);
    } break;
    case 0216: { // 10 001 110
      // X: open. interpreted as 1.
      bits = (bits << 1) | 1;
      ++num_bits;
    } break;
    case 0200: { // 10 000 000
      if (num_bits == 12) {     // exact length of expected message
        state = SS_STOPPING;
        code_state = 8;         // we have received 8 bits of the stop code
      } else {
        messagebox1 = 4;
        //DEBUGFLIP(0);
        next_capture();
        //DEBUGFLIP(0);
      }
    } break;
    default: {
      ++bits_ever;
      messagebox1 = 5;
      messagebox2 = code_bits;
      //DEBUGFLIP(0);
      next_capture();
      //DEBUGFLIP(0);
    }
    }
  }
  sei();
}


bool
NexaRXInstance::getMessage(int& house, int& device, bool& state)
{
  static int prev_bits = 0;
  if (bits_ever != prev_bits) {
    cli();
    house = bits_ever;
    device = messagebox2;
    messagebox1 = 0;
    messagebox2 = 0;
    prev_bits = bits_ever;
    sei();
    state = false;
    return true;
  }
  return false;
}

void
NexaRXInstance::loop()
{
  //DEBUG(0, NEXA_READ);
}
