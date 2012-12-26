#include <avr/io.h>
#include <avr/interrupt.h>

#include "NexaRX.h"

//#define DEBUG 1

// // radio:
#define NEXA_PORT PINC
#define NEXA_DDR DDRC
#define NEXA_MASK (1 << 4)
#define NEXA_PCINT_MSK PCMSK1
#define NEXA_PCINT PCINT12
#define NEXA_PCICR_MASK (1 << PCIE1)
#define NEXA_INT_vect PCINT1_vect

// sw1:
// #define NEXA_PORT PIND
// #define NEXA_DDR DDRD
// #define NEXA_MASK (1 << 6)
// #define NEXA_PCINT_MSK PCMSK2
// #define NEXA_PCINT PCINT22
// #define NEXA_PCICR_MASK (1 << PCIE2)

// debug2:
// #define NEXA_PORT PIND
// #define NEXA_DDR DDRD
// #define NEXA_MASK (1 << 5)
// #define NEXA_PCINT_MSK PCMSK2
// #define NEXA_PCINT PCINT21
// #define NEXA_PCICR_MASK (1 << PCIE2)

#define NEXA_READ !!(NEXA_PORT & NEXA_MASK)

#ifdef DEBUG
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

#define DEBUGPIN(x, y) do { DEBUG##x##_PORT = (DEBUG##x##_PORT & ~DEBUG##x##_MASK) | ((y) << DEBUG##x##_IDX); } while (0)

static bool debug_value0;
static bool debug_value1;
static bool debug_count;

#define DEBUGFLIP(x) DEBUGPIN(x, (debug_value##x ^= 1))
#define DEBUGRESET(x) DEBUGPIN(x, (debug_value##x = 0))

struct DebugMessage {
  unsigned char idx;
  const char* msg;
  unsigned long value1, value2, value3, value4;
};
#else // #ifdef DEBUG
#define DEBUGPIN(x, y)
#define DEBUGFLIP(x)
#define DEBUGRESET(x)
#endif // #ifdef DEBUG #else

struct Code {
  unsigned char idx;
  unsigned long bits;
};

template <typename T, int MAX_MESSAGES>
struct MessageQueue {
  const unsigned    messages_max = MAX_MESSAGES;
  T                 messages[MAX_MESSAGES];
  unsigned          messages_wr;
  unsigned          messages_rd;
  volatile unsigned messages_count; // allows using all slots and accessing the data without cli/sei
  unsigned char     messages_sent;

  T* sendBegin()
  {
    ++messages_sent;
    if (messages_count == messages_max) {
      return 0;
    } else {
      T* m = &messages[messages_wr];
      m->idx = messages_sent;
      return m;
    }
  }
  void sendFinish()
  {
    if (++messages_wr == messages_max) {
      messages_wr = 0;
    }
    if (++messages_count > messages_max) {
      messages_count = messages_max;
    }
  }
  bool receive(T& msg)
  {
    if (messages_count) {
      cli();
      if (messages_count) {
        --messages_count;
        msg = messages[messages_rd];
        if (++messages_rd == messages_max) {
          messages_rd = 0;
        }
      }
      sei();
      return true;
    } else {
      return false;
    }
  }
};

#ifdef DEBUG
MessageQueue<DebugMessage, 50> debugs;
#endif // DEBUG
MessageQueue<Code, 10> codes;

static const int prescaler = 1;
static const double systemHz = 16000000;
static const double bitLength = 350.0 / 1000000.0;
static const unsigned int finetuning = 47 / prescaler; // fine-tuned with an oscilloscope
static const unsigned int timerCycles = systemHz / prescaler / (1 / bitLength);
static const unsigned int timerPreload = 65536 - timerCycles + finetuning;

// when (TCNT1 >= 65536 - timerAdjustThreshold) || (TCNT1 <=
// timerAdjustThreshold) in IO handler, we can fudge the timer offset
// a bit to accommodate for different clock rate
static const unsigned int timerAdjustThreshold = systemHz / prescaler / (1 / (bitLength / 6));

NexaRXInstance NexaRX;

enum SeqState {
  SS_NOT_CAPTURING,             // waiting for some signal
  SS_OTHER,                     // wait for the remaining symbols
  SS_STOPPING                   // wait for the stop sequence to finish
};

static SeqState state;
static unsigned long bits;
static unsigned char num_bits;
static unsigned char code_bits; // code bits received, most recent one is bit #0
static unsigned char code_len; // number of elements of code received

static unsigned int debug_count_io; // debug counter
static unsigned int debug_count_timer; // debug counter

static bool     previous_input_state; // what we know the current state to be, updated in IO interrupt
static unsigned previous_input_time; // when was this last input received
static unsigned input_ones, input_zeroes; // keep track of the number of oens and zeroes during sample interval
static bool     timer_adjusted; // did we just adjust the timer?

#ifdef DEBUG
extern void debug();
extern void debug(const char*);
extern void debug(unsigned);

// assumes interrupts are disabled
static void
debug_message_send(const char* msg, unsigned long value1 = 0, unsigned long value2 = 0, unsigned long value3 = 0, unsigned long value4 = 0)
{
  DebugMessage* m = debugs.sendBegin();
  if (!m) return;
  m->msg = msg;
  m->value1 = value1;
  m->value2 = value2;
  m->value3 = value3;
  m->value4 = value4;
  debugs.sendFinish();
}
#endif

//#define DEBUG_MESSAGE_SEND(x) debug_message_send x
#define DEBUG_MESSAGE_SEND(x)

static void
message_send(unsigned long bits)
{
  Code* m = codes.sendBegin();
  if (!m) return;
  m->bits = bits;
  codes.sendFinish();
}

static
void
setup_timer()
{
  // initialize timer1 
  TCCR1A = 0;
  TCCR1B = 0;

//  TCCR1B |= (1 << CS12);    // 256 prescaler 
  if (prescaler == 1) {
    TCCR1B |= (1 << CS10);    // no prescaler 
  } else if (prescaler == 8) {
    TCCR1B |= (1 << CS11);    // /8
  } else if (prescaler == 64) {
    TCCR1B |= (1 << CS10) | (1 << CS11);    // /64
  } else if (prescaler == 256) {
    TCCR1B |= (1 << CS12);    // /256
  } else if (prescaler == 1024) {
    TCCR1B |= (1 << CS12) | (1 << CS11);    // /1024
  }
//  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

static
void
setup_io()
{
  // enable IO interrupt
  NEXA_PCINT_MSK |= (1 << NEXA_PCINT);
  // enable Port Change interrupt
  PCICR |= NEXA_PCICR_MASK;

  // disable IO interrupt
  //NEXA_PCINT_MSK &= ~(1 << NEXA_PCINT);
  //PCICR &= ~NEXA_PCICR_MASK;
}

// start or restart timer
static
void
start_timer()
{
  TCNT1 = timerPreload;
  TIFR1 |= (1 << TOV1);
  TIMSK1 |= (1 << TOIE1);
}

static
void
setup_debug()
{
#ifdef DEBUG
  DEBUG0_DDR |= DEBUG0_MASK;
  DEBUG1_DDR |= DEBUG1_MASK;
  // DEBUG2_DDR |= DEBUG2_MASK;
#endif
}

static
void
stop_timer()
{
  TIMSK1 &= ~(1 << TOIE1);
}

static
void
reset_input()
{
  input_ones = 0;
  input_zeroes = 0;
}

static
void
reset_capture()
{
  DEBUG_MESSAGE_SEND(("rc"));
  state = SS_NOT_CAPTURING;
  bits = 0;
  num_bits = 0;
  code_len = 0;
  code_bits = 0;
  reset_input();
}

static
void
start_capture()
{
  DEBUG_MESSAGE_SEND(("sc", debug_count_io, debug_count_timer));
  start_timer();
  reset_capture();
  state = SS_OTHER;
  previous_input_time = timerPreload;
}

static
void
next_capture()
{
  DEBUG_MESSAGE_SEND(("nc"));
  stop_timer();
  reset_capture();

#ifdef DEBUG
  debug_count = 0;
  //DEBUGRESET(0);
  DEBUGRESET(1);
  DEBUGFLIP(0);
#endif // #ifdef DEBUG
}

void
NexaRXInstance::setup()
{
  // input pin
  NEXA_DDR &= ~NEXA_MASK;

#ifdef DEBUG
  debug(timerPreload);
  debug();
#endif

  cli();

  setup_debug();
  setup_io();
  setup_timer();

  sei();
}

static void
handle_message(unsigned long bits)
{
  message_send(bits);
}

static void
process_bit(bool value)
{
  switch (state) {
  case SS_OTHER: {
    code_bits = (code_bits << 1) | value;
    ++code_len;
  } break;
  case SS_STOPPING: {
    ++code_len;
    if (value) {
      DEBUG_MESSAGE_SEND(("pb", 0, code_len, value));
      // something detected during stop
      //DEBUGFLIP(0);
      next_capture();
      //DEBUGFLIP(0);
    }
    if (code_len == 32) {
      handle_message(bits);
      //DEBUGFLIP(0);
      next_capture();
      //DEBUGFLIP(0);
    }
  } break;
  }
}

static void
process_code()
{
  code_len = 0;
  switch (code_bits) {
  case 0210: { // 10 001 000
    bits <<= 1;
    ++num_bits;
    DEBUG_MESSAGE_SEND(("pc", code_bits, 0, bits, num_bits));
  } break;
  case 0356: { // 11 101 110
    // 1: unsupported
    DEBUG_MESSAGE_SEND(("0356"));
    //DEBUGFLIP(0);
    next_capture();
    //DEBUGFLIP(0);
  } break;
  case 0216: { // 10 001 110
    // X: open. interpreted as 1.
    bits = (bits << 1) | 1;
    ++num_bits;
    DEBUG_MESSAGE_SEND(("pc", code_bits, 1, bits, num_bits));
  } break;
  case 0200: { // 10 000 000
    if (num_bits == 12) {     // exact length of expected message
      DEBUG_MESSAGE_SEND(("s"));
      state = SS_STOPPING;
      code_len = 8;         // we have received 8 bits of the stop code
    } else {
      DEBUG_MESSAGE_SEND(("inob", num_bits, bits));
      next_capture();
    }
  } break;
  default: {
    DEBUG_MESSAGE_SEND(("ic", debug_count_io, debug_count_timer, code_bits));
    next_capture();
  }
  }
  code_bits = 0;
}

static
void
collect_input(unsigned delta)
{
  if (previous_input_state) {
    // DEBUG_MESSAGE_SEND(("collect:input_ones", delta, input_ones, input_zeroes));
    input_ones += delta;
  } else {
    // DEBUG_MESSAGE_SEND(("collect:input_zeroes", delta, input_ones, input_zeroes));
    input_zeroes += delta;
  }
}

static void
read_bit()
{
  //DEBUGFLIP(0);
  process_bit(input_ones > input_zeroes);
  DEBUG_MESSAGE_SEND(("rb", input_ones, input_zeroes, code_len, code_bits));
  reset_input();
  //DEBUGFLIP(0);

  if (state == SS_OTHER && code_len == 8) {
    process_code();
  }
}

ISR(NEXA_INT_vect)
{
  DEBUGFLIP(1);
  unsigned t = TCNT1;
  bool input = NEXA_READ;
  unsigned delta = t - previous_input_time;
  DEBUG_MESSAGE_SEND(("I2", input, t, previous_input_time, delta));
  previous_input_time = t;
  collect_input(delta);
#ifdef DEBUG
  ++debug_count_io;
  ++debug_count;
#endif
  previous_input_state = input;

  if (state == SS_NOT_CAPTURING) {
    start_capture();
  } else if (!timer_adjusted) {
    // let's skip the annoying(?) situation
    if (t >= timerPreload) {
      if (t >= 65536 - timerAdjustThreshold) {
        // ok, the end of the bit arrived a little bit early, perform
        // reading immediately
        // DEBUGFLIP(0);
        DEBUG_MESSAGE_SEND(("a1", t));
        read_bit();
        start_timer();
        previous_input_time = timerPreload;
        timer_adjusted = true;
      } else if (t <= timerPreload + timerAdjustThreshold) {
        // DEBUGFLIP(0);
        // DEBUGFLIP(1);
        // ok, the end of the previous bit arrived a little bit late, so
        // let's imagine the bit really started here
        DEBUG_MESSAGE_SEND(("a2", t));
        start_timer();
        previous_input_time = timerPreload;
        timer_adjusted = true;
      }
    }
  }
}

ISR(TIMER1_OVF_vect)
{
  ++debug_count_timer;
  TCNT1 -= timerCycles;
  DEBUGFLIP(0);
  unsigned delta = 65535 - previous_input_time;
  // move previous_input_time into current time coordinates
  previous_input_time = timerPreload;
  collect_input(delta);
  timer_adjusted = false;
  read_bit();
  //DEBUGFLIP(0);
}

bool
NexaRXInstance::getMessage(int& house, int& device, bool& state)
{
#ifdef DEBUG
  DebugMessage msg;
  if (debugs.receive(msg)) {
    debug(msg.idx);
    debug(msg.msg);
    debug(msg.value1);
    debug(msg.value2);
    debug(msg.value3);
    debug(msg.value4);
    debug();
  }
#endif
  Code code;
  if (codes.receive(code)) {
    house = code.bits >> 8;
    device = (code.bits >> 4) & 0x0f;
    state = code.bits & 1;
    return true;
  } else {
    return false;
  }
}

void
NexaRXInstance::loop()
{
  //DEBUGPIN(0, NEXA_READ);
}
