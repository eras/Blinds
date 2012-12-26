#include "NexaRX.h"
#include "EEPROM.h"

// -*- mode: c++ -*-
int motor1_pins[] = { 4, 3, 2, 1 };
const int num_pins = sizeof(motor1_pins) / sizeof(*motor1_pins);
long unsigned step_interval = 2000;
const bool powersave = true;
const int full_revolution = 509;

#define MOTOR1_PORT PORTB
#define MOTOR1_DDR DDRB
//#define MOTOR1_SET(VALUE) do { MOTOR1_PORT = (MOTOR1_PORT & (~MOTOR1_MASK)) | (VALUE); } while (0)
#define MOTOR1_SET(VALUE) do { MOTOR1_PORT = (VALUE); } while (0)

const int green_led_pin = 3;
const int red_led_pin = 2;
const int sw1_pin = 6;
const int sw2_pin = 7;

#define SEQUENCE 2

#define VERSION "1.0"

const int closed_cycles = 260;
const int closed2_cycles = -260;
const int open_cycles = 0;
const int overshoot_cycles = 50;
const int small_turn = full_revolution / 20;

unsigned int green_led_ms_left = 0; // how many milliseconds to keep the green led on?

bool powered = false;

int address_house;
int address_device;

bool learning_address;

void swap(int& a, int& b)
{
  int tmp = a;
  a = b;
  b = tmp;
}

void reverse(int* first, int* last)
{
  --last;
  while (first < last) {
    swap(*first, *last);
    ++first;
    --last;
  }
}


// from http://wordaligned.org/articles/next-permutation which lifted
// it off some implementation
bool next_permutation(int* first, int* last)
{
    if (first == last)
        return false;
    int* i = first;
    ++i;
    if (i == last)
        return false;
    i = last;
    --i;
        
    for(;;)
    {
        int* ii = i;
        --i;
        if (*i < *ii)
        {
            int* j = last;
            while (!(*i < *--j))
            {}
            swap(*i, *j);
            reverse(ii, last);
            return true;
        }
        if (i == first)
        {
            reverse(first, last);
            return false;
        }
    }
}


const int sequence[] = { 
#if (SEQUENCE==1)
//  1, 3, 2, 4 
//  001,
//  004,
//  002,
//  010

  001,
  002,
  004,
  010

#elif (SEQUENCE==2)
// 1+2, 2+3, 3+4, 4+1
   003,
   006,
   014,
   011

#elif (SEQUENCE==3)
// 1, 1+2, 2, 2+3, 3, 3+4, 4, 4+1
   001,
   003,
   002,
   006,
   004,
   014,
   010,
   011
#endif // SEQUENCE


};

const int sequence_xor = 0x00;

const int len_sequence = sizeof(sequence) / sizeof(*sequence);

unsigned char motor1_bits[len_sequence];

int at_seq = 0;

void reset_pins()
{
  powered = false;
  MOTOR1_SET(0);
}


void setup_motor1()
{
  unsigned ddr = 0u;
  for (int pin = 0; pin < num_pins; ++pin) {
    ddr |= (1u << motor1_pins[pin]);
  }
  MOTOR1_DDR |= ddr;
  for (int at = 0; at < len_sequence; ++at) {
    unsigned char value = 0;
    int state = sequence[at];
    for (int pin = 0; pin < num_pins; ++pin) {
      if (state & (1u << pin)) {
        value |= (1u << motor1_pins[pin]);
      }
    }
    motor1_bits[at] = value;
  }
}

void save_address()
{
  EEPROM.write(0, address_house);
  EEPROM.write(1, address_device);
  EEPROM.write(2, address_device ^ address_house ^ 0x42);
}

bool load_address()
{
  address_house = EEPROM.read(0);
  address_device = EEPROM.read(1);
  return EEPROM.read(2) == address_house ^ address_device ^ 0x42;
}                    

void setup() {
  Serial.begin(115200); 
  Serial.println("START " VERSION);
  pinMode(green_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(sw1_pin, INPUT);
  pinMode(sw2_pin, INPUT);

  NexaRX.setup();

  setup_motor1();
  reset_pins();

  while (digitalRead(sw1_pin) || digitalRead(sw2_pin)) {
    digitalWrite(green_led_pin, HIGH);
    learning_address = true;
    delay(100);
  }
  if (learning_address) {
    delay(1000);
  }
  // led turned off in main loop

  if (!learning_address) {
    if (!load_address()) {
      learning_address = true;
    }
  }
}

void step_delay()
{
  if (step_interval > 30000) {
    delay(step_interval >> 10);
  } else {
    delayMicroseconds(step_interval);
  }
}

void turn(int direction)
{
  if (!powered) {
    MOTOR1_SET(motor1_bits[at_seq]);
    step_delay();
    powered = true;
  }
  for (int c = 0; c < len_sequence; ++c) {
    //Serial.print(".");
    //digitalWrite(led, at_seq == 0 ? HIGH : LOW);
    if (direction) {
      ++at_seq;
      if (at_seq == len_sequence) {
        at_seq = 0;
      }
    } else {
      if (at_seq == 0) {
        at_seq = len_sequence - 1;
      } else {
        --at_seq;
      }
    }
    MOTOR1_SET(motor1_bits[at_seq]);
    step_delay();
  }
}

long int turning = 0; // orientation we want to be in relative to current
long int orientation = 0;
enum overshoot_state {
  OVERSHOOT_UNCONFIGURED,
  OVERSHOOT_NONE,
  OVERSHOOT_READY,
  OVERSHOOT_SHOOTING,
  OVERSHOOT_RETURNING
} overshoot = OVERSHOOT_UNCONFIGURED;

void next_pin_order()
{
  next_permutation(motor1_pins, motor1_pins + num_pins);
  setup_motor1();
}

void dump_pin_order()
{
  Serial.print("Pin order:");
  for (int c = 0; c < num_pins; ++c) {
    Serial.print(' ');
    Serial.print(motor1_pins[c]);
  }
  Serial.println();
}

bool open()
{
  if (orientation != open_cycles) {
    turning = open_cycles - orientation;
    overshoot = OVERSHOOT_READY;
    Serial.println("o");
    return true;
  } else {
    return false;
  }
}

void reset_orientation()
{
  reset_pins();
  overshoot = OVERSHOOT_NONE;
  turning = 0;
  orientation = 0;
  Serial.print("O ");
  Serial.print(orientation);
  Serial.println();
}

bool close()
{
  if (orientation < closed_cycles) {
    turning = closed_cycles - orientation;
    overshoot = OVERSHOOT_READY;
    Serial.println("c");
    return true;
  } else {
    return false;
  }
}

unsigned delta_millis()
{
  static unsigned int prev_millis = 0;
  unsigned int cur_millis = millis();
  unsigned int delta = cur_millis - prev_millis;
  prev_millis = cur_millis;
  return delta;
}

void maintain_green_led(unsigned delta)
{
  if (green_led_ms_left > 0) {
    unsigned prev_green_led_ms_left = green_led_ms_left;
    green_led_ms_left -= delta;
    if (green_led_ms_left > prev_green_led_ms_left) {
      green_led_ms_left = 0;
    }
  }

  digitalWrite(green_led_pin, green_led_ms_left > 0 ? HIGH : LOW);
}

void control_loop()
{
  unsigned delta = delta_millis();

  maintain_green_led(delta);
  digitalWrite(red_led_pin, (overshoot == OVERSHOOT_UNCONFIGURED && ((millis() >> 10) & 1)) ? HIGH : LOW);

  if (turning != 0) {
    long int old_turning = turning;
    turn(turning > 0);
    if (turning > 0) {
      --turning;
      ++orientation;
    }
    if (turning < 0) {
      ++turning;
      --orientation;
    }
    if (turning == 0) {
      // old_turning == -1 or 1
      switch (overshoot) {
      case OVERSHOOT_UNCONFIGURED: break;
      case OVERSHOOT_NONE:
      case OVERSHOOT_RETURNING:
        if (powersave) {
          reset_pins();
        }
        overshoot = OVERSHOOT_NONE;
        break;
      case OVERSHOOT_READY:
        turning = old_turning * overshoot_cycles;
        overshoot = OVERSHOOT_SHOOTING;
        break;
      case OVERSHOOT_SHOOTING:
        turning = -old_turning * overshoot_cycles;
        overshoot = OVERSHOOT_RETURNING;
        break;
      }
    }
  }
  {
    int house, device;
    bool state;
    if (NexaRX.getMessage(house, device, state) && overshoot == OVERSHOOT_NONE) {
      if (house == address_house && device == address_device) {
        green_led_ms_left = 200;
        if (state) {
          open();
        } else {
          close();
        }
      }
    }
  }
  if (overshoot == OVERSHOOT_NONE) {
    if (digitalRead(sw1_pin)) {
      close();
    } else if (digitalRead(sw2_pin)) {
      open();
    }
  } else if (overshoot == OVERSHOOT_UNCONFIGURED) {
    bool sw1 = digitalRead(sw1_pin);
    bool sw2 = digitalRead(sw2_pin);
    if (sw1 && sw2) {
      digitalWrite(green_led_pin, HIGH);
      reset_orientation();
      while (digitalRead(sw1_pin) && digitalRead(sw2_pin)) {
        delay(100);
      }
      delay(1000);
    } else { 
      if (turning == 0) {
        if (sw1) {
          turning += small_turn;
        }
        if (sw2) {
          turning -= small_turn;
        }
      }
    }
  }
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == ' ') {
      Serial.print("sw1 ");
      Serial.print(digitalRead(sw1_pin));
      Serial.print(" sw2 ");
      Serial.print(digitalRead(sw2_pin));
      Serial.println();
    } else if (ch == '>') {
      turning += full_revolution;
      Serial.println(">");
    } else if (ch == '<') {
      turning += -full_revolution;
      Serial.println("<");
    } else if (ch == ',') {
      turning += small_turn;
      Serial.println(",");
    } else if (ch == '.') {
      turning += -small_turn;
      Serial.println(".");
    } else if (ch == 'c') {
      if (overshoot != OVERSHOOT_UNCONFIGURED) {
        close();
      }
    } else if (ch == 'C') {
      if (overshoot != OVERSHOOT_UNCONFIGURED && orientation > closed2_cycles) {
        turning = closed2_cycles - orientation;
        overshoot = OVERSHOOT_READY;
        Serial.println("C");
      }
    } else if (ch == 'o') {
      if (overshoot != OVERSHOOT_UNCONFIGURED) {
        open();
      }
    } else if (ch == '0') {
      reset_orientation();
    } else if (ch == '?') {
      Serial.print("O ");
      Serial.print(orientation);
      Serial.println();
    } else if (ch == 'N') {
      next_pin_order();
      dump_pin_order();
      turning = full_revolution;
      orientation = 0;
    } else if (ch == 'n') {
      dump_pin_order();
    } else if (ch == '+') {
      step_interval += 100;
      Serial.print("Step interval: ");
      Serial.println(step_interval);
    } else if (ch == '-') {
      step_interval -= 100;
      if (step_interval < 100) {
        step_interval = 100;
      }
      Serial.print("Step interval: ");
      Serial.println(step_interval);
    } else if (ch == 'd') {
      for (int c = 0; c < len_sequence; ++c) {
        Serial.println((unsigned int) motor1_bits[c]);
      }
    }
  }
}

void learning_loop()
{
  static bool has_candidate = false;
  unsigned delta = delta_millis();
  maintain_green_led(delta);
  digitalWrite(red_led_pin, (has_candidate || ((millis() >> 8) & 1)) ? HIGH : LOW);
  if (!has_candidate) {
    int house, device;
    bool state;
    if (NexaRX.getMessage(house, device, state) && state) {
      address_house = house;
      address_device = device;
      has_candidate = true;
    }
  } else {
    int house, device;
    bool state;
    if (NexaRX.getMessage(house, device, state) &&
        house == address_house && device == address_device) {
      green_led_ms_left = 200;
    }
    if (digitalRead(sw1_pin) || digitalRead(sw2_pin)) {
      digitalWrite(green_led_pin, HIGH);
      while (digitalRead(sw1_pin) || digitalRead(sw2_pin)) {
        delay(100);
      }
      digitalWrite(green_led_pin, HIGH);
      delay(1000);
      save_address();
      learning_address = false;
    }
  }
}

void loop()
{
  if (learning_address) {
    learning_loop();
  } else {
    control_loop();
  }
}
