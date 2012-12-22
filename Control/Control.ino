// -*- mode: c++ -*-
int pins[] = { 9, 10, 11, 12 };
const int num_pins = sizeof(pins) / sizeof(*pins);
long unsigned step_interval = 200000;
const bool powersave = true;
const int full_revolution = 509;

const int sw1_pin = 6;
const int sw2_pin = 7;

#define SEQUENCE 2

#define VERSION "0.3"

const int closed_cycles = 260;
const int closed2_cycles = -260;
const int open_cycles = 0;
const int overshoot_cycles = 50;
const int small_turn = full_revolution / 20;

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

const int led = 13;

int at_seq = 0;
int cur_state = 0;

void reset_pins()
{
  for (int c = 0; c < num_pins; ++c) {
    digitalWrite(pins[c], LOW);
  }
}


void setup() {
  Serial.begin(115200); 
  Serial.println("START " VERSION);
  // set the digital pin as output:
  for (int c = 0; c < num_pins; ++c) {
    pinMode(pins[c], OUTPUT);
  }
  pinMode(led, OUTPUT);
  pinMode(sw1_pin, INPUT);
  pinMode(sw2_pin, INPUT);
  reset_pins();
}

void turn(int direction)
{
  for (int c = 0; c < len_sequence; ++c) {
    //Serial.print(".");
    digitalWrite(led, at_seq == 0 ? HIGH : LOW);
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
    int state = sequence[at_seq] ^ sequence_xor;
    int changed = state ^ cur_state;
    for (int c = 0; c < num_pins; ++c) {
      if (changed & (1 << c)) {
        int value = state & (1 << c) ? HIGH : LOW;
        digitalWrite(pins[c], value);
      }
    }
    cur_state = state;
    if (step_interval > 30000) {
      delay(step_interval >> 10);
    } else {
      delayMicroseconds(step_interval);
    }
  }
}

long int turning = 0; // orientation we want to be in relative to current
long int orientation = 0;
enum overshoot_state {
  OVERSHOOT_NONE,
  OVERSHOOT_READY,
  OVERSHOOT_SHOOTING,
  OVERSHOOT_RETURNING
} overshoot = OVERSHOOT_NONE;

void next_pin_order()
{
  next_permutation(pins, pins + num_pins);
}

void dump_pin_order()
{
  Serial.print("Pin order:");
  for (int c = 0; c < num_pins; ++c) {
    Serial.print(' ');
    Serial.print(pins[c]);
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

void loop()
{
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
        case OVERSHOOT_NONE:
        case OVERSHOOT_RETURNING:
          if (powersave) {
            reset_pins();
          }
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
  if (turning == 0) {
    if (digitalRead(sw1_pin)) {
      if (!open()) {
        close();
      }
    } else if (digitalRead(sw2_pin)) {
      open();
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
      close();
    } else if (ch == 'C') {
      if (orientation > closed2_cycles) {
        turning = closed2_cycles - orientation;
        overshoot = OVERSHOOT_READY;
        Serial.println("C");
      }
    } else if (ch == 'o') {
      open();
    } else if (ch == '0') {
      reset_pins();
      overshoot = OVERSHOOT_NONE;
      turning = 0;
      orientation = 0;
      Serial.print("O ");
      Serial.print(orientation);
      Serial.println();
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
    }
  }
}

