// -*- mode: c++ -*-

#include "NexaRX.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup");
  NexaRX.setup();
}

bool newline = true;

void debug(unsigned x)
{
  if (!newline) Serial.print(" ");
  Serial.print(x);
  newline = false;
}

void debug(const char* x)
{
  if (!newline) Serial.print(" ");
  Serial.print(x);
  newline = false;
}

void debug()
{
  Serial.println();
  newline = true;
}

void loop()
{
  int house;
  int device;
  bool state;
  if (NexaRX.getMessage(house, device, state)) {
    noInterrupts();
    Serial.print("Got message: ");
    Serial.print(house);
    Serial.print(".");
    Serial.print(device);
    Serial.print(" ");
    Serial.print(state ? "ON" : "OFF");
    Serial.println();
    interrupts();
  }
  NexaRX.loop();
}
