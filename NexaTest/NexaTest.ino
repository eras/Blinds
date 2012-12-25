// -*- mode: c++ -*-

#include "NexaRX.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup");
  NexaRX.setup();
}

void debug(unsigned x)
{
  Serial.print("debug: ");
  Serial.println(x);
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
