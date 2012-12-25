// -*- mode: c++ -*-

#ifndef NEXARX_H
#define NEXARX_H

class NexaRXInstance {
public:
  NexaRXInstance() {}

  static void setup();
  static bool getMessage(int& house, int& device, bool& state);
  static void loop();
};

extern NexaRXInstance NexaRX;

#endif // NEXARX_H
