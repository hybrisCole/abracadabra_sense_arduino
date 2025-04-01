#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <Arduino.h>

class LEDManager {
private:
  int pin;
  bool state;

public:
  LEDManager(int ledPin) : pin(ledPin), state(false) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
  }

  void toggle() {
    state = !state;
    digitalWrite(pin, state);
  }

  void setOn() {
    state = true;
    digitalWrite(pin, state);
  }

  void setOff() {
    state = false;
    digitalWrite(pin, state);
  }
};

#endif