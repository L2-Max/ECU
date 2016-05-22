#ifndef __VSS_H__
#define __VSS_H__

#include <Arduino.h>

#define PIN_VSS 3

struct VSS
{
  VSS();
  ~VSS();

  void read();

  uint32_t _pulses_h;
  uint32_t _pulses_l;

  uint32_t _meters;
};

#endif
