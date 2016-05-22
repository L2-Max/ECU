#ifndef __VSS_H__
#define __VSS_H__

#include <Arduino.h>

#define PIN_VSS 3

struct VSS
{
  VSS();
  ~VSS();

  void read();
  void read_Speed( unsigned long aNow_MS );

  uint32_t _pulses_total;

  volatile uint32_t _pulses[ 2 ];
  volatile bool _slot_pulses;
  
  float _meters;
  uint32_t _speed;

  uint32_t _next_sample_ms;
};

#endif
