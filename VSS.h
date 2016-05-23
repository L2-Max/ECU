#ifndef __VSS_H__
#define __VSS_H__

#include <Arduino.h>

#define PIN_VSS 3

struct Pulses
{
  Pulses()
  {
    _usecs = 0;
    _count = 0;
  }
  
  volatile uint32_t _usecs;
  volatile uint32_t _count;
};

struct VSS
{
  VSS();
  ~VSS();

  void read();
  void read_Speed( unsigned long aNow_MS );

  volatile bool _slot_pulses;

  uint32_t _pulses_total;

  Pulses _pulses[ 2 ];
  
  bool _is_On;
  
  float _meters;
  uint32_t _speed;

  uint32_t _next_sample_ms;
};

#endif
