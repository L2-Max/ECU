#ifndef __TPS_H__
#define __TPS_H__

#include "Average.h"

#include <Arduino.h>

struct ECU;

struct TPS
{
  TPS( ECU& anECU );
  ~TPS();

  void read( unsigned long aNow_MS );
  unsigned long read_Vcc();

  ECU& _ecu;

  bool _isOpen;

  Average< short > _value_average;
  
  short _value_last;
  short _value_closed;
  short _value_jitter_counter;

  uint16_t _counter_close;
  uint16_t _counter_open;

  unsigned long _next_sample_ms;
};

#endif
