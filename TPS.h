#ifndef __TPS_H__
#define __TPS_H__

#include "Average.h"

struct ECU;

struct TPS
{
  TPS( ECU& anECU );
  ~TPS();

  void read( unsigned long aNow_MS );
  unsigned long read_Vcc();

  ECU& _ecu;

  bool _isOpen;

  short _value;
  short _value_last;
  short _value_closed;
  short _value_jitter_counter;

  unsigned long _next_sample_ms;
};

#endif
