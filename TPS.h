#ifndef __TPS_H__
#define __TPS_H__

#include "Average.h"

struct TPS
{
  TPS();

  void read( unsigned long aNow_MS );
  unsigned long read_Vcc();

  bool _isOpen;

  Average< unsigned short > _value_average;
  
  unsigned short _value;

  unsigned long _last_sample;
};

#endif
