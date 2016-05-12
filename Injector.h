#ifndef __INJECTOR_H__
#define __INJECTOR_H__

#define PIN_INJECTOR 2

struct ECU;

struct Periods
{
  volatile unsigned long _count;
  volatile unsigned long _usecs;
};

struct Injector
{
  Injector( ECU& anEcu );

  void read();

  Periods read_periods();
  Periods read_periods_on();

  bool _is_On;

  volatile bool _slot_periods;
  volatile bool _slot_periods_on;

  Periods _private_periods;
  
  Periods _periods[ 2 ];
  Periods _periods_on[ 2 ];

  Periods _last_periods;
  Periods _last_periods_on;
  
  char _state;

  ECU& _ecu;
};

#endif

