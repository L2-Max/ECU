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

  const Periods& read_periods();
  const Periods& read_periods_on();

  static const Periods& read_periods( Periods& aPeriods, Periods& aLast );

  bool _is_On;

  volatile unsigned char _interrupted;
  
  volatile bool _slot_periods;
  volatile bool _slot_periods_on;

  volatile unsigned long _periods_count;
  volatile unsigned long _periods_on_count;

  Periods _periods[ 2 ];
  Periods _periods_on[ 2 ];
  
  char _state;

  ECU& _ecu;
};

#endif

