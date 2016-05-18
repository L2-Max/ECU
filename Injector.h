#ifndef __INJECTOR_H__
#define __INJECTOR_H__

#define PIN_INJECTOR 2

struct ECU;

struct Periods
{
  Periods()
  {
    _count = 0;
  }
  
  volatile unsigned long _count;
  volatile unsigned long _usecs;
};

struct Injector
{
  Injector( ECU& anEcu );

  void read();

  void read_periods( Periods& aPeriods );
  void read_periods_on( Periods& aPeriods );

  bool _is_On;

  volatile bool _slot_periods;
  volatile bool _slot_periods_on;

  unsigned long _periods_count;
  unsigned long _periods_on_count;
  unsigned long _periods_on_usecs;

  Periods _periods[ 2 ];
  Periods _periods_on[ 2 ];
  
  char _state;

  ECU& _ecu;
};

#endif

