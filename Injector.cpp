#include "Injector.h"

#include "ECU.h"

#include <Arduino.h>

#define INJECTOR_STATE_THRESHOLD 1
#define INJECTOR_STATE_JITTER 1

Injector::Injector( ECU& anEcu ) :
   _ecu( anEcu ), _is_On( false ), _slot_periods( false ), _slot_periods_on( false ), _state( 0 )
{
  memset( &_private_periods, 0, sizeof( _private_periods ) );
  memset( &_private_periods_on, 0, sizeof( _private_periods_on ) );
  
  memset( _periods, 0, sizeof( _periods ) );
  memset( _periods_on, 0, sizeof( _periods_on ) );
  
  memset( &_last_periods, 0, sizeof( _last_periods ) );
  memset( &_last_periods_on, 0, sizeof( _last_periods_on ) );
  
  pinMode( PIN_INJECTOR, INPUT_PULLUP );
}

void Injector::read()
{
  int theState( digitalRead( PIN_INJECTOR ) );
  
  if( theState == LOW )
  {
    //if( ++_state > INJECTOR_STATE_THRESHOLD )
    {
      _state = INJECTOR_STATE_THRESHOLD;
    }
  }
  else if( theState == HIGH )
  {
    //if( --_state < -INJECTOR_STATE_THRESHOLD )
    {
      _state = -INJECTOR_STATE_THRESHOLD;
    }
  }
  
  if( ( ( _state >= INJECTOR_STATE_JITTER ) && !_is_On ) || ( ( _state <= -INJECTOR_STATE_JITTER ) && _is_On ) )
  {
    if( !_is_On )
    {
      _private_periods._usecs = micros();
      ++_private_periods._count;

      _periods[ _slot_periods ] = _private_periods;
    }
    else
    {
      _private_periods_on._usecs += ( micros() - _periods[ _slot_periods ]._usecs );
      ++_private_periods_on._count;
      
      _periods_on[ _slot_periods_on ] = _private_periods_on;
    }

    _is_On = !_is_On;
  }
}

const Periods& Injector::read_periods()
{
  _slot_periods = !_slot_periods;
  
  return read_periods( _periods[ !_slot_periods ], _last_periods );
}

const Periods& Injector::read_periods_on()
{
  _slot_periods_on = !_slot_periods_on;
  
  return read_periods( _periods_on[ !_slot_periods_on ], _last_periods_on );
}

const Periods& Injector::read_periods( Periods& aPeriods, Periods& aLast )
{
  static Periods ret;
  
  if( aPeriods._count && aPeriods._usecs )
  {
     ret._usecs = ( aPeriods._usecs - aLast._usecs );
     ret._count = ( aPeriods._count - aLast._count );
  }
  else
  {
    ret._usecs = 0;
    ret._count = 0;
  }

  aLast = aPeriods;

  aPeriods._count = 0;
  aPeriods._usecs = 0;

  return ret;
}

