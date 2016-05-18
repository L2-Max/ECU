#include "Injector.h"

#include "ECU.h"

#include <Arduino.h>

#define INJECTOR_STATE_THRESHOLD 1
#define INJECTOR_STATE_JITTER 1

Injector::Injector( ECU& anEcu ) :
   _ecu( anEcu ), _is_On( false ), _slot_periods( false ), _slot_periods_on( false ), _state( 0 ),
   _interrupted( 0 ), _periods_count( 0 ), _periods_on_count( 0 )
{
  memset( _periods, 0, sizeof( _periods ) );
  memset( _periods_on, 0, sizeof( _periods_on ) );
  
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
      ++_periods_count;
      
      _periods[ _slot_periods ]._usecs = micros();
      _periods[ _slot_periods ]._count = _periods_count;

      _interrupted = 1;
    }
    else
    {
      ++_periods_on_count;
      
      _periods_on[ _slot_periods_on ]._usecs += ( micros() - _periods[ _slot_periods ]._usecs );
      _periods_on[ _slot_periods_on ]._count = _periods_on_count;
    }

    _is_On = !_is_On;
  }
}

const Periods& Injector::read_periods()
{
  static Periods ret;

  if( _interrupted )
  {
    Periods theLastPeriods( _periods[ !_slot_periods ] );
    
    _slot_periods = !_slot_periods;

    _interrupted = 0;
    
    if( theLastPeriods._usecs )
    {
       ret._usecs = ( _periods[ !_slot_periods ]._usecs - theLastPeriods._usecs );
       ret._count = ( _periods[ !_slot_periods ]._count - theLastPeriods._count );
    }
  }
  else
  {
     ret._usecs = 0;
     ret._count = 0;
  }

  return ret;//read_periods( _periods[ !_slot_periods ], thePeriods );
}

const Periods& Injector::read_periods_on()
{
  Periods thePeriods( _periods_on[ !_slot_periods_on ] );
  
  _slot_periods_on = !_slot_periods_on;
  
  return read_periods( _periods_on[ !_slot_periods_on ], thePeriods );
}

const Periods& Injector::read_periods( Periods& aPeriods, Periods& aLast )
{
  static Periods ret;
  
  if( aLast._usecs )
  {
     ret._usecs = ( aPeriods._usecs - aLast._usecs );
     ret._count = ( aPeriods._count - aLast._count );
  }
  else
  {
    ret._usecs = 0;
    ret._count = 0;
  }

  return ret;
}

