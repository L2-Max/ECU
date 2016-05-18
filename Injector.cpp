#include "Injector.h"

#include "ECU.h"

#include <Arduino.h>

#define INJECTOR_STATE_THRESHOLD 1
#define INJECTOR_STATE_JITTER 1

Injector::Injector( ECU& anEcu ) :
   _ecu( anEcu ), _is_On( false ), _slot_periods( false ), _slot_periods_on( false ), _state( 0 ),
   _periods_count( 0 ), _periods_on_count( 0 ), _periods_on_usecs( 0 )
{
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

      _periods_on_usecs = micros();
      
      _periods[ _slot_periods ]._usecs = _periods_on_usecs;
      _periods[ _slot_periods ]._count = _periods_count;
    }
    else
    {
      ++_periods_on_count;
      
      _periods_on[ _slot_periods_on ]._usecs += ( micros() - _periods_on_usecs );
      _periods_on[ _slot_periods_on ]._count = _periods_on_count;
    }

    _is_On = !_is_On;
  }
}

void Injector::read_periods( Periods& aPeriods )
{
  if( _periods[ _slot_periods ]._count )
  {
    Periods theLastPeriods( _periods[ !_slot_periods ] );
    
    _periods[ !_slot_periods ]._count = 0;
    
    _slot_periods = !_slot_periods;

    if( theLastPeriods._count )
    {
       aPeriods._usecs = ( _periods[ !_slot_periods ]._usecs - theLastPeriods._usecs );
       aPeriods._count = ( _periods[ !_slot_periods ]._count - theLastPeriods._count );
    }
  }
}

void Injector::read_periods_on( Periods& aPeriods )
{
  if( _periods_on[ _slot_periods_on ]._usecs )
  {
    Periods theLastPeriods( _periods_on[ !_slot_periods_on ] );
    
    _periods_on[ !_slot_periods_on ]._count = 0;
    
    _slot_periods_on = !_slot_periods_on;

    if( theLastPeriods._count )
    {
       aPeriods._usecs = _periods_on[ !_slot_periods_on ]._usecs;
       aPeriods._count = ( _periods_on[ !_slot_periods_on ]._count - theLastPeriods._count );

       _periods_on[ !_slot_periods_on ]._usecs = 0;
    }
  }
}

