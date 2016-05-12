#include "Injector.h"

#include "ECU.h"

#include <Arduino.h>

#define INJECTOR_STATE_THRESHOLD 1
#define INJECTOR_STATE_JITTER 1

Injector::Injector( ECU& anEcu ) :
   _ecu( anEcu ), _is_On( false ), _slot_periods( false ), _slot_periods_on( false ), _state( 0 )
{
  memset( &_private_periods, 0, sizeof( _private_periods ) );
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
      _periods_on[ _slot_periods_on ]._usecs += ( micros() - _periods[ _slot_periods ]._usecs );
      
      ++_periods_on[ _slot_periods_on ]._count;
    }

    _is_On = !_is_On;

    _ecu._is_Running = true;
  }
}

Periods Injector::read_periods()
{
  Periods ret;
  
  _slot_periods = !_slot_periods;
  
  Periods thePeriods( _periods[ !_slot_periods ] );

  ret._usecs = ( thePeriods._usecs - _last_periods._usecs );
  ret._count = ( thePeriods._count - _last_periods._count );

  _last_periods = thePeriods;

  return ret;
}

Periods Injector::read_periods_on()
{
  Periods ret;
  
  _slot_periods_on = !_slot_periods_on;
  
  Periods thePeriods( _periods_on[ !_slot_periods_on ] );

  ret._usecs = ( thePeriods._usecs - _last_periods_on._usecs );
  ret._count = ( thePeriods._count - _last_periods_on._count );

  _last_periods_on = thePeriods;

  return ret;
}

  
