#include "VSS.h"

#include "EEPROM_Defs.h"

#include <Arduino.h>

#define VSS_SAMPLE_MS 250
#define VSS_PULSE_PER_METER 29.

VSS::VSS() : _pulses_total( 0 ), _meters( 0 ), _speed( 0 ), _next_sample_ms( 0 ), _slot_pulses( false ), _is_On( false )
{
  EEPROM.get( ECU_VSS_METERS, _meters );

  pinMode( PIN_VSS, INPUT );

  memset( &_pulses, 0, sizeof( _pulses ) );
}

VSS::~VSS()
{
  EEPROM.put( ECU_VSS_METERS, _meters );
}

void VSS::read()
{
  int theState( digitalRead( PIN_VSS ) );

  if( ( theState == HIGH ) && !_is_On || ( theState == LOW ) && _is_On )
  {
    if( _is_On )
    {
      _pulses[ _slot_pulses ]._usecs = micros();
      _pulses[ _slot_pulses ]._count = ++_pulses_total;
    }

    _is_On = !_is_On;
  }
}

void VSS::read_Speed( unsigned long aNow_MS )
{
  if( aNow_MS >= _next_sample_ms )
  {
    _next_sample_ms = ( aNow_MS + VSS_SAMPLE_MS );

    Pulses thePulses;
    
    if( _pulses[ _slot_pulses ]._count )
    {
      Pulses theLastPulses( _pulses[ !_slot_pulses ] );
  
      _pulses[ !_slot_pulses ]._count = 0;
      _pulses[ !_slot_pulses ]._usecs = 0;
  
      _slot_pulses = !_slot_pulses;
  
      if( theLastPulses._count )
      {
        thePulses._usecs = ( _pulses[ !_slot_pulses ]._usecs - theLastPulses._usecs );
        thePulses._count = ( _pulses[ !_slot_pulses ]._count - theLastPulses._count );
      }
    }

    _meters += ( thePulses._count / VSS_PULSE_PER_METER );
    _speed = ( 1000. * 60. * 60. / VSS_PULSE_PER_METER / thePulses._usecs * thePulses._count );
  }
}

