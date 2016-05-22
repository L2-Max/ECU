#include "VSS.h"

#include "EEPROM_Defs.h"

#include <EEPROM.h>
#include <Arduino.h>

#define VSS_SAMPLE_MS 1000
#define VSS_PULSE_PER_METER 28.85

VSS::VSS() : _pulses_total( 0 ), _meters( 0 ), _speed( 0 ), _next_sample_ms( 0 ), _slot_pulses( false )
{
  EEPROM.get( ECU_VSS_METERS, _meters );

  pinMode( PIN_VSS, INPUT );

  _pulses[ 0 ] = 0;
  _pulses[ 1 ] = 0;
}

VSS::~VSS()
{
  EEPROM.put( ECU_VSS_METERS, _meters );
}

void VSS::read()
{
  if( digitalRead( PIN_VSS ) == LOW )
  {
    ++_pulses_total;
    
    _pulses[ _slot_pulses ] = _pulses_total;
  }
}

void VSS::read_Speed( unsigned long aNow_MS )
{
  if( aNow_MS >= _next_sample_ms )
  {
    _next_sample_ms = ( aNow_MS + VSS_SAMPLE_MS );

    uint32_t thePulses( 0 );
    
    if( _pulses[ _slot_pulses ] )
    {
      uint32_t theLastPulses( _pulses[ !_slot_pulses ] );
  
      _pulses[ !_slot_pulses ] = 0;
  
      _slot_pulses = !_slot_pulses;
  
      if( theLastPulses )
      {
        thePulses = ( _pulses[ !_slot_pulses ] - theLastPulses );
      }
    }

    _meters += thePulses / VSS_PULSE_PER_METER;
    _speed = ( thePulses / VSS_PULSE_PER_METER ) * ( 1000. / VSS_SAMPLE_MS ) * 60. * 60 / 1000.;
  }
}

