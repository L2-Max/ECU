#include "VSS.h"

#include "EEPROM_Defs.h"

#include <EEPROM.h>
#include <Arduino.h>

VSS::VSS() : _pulses_h( 0 ), _pulses_l( 0 )
{
  EEPROM.get( ECU_VSS_PULSES_H, _pulses_h );
  EEPROM.get( ECU_VSS_PULSES_L, _pulses_l );
}

VSS::~VSS()
{
  EEPROM.put( ECU_VSS_PULSES_H, _pulses_h );
  EEPROM.put( ECU_VSS_PULSES_L, _pulses_l );
}

void VSS::read()
{
  if( digitalRead( PIN_VSS ) == HIGH )
  {
    ++_pulses_l;
  
    if( _pulses_l == 1000000000UL )
    {
      _pulses_l = 0;
      ++_pulses_h;
    }
  }
}

