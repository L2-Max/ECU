#ifndef __EEPROM_DEFS_H__
#define __EEPROM_DEFS_H__

#if 0
#include <EEPROM.h>
#else
#include <inttypes.h>
struct EEPROM_STUB
{
  static void get( uint32_t, ... ) {}
  static void put( uint32_t, ... ) {}
};

static EEPROM_STUB EEPROM;
#endif

#define ECU_RESET_FLAG_ADDR   0
#define ECU_MPG_IDLE_ADDR     ( ECU_RESET_FLAG_ADDR + 1 )
#define ECU_MPG_RUN_ADDR      ( ECU_MPG_IDLE_ADDR + 8 )
#define ECU_MAX_RPM           ( ECU_MPG_RUN_ADDR + 8 )
#define ECU_VSS_PULSES_H      ( ECU_MAX_RPM + 2 )
#define ECU_VSS_PULSES_L      ( ECU_VSS_PULSES_H + 4 )
#define ECU_VSS_METERS        ( ECU_VSS_PULSES_L + 4 )
#define ECU_TPS_CLOSED_MV     ( ECU_VSS_METERS + 4 )

#endif
