#ifndef __EEPROM_DEFS_H__
#define __EEPROM_DEFS_H__

#define ECU_RESET_FLAG_ADDR   0
#define ECU_MPG_TOTAL_ADDR    ( ECU_RESET_FLAG_ADDR + 1 )
#define ECU_MAX_RPM           ( ECU_MPG_TOTAL_ADDR + 8 )
#define ECU_VSS_PULSES_H      ( ECU_MAX_RPM + 2 )
#define ECU_VSS_PULSES_L      ( ECU_VSS_PULSES_H + 4 )
#define ECU_VSS_METERS        ( ECU_VSS_PULSES_L + 4 )

#endif
