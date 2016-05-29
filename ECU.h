#ifndef __ECU_H__
#define __ECU_H__

#include "IAC.h"
#include "Injector.h"
#include "TPS.h"
#include "ECT.h"
#include "VSS.h"
#include "MAP.h"

#include "Average.h"

//#define ECU_SYSTEM_RESET

struct ECU
{
  enum E_State
  {
    sInit,
    sUninit,
    sWforEstarting,
    sIdling,
    sRunning,
    sResetting
  };
  
  ECU();
  ~ECU();

  static void Iterrupt_Injector_Change();
  static void Iterrupt_VSS_Change();
  
  void run( unsigned long aNow_MS );

  void calculate_target_RPM( unsigned long aNow );
  
  void read_RPM( unsigned long aNow );
  void read_Fueling( unsigned long aNow );

  IAC _iac;
  TPS _tps;
  ECT _ect;
  VSS _vss;
  MAP _map;
  Injector _injector;

  E_State _state;

  void ( ECU::*_state_handler )();

  void init();
  void uninit();

  void wait_for_engine_starting();

  void engine_idling();

  Average< unsigned short > _rpm_average;
  Average< unsigned long > _periods_on_average;

  short _rpm;
  short _rpm_target;
  unsigned short _rpm_max;

  uint16_t _idle_timer;
  
  unsigned char _rpm_zero_counter;
  unsigned char _periods_on_zero_counter;

  long _last_idle_position;
  
  unsigned long _next_sample_ms;

  uint64_t _total_periods_on;
};

#endif

