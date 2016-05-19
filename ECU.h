#ifndef __ECU_H__
#define __ECU_H__

#include "IAC.h"
#include "Injector.h"
#include "TPS.h"

#include "Average.h"

struct ECU
{
  enum E_State
  {
    sInit,
    sUninit,
    sWforEstarting,
    sWforEgetsSrpm,
    sWforEidlingAstartup,
    sIdling,
    sRunning,
    sResetting
  };
  
  ECU();
  ~ECU();

  static void Iterrupt_Injector_Change();
  
  void run( unsigned long aNow_MS );

  void calculate_target_RPM( unsigned long aNow );
  
  void read_RPM( unsigned long aNow );
  void read_Fueling( unsigned long aNow );

  IAC _iac;
  TPS _tps;
  Injector _injector;

  E_State _state;

  void ( ECU::*_state_handler )();

  void init();
  void uninit();

  void wait_for_engine_starting();
  void wait_for_engine_gets_startup_rpm();
  void wait_for_engine_idling_after_startup();

  void engine_idling();

  Average< unsigned short > _rpm_average;
  Average< unsigned long > _periods_on_average;

  short _rpm;
  short _rpm_target;
  unsigned short _rpm_max;
  
  unsigned char _rpm_zero_counter;
  unsigned char _periods_on_zero_counter;

  bool _last_tps_state;

  unsigned short _last_idle_steps;
  unsigned long _last_sample_usecs;

  uint64_t _total_periods_on;
};

#endif

