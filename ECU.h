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
    sRunning
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

  short _rpm;
  short _rpm_target;
  unsigned char _rpm_zero_counter;

  unsigned char _startup_timer;

  unsigned long _last_sample_usecs;
  unsigned long _last_rpm_change_usecs;
};

#endif

