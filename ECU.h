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
    sInitial,
    sStarting,
    sIdling,
    sRunning,
    sShutDown,
    sWait,
    sFinish
  };
  
  ECU();
  ~ECU();

  static void Iterrupt_Injector_Change();
  
  bool run();

  void calculate_target_RPM( unsigned long aNow );
  
  void read_RPM( unsigned long aNow );
  void read_Fueling( unsigned long aNow );

  IAC _iac;
  TPS _tps;
  Injector _injector;

  E_State _state;

  Average< unsigned short > _rpm_average;

  short _rpm;
  short _rpm_target;
  unsigned char _rpm_zero_counter;

  unsigned char _startup_timer;

  bool _last_tps_state;
  unsigned long _last_sample_usecs;
  unsigned long _last_rpm_change_usecs;
};

#endif

