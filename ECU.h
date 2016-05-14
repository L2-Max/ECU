#ifndef __ECU_H__
#define __ECU_H__

#include "IAC.h"
#include "Injector.h"
#include "TPS.h"

#include "Average.h"

struct ECU
{
  ECU();

  static void Iterrupt_Injector_Change();
  
  void run();

  void calculate_target_RPM( unsigned long aNow );
  
  void read_RPM( unsigned long aNow );
  void read_Fueling( unsigned long aNow );

  bool isRunning()const;

  IAC _iac;
  TPS _tps;
  Injector _injector;

  Average< unsigned short > _rpm_average;
  
  short _rpm;
  short _rpm_target;

  unsigned char _startup_timer;

  unsigned long _last_sample_usecs;
  unsigned long _last_rpm_change_usecs;
};

#endif

