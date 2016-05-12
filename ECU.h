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

  void calculate_target_RPM();
  void calculate_RPM( unsigned long aNow );

  IAC _iac;
  TPS _tps;
  Injector _injector;

  bool _is_Running;

  Average< unsigned short > _rpm_average;
  
  short _rpm;
  short _rpm_target;

  unsigned long _last_sample_usecs;

  unsigned long _cycles;
};

#endif

