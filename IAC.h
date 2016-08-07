#ifndef __IAC_H__
#define __IAC_H__

#include "Average.h"

#include <AccelStepper.h>
//#include <Stepper.h>

struct ECU;

struct IAC
{
  enum EState
  {
    sReady,
    sResetting,
    sSetting
  };
  
  IAC( ECU& anECU );
  ~IAC();

  void step( short aSteps );
  void stepTo( short aSteps );
  
  void control_RPM( unsigned long aNow_MS );
  
  void run();
  void reset();

  void Set_Enabled( bool anEnabled );

  ECU& _ecu;
  
  AccelStepper _stepper;

  EState _state;
  
  float _Kp;
  float _Ki;
  float _Kd;

  short _last_target_rpm;
  short _last_error;
  short _integral;
  short _derivative;

  unsigned long _next_control_ms;

  bool _is_Enabled;

  uint16_t _down_timer;

  //Average< short > _error_average;
};

#endif
