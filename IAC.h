#ifndef __IAC_H__
#define __IAC_H__

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
  void control_RPM( unsigned long aNow );
  void reset();

  ECU& _ecu;
  
  AccelStepper _stepper;

  EState _state;
  
  const float _Kp = 1;
  const float _Ki = .3;
  const float _Kd = .3;

  short _last_target_rpm;
  short _last_error;
  short _integral;
  short _derivative;

  unsigned long _last_control;
};

#endif
