#ifndef __IAC_H__
#define __IAC_H__

//#include <AccelStepper.h>
#include <Stepper.h>

#define ECU_IDLE_RPM 1000
#define ECU_STARTUP_RPM 1800
#define ECU_RPM_TOLERANCE 25

struct ECU;

struct IAC
{
  IAC( ECU& anECU );
  ~IAC();

  void step( short aSteps );
  void control_RPM( unsigned long aNow );
  void reset();

  ECU& _ecu;
  
  Stepper _stepper;

  bool _is_Reset;
  bool _is_SetStartPos;

  short _pos;
  short _steps_ToGo;

  const float _Kp = 1;
  const float _Ki = .0;
  const float _Kd = .8;

  short _last_target_rpm;
  short _last_error;
  short _integral;
  short _derivative;

  unsigned long _last_control;
  unsigned long _last_step;
};

#endif
