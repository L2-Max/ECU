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
  void stepTo( unsigned short aSteps );
  
  void control_RPM( unsigned long aNow_MS );
  
  void run();
  void reset();

  void Set_Enabled( bool anEnabled );

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

  unsigned long _next_control_ms;

  bool _is_Enabled;
};

#endif
