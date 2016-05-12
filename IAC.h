#ifndef __IAC_H__
#define __IAC_H__

#include <Servo.h>

struct ECU;

struct IAC
{
  IAC( ECU& anECU );

  void step( char aSteps );
  void control_RPM( unsigned long aNow );

  ECU& _ecu;
  
  Servo _servo;
  short _pos;

  const float _Kp = 2;
  const float _Ki = .1;
  const float _Kd = .5;

  short _last_error;
  
  short _integral;
  short _derivative;

  unsigned long _last_control;
};

#endif
