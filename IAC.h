#ifndef __IAC_H__
#define __IAC_H__

#include <Servo.h>

#define ECU_IDLE_RPM 1000
#define ECU_STARTUP_RPM 1300
#define ECU_RPM_TOLERANCE 25

struct ECU;

struct IAC
{
  IAC( ECU& anECU );

  void step( char aSteps );
  void control_RPM( unsigned long aNow );

  unsigned char  read_pos_EEPROM();
  void write_pos_EEPROM();

  ECU& _ecu;
  
  Servo _servo;
  short _pos;

  const float _Kp = 1;
  const float _Ki = .0;
  const float _Kd = .0;

  short _last_target_rpm;

  unsigned char _rpm_to_pos[ ( ECU_STARTUP_RPM - ECU_IDLE_RPM ) / ECU_RPM_TOLERANCE + 1 ];

  short _last_error;
  unsigned long _servo_delay;
  
  short _integral;
  short _derivative;

  unsigned long _last_control;
};

#endif
