#include "IAC.h"

#include "ECU.h"

#include <Arduino.h>

#define PIN_SERVO 4

#define SERVO_INIT_MS 1900

#define IAC_POS 180
#define IAC_MAX_STEPS 100.

#define IAC_CONTROL_US 250000

#define IAC_RPM_LOW_THRESHOLD 800.
#define IAC_RPM_HIGH_THRESHOLD 2000.

#define IAC_ERROR_MAX 60
#define IAC_I_MAX 20
#define IAC_D_MAX 20

IAC::IAC( ECU& anECU ) :
  _ecu( anECU ), _pos( 0 ), _last_control( 0 ), _last_error( 0 ), _integral( 0 ), _derivative( 0 )
{
  _servo.attach( PIN_SERVO, ( IAC_POS - IAC_MAX_STEPS ), IAC_POS );
  _servo.write( IAC_POS );

  //step( 30 );
}

void IAC::step( char aSteps )
{
  _pos += aSteps;

  if( _pos > IAC_MAX_STEPS )
  {
    _pos = IAC_MAX_STEPS;
  }
  else if( _pos < 0 )
  {
    _pos = 0;
  }

  if( aSteps < 0 )
  {
    _servo.write( IAC_POS - _pos );

//    delay( 15 );
  }
  else
  _servo.write( IAC_POS - _pos );
}

void IAC::control_RPM( unsigned long aNow )
{
  if( ( aNow - _last_control ) >= IAC_CONTROL_US )
  {
    unsigned long theServoDelay( 0 );
    
    if( !_ecu._tps._isOpen )
    {
      if( _ecu._rpm >= IAC_RPM_LOW_THRESHOLD && _ecu._rpm <= IAC_RPM_HIGH_THRESHOLD )
      {
        short theError( ( _ecu._rpm_target - _ecu._rpm ) / 
                        ( ( IAC_RPM_HIGH_THRESHOLD - IAC_RPM_LOW_THRESHOLD ) / IAC_MAX_STEPS ) );

        if( theError > IAC_ERROR_MAX )
        {
          theError = IAC_ERROR_MAX;
        }
        else if( theError < -IAC_ERROR_MAX )
        {
          theError = -IAC_ERROR_MAX;
        }

        _integral += theError * ( float( aNow - _last_control ) / 1000000.  );
        _derivative = ( theError - _last_error ) / ( float( aNow - _last_control ) / 1000000. );

        if( _integral > IAC_I_MAX )
        {
          _integral = IAC_I_MAX;
        }
        else if( _integral < -IAC_I_MAX )
        {
          _integral = -IAC_I_MAX;
        }

        if( _derivative > IAC_D_MAX )
        {
          _derivative = IAC_D_MAX;
        }
        else if( _derivative < -IAC_D_MAX )
        {
          _derivative = -IAC_D_MAX;
        }

        short theSteps( _Kp * theError + _Ki * _integral + _Kd * _derivative );

        _last_error = theError;

        step( theSteps );

        if( theSteps )
        {
          //theServoDelay = 150;
        }
      }
    }
    else
    {
      //step( -_pos );
      //step( 30 );
    }

    _last_control = aNow + theServoDelay;
  }
}

