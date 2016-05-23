#include "IAC.h"

#include "ECU.h"

#include <Arduino.h>
#include <EEPROM.h>

#define IAC_MAX_STEPS 5000.

#define IAC_CONTROL_MS 200.

#define IAC_ERROR_MAX 50
#define IAC_I_MAX 50
#define IAC_D_MAX 50

#define IAC_START_POS 2500
#define IAC_SPEED 700

IAC::IAC( ECU& anECU ) :
  _ecu( anECU ), _last_control( 0 ), _last_error( 0 ), _integral( 0 ), _derivative( 0 ),
  _last_target_rpm( 0 ), _stepper( AccelStepper::HALF4WIRE, 12, 10, 11, 8 ),
  _state( sReady ), _is_Enabled( false )
{
  _stepper.setMaxSpeed( 2000. );
  _stepper.setAcceleration( 1000. );

  _stepper.setCurrentPosition( IAC_START_POS );
}

IAC::~IAC()
{
}

void IAC::step( short aSteps )
{
  if( ( _stepper.currentPosition() + aSteps ) > IAC_MAX_STEPS )
  {
    _stepper.move( IAC_MAX_STEPS - _stepper.currentPosition() );
  }
  else if( ( _stepper.currentPosition() + aSteps ) < 0 )
  {
    _stepper.move( -_stepper.currentPosition() );
  }
  else
  {
    _stepper.move( aSteps );
  }
  
  _stepper.setSpeed( IAC_SPEED );
}

void IAC::stepTo( unsigned short aSteps )
{
  if( aSteps > IAC_MAX_STEPS )
  {
    aSteps = IAC_MAX_STEPS;
  }

  _stepper.moveTo( aSteps );
  _stepper.setSpeed( IAC_SPEED );
}

void IAC::control_RPM( unsigned long aNow_MS )
{
  if( aNow_MS >= _last_control )
  {
    _last_control = ( aNow_MS + IAC_CONTROL_MS );
    
    if( _state == sReady )
    {
      if( _is_Enabled )
      {
        short theError( ( _ecu._rpm_target - _ecu._rpm ) );

        if( theError > IAC_ERROR_MAX )
        {
          theError = IAC_ERROR_MAX;
        }
        else if( theError < -IAC_ERROR_MAX )
        {
          theError = -IAC_ERROR_MAX;
        }

        _integral += theError * ( IAC_CONTROL_MS / 1000. );
        _derivative = ( theError - _last_error ) / ( IAC_CONTROL_MS / 1000. );

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

        step( _Kp * theError + _Ki * _integral + _Kd * _derivative );

        _last_error = theError;
      }
    }
  }
}

void IAC::run()
{
  if( !_stepper.distanceToGo() )
  {
    if( _state == sResetting )
    {
      _state = sSetting;

      _stepper.setCurrentPosition( 0 );
      
      step( IAC_START_POS );
    }
    else if( _state == sSetting )
    {
      _state = sReady;
    }
  }
  else
  {
    _stepper.runSpeedToPosition();
  }
}

void IAC::reset()
{
  _state = sResetting;

#ifdef ECU_SYSTEM_RESET
  _stepper.move( -IAC_MAX_STEPS );
#else
  _stepper.move( -_stepper.currentPosition() - 200 );
#endif

  _stepper.setSpeed( IAC_SPEED );
}

void IAC::Set_Enabled( bool anEnabled )
{
  _is_Enabled = anEnabled;
}

