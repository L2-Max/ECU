#include "IAC.h"

#include "ECU.h"

#include "EEPROM_Defs.h"

#include <Arduino.h>

#define IAC_MAX_STEPS 6000.

#define IAC_CONTROL_MS 200.

#define IAC_ERROR_MAX 50
#define IAC_I_MAX 50
#define IAC_D_MAX 50

#define IAC_START_POS 2500
#define IAC_SPEED 600

IAC::IAC( ECU& anECU ) :
  _ecu( anECU ), _next_control_ms( 0 ), _last_error( 0 ), _integral( 0 ), _derivative( 0 ),
  _last_target_rpm( 0 ), _stepper( AccelStepper::HALF4WIRE, 12, 10, 11, 8 ),
  _state( sReady ), _is_Enabled( false ), _Kp( .7 ), _Ki( .0 ), _Kd( .3 ),
  _down_timer( 0 )//, _error_average( 1000. / IAC_CONTROL_MS )
{
  _stepper.setMaxSpeed( 1000. );
  _stepper.setAcceleration( 1000. );

  _stepper.setCurrentPosition( 0 );
}

IAC::~IAC()
{
}

void IAC::step( short aSteps )
{
  _stepper.move( aSteps );
  _stepper.setSpeed( IAC_SPEED );
}

void IAC::stepTo( short aSteps )
{
  _stepper.moveTo( aSteps );
  _stepper.setSpeed( IAC_SPEED );
}

void IAC::control_RPM( unsigned long aNow_MS )
{
  if( aNow_MS >= _next_control_ms )
  {
    _next_control_ms = ( aNow_MS + IAC_CONTROL_MS );
    
    if( _state == sReady )
    {
        short theError( ( _ecu._rpm_target - _ecu._rpm ) );

        //_error_average.push( theError );

        _integral += theError/*_error_average.average()*/ * ( IAC_CONTROL_MS / 1000. );
        _derivative = ( theError/*_error_average.average()*/ - _last_error ) / ( IAC_CONTROL_MS / 1000. );

        if( theError > IAC_ERROR_MAX )
        {
          theError = IAC_ERROR_MAX;
        }
        else if( theError < -IAC_ERROR_MAX )
        {
          theError = -IAC_ERROR_MAX;
        }
        
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

        if( theError /*_error_average.average()*/ < 0 && _integral < 0 )
        {
          if( _down_timer && _derivative <= 0 )
          {
            --_down_timer;
          }
        }
        else
        {
          _down_timer = ( 7000. / IAC_CONTROL_MS );
        }

        if( _is_Enabled && ( !_down_timer || theError /*_error_average.average()*/ >= 0 ) )
        {
          step( _Kp * theError/*_error_average.average()*/ + _Ki * _integral + _Kd * _derivative );
        }

      _last_error = theError;//_error_average.average();
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

      step( -( IAC_MAX_STEPS - IAC_START_POS ) );
    }
    else if( _state == sSetting )
    {
      _stepper.setCurrentPosition( 0 );
      
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
#ifdef ECU_SYSTEM_RESET
  _state = sResetting;
  step( IAC_MAX_STEPS );
#else
  _state = sSetting;
  step( 1000 );
#endif
}

void IAC::Set_Enabled( bool anEnabled )
{
  if( _is_Enabled && !anEnabled )
  {
    _stepper.stop();
  }
  
  _is_Enabled = anEnabled;
}

