#include "IAC.h"

#include "ECU.h"

#include <Arduino.h>
#include <EEPROM.h>

#define IAC_MAX_STEPS 2000.

#define IAC_CONTROL_US 100000

#define IAC_RPM_LOW_THRESHOLD 500.
#define IAC_RPM_HIGH_THRESHOLD 2000.

#define IAC_ERROR_MAX 50
#define IAC_I_MAX 100
#define IAC_D_MAX 100

#define IAC_CURR_POS_ADDR 0
#define IAC_STARTUP_POS_ADDR 2

IAC::IAC( ECU& anECU ) :
  _ecu( anECU ), _pos( IAC_MAX_STEPS ), _steps_ToGo( 0 ),
  _last_control( 0 ), _last_step( 0 ), _last_error( 0 ), _integral( 0 ), _derivative( 0 ),
  _last_target_rpm( 0 ), _stepper( 2100, 11, 9, 10, 8 ), _is_Reset( false ), _is_SetStartPos( false )
{
  //_stepper.moveTo( -IAC_MAX_STEPS );
  _stepper.setSpeed( 12 );

  _stepper.step( -IAC_MAX_STEPS );
  _stepper.step( 1000 );

  //EEPROM.get( IAC_CURR_POS_ADDR, _pos );
}

IAC::~IAC()
{
  //EEPROM.put( IAC_CURR_POS_ADDR, _pos );
}

void IAC::step( short aSteps )
{
  _steps_ToGo = aSteps;

  if( ( _pos + _steps_ToGo ) > IAC_MAX_STEPS )
  {
    _steps_ToGo = ( IAC_MAX_STEPS - _pos );
  }
  else if( ( _pos + _steps_ToGo ) < 0 )
  {
    _steps_ToGo = -_pos;
  }
  
  //_stepper.setSpeed( 700. );
}

void IAC::control_RPM( unsigned long aNow )
{
  if( !_is_Reset )
  {
    if( ( aNow - _last_control ) >= ( IAC_CONTROL_US ) )
    {
      if( !_ecu._tps._isOpen )
      {
        if( /*_ecu._rpm >= IAC_RPM_LOW_THRESHOLD && */ _ecu._rpm <= IAC_RPM_HIGH_THRESHOLD )
        {
          short theSteps( 0 );
          
          if( _ecu.isRunning() /*&& _ecu._rpm_target == ECU_IDLE_RPM &&
              _ecu._rpm > ( ECU_IDLE_RPM - 200 ) && _ecu._rpm < ( ECU_IDLE_RPM + 200 )*/ )
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
    
            theSteps = ( _Kp * theError + _Ki * _integral + _Kd * _derivative );
    
            _last_error = theError;
          }
  
          step( theSteps );
        }
      }
  
      _last_control = aNow;
    }
  }
  
  if( ( aNow - _last_step ) >= 2400 )
  {
    _last_step = aNow;
    
    if( _steps_ToGo > 0 )
    {
       _stepper.step( 1 );

       --_steps_ToGo;
       ++_pos;
    }
    else if( _steps_ToGo < 0 )
    {
      _stepper.step( -1 );

       ++_steps_ToGo;
       --_pos;
    }
    else
    {
      if( _is_Reset )
      {
        if( !_is_SetStartPos )
        {
          _is_SetStartPos = true;

          step( 1000 );
        }
        else
        {
           _is_Reset = false;
        }
      }
    }
  }
  
  //_stepper.runSpeedToPosition();
}

void IAC::reset()
{
  _is_Reset = true;

  step( -IAC_MAX_STEPS );
}

