#include "IAC.h"

#include "ECU.h"

#include <Arduino.h>
#include <EEPROM.h>

#define PIN_SERVO 4

#define SERVO_INIT_MS 1900

#define IAC_POS 180
#define IAC_MAX_STEPS 80.

#define IAC_CONTROL_US 250000

#define IAC_RPM_LOW_THRESHOLD 500.
#define IAC_RPM_HIGH_THRESHOLD 2000.

#define IAC_ERROR_MAX 1
#define IAC_I_MAX 5
#define IAC_D_MAX 5

IAC::IAC( ECU& anECU ) :
  _ecu( anECU ), _pos( 0 ), _last_control( 0 ), _last_error( 0 ), _integral( 0 ), _derivative( 0 ), _servo_delay( 0 ),
  _last_target_rpm( 0 )
{
  memset( _rpm_to_pos, -1, sizeof( _rpm_to_pos ) );
  
  _servo.attach( PIN_SERVO, ( IAC_POS - IAC_MAX_STEPS ), IAC_POS );
  _servo.write( IAC_POS );

  _rpm_to_pos[ 0 ] = 36;
  _rpm_to_pos[ 1 ] = 36;
  _rpm_to_pos[ 2 ] = 36;
  _rpm_to_pos[ 3 ] = 36;
  _rpm_to_pos[ 4 ] = 36;
  _rpm_to_pos[ 5 ] = 36;
  _rpm_to_pos[ 6 ] = 36;
  _rpm_to_pos[ 7 ] = 36;
  _rpm_to_pos[ 8 ] = 45;
  _rpm_to_pos[ 9 ] = 50;
  _rpm_to_pos[ 10 ] = 55;
  _rpm_to_pos[ 11 ] = 60;
  _rpm_to_pos[ 12 ] = 65;
  //_rpm_to_pos[ 13 ] = 60;
  //_rpm_to_pos[ 14 ] = 60;
  //_rpm_to_pos[ 15 ] = 60;
  //_rpm_to_pos[ 16 ] = 36;
  //_rpm_to_pos[ 17 ] = 36;
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
  }
  else
  {
    _servo.write( IAC_POS - _pos );
  }
}

void IAC::control_RPM( unsigned long aNow )
{
  if( ( aNow - _last_control ) >= ( IAC_CONTROL_US + _servo_delay ) )
  {
    _servo_delay = 0;
    
    if( !_ecu._tps._isOpen )
    {
      if( /*_ecu._rpm >= IAC_RPM_LOW_THRESHOLD && */ _ecu._rpm <= IAC_RPM_HIGH_THRESHOLD )
      {
        short theSteps( 0 );
        
        if( _ecu._rpm_target != _last_target_rpm )
        {
          _last_target_rpm = _ecu._rpm_target;
          
          short theNewPos( read_pos_EEPROM() );

          if( theNewPos )
          {
            theSteps = ( theNewPos - _pos );
            
            //_servo_delay += 1000000;
          }
        }

        if( !theSteps && _ecu.isRunning() && _ecu._rpm_target == ECU_IDLE_RPM &&
            _ecu._rpm > ( ECU_IDLE_RPM - 200 ) && _ecu._rpm < ( ECU_IDLE_RPM + 200 ) )
        {
          short theError( ( _ecu._rpm_target - _ecu._rpm ) / 
                          ( ( IAC_RPM_HIGH_THRESHOLD - IAC_RPM_LOW_THRESHOLD ) / IAC_MAX_STEPS ) );

          /*if( _ecu._rpm_target != ECU_IDLE_RPM &&
             _ecu._rpm > ( _ecu._rpm_target - ECU_RPM_TOLERANCE ) && _ecu._rpm < ( _ecu._rpm_target + ECU_RPM_TOLERANCE ) )
          {
            theError = 0;
          }*/
  
          if( theError > IAC_ERROR_MAX )
          {
            theError = IAC_ERROR_MAX;
          }
          else if( theError < -IAC_ERROR_MAX )
          {
            theError = -IAC_ERROR_MAX;
          }
  
          _integral += theError * ( float( aNow - _last_control + _servo_delay ) / 1000000.  );
          _derivative = ( theError - _last_error ) / ( float( aNow - _last_control + _servo_delay ) / 1000000. );
  
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

          if( !theError )
          {
            write_pos_EEPROM();
          }
        }

        step( theSteps );

        if( theSteps )
        {
          _servo_delay += 0;//250000;
        }
        else
        {
          _servo_delay = 0;
        }
      }
    }
    else
    {
      //step( -_pos );
      //step( 30 );
    }

    _last_control = aNow;
  }
}

unsigned char IAC::read_pos_EEPROM()
{
  unsigned char theAddr( ( _ecu._rpm_target - ECU_IDLE_RPM ) / ECU_RPM_TOLERANCE );

  if( _rpm_to_pos[ theAddr ] == 255 )
  {
     _rpm_to_pos[ theAddr ] = EEPROM.read( theAddr );
  }
  
  return _rpm_to_pos[ theAddr ];
}

void IAC::write_pos_EEPROM()
{
  unsigned char theAddr( ( _ecu._rpm_target - ECU_IDLE_RPM ) / ECU_RPM_TOLERANCE );

  _rpm_to_pos[ theAddr ] = _pos;

  //EEPROM.update( theAddr, _rpm_to_pos[ theAddr ] );
}

