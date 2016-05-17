#include "ECU.h"

#include "EEPROM_Defs.h"

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ECU_SAMPLING_US 100000

#define DISPLAY_INTERVAL 300

#define ECU_POWER_PIN 4

LiquidCrystal_I2C g_lcd( 0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );

unsigned long g_LastMillis( millis() );

ECU* g_ECU( 0 );

void setup()
{
  Serial.begin(19200);

  for( int i( 0 ); i < EEPROM.length() ; i++ )
  {
//    EEPROM.write( i, 0 );
  }
  
  g_lcd.begin(16,2);

  g_lcd.home ();
  g_lcd.print( "   Hello!" );  
  g_lcd.setCursor( 0, 1 );

  g_lcd.print( "     l2ECU" );

  delay( 1000 );

  g_lcd.home();
  g_lcd.clear();

  pinMode( 8, OUTPUT );
  digitalWrite( 8, LOW );
}

unsigned long g_Cycles( 0 );

void loop()
{
  if( !g_ECU )
  {
    g_ECU = new ECU();
  }
  
  for(;;)
  {
    unsigned long theNow( millis() );

    if( !g_ECU->run() )
    {
      break;
    }

    if( ( theNow - g_LastMillis ) >= DISPLAY_INTERVAL )
    {
      g_LastMillis = theNow;
      
      //g_lcd.home();
      //g_lcd.clear();

      Serial.print( "c" );
      Serial.print( g_Cycles / DISPLAY_INTERVAL );

      Serial.print( " e" );
      Serial.print( g_ECU->_iac._last_error );
      
      Serial.print( " i" );
      Serial.print( g_ECU->_iac._integral );

      Serial.print( " d" );
      Serial.print( g_ECU->_iac._derivative );
      
      //g_lcd.setCursor( 0, 1 );

      Serial.print( " r" );
      Serial.print( g_ECU->_rpm );

      Serial.print( " t" );
      Serial.print( g_ECU->_rpm_target );

      Serial.print( " p" );
      Serial.print( static_cast< short >( g_ECU->_iac._stepper.currentPosition() ) );

      Serial.print( " tps" );
      Serial.print( g_ECU->_tps._value );

      Serial.print( " s" );
      Serial.print( g_ECU->_state );

      Serial.println();

      g_Cycles = 0;
    }

    ++g_Cycles;
  }

  delete g_ECU;
  g_ECU = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////
#define ECU_IDLE_RPM 1000
#define ECU_STARTUP_RPM 1500
#define ECU_RPM_TOLERANCE 25

ECU::ECU() :
  _iac( *this ), _injector( *this ), _rpm( 0 ), _rpm_target( ECU_IDLE_RPM ), _last_sample_usecs( 0 ),
  _rpm_average( 2 ), _last_rpm_change_usecs( 0 ), _state( sSetup ), _rpm_zero_counter( 0 )
{
  pinMode( ECU_POWER_PIN, OUTPUT );
  
  attachInterrupt( digitalPinToInterrupt( PIN_INJECTOR ), ECU::Iterrupt_Injector_Change, CHANGE );

  if( !EEPROM.read( ECU_RESET_FLAG_ADDR ) )
  {
    _iac.reset();
  }

  //EEPROM.update( ECU_RESET_FLAG_ADDR, 0 );
}

ECU::~ECU()
{
  digitalWrite( ECU_POWER_PIN, LOW );
}

void ECU::Iterrupt_Injector_Change()
{
  g_ECU->_injector.read();
}

bool ECU::run()
{
  bool ret( true );
  
  unsigned long theNow( micros() );

  if( ( theNow - _last_sample_usecs ) >= ECU_SAMPLING_US )
  {
    read_RPM( theNow );
    read_Fueling( theNow );

    calculate_target_RPM( theNow );

    if( _state == sSetup )
    {
      if( _iac._state == IAC::sReady )
      {
        _state = sInitial;

        EEPROM.update( ECU_RESET_FLAG_ADDR, 1 );
      }
    }
    else if( _state == sInitial )
    {
      if( _rpm >= ( ECU_IDLE_RPM - 500 ) )
      {
        if( _rpm > ECU_STARTUP_RPM )
        {
           _state = sStarting;

           EEPROM.update( ECU_RESET_FLAG_ADDR, 0 );
        }
        else
        {
          _iac.step( 100 );
        }
      }
    }
    else if( _state == sStarting )
    {
      digitalWrite( ECU_POWER_PIN, HIGH );

      if( _rpm )
      {
        if( _rpm >= ( ECU_IDLE_RPM - 500 ) && _rpm <= ( ECU_IDLE_RPM + 200 ) )
        {
          _state = sIdling;
        }
        else
        {
          _iac.step( -100 );
        }
      }
      else
      {
        _state = sShutDown;
      }
    }
    else if( _state == sRunning || _state == sIdling )
    {
      if( _rpm )
      {
        if( !_tps._isOpen )
        {
          if( _rpm >= ( ECU_IDLE_RPM - 500 ) && _rpm <= ( ECU_IDLE_RPM + 200 ) )
          {
            _state = sIdling;
          }
        }
        else if( _tps._isOpen )
        {
          _state = sRunning;
        }
      }
      else
      {
        _state = sShutDown;
      }
    }
    else if( _state == sShutDown )
    {
      _state = sWait;
      
      _iac.reset();
    }
    else if( _state == sWait )
    {
      if( _iac._state == IAC::sReady )
      {
        _state = sFinish;

        EEPROM.update( ECU_RESET_FLAG_ADDR, 1 );
      }
    }
    else if( _state == sFinish )
    {
      ret = false;
    }
    
    _last_sample_usecs = theNow;
  }

  _tps.read( theNow );
  _iac.control_RPM( theNow );

  return ret;
}

void ECU::calculate_target_RPM( unsigned long aNow )
{
  /*if( _state == sRunning )
  {
    if( ( aNow - _last_rpm_change_usecs ) > 1000000 * 5 )
    {
      _last_rpm_change_usecs = aNow;
      
      _rpm_target -= 100;

      if( _rpm_target < ( ECU_IDLE_RPM + 25 ) )
      {
        _rpm_target = ECU_IDLE_RPM;
      }
    }

    if( _tps._isOpen )
    {
      _rpm_target = 1200;
    }
  }
  else
  {
    _rpm_target = ECU_STARTUP_RPM;
  }*/
}

void ECU::read_RPM( unsigned long aNow )
{
  Periods thePeriods( _injector.read_periods() );
  
  if( thePeriods._count )
  {
    unsigned short theRpm( ( aNow - _last_sample_usecs ) * 30. /
                             thePeriods._usecs * thePeriods._count / ( ECU_SAMPLING_US / 1000000. ) );

    _rpm_average.push( theRpm );
    _rpm = _rpm_average.average();

    _rpm_zero_counter = ( 1000000 / ECU_SAMPLING_US / 2 );
  }
  else if( !--_rpm_zero_counter )
  {
    _rpm = 0;
  }
}

void ECU::read_Fueling( unsigned long aNow )
{
  Periods thePeriods( _injector.read_periods_on() );
  
  if( thePeriods._count )
  {
  }
}

