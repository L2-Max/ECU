#include "ECU.h"

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ECU_SAMPLING_US 250000

#define DISPLAY_INTERVAL 250

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
  
  g_ECU = new ECU();
  
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
  for(;;)
  {
    unsigned long theNow( millis() );

    g_ECU->run();

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
      Serial.print( static_cast< short >( g_ECU->_iac._pos ) );

      Serial.print( " l" );
      Serial.print( ( g_ECU->_tps._isOpen ? "O" : "C" ) );

      Serial.print( " v" );
      Serial.print( g_ECU->_tps._value );

      Serial.println();

      g_Cycles = 0;
    }

    ++g_Cycles;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
ECU::ECU() :
  _iac( *this ), _injector( *this ), _rpm( 0 ), _rpm_target( ECU_IDLE_RPM ), _last_sample_usecs( 0 ),
  _rpm_average( 5 ), _last_rpm_change_usecs( 0 )
{
  attachInterrupt( digitalPinToInterrupt( PIN_INJECTOR ), ECU::Iterrupt_Injector_Change, CHANGE );
}

void ECU::Iterrupt_Injector_Change()
{
  g_ECU->_injector.read();
}

void ECU::run()
{
  unsigned long theNow( micros() );

  if( ( theNow - _last_sample_usecs ) >= ECU_SAMPLING_US )
  {
    read_RPM( theNow );
    read_Fueling( theNow );

    calculate_target_RPM( theNow );

    _last_sample_usecs = theNow;
  }

  _tps.read( theNow );
  _iac.control_RPM( theNow );
}

void ECU::calculate_target_RPM( unsigned long aNow )
{
  if( isRunning() )
  {
    if( ( aNow - _last_rpm_change_usecs ) > 1000000 * 2 )
    {
      _last_rpm_change_usecs = aNow;
      
      _rpm_target -= 25;

      if( _rpm_target < ( ECU_IDLE_RPM + 25 * 6 ) )
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
  }
}

void ECU::read_RPM( unsigned long aNow )
{
  Periods thePeriods( _injector.read_periods() );
  
  if( thePeriods._count )
  {
    unsigned short theRpm( ( aNow - _last_sample_usecs ) * 60. /
                             thePeriods._usecs * thePeriods._count / ( ECU_SAMPLING_US / 1000000. ) / 4. );

    _rpm_average.push( theRpm );
  }
  else
  {
    _rpm_average.push( 0 );
  }

  _rpm = _rpm_average.average();
}

void ECU::read_Fueling( unsigned long aNow )
{
  Periods thePeriods( _injector.read_periods_on() );
  
  if( thePeriods._count )
  {
  }
}

bool ECU::isRunning()const
{
  return _rpm;
}

