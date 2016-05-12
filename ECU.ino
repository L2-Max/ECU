#include <LiquidCrystal_I2C.h>

#include "ECU.h"

#define ECU_SAMPLING_US 250000
#define ECU_IDLE_RPM 1000
#define ECU_RPM_TOLERANCE 100

#define DISPLAY_INTERVAL 250

LiquidCrystal_I2C g_lcd( 0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );

unsigned long g_LastMillis( millis() );
unsigned long g_DisplayMS( 0 );

ECU* g_ECU( 0 );

void setup()
{
  Serial.begin(19200);
  
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

unsigned long g_LastPulse( 0 );
unsigned long g_Last_Cycles( 0 );

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
      Serial.print( ( g_ECU->_cycles - g_Last_Cycles ) / DISPLAY_INTERVAL );

      Serial.print( " e" );
      Serial.print( g_ECU->_iac._last_error );
      
      Serial.print( " i" );
      Serial.print( g_ECU->_iac._integral );

      Serial.print( " d" );
      Serial.print( g_ECU->_iac._derivative );
      
      //g_lcd.setCursor( 0, 1 );

      Serial.print( " r" );
      Serial.print( g_ECU->_rpm );

      Serial.print( " p" );
      Serial.print( static_cast< short >( g_ECU->_iac._pos ) );

      Serial.print( " l" );
      Serial.print( ( g_ECU->_tps._isOpen ? "O" : "C" ) );

      Serial.print( " v" );
      Serial.println( g_ECU->_tps._value );

      g_DisplayMS = ( millis() - g_LastMillis );
      g_Last_Cycles = g_ECU->_cycles;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
ECU::ECU() :
  _iac( *this ), _injector( *this ), _rpm( 0 ), _rpm_target( ECU_IDLE_RPM ), _last_sample_usecs( 0 ), _cycles( 0 ),
  _rpm_average( 5 ), _is_Running( false )
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
    calculate_RPM( theNow );
    calculate_target_RPM();

    _last_sample_usecs = theNow;
  }

  _tps.read( theNow );
  _iac.control_RPM( theNow );

  ++_cycles;
}

void ECU::calculate_target_RPM()
{
  if( _rpm )
  {
     _rpm_target = ECU_IDLE_RPM;
  }
  else
  {
    _rpm_target = 0;
  }
}

void ECU::calculate_RPM( unsigned long aNow )
{
  Periods thePeriods( _injector.read_periods() );
  
  if( _is_Running )
  {
    unsigned short theRpm( ( aNow - _last_sample_usecs ) * 60. /
                             thePeriods._usecs * thePeriods._count / ( ECU_SAMPLING_US / 1000000. ) );

    _rpm_average.push( theRpm / 4. );
  }
  else
  {
    _rpm_average.push( 0 );
  }

  _rpm = _rpm_average.average();

  _is_Running = false;
}

