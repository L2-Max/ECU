#include "ECU.h"

#include "EEPROM_Defs.h"

//#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ECU_SAMPLING_MS 50

#define DISPLAY_INTERVAL_MS 900

#define ECU_POWER_PIN 4

//LiquidCrystal_I2C g_lcd( 0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );

ECU* g_ECU( 0 );

void setup()
{
  Serial.begin(19200);

  for( int i( 0 ); i < EEPROM.length() ; i++ )
  {
    //EEPROM.write( i, 0 );
  }
  
/*  g_lcd.begin(16,2);

  g_lcd.home ();
  g_lcd.print( "   Hello!" );  
  g_lcd.setCursor( 0, 1 );

  g_lcd.print( "     l2ECU" );

  delay( 1000 );

  g_lcd.home();
  g_lcd.clear();

  pinMode( 8, OUTPUT );
  digitalWrite( 8, LOW );*/

  pinMode( 5, OUTPUT );
}

unsigned long g_Cycles( 0 );

unsigned long g_LastMS( 0 );
unsigned long g_NextMS( 0 );

unsigned long g_Display_MS( 0 );

bool g_Reset( false );

void simulator()
{
  static int g_PinState( HIGH );
  static volatile unsigned long g_NextHIGH( 0 );
  static volatile unsigned long g_HIGHCount( 0 );

  static unsigned long g_Stimer( 300 );
  static unsigned short g_Delay( 19000 );
  static unsigned short g_Off_Delay( 500 );
  
  unsigned long theNow( micros() );

  if( g_Off_Delay )
  {
    if( g_PinState == HIGH && ( theNow - g_NextHIGH ) >= g_Delay )
    {
      if( g_Stimer )
      {
        if( !--g_Stimer )
        {
          g_Delay = 29990;
        }
      }
      
      g_NextHIGH = theNow;
      
      ++g_HIGHCount;
      
      g_PinState = LOW;
      
      digitalWrite( 5, g_PinState );
    }
    else if( g_PinState == LOW && ( theNow - g_NextHIGH ) >= 4000 )
    {
      g_PinState = HIGH;
      
      digitalWrite( 5, g_PinState );

      --g_Off_Delay;
    }
  }
}

void loop()
{
  if( !g_ECU )
  {
    g_ECU = new ECU();
  }
  
  for( ; !g_Reset ; )
  {
    //simulator();
    
    unsigned long theNow( millis() );

    g_ECU->run( theNow );

    if( theNow >= g_NextMS )
    {
      g_NextMS = ( theNow + DISPLAY_INTERVAL_MS - ( theNow - g_NextMS ) );
      g_LastMS = millis();
      
      //g_lcd.home();
      //g_lcd.clear();

      Serial.print( "c" );
      Serial.print( g_Cycles / ( DISPLAY_INTERVAL_MS - g_Display_MS ) );

      Serial.print( " e" );
      Serial.print( g_ECU->_iac._last_error );
      
      Serial.print( " i" );
      Serial.print( g_ECU->_iac._integral );

      Serial.print( " d" );
      Serial.print( g_ECU->_iac._derivative );

      Serial.print( " r_m" );
      Serial.print( g_ECU->_rpm_max );

      Serial.println();
      
      //g_lcd.setCursor( 0, 1 );

      Serial.print( "r" );
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

      Serial.print( "pw" );
      Serial.print( g_ECU->_periods_on_average.average() );

      Serial.print( " lh" );
      Serial.print( float( g_ECU->_periods_on_average.average() * g_ECU->_rpm * 2 ) * .000000123, 3 );

      Serial.print( " lt" );
      Serial.print( float( g_ECU->_total_periods_on ) * ( .000000123 / 60. ), 3 );
      
      Serial.println();

      g_Display_MS = ( millis() - g_LastMS );
      
      g_Cycles = 0;
    }

    ++g_Cycles;
  }

  delete g_ECU;
  g_ECU = 0;

  g_Reset = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
#define ECU_IDLE_RPM 1000
#define ECU_STARTUP_RPM 1500
#define ECU_RPM_TOLERANCE 25

ECU::ECU() :
  _iac( *this ), _injector( *this ), _rpm( 0 ), _rpm_target( ECU_IDLE_RPM ), _last_sample_usecs( 0 ),
  _rpm_average( 3 ), _state( sInit ), _rpm_zero_counter( 0 ), _state_handler( &ECU::init ),
  _last_idle_steps( 0 ), _last_tps_state( _tps._isOpen ), _periods_on_average( 1 ), _periods_on_zero_counter( 0 ),
  _rpm_max( 0 ), _total_periods_on( 0 )
{
  pinMode( ECU_POWER_PIN, OUTPUT );
  
  attachInterrupt( digitalPinToInterrupt( PIN_INJECTOR ), ECU::Iterrupt_Injector_Change, CHANGE );

  if( !EEPROM.read( ECU_RESET_FLAG_ADDR ) )
  {
    _iac.reset();
  }

  EEPROM.get( ECU_MPG_TOTAL_ADDR, _total_periods_on );
}

ECU::~ECU()
{
  EEPROM.put( ECU_MPG_TOTAL_ADDR, _total_periods_on );
  
  digitalWrite( ECU_POWER_PIN, LOW );
}

void ECU::Iterrupt_Injector_Change()
{
  g_ECU->_injector.read();
}

void ECU::run( unsigned long aNow_MS )
{
  if( aNow_MS >= _last_sample_usecs )
  {
    read_RPM( aNow_MS );
    read_Fueling( aNow_MS );

    calculate_target_RPM( aNow_MS );

    _tps.read( aNow_MS );

    ( this->*_state_handler )();

    _iac.control_RPM( aNow_MS );

    _last_sample_usecs = ( aNow_MS + ECU_SAMPLING_MS );
  }

  _iac.run();
}

void ECU::init()
{
  _state = sInit;
  
  if( _iac._state == IAC::sReady )
  {
    _state_handler = &ECU::wait_for_engine_starting;
  
    EEPROM.update( ECU_RESET_FLAG_ADDR, 1 );
  }
}

void ECU::uninit()
{
  _state = sUninit;
  
  if( _iac._state == IAC::sReady )
  {
    EEPROM.update( ECU_RESET_FLAG_ADDR, 1 );

    g_Reset = true;
  }
}

void ECU::wait_for_engine_starting()
{
  _state = sWforEstarting;
  
  if( _rpm )
  {
    _state_handler = &ECU::wait_for_engine_gets_startup_rpm;
  }
}

void ECU::wait_for_engine_gets_startup_rpm()
{
  _state = sWforEgetsSrpm;
  
  if( _rpm )
  {
    if( _rpm > ECU_STARTUP_RPM )
    {
       _state_handler = &ECU::wait_for_engine_idling_after_startup;
    }
    else
    {
      EEPROM.update( ECU_RESET_FLAG_ADDR, 0 );
      
      _iac.step( 100 );
    }
  }
}

void ECU::wait_for_engine_idling_after_startup()
{
  _state = sWforEidlingAstartup;
  
  if( _rpm )
  {
    if( _rpm >= ( ECU_IDLE_RPM - 500 ) && _rpm <= ( ECU_IDLE_RPM + 200 ) )
    {
      digitalWrite( ECU_POWER_PIN, HIGH );

      _state_handler = &ECU::engine_idling;
    }
    else
    {
      _iac.step( -100 );
    }
  }
  else
  {
    _iac.reset();

    _state_handler = &ECU::uninit;
  }
}

void ECU::engine_idling()
{
  _state = sIdling;
  
  if( _rpm )
  {
    if( !_tps._isOpen )
    {
      if( _rpm >= ( ECU_IDLE_RPM - 500 ) && _rpm <= ( ECU_IDLE_RPM + 200 ) )
      {
        _state = sIdling;

        if( _rpm >= ( ECU_IDLE_RPM - 50 ) && _rpm <= ( ECU_IDLE_RPM + 50 ) )
        {
          _last_idle_steps = _iac._stepper.currentPosition();
        }
      }

      if( _last_tps_state != _tps._isOpen )
      {
         _iac.stepTo( _last_idle_steps );
      }
    }
    else if( _tps._isOpen )
    {
      _state = sRunning;

      if( _last_tps_state != _tps._isOpen )
      {
        if( _last_idle_steps )
        {
          _iac.stepTo( _last_idle_steps + 800 );
        }
      }
    }

    _last_tps_state = _tps._isOpen;
  }
  else
  {
    _iac.reset();

    _state_handler = &ECU::uninit;
  }
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
  Periods thePeriods;
  
  _injector.read_periods( thePeriods );
  
  if( thePeriods._count )
  {
    unsigned short theRpm( ( 1000000. * 30 ) / thePeriods._usecs * thePeriods._count );

    if( theRpm > 200 )
    {
      _rpm_average.push( theRpm );
      _rpm = _rpm_average.average();
  
      _rpm_zero_counter = ( 1000 / ECU_SAMPLING_MS / 2 );
  
      if( _rpm > _rpm_max )
      {
        _rpm_max = _rpm;
      }
    }
  }
  else
  {
    if( !_rpm_zero_counter )
    {
      _rpm = 0;
    }
    else
    {
      --_rpm_zero_counter;
    }
  }
}

void ECU::read_Fueling( unsigned long aNow )
{
  Periods thePeriods;

  _injector.read_periods_on( thePeriods );
  
  if( thePeriods._count )
  {
    _periods_on_average.push( float( thePeriods._usecs ) / thePeriods._count );
    _total_periods_on += thePeriods._usecs;

    _periods_on_zero_counter = ( 1000 / ECU_SAMPLING_MS / 2 );
  }
  else
  {
    if( !_periods_on_zero_counter )
    {
      _periods_on_average.push( 0 );
    }
    else
    {
      --_periods_on_zero_counter;
    }
  }
}

