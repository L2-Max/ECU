#include "ECU.h"

#include "EEPROM_Defs.h"

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ECU_SAMPLING_MS 50

#define DISPLAY_INTERVAL_MS 666

#define ECU_POWER_PIN 4

LiquidCrystal_I2C g_lcd( 0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );

ECU* g_ECU( 0 );

void setup()
{
  Serial.begin( 115200 );

#ifdef ECU_SYSTEM_RESET
  for( int i( 0 ); i < EEPROM.length() ; i++ )
  {
    EEPROM.write( i, 0 );
  }
#endif
  
  g_lcd.begin(16,2);
  g_lcd.home ();
  g_lcd.print( "   Hello!" ); 
  
  g_lcd.setCursor( 0, 1 );

  g_lcd.print( "     l2ECU" );

  delay( 2000 );

  g_lcd.home();
  g_lcd.clear();

  pinMode( 8, OUTPUT );
  digitalWrite( 8, LOW );
}

unsigned long g_LastMS( 0 );
unsigned long g_NextMS( 0 );

unsigned long g_Display_MS( 0 );

bool g_Reset( false );

void loop()
{
  if( !g_ECU )
  {
    g_ECU = new ECU();
  }
  
  for( uint32_t I_cycle( 0 ); !g_Reset ; ++I_cycle )
  {
    unsigned long theNow( millis() );

    g_ECU->run( theNow );

    if( theNow >= g_NextMS )
    {
      g_NextMS = ( theNow + DISPLAY_INTERVAL_MS - ( theNow - g_NextMS ) );
      g_LastMS = millis();

      if( Serial.available() )
      {
        String theStr( Serial.readString() );
        String theK( theStr.substring( 0, 1 ) );
        float theVal( theStr.substring( 1 ).toFloat() );

        if( theK == "p" )
        {
          g_ECU->_iac._Kp = theVal;
        }
        else if( theK == "i" )
        {
          g_ECU->_iac._Ki = theVal;
        }
        else if( theK == "d" )
        {
          g_ECU->_iac._Kd = theVal;
        }
      }
      
      Serial.println();

      Serial.print( F( "SYS" ) );
      
      Serial.print( F( "\tCLK " ) );
      Serial.println( float( I_cycle ) / ( DISPLAY_INTERVAL_MS - g_Display_MS ) );

      Serial.print( F( "ECU" ) );
      
      Serial.print( F( "\tRPM " ) );
      Serial.print( g_ECU->_rpm );

      Serial.print( F( "\tRPM_TRGT " ) );
      Serial.print( g_ECU->_rpm_target );

      Serial.print( F( "\tSTATE " ) );
      Serial.print( g_ECU->_state );

      Serial.print( F( "\tRPM_MAX " ) );
      Serial.println( g_ECU->_rpm_max );

      Serial.print( F( "\tI_LP " ) );
      Serial.print( g_ECU->_last_idle_position );

      Serial.print( F( "\tI_TMR " ) );
      Serial.println( g_ECU->_idle_timer );
      
      Serial.print( F( "INJ" ) );

      Serial.print( F( "\tPOA " ) );
      Serial.println( g_ECU->_periods_on_average.average() );

      Serial.print( F( "IAC" ) );
      
      Serial.print( "\tKp " );
      Serial.print( g_ECU->_iac._Kp );

      Serial.print( "\tKi " );
      Serial.print( g_ECU->_iac._Ki );

      Serial.print( "\tKd " );
      Serial.println( g_ECU->_iac._Kd );
      
      Serial.print( F( "\tE " ) );
      Serial.print( g_ECU->_iac._last_error );
      
      Serial.print( F( "\tI " ) );
      Serial.print( g_ECU->_iac._integral );

      Serial.print( F( "\tD " ) );
      Serial.println( g_ECU->_iac._derivative );

      Serial.print( F( "\tPOS " ) );
      Serial.println( static_cast< short >( g_ECU->_iac._stepper.currentPosition() ) );

      Serial.print( F( "TPS" ) );

      Serial.print( F( "\tV " ) );
      Serial.print( g_ECU->_tps._value_average.average() );

      Serial.print( F( "\tV_C " ) );
      Serial.print( g_ECU->_tps._value_closed );

      Serial.print( F( "\tSTATE " ) );
      Serial.println( g_ECU->_tps._isOpen );

      Serial.print( F( "\tO " ) );
      Serial.print( g_ECU->_tps._counter_open );

      Serial.print( F( "\tC " ) );
      Serial.println( g_ECU->_tps._counter_close );

      Serial.print( F( "ECT" ) );

      Serial.print( F( "\tTEMP " ) );
      Serial.println( g_ECU->_ect._temperature );

      Serial.print( F( "MAP" ) );
      
      Serial.print( F( "\tP " ) );
      Serial.println( g_ECU->_map._pressure );

      Serial.print( F( "VSS" ) );

      Serial.print( F( "\tSPEED " ) );
      Serial.print( g_ECU->_vss._speed );

      Serial.print( F( "\tKM " ) );
      Serial.println( g_ECU->_vss._meters / 1000., 3 );
      
      Serial.print( F( "FLOW" ) );
      
      Serial.print( F( "\tL/H " ) );
      Serial.print( float( g_ECU->_periods_on_average.average() * g_ECU->_rpm * 2 ) * .000000123, 3 );

      Serial.print( F( "\tL_TOT " ) );
      Serial.println( float( g_ECU->_total_periods_on * 4 ) * ( .000000123 / 60. ), 3 );

      /////////////////////////////////////////////////////////
      /*g_lcd.home();
      g_lcd.clear();

      g_lcd.print( F( "c" ) );
      g_lcd.print( I_cycle / ( DISPLAY_INTERVAL_MS - g_Display_MS ) );
      
      g_lcd.print( F( " tps" ) );
      g_lcd.print( g_ECU->_tps._value );

      g_lcd.setCursor( 0, 1 );

      g_lcd.print( F( "s" ) );
      g_lcd.print( g_ECU->_vss._speed );

      g_lcd.print( F( " m" ) );
      g_lcd.print( g_ECU->_vss._meters );

      g_lcd.print( F( " r" ) );
      g_lcd.print( g_ECU->_rpm );*/

      g_Display_MS = ( millis() - g_LastMS );
      
      I_cycle = 0;
    }
  }

  delete g_ECU;
  g_ECU = 0;

  g_Reset = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
#define ECU_RPM_IDLE 1000
#define ECU_RPM_WARMUP 1800
#define ECU_RPM_IDLE_TOLERANCE 25

ECU::ECU() :
  _iac( *this ), _injector( *this ), _tps( *this ), _rpm( 0 ), _rpm_target( ECU_RPM_IDLE ), _next_sample_ms( 0 ),
  _rpm_average( 3 ), _state( sInit ), _rpm_zero_counter( 0 ), _state_handler( &ECU::init ),
  _last_idle_position( 0 ), _periods_on_average( 1 ), _periods_on_zero_counter( 0 ),
  _rpm_max( 0 ), _total_periods_on( 0 ), _idle_timer( 0 )
{
  pinMode( ECU_POWER_PIN, OUTPUT );
  
  attachInterrupt( digitalPinToInterrupt( PIN_INJECTOR ), ECU::Iterrupt_Injector_Change, CHANGE );
  attachInterrupt( digitalPinToInterrupt( PIN_VSS ), ECU::Iterrupt_VSS_Change, CHANGE );

#ifdef ECU_SYSTEM_RESET
  _iac.reset();
#endif

  EEPROM.get( ECU_MPG_TOTAL_ADDR, _total_periods_on );
  EEPROM.get( ECU_MAX_RPM, _rpm_max );
}

ECU::~ECU()
{
  EEPROM.put( ECU_MPG_TOTAL_ADDR, _total_periods_on );
  EEPROM.put( ECU_MAX_RPM, _rpm_max );
  
  digitalWrite( ECU_POWER_PIN, LOW );
}

void ECU::Iterrupt_Injector_Change()
{
  g_ECU->_injector.read();
}

void ECU::Iterrupt_VSS_Change()
{
  g_ECU->_vss.read();
}

void ECU::run( unsigned long aNow_MS )
{
  if( aNow_MS >= _next_sample_ms )
  {
    _next_sample_ms = ( aNow_MS + ECU_SAMPLING_MS );
    
    read_RPM( aNow_MS );
    read_Fueling( aNow_MS );

    _vss.read_Speed( aNow_MS );

    _tps.read( aNow_MS );
    _ect.read( aNow_MS );
    _map.read( aNow_MS );

    calculate_target_RPM( aNow_MS );

    ( this->*_state_handler )();

    _iac.control_RPM( aNow_MS );
  }

  static uint8_t g_IAC_Run( 0 );

  if( !--g_IAC_Run )
  {
     _iac.run();
  }
}

void ECU::init()
{
  _state = sInit;
  
  if( _iac._state == IAC::sReady )
  {
    _state_handler = &ECU::wait_for_engine_starting;
  }
}

void ECU::uninit()
{
  _state = sUninit;
  
  if( _iac._state == IAC::sReady )
  {
    g_Reset = true;
  }
}

void ECU::wait_for_engine_starting()
{
  _state = sWforEstarting;
  
  if( _rpm > 500 )
  {
    _state_handler = &ECU::engine_idling;
    
    digitalWrite( ECU_POWER_PIN, HIGH );
  }
}

void ECU::engine_idling()
{
  if( _rpm )
  {
    if( !_tps._isOpen && _vss._speed < 10 )
    {
      _state = sIdling;

      if( !_idle_timer )
      {
        _iac.Set_Enabled( true );
  
        //if( _rpm >= ( _rpm_target - ECU_RPM_IDLE_TOLERANCE ) && _rpm <= ( _rpm_target + ECU_RPM_IDLE_TOLERANCE ) )
        {
          _last_idle_position = _iac._stepper.currentPosition();
        }
      }
      else
      {
        --_idle_timer;
      }
    }
    else
    {
      _state = sRunning;

      _iac.Set_Enabled( false );
    }

    if( _state == sRunning )
    {
      if( _tps._isPartOpen || _vss._speed > 10 )
      {
        _idle_timer = ( 2000. / ECU_SAMPLING_MS );

        if( _rpm < ( ECU_RPM_IDLE + 1000 ) )
        {
          _iac.stepTo( _last_idle_position + 100 );
        }
        else
        {
          _iac.stepTo( _last_idle_position + 100 );
        }
      }
    }
  }
  else
  {
    _iac.reset();
  
    _state_handler = &ECU::uninit;
  }
}

void ECU::calculate_target_RPM( unsigned long aNow )
{
  if( _ect._temperature > 35 )
  {
    _rpm_target = ECU_RPM_IDLE;
  }
  else if( _ect._temperature > 0 )
  {
    _rpm_target = ( ECU_RPM_WARMUP - ( ( ECU_RPM_WARMUP - ECU_RPM_IDLE ) / 46.6 * _ect._temperature ) );
  }
  else
  {
    _rpm_target = ECU_RPM_WARMUP;
  }
}

void ECU::read_RPM( unsigned long aNow )
{
  Periods thePeriods;
  
  _injector.read_periods( thePeriods );
  
  if( thePeriods._count )
  {
    unsigned short theRpm( ( 1000000. * 120 ) / thePeriods._usecs * thePeriods._count );

    if( theRpm > 500 )
    {
      _rpm_average.push( theRpm );
      _rpm = _rpm_average.average();
  
      _rpm_zero_counter = ( 1000 / ECU_SAMPLING_MS );
  
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

