#include "TPS.h"

#include "EEPROM_Defs.h"

#define PIN_TPS A0

#define TPS_SAMPLING_MS 10

#define TPS_JITTER 20.
#define TPS_JITTER_COUNTER  ( 1000. / TPS_SAMPLING_MS )
#define TPS_JITTER_COUNTER_MAX  ( TPS_JITTER_COUNTER + ( 1000. / TPS_SAMPLING_MS ) )

TPS::TPS( ECU& anECU ) :
  _ecu( anECU ), _next_sample_ms( 0 ), _value_closed( 0 ), _value_jitter_counter( 0 ), _value_average( 100. / TPS_SAMPLING_MS ),
  _counter_close( 0 ), _counter_open( 0 ), _isOpen( false )
{
  EEPROM.get( ECU_TPS_CLOSED_MV, _value_closed );

  if( !_value_closed )
  {
    _value_closed = 600;
  }
  else
  {
    _value_closed += 30;
  }

  _value_closed = 503;
  
  pinMode( PIN_TPS, INPUT );
}

TPS::~TPS()
{
  EEPROM.put( ECU_TPS_CLOSED_MV, _value_closed );
}

void TPS::read( unsigned long aNow_MS )
{
  if( aNow_MS >= _next_sample_ms )
  {
    _next_sample_ms = ( aNow_MS + TPS_SAMPLING_MS );

    _value_last = analogRead( PIN_TPS ) / 1023. * 5000.;

    _value_average.push( _value_last );
    //_value_average.push( 350. + analogRead( PIN_TPS ) / 1023. * 350. + rand() % 15 );

    if( _value_average.average() > 400 && _value_average.average() < 600 )
    {
      if( abs( _value_average.average() - _value_last ) < TPS_JITTER )
      {
        if( _value_jitter_counter < TPS_JITTER_COUNTER_MAX )
        {
          ++_value_jitter_counter;
        }
      }
      else
      {
        _value_jitter_counter = 0;
      }
  
      if( _value_jitter_counter >= TPS_JITTER_COUNTER )
      {
        if( _value_average.average() < _value_closed )
        {
          //--_value_closed;
        }
        else if( _value_average.average() > _value_closed )
        {
          if( _value_jitter_counter < TPS_JITTER_COUNTER_MAX )
          {
            //++_value_closed;
          }
        }
      }
    }

    _isOpen = ( _value_average.average() > ( _value_closed + 10. ) );

    if( _isOpen )
    {
      ++_counter_open;
    }
    else
    {
      ++_counter_close;
    }
  }
}

unsigned long TPS::read_Vcc()
{
  /*#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay( 2 );
  
  ADCSRA |= _BV( ADSC );
  
  while( bit_is_set( ADCSRA, ADSC ) );

  uint8_t low( ADCL );
  uint8_t high( ADCH );

  unsigned long ret( ( high << 8 ) | low );

  ret = ( 1125300L / ret );
  
  return ret;*/
  return 0;
}
  
