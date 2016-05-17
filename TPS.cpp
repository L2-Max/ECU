#include "TPS.h"

#include <Arduino.h>

#define PIN_TPS A0

#define TPS_SAMPLING_MS 100000
#define TPS_VALUE_CLOSED 520.

TPS::TPS() : _last_sample( 0 ), _value( 0 ), _value_average( 3 )
{
  pinMode( PIN_TPS, INPUT );
  //pinMode( A1, INPUT );
  //analogReference( EXTERNAL );
}

void TPS::read( unsigned long aNow )
{
  if( ( aNow - _last_sample ) >= TPS_SAMPLING_MS )
  {
    _last_sample = aNow;

    _value_average.push( analogRead( PIN_TPS ) / 1023. * read_Vcc() );
    _value = _value_average.average();

    _isOpen = ( _value > TPS_VALUE_CLOSED );
  }
}

unsigned long TPS::read_Vcc()
{
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
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
  
  return ret;
}
  
