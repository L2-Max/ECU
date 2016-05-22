#include "ECT.h"

#include <Arduino.h>

#define PIN_ECT A1

#define ECT_SAMPLING_MS 100

const Voltage_Temperature_Pair ECT::_temperature_approx_table[] =
{
  { .0, 150 },
  { .305, 122 },
  { .45, 110 },
  { .55, 100 },
  { .65, 90 },
  { .81, 80 },
  { 1.13, 70 },
  { 1.426, 60 },
  { 1.67, 50 },
  { 2.02, 40 },
  { 3.05, 20 },
  { 5., -30 }
};

ECT::ECT() : _last_sample( 0 ), _temperature( 0 )
{
  pinMode( PIN_ECT, INPUT );
}

ECT::~ECT()
{}

void ECT::read( unsigned long aNow_MS )
{
  if( aNow_MS >= _last_sample )
  {
    _last_sample = ( aNow_MS + ECT_SAMPLING_MS );

    _temperature = temperature_approximate( analogRead( PIN_ECT ) / 1023. * 4.999 );
  }
}

short ECT::temperature_approximate( float aVoltage )const
{
  short ret( 0 );

  for( int i( 0 ); i < sizeof( _temperature_approx_table ) / sizeof( *_temperature_approx_table ) - 1; ++i )
  {
    if( aVoltage >= _temperature_approx_table[ i ]._voltage && aVoltage < _temperature_approx_table[ i + 1 ]._voltage )
    {
      ret = _temperature_approx_table[ i ]._temperature - 
            ( ( aVoltage - _temperature_approx_table[ i ]._voltage ) /
              ( ( _temperature_approx_table[ i + 1 ]._voltage - _temperature_approx_table[ i ]._voltage ) /
                ( _temperature_approx_table[ i ]._temperature - _temperature_approx_table[ i + 1 ]._temperature ) ) );
      break;
    }
  }

  return ret;
}

