#include "MAP.h"

#include <Arduino.h>

#define PIN_MAP A2

#define MAP_SAMPLING_MS 100

MAP::MAP() : _last_sample( 0 ), _pressure( 0 )
{
  pinMode( PIN_MAP, INPUT );
}

MAP::~MAP()
{}

void MAP::read( unsigned long aNow_MS )
{
  if( aNow_MS >= _last_sample )
  {
    _last_sample = ( aNow_MS + MAP_SAMPLING_MS );

    _pressure = ( analogRead( PIN_MAP ) / 1023. * 175. );
  }
}
