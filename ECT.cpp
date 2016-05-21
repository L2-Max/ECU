#include "ECT.h"

#include <Arduino.h>

#define PIN_ECT A1

#define ECT_SAMPLING_MS 100

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

    _temperature = 147. - ( ( analogRead( PIN_ECT ) / 1023. ) * 215. );
  }
}

