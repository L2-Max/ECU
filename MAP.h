#ifndef __MAP_H__
#define __MAP_H__

struct MAP
{
  MAP();
  ~MAP();

  void read( unsigned long aNow_MS );

  unsigned long _last_sample;

  short _pressure;
};

#endif
