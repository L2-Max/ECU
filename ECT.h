#ifndef __ECT_H__
#define __ECT_H__

struct ECT
{
  ECT();
  ~ECT();

  void read( unsigned long aNow_MS );

  unsigned long _last_sample;

  short _temperature;
};

#endif
