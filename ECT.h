#ifndef __ECT_H__
#define __ECT_H__

struct Voltage_Temperature_Pair
{
  float _voltage;
  short _temperature;
};

struct ECT
{
  ECT();
  ~ECT();

  void read( unsigned long aNow_MS );
  short temperature_approximate( float aVoltage )const;

  unsigned long _last_sample;

  short _temperature;
  static const Voltage_Temperature_Pair _temperature_approx_table[];
};

#endif
