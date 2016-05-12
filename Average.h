#ifndef __AVERAGE_H__
#define __AVERAGE_H__

template< class T >
class Average
{
public:
  Average( unsigned short aSize );
  ~Average();

  void push( T aValue );
  T average()const;

private:
  T* _values;

  T _sum;
  
  unsigned short _size;
  unsigned short _count;
  unsigned short _I_value;
};

#endif
