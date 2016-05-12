#include "Average.h"

#include <Arduino.h>

template< class T >
Average< T >::Average( unsigned short aSize ) :
  _values( new T[ aSize ] ), _size( aSize ), _count( 0 ), _I_value( 0 ), _sum( 0 )
{
  memset( _values, 0, sizeof( *_values ) * _size );
}

template< class T >
Average< T >::~Average() {}

template< class T >
void Average< T >::push( T aValue )
{
  if( _count )
  {
     _sum -= _values[ _I_value ];
  }
  
  _values[ _I_value ] = aValue;

  if( _count < _size )
  {
    ++_count;
  }

  if( ++_I_value >= _size )
  {
    _I_value = 0;
  }

  _sum += aValue;
}

template< class T >
T Average< T >::average()const
{
  T ret( 0 );

  if( _count )
  {
    ret = _sum / _count;
  }
  
  return ret;
}

template class Average< unsigned short >;
