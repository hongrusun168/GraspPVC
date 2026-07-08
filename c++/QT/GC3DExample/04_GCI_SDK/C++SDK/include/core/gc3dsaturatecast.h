/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3DSATURATECAST_H
#define GC3DSATURATECAST_H
#include<string>
namespace gc3d {

template<typename _Tp> static inline _Tp saturate_cast(char v)    { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(short v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(unsigned v) { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(int v)      { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(float v)    { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(double v)   { return _Tp(v); }

}
#endif // GC3DSATURATECAST_H
