/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3DCORETYPED_H
#define GC3DCORETYPED_H
#include<vector>
#include "gc3dsaturatecast.h"
#include<cmath>
#include<iostream>
#include <algorithm>
#include<assert.h>
#include <limits.h>
#ifdef GCI_OS_WIN32
#define  DLLEXPORT __declspec(dllexport)
#else
#define  DLLEXPORT
#endif

//常数定义
#define GCI_PI   3.1415926535897932384626433832795
#define GCI_2PI  6.283185307179586476925286766559
#define GCI_ZMAX INT_MAX
#define GCI_ZMIN INT_MIN
#define GCI_Epsilon   1e-11

typedef unsigned char uchar;
typedef unsigned short ushort;
namespace gc3d {



//////////////////////////////// GSize_ ////////////////////////////////

/** @brief Template class for specifying the size of an image or rectangle.

The class includes two members called width and height. The structure can be converted to and from
the old GCILAB structures . The same set of arithmetic and comparison
operations as for Point_ is available.

GCILAB defines the following GSize_\<\> aliases:
@code
    typedef GSize_<int> Size2i;
    typedef Size2i Size;
    typedef GSize_<float> Size2f;
@endcode
*/
template<typename _Tp> class GSize_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GSize_();
    GSize_(_Tp _width, _Tp _height);
    GSize_(const GSize_& sz);
    GSize_(GSize_&& sz)  ;

    GSize_& operator = (const GSize_& sz);
    GSize_& operator = (GSize_&& sz)  ;
    //! the area (width*height)
    _Tp area() const;
    //! aspect ratio (width/height)
    double aspectRatio() const;
    //! true if empty
    bool empty() const;

    //! conversion of another data type.
    template<typename _Tp2> operator GSize_<_Tp2>() const;

    _Tp width; //!< the width
    _Tp height; //!< the height
};

typedef GSize_<int> Size2i;
typedef GSize_<float> Size2f;
typedef GSize_<double> Size2d;
typedef Size2i Size;

template<typename _Tp> class DLLEXPORT  GPoint_
{
    public:
    typedef _Tp value_type;

    //! default constructor
    GPoint_();
    GPoint_(_Tp _x, _Tp _y);
    GPoint_(const GPoint_& pt);
    GPoint_(GPoint_&& pt) ;


    GPoint_& operator = (const GPoint_& pt);
    GPoint_& operator = (GPoint_&& pt) ;


    //! conversion to another data type
    template<typename _Tp2> operator GPoint_<_Tp2>() const;

    //! dot product
    _Tp dot(const GPoint_& pt) const;
    //! dot product computed in double-precision arithmetics
    double ddot(const GPoint_& pt) const;
    //! cross-product
    double cross(const GPoint_& pt) const;

    _Tp x; //!< x coordinate of the point
    _Tp y; //!< y coordinate of the point
};

typedef GPoint_<int> GPointi;
typedef GPoint_<float> GPointf;
typedef GPoint_<double> GPointd;
typedef GPointi GPoint;


//////////////////////////////// GPoint3_ ////////////////////////////////

/** @brief Template class for 3D points specified by its coordinates `x`, `y` and `z`.

An instance of the class is interchangeable with the C structure GCIPoint2D32f . Similarly to
Point_ , the coordinates of 3D points can be converted to another type. The vector arithmetic and
comparison operations are also supported.

The following GGPoint3_\<\> aliases are available:
@code
    typedef GGPoint3_<int> Point3i;
    typedef GGPoint3_<float> Point3f;
    typedef GGPoint3_<double> Point3d;
@endcode
@see gc3d::Point3i, gc3d::GPoint3f and gc3d::GPoint3d
*/
template<typename _Tp> class GPoint3_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GPoint3_();
    GPoint3_(_Tp _x, _Tp _y, _Tp _z);
    GPoint3_(const GPoint3_& pt);
    GPoint3_(GPoint3_&& pt);
    explicit GPoint3_(const GPoint_<_Tp>& pt);

    GPoint3_& operator = (const GPoint3_& pt);
    GPoint3_& operator = (GPoint3_&& pt) ;
    //! conversion to another data type
    template<typename _Tp2> operator GPoint3_<_Tp2>() const;

    //! dot product
    _Tp dot(const GPoint3_& pt) const;
    //! dot product computed in double-precision arithmetics
    double ddot(const GPoint3_& pt) const;
    //! cross product of the 2 3D points
    GPoint3_ cross(const GPoint3_& pt) const;
    _Tp x; //!< x coordinate of the 3D point
    _Tp y; //!< y coordinate of the 3D point
    _Tp z; //!< z coordinate of the 3D point
};

typedef GPoint3_<int> GPoint3i;
typedef GPoint3_<float> GPoint3f;
typedef GPoint3_<double> GPoint3d;
typedef GPoint3i GPoint3;

//////////////////////////////// GRect_ ////////////////////////////////

/** @brief Template class for 2D rectangles

described by the following parameters:
-   Coordinates of the top-left corner. This is a default interpretation of GRect_::x and GRect_::y
    in GCILAB. Though, in your algorithms you may count x and y from the bottom-left corner.
-   Rectangle width and height.

GCILAB typically assumes that the top and left boundary of the rectangle are inclusive, while the
right and bottom boundaries are not. For example, the method GRect_::contains returns true if

\f[x  \leq pt.x < x+width,
      y  \leq pt.y < y+height\f]

Virtually every loop over an image ROI in GCILAB (where ROI is specified by GRect_\<int\> ) is
implemented as:
@code
    for(int y = roi.y; y < roi.y + roi.height; y++)
        for(int x = roi.x; x < roi.x + roi.width; x++)
        {
            // ...
        }
@endcode
In addition to the class members, the following operations on rectangles are implemented:
-   \f$\texttt{rect} = \texttt{rect} \pm \texttt{point}\f$ (shifting a rectangle by a certain offset)
-   \f$\texttt{rect} = \texttt{rect} \pm \texttt{size}\f$ (expanding or shrinking a rectangle by a
    certain amount)
-   rect += point, rect -= point, rect += size, rect -= size (augmenting operations)
-   rect = rect1 & rect2 (rectangle intersection)
-   rect = rect1 | rect2 (minimum area rectangle containing rect1 and rect2 )
-   rect &= rect1, rect |= rect1 (and the corresponding augmenting operations)
-   rect == rect1, rect != rect1 (rectangle comparison)

This is an example how the partial ordering on rectangles can be established (rect1 \f$\subseteq\f$
rect2):
@code
    template<typename _Tp> inline bool
    operator <= (const GRect_<_Tp>& r1, const GRect_<_Tp>& r2)
    {
        return (r1 & r2) == r1;
    }
@endcode

*/
template<typename _Tp> class GRect_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GRect_();
    GRect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
    GRect_(const GRect_& r);
    GRect_(GRect_&& r)  ;
    GRect_(const GPoint_<_Tp>& org, const GSize_<_Tp>& sz);
    GRect_(const GPoint_<_Tp>& pt1, const GPoint_<_Tp>& pt2);

    GRect_& operator = ( const GRect_& r );
    GRect_& operator = ( GRect_&& r )  ;
    //! the top-left corner
    GPoint_<_Tp> tl() const;
    //! the bottom-right corner
    GPoint_<_Tp> br() const;

    //! size (width, height) of the rectangle
    GSize_<_Tp> size() const;
    //! area (width*height) of the rectangle
    _Tp area() const;
    //! true if empty
    bool empty() const;

    //! conversion to another data type
    template<typename _Tp2> operator GRect_<_Tp2>() const;

    //! checks whether the rectangle contains the point
    bool contains(const GPoint_<_Tp>& pt) const;

    _Tp x; //!< x coordinate of the top-left corner
    _Tp y; //!< y coordinate of the top-left corner
    _Tp width; //!< width of the rectangle
    _Tp height; //!< height of the rectangle
};

typedef GRect_<int> GRect2i;
typedef GRect_<float> GRect2f;
typedef GRect_<double> GRect2d;
typedef GRect2i GRect;



////////////////////////////////// 2D Point ///////////////////////////////

template<typename _Tp> inline
GPoint_<_Tp>::GPoint_()
    : x(0), y(0) {}

template<typename _Tp> inline
GPoint_<_Tp>::GPoint_(_Tp _x, _Tp _y)
    : x(_x), y(_y) {}

template<typename _Tp> inline
GPoint_<_Tp>::GPoint_(const GPoint_& pt)
    : x(pt.x), y(pt.y) {}

template<typename _Tp> inline
GPoint_<_Tp>::GPoint_(GPoint_&& pt)
    : x(std::move(pt.x)), y(std::move(pt.y)) {}


template<typename _Tp> inline
GPoint_<_Tp>& GPoint_<_Tp>::operator = (const GPoint_& pt)
{
    x = pt.x; y = pt.y;
    return *this;
}

template<typename _Tp> inline
GPoint_<_Tp>& GPoint_<_Tp>::operator = (GPoint_&& pt)
{
    x = std::move(pt.x); y = std::move(pt.y);
    return *this;
}

template<typename _Tp> template<typename _Tp2> inline
GPoint_<_Tp>::operator GPoint_<_Tp2>() const
{
    return GPoint_<_Tp2>(saturate_cast<_Tp2>(x), saturate_cast<_Tp2>(y));
}

template<typename _Tp> inline
_Tp GPoint_<_Tp>::dot(const GPoint_& pt) const
{
    return saturate_cast<_Tp>(x*pt.x + y*pt.y);
}

template<typename _Tp> inline
double GPoint_<_Tp>::ddot(const GPoint_& pt) const
{
    return (double)x*(double)(pt.x) + (double)y*(double)(pt.y);
}

template<typename _Tp> inline
double GPoint_<_Tp>::cross(const GPoint_& pt) const
{
    return (double)x*pt.y - (double)y*pt.x;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator += (GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator -= (GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator *= (GPoint_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator *= (GPoint_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator *= (GPoint_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator /= (GPoint_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator /= (GPoint_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    return a;
}

template<typename _Tp> static inline
GPoint_<_Tp>& operator /= (GPoint_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    return a;
}

template<typename _Tp> static inline
double norm(const GPoint_<_Tp>& pt)
{
    return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y);
}

template<typename _Tp> static inline
bool operator == (const GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return a.x == b.x && a.y == b.y;
}

template<typename _Tp> static inline
bool operator != (const GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return a.x != b.x || a.y != b.y;
}

template<typename _Tp> static inline
GPoint_<_Tp> operator + (const GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(a.x + b.x), saturate_cast<_Tp>(a.y + b.y) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator - (const GPoint_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(a.x - b.x), saturate_cast<_Tp>(a.y - b.y) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator - (const GPoint_<_Tp>& a)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(-a.x), saturate_cast<_Tp>(-a.y) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (const GPoint_<_Tp>& a, int b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(a.x*b), saturate_cast<_Tp>(a.y*b) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (int a, const GPoint_<_Tp>& b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(b.x*a), saturate_cast<_Tp>(b.y*a) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (const GPoint_<_Tp>& a, float b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(a.x*b), saturate_cast<_Tp>(a.y*b) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (float a, const GPoint_<_Tp>& b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(b.x*a), saturate_cast<_Tp>(b.y*a) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (const GPoint_<_Tp>& a, double b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(a.x*b), saturate_cast<_Tp>(a.y*b) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator * (double a, const GPoint_<_Tp>& b)
{
    return GPoint_<_Tp>( saturate_cast<_Tp>(b.x*a), saturate_cast<_Tp>(b.y*a) );
}

template<typename _Tp> static inline
GPoint_<_Tp> operator / (const GPoint_<_Tp>& a, int b)
{
    GPoint_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
GPoint_<_Tp> operator / (const GPoint_<_Tp>& a, float b)
{
    GPoint_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
GPoint_<_Tp> operator / (const GPoint_<_Tp>& a, double b)
{
    GPoint_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}



//////////////////////////////// 3D Point ///////////////////////////////

template<typename _Tp> inline
GPoint3_<_Tp>::GPoint3_()
    : x(0), y(0), z(0) {}

template<typename _Tp> inline
GPoint3_<_Tp>::GPoint3_(_Tp _x, _Tp _y, _Tp _z)
    : x(_x), y(_y), z(_z) {}

template<typename _Tp> inline
GPoint3_<_Tp>::GPoint3_(const GPoint3_& pt)
    : x(pt.x), y(pt.y), z(pt.z) {}

template<typename _Tp> inline
GPoint3_<_Tp>::GPoint3_(GPoint3_&& pt)
    : x(std::move(pt.x)), y(std::move(pt.y)), z(std::move(pt.z)) {}

template<typename _Tp> inline
GPoint3_<_Tp>::GPoint3_(const GPoint_<_Tp>& pt)
    : x(pt.x), y(pt.y), z(_Tp()) {}


template<typename _Tp> template<typename _Tp2> inline
GPoint3_<_Tp>::operator GPoint3_<_Tp2>() const
{
    return GPoint3_<_Tp2>(saturate_cast<_Tp2>(x), saturate_cast<_Tp2>(y), saturate_cast<_Tp2>(z));
}

template<typename _Tp> inline
GPoint3_<_Tp>& GPoint3_<_Tp>::operator = (const GPoint3_& pt)
{
    x = pt.x; y = pt.y; z = pt.z;
    return *this;
}

template<typename _Tp> inline
GPoint3_<_Tp>& GPoint3_<_Tp>::operator = (GPoint3_&& pt)
{
    x = std::move(pt.x); y = std::move(pt.y); z = std::move(pt.z);
    return *this;
}

template<typename _Tp> inline
_Tp GPoint3_<_Tp>::dot(const GPoint3_& pt) const
{
    return saturate_cast<_Tp>(x*pt.x + y*pt.y + z*pt.z);
}

template<typename _Tp> inline
double GPoint3_<_Tp>::ddot(const GPoint3_& pt) const
{
    return (double)x*pt.x + (double)y*pt.y + (double)z*pt.z;
}

template<typename _Tp> inline
GPoint3_<_Tp> GPoint3_<_Tp>::cross(const GPoint3_<_Tp>& pt) const
{
    return GPoint3_<_Tp>(y*pt.z - z*pt.y, z*pt.x - x*pt.z, x*pt.y - y*pt.x);
}


template<typename _Tp> static inline
GPoint3_<_Tp>& operator += (GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator -= (GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator *= (GPoint3_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator *= (GPoint3_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator *= (GPoint3_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator /= (GPoint3_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator /= (GPoint3_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    return a;
}

template<typename _Tp> static inline
GPoint3_<_Tp>& operator /= (GPoint3_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    return a;
}

template<typename _Tp> static inline
double norm(const GPoint3_<_Tp>& pt)
{
    return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y + (double)pt.z*pt.z);
}

template<typename _Tp> static inline
bool operator == (const GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

template<typename _Tp> static inline
bool operator != (const GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator + (const GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(a.x + b.x), saturate_cast<_Tp>(a.y + b.y), saturate_cast<_Tp>(a.z + b.z));
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator - (const GPoint3_<_Tp>& a, const GPoint3_<_Tp>& b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(a.x - b.x), saturate_cast<_Tp>(a.y - b.y), saturate_cast<_Tp>(a.z - b.z));
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator - (const GPoint3_<_Tp>& a)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(-a.x), saturate_cast<_Tp>(-a.y), saturate_cast<_Tp>(-a.z) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (const GPoint3_<_Tp>& a, int b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(a.x*b), saturate_cast<_Tp>(a.y*b), saturate_cast<_Tp>(a.z*b) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (int a, const GPoint3_<_Tp>& b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (const GPoint3_<_Tp>& a, float b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(a.x * b), saturate_cast<_Tp>(a.y * b), saturate_cast<_Tp>(a.z * b) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (float a, const GPoint3_<_Tp>& b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (const GPoint3_<_Tp>& a, double b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(a.x * b), saturate_cast<_Tp>(a.y * b), saturate_cast<_Tp>(a.z * b) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator * (double a, const GPoint3_<_Tp>& b)
{
    return GPoint3_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a) );
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator / (const GPoint3_<_Tp>& a, int b)
{
    GPoint3_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator / (const GPoint3_<_Tp>& a, float b)
{
    GPoint3_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
GPoint3_<_Tp> operator / (const GPoint3_<_Tp>& a, double b)
{
    GPoint3_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}



////////////////////////////////// Rect /////////////////////////////////

template<typename _Tp> inline
GRect_<_Tp>::GRect_()
    : x(0), y(0), width(0), height(0) {}

template<typename _Tp> inline
GRect_<_Tp>::GRect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height)
    : x(_x), y(_y), width(_width), height(_height) {}

template<typename _Tp> inline
GRect_<_Tp>::GRect_(const GRect_<_Tp>& r)
    : x(r.x), y(r.y), width(r.width), height(r.height) {}

template<typename _Tp> inline
GRect_<_Tp>::GRect_(GRect_<_Tp>&& r)
    : x(std::move(r.x)), y(std::move(r.y)), width(std::move(r.width)), height(std::move(r.height)) {}

template<typename _Tp> inline
GRect_<_Tp>::GRect_(const GPoint_<_Tp>& org, const GSize_<_Tp>& sz)
    : x(org.x), y(org.y), width(sz.width), height(sz.height) {}

template<typename _Tp> inline
GRect_<_Tp>::GRect_(const GPoint_<_Tp>& pt1, const GPoint_<_Tp>& pt2)
{
    x = std::min(pt1.x, pt2.x);
    y = std::min(pt1.y, pt2.y);
    width = std::max(pt1.x, pt2.x) - x;
    height = std::max(pt1.y, pt2.y) - y;
}

template<typename _Tp> inline
GRect_<_Tp>& GRect_<_Tp>::operator = ( const GRect_<_Tp>& r )
{
    x = r.x;
    y = r.y;
    width = r.width;
    height = r.height;
    return *this;
}

template<typename _Tp> inline
GRect_<_Tp>& GRect_<_Tp>::operator = ( GRect_<_Tp>&& r )
{
    x = std::move(r.x);
    y = std::move(r.y);
    width = std::move(r.width);
    height = std::move(r.height);
    return *this;
}

template<typename _Tp> inline
GPoint_<_Tp> GRect_<_Tp>::tl() const
{
    return GPoint_<_Tp>(x,y);
}

template<typename _Tp> inline
GPoint_<_Tp> GRect_<_Tp>::br() const
{
    return GPoint_<_Tp>(x + width, y + height);
}

template<typename _Tp> inline
GSize_<_Tp> GRect_<_Tp>::size() const
{
    return GSize_<_Tp>(width, height);
}

template<typename _Tp> inline
_Tp GRect_<_Tp>::area() const
{
    const _Tp result = width * height;
//    std::assert(!std::numeric_limits<_Tp>::is_integer
//                 || width == 0 || result / width == height); // make sure the result fits in the return value
    return result;
}

template<typename _Tp> inline
bool GRect_<_Tp>::empty() const
{
    return width <= 0 || height <= 0;
}

template<typename _Tp> template<typename _Tp2> inline
GRect_<_Tp>::operator GRect_<_Tp2>() const
{
    return GRect_<_Tp2>(saturate_cast<_Tp2>(x), saturate_cast<_Tp2>(y), saturate_cast<_Tp2>(width), saturate_cast<_Tp2>(height));
}

template<typename _Tp> inline
bool GRect_<_Tp>::contains(const GPoint_<_Tp>& pt) const
{
    return x <= pt.x && pt.x < x + width && y <= pt.y && pt.y < y + height;
}


template<typename _Tp> static inline
GRect_<_Tp>& operator += ( GRect_<_Tp>& a, const GPoint_<_Tp>& b )
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

template<typename _Tp> static inline
GRect_<_Tp>& operator -= ( GRect_<_Tp>& a, const GPoint_<_Tp>& b )
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

template<typename _Tp> static inline
GRect_<_Tp>& operator += ( GRect_<_Tp>& a, const GSize_<_Tp>& b )
{
    a.width += b.width;
    a.height += b.height;
    return a;
}

template<typename _Tp> static inline
GRect_<_Tp>& operator -= ( GRect_<_Tp>& a, const GSize_<_Tp>& b )
{
    const _Tp width = a.width - b.width;
    const _Tp height = a.height - b.height;
    CV_DbgAssert(width >= 0 && height >= 0);
    a.width = width;
    a.height = height;
    return a;
}

template<typename _Tp> static inline
GRect_<_Tp>& operator &= ( GRect_<_Tp>& a, const GRect_<_Tp>& b )
{
    _Tp x1 = std::max(a.x, b.x);
    _Tp y1 = std::max(a.y, b.y);
    a.width = std::min(a.x + a.width, b.x + b.width) - x1;
    a.height = std::min(a.y + a.height, b.y + b.height) - y1;
    a.x = x1;
    a.y = y1;
    if( a.width <= 0 || a.height <= 0 )
        a = GRect();
    return a;
}

template<typename _Tp> static inline
GRect_<_Tp>& operator |= ( GRect_<_Tp>& a, const GRect_<_Tp>& b )
{
    if (a.empty()) {
        a = b;
    }
    else if (!b.empty()) {
        _Tp x1 = std::min(a.x, b.x);
        _Tp y1 = std::min(a.y, b.y);
        a.width = std::max(a.x + a.width, b.x + b.width) - x1;
        a.height = std::max(a.y + a.height, b.y + b.height) - y1;
        a.x = x1;
        a.y = y1;
    }
    return a;
}

template<typename _Tp> static inline
bool operator == (const GRect_<_Tp>& a, const GRect_<_Tp>& b)
{
    return a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height;
}

template<typename _Tp> static inline
bool operator != (const GRect_<_Tp>& a, const GRect_<_Tp>& b)
{
    return a.x != b.x || a.y != b.y || a.width != b.width || a.height != b.height;
}

template<typename _Tp> static inline
GRect_<_Tp> operator + (const GRect_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return GRect_<_Tp>( a.x + b.x, a.y + b.y, a.width, a.height );
}

template<typename _Tp> static inline
GRect_<_Tp> operator - (const GRect_<_Tp>& a, const GPoint_<_Tp>& b)
{
    return GRect_<_Tp>( a.x - b.x, a.y - b.y, a.width, a.height );
}

template<typename _Tp> static inline
GRect_<_Tp> operator + (const GRect_<_Tp>& a, const GSize_<_Tp>& b)
{
    return GRect_<_Tp>( a.x, a.y, a.width + b.width, a.height + b.height );
}

template<typename _Tp> static inline
GRect_<_Tp> operator - (const GRect_<_Tp>& a, const GSize_<_Tp>& b)
{
    const _Tp width = a.width - b.width;
    const _Tp height = a.height - b.height;
    CV_DbgAssert(width >= 0 && height >= 0);
    return GRect_<_Tp>( a.x, a.y, width, height );
}

template<typename _Tp> static inline
GRect_<_Tp> operator & (const GRect_<_Tp>& a, const GRect_<_Tp>& b)
{
    GRect_<_Tp> c = a;
    return c &= b;
}

template<typename _Tp> static inline
GRect_<_Tp> operator | (const GRect_<_Tp>& a, const GRect_<_Tp>& b)
{
    GRect_<_Tp> c = a;
    return c |= b;
}

////////////////////////////////// Size /////////////////////////////////

template<typename _Tp> inline
GSize_<_Tp>::GSize_()
    : width(0), height(0) {}

template<typename _Tp> inline
GSize_<_Tp>::GSize_(_Tp _width, _Tp _height)
    : width(_width), height(_height) {}

template<typename _Tp> inline
GSize_<_Tp>::GSize_(const GSize_& sz)
    : width(sz.width), height(sz.height) {}

template<typename _Tp> inline
GSize_<_Tp>::GSize_(GSize_&& sz)
    : width(std::move(sz.width)), height(std::move(sz.height)) {}

template<typename _Tp> template<typename _Tp2> inline
GSize_<_Tp>::operator GSize_<_Tp2>() const
{
    return GSize_<_Tp2>(saturate_cast<_Tp2>(width), saturate_cast<_Tp2>(height));
}

template<typename _Tp> inline
GSize_<_Tp>& GSize_<_Tp>::operator = (const GSize_<_Tp>& sz)
{
    width = sz.width; height = sz.height;
    return *this;
}

template<typename _Tp> inline
GSize_<_Tp>& GSize_<_Tp>::operator = (GSize_<_Tp>&& sz)
{
    width = std::move(sz.width); height = std::move(sz.height);
    return *this;
}

template<typename _Tp> inline
_Tp GSize_<_Tp>::area() const
{
    const _Tp result = width * height;
//    std::assert(!std::numeric_limits<_Tp>::is_integer
//                 || width == 0 || result / width == height); // make sure the result fits in the return value
    return result;
}

template<typename _Tp> inline
double GSize_<_Tp>::aspectRatio() const
{
    return width / static_cast<double>(height);
}

template<typename _Tp> inline
bool GSize_<_Tp>::empty() const
{
    return width <= 0 || height <= 0;
}


template<typename _Tp> static inline
GSize_<_Tp>& operator *= (GSize_<_Tp>& a, _Tp b)
{
    a.width *= b;
    a.height *= b;
    return a;
}

template<typename _Tp> static inline
GSize_<_Tp> operator * (const GSize_<_Tp>& a, _Tp b)
{
    GSize_<_Tp> tmp(a);
    tmp *= b;
    return tmp;
}

template<typename _Tp> static inline
GSize_<_Tp>& operator /= (GSize_<_Tp>& a, _Tp b)
{
    a.width /= b;
    a.height /= b;
    return a;
}

template<typename _Tp> static inline
GSize_<_Tp> operator / (const GSize_<_Tp>& a, _Tp b)
{
    GSize_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
GSize_<_Tp>& operator += (GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    a.width += b.width;
    a.height += b.height;
    return a;
}

template<typename _Tp> static inline
GSize_<_Tp> operator + (const GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    GSize_<_Tp> tmp(a);
    tmp += b;
    return tmp;
}

template<typename _Tp> static inline
GSize_<_Tp>& operator -= (GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    a.width -= b.width;
    a.height -= b.height;
    return a;
}

template<typename _Tp> static inline
GSize_<_Tp> operator - (const GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    GSize_<_Tp> tmp(a);
    tmp -= b;
    return tmp;
}

template<typename _Tp> static inline
bool operator == (const GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    return a.width == b.width && a.height == b.height;
}

template<typename _Tp> static inline
bool operator != (const GSize_<_Tp>& a, const GSize_<_Tp>& b)
{
    return !(a == b);
}



}



#endif // GTYPES_H
