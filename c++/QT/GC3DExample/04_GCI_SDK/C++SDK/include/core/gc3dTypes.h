/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3DTYPES_H
#define GC3DTYPES_H

#include "gc3dCoreTypes.h"
#include<vector>

#include "gc3d.h"

/**
 *  @brief GCISHAPE 形状描述结构体
  */
enum GCIShape {
    GCI_SHAPE_NONE     =      -1,                                        //!<未初始化默认参数
    GCI_SHAPE_RECT      =      0,                                        //!<0:旋转矩形
    GCI_SHAPE_CIRCLE   =       1,                                        //!<1:圆形
    GCI_SHAPE_LINE      =      2,                                        //!<2：线
    GCI_SHAPE_CONTOUR   =      3                                         //!<3：若干点
};

/**
 *  @brief GCICONTOURGROUND 轮廓查找参数
  */
enum GCIContourGround{
    GCI_BLACK_BACKGROUND = 0,           //!<黑色为背景
    GCI_WHITE_BACKGROUND = 1            //!<白色为背景
};

/**
 *  @brief GCIThresholdTypes 轮廓查找参数
  */
enum GCIThresholdTypes{
    GCI_THRESH_BINARY=0,            //!
    GCI_THRESH_BINARY_INV
};

/**
 *  @brief preImageType 预处理类型 0 不处理 1 中值滤波 2 二值化 3 中值滤波加二值化
 */
enum preImageType {
    PRE_NOTHING = 0,             //!<不做处理
    PRE_MEDIAN = 1,              //!<中值滤波
    PRE_BINARY = 2,              //!<二值化
    PRE_MEDIANANDBINARY = 3      //!<中值率波+二值化

};

/**
 *  @brief modelType 模板匹配类型 0 灰度匹配 2 二值轮廓 3 canny边缘
 */
enum modelType {
    MODELTYPE_GRAY = 0,                //!<灰度图匹配
    MODELTYPE_BINARYEDGE = 1,          //!<二值轮廓匹配
    MODELTYPE_CANNY = 2                //!<canny边缘轮廓匹配

};



/**
 *  @brief GCIColorConversionCodes 图像类型转化
  */
enum GCIColorConversionCodes{
    GCI_COLOR_RGB2GRAY=0,
    GCI_COLOR_GRAY2RGB=1,
};


/**
 *  @brief GCIIMGDATATYPE 图像数据类型
  */
enum GCIImageDataType{
    GCI_8UC1 = 0,           //!<8位灰度图类型
    GCI_8UC3 = 1            //!<24位彩色图像类型
};

/**
 *  @brief GCIMatchType 模板匹配方法
  */
enum GCIMatchType{
    GCI_TM_SQDIFF = 0,       //!<平方差匹配
    GCI_TM_CCOEFF            //!<相关性匹配
};

/**
 *  @brief GCIFileStorageType 文件读写模式
  */
enum GCIFileStorageType{
    GCI_FILE_READ = 0,        //!<读模式
    GCI_FILE_WRITE            //!<写模式
};


namespace gc3d {

template<typename _Tp> class DLLEXPORT Scalar_
{
public:
    typedef _Tp value_type;

    //! default constructor
    Scalar_();
    Scalar_(const _Tp _v0,const _Tp _v1,const _Tp _v2);

    Scalar_& operator = ( const Scalar_& r );
    _Tp val[3];
public:
    _Tp v0;
    _Tp v1;
    _Tp v2;
};

typedef Scalar_<int> Scalar3i;
typedef Scalar_<float> Scalar3f;
typedef Scalar_<double> Scalar3d;
typedef Scalar3i Scalar;


template<typename _Tp> class DLLEXPORT GCircle_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GCircle_();
    GCircle_(const GPoint_<_Tp> center, double radius);


    GCircle_& operator = ( const GCircle_& r );


public:
    GPoint_<_Tp> center;
    double radius;
};

typedef GCircle_<int> GCircle2i;
typedef GCircle_<float> GCircle2f;
typedef GCircle_<double> GCircle2d;
typedef GCircle2i GCircle;



template<typename _Tp> class DLLEXPORT GLine_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GLine_();
    GLine_(const GPoint_<_Tp> p1, const GPoint_<_Tp> p2);


    GLine_& operator = ( const GLine_& r );
public:
    GPoint_<_Tp> p1;
    GPoint_<_Tp> p2;
    double k;
    double b;
};

typedef GLine_<int> GLine2i;
typedef GLine_<float> GLine2f;
typedef GLine_<double> GLine2d;
typedef GLine2i GLine;



template<typename _Tp> class DLLEXPORT GRotationRect_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GRotationRect_();
    GRotationRect_(const GPoint_<_Tp> p1, const GPoint_<_Tp> p2,const GPoint_<_Tp> p3, const GPoint_<_Tp> p4);


    GRotationRect_& operator = ( const GRotationRect_& r );
public:
    GPoint_<_Tp> p1;
    GPoint_<_Tp> p2;
    GPoint_<_Tp> p3;
    GPoint_<_Tp> p4;
    GPoint_<_Tp> center;
    GSize_<float> size;
    float angle;
};

typedef GRotationRect_<int> GRotationRect2i;
typedef GRotationRect_<float> GRotationRect2f;
typedef GRotationRect_<double> GRotationRect2d;
typedef GRotationRect2i GRotationRect;



template<typename _Tp> class DLLEXPORT GRegion_
{
public:
    typedef _Tp value_type;

    //! default constructor
    GRegion_();
    GRegion_(GCIShape shapeType,GRotationRect_<_Tp> rect,GCircle_<_Tp> circle,std::vector<GPoint_<_Tp>>&points,GLine_<_Tp> line);
    GRegion_(GCIShape shapeType,GRotationRect_<_Tp> rect,GCircle_<_Tp> circle,std::vector<GPoint_<_Tp>> points,GLine_<_Tp> line);

    GRegion_& operator = ( const GRegion_& r );
public:
    GCIShape shapeType  = GCI_SHAPE_NONE;
    GRotationRect_<_Tp>         rect;
    GCircle_<_Tp>             circle;
    std::vector<GPoint_<_Tp>> points;
    GLine_<_Tp>                 line;

};

typedef GRegion_<int> GRegion2i;
typedef GRegion_<float> GRegion2f;
typedef GRegion_<double> GRegion2d;
typedef GRegion2i GRegion;


////////////////////////////////// Scalar /////////////////////////////////

template<typename _Tp> inline
Scalar_<_Tp>::Scalar_()
{
    v0=0;
    v1=0;
    v2=0;
    val[0] = 0;
    val[1] = 0;
    val[2] = 0;
}

template<typename _Tp> inline
Scalar_<_Tp>::Scalar_(const _Tp _v0, const _Tp _v1,const _Tp _v2)
    : v0(_v0),v1(_v1),v2(_v2) {
    val[0] = v0;
    val[1] = v1;
    val[2] = v2;
}

template<typename _Tp> inline
Scalar_<_Tp>& Scalar_<_Tp>::operator = ( const Scalar_<_Tp>& r )
{
    v0 = r.v0;
    v1 = r.v1;
    v2 = r.v2;
    val[0] = r.v0;
    val[1] = r.v1;
    val[2] = r.v2;
    return *this;
}



////////////////////////////////// Circle /////////////////////////////////

template<typename _Tp> inline
GCircle_<_Tp>::GCircle_()
{
    center.x=0;
    center.y=0;
    radius=0;
}

template<typename _Tp> inline
GCircle_<_Tp>::GCircle_(const GPoint_<_Tp> _center, double _radius)
    : center(_center),radius(_radius) {}

template<typename _Tp> inline
GCircle_<_Tp>& GCircle_<_Tp>::operator = ( const GCircle_<_Tp>& r )
{
    center=r.center;
    radius=r.radius;
    return *this;
}



////////////////////////////////// Line /////////////////////////////////

template<typename _Tp> inline
GLine_<_Tp>::GLine_()
{
    p1.x=0;
    p1.y=0;
    p2.x=0;
    p2.y=0;
    k=0;
    b=0;
}

template<typename _Tp> inline
GLine_<_Tp>::GLine_(const GPoint_<_Tp> _p1, const GPoint_<_Tp> _p2)
    : p1(_p1),p2(_p2) {
    if(p1.x!=p2.x){
        float v0 = p2.y-p1.y;
        float v1 = p2.x-p1.x;
        k = v1/v0;
        b = p1.y-k*p1.x;
    }
}

template<typename _Tp> inline
GLine_<_Tp>& GLine_<_Tp>::operator = ( const GLine_<_Tp>& line )
{
    p1=line.p1;
    p2=line.p2;
    k = line.k;
    b = line.b;
    return *this;
}






////////////////////////////////// GRotationRect /////////////////////////////////

template<typename _Tp> inline
GRotationRect_<_Tp>::GRotationRect_()
{
    p1.x=0;
    p1.y=0;
    p2.x=0;
    p2.y=0;
    p3.x=0;
    p3.y=0;
    p4.x=0;
    p4.y=0;
}

template<typename _Tp> inline
GRotationRect_<_Tp>::GRotationRect_(const GPoint_<_Tp> _p1, const GPoint_<_Tp> _p2,const GPoint_<_Tp> _p3, const GPoint_<_Tp> _p4)
    : p1(_p1),p2(_p2),p3(_p3),p4(_p4) {}

template<typename _Tp> inline
GRotationRect_<_Tp>& GRotationRect_<_Tp>::operator = ( const GRotationRect_<_Tp>& line )
{
    p1=line.p1;
    p2=line.p2;
    p3=line.p3;
    p4=line.p4;
    center = line.center;
    angle = line.angle;
    size = line.size;
    return *this;
}


////////////////////////////////// GRegion_ /////////////////////////////////

template<typename _Tp> inline
GRegion_<_Tp>::GRegion_()
{
    shapeType=GCI_SHAPE_NONE;
    points.clear();
}

template<typename _Tp> inline
GRegion_<_Tp>::GRegion_(GCIShape _shapeType,GRotationRect_<_Tp> _rect,GCircle_<_Tp> _circle,std::vector<GPoint_<_Tp>>&_points,GLine_<_Tp> _line)
{
    shapeType=_shapeType;
    rect=_rect;
    circle=_circle;
    points=_points;
    line=_line;
}


template<typename _Tp> inline
GRegion_<_Tp>::GRegion_(GCIShape _shapeType,GRotationRect_<_Tp> _rect,GCircle_<_Tp> _circle,std::vector<GPoint_<_Tp>> _points,GLine_<_Tp> _line)
{
    shapeType=_shapeType;
    rect=_rect;
    circle=_circle;
    points=_points;
    line=_line;
}

template<typename _Tp> inline
GRegion_<_Tp>& GRegion_<_Tp>::operator = ( const GRegion_<_Tp>& greg )
{
    shapeType=greg.shapeType;
    rect=greg.rect;
    circle=greg.circle;
    points=greg.points;
    line=greg.line;
    return *this;
}



}



#endif // GTYPES_H
