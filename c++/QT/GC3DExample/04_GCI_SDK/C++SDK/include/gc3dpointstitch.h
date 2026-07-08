/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2024, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#ifndef GC3DPOINTSTITCH_H
#define GC3DPOINTSTITCH_H
#include <vector>
#include<array>
#include "core/gc3dTypes.h"

namespace gc3d {

DLLEXPORT  void genUndistortIndex(int*& undistortIndex,double* innerParams,double* ks,int imgW,int imgH);

/**
* @brief undistortMetaData  对metaData去畸变
* @param [inout]  data                      //!<需要去畸变的metaData
* @param [in]  unDisIndex                   //!<畸变索引，可以由genUndistortIndex得到
* @return
*/
DLLEXPORT  void undistortMetaData(gc3d::GC3DMetaData& data, int* unDisIndex);

/**
* @brief conCatData  对metadata进行拼接
* @param [in]  srcData              //!<起始拼接图
* @param [in]  conData              //!<拼接图
* @param [inOut]  resData           //!<拼接好的图
* @param [in]  ax1                  //!<第一个坐标
* @param [in]  ax2                  //!<第二个坐标
* @param [in]  kx                   //!<x方向系数
* @param [in]  ky                   //!<y方向系数
* @param [in]  kz                   //!<z方向系数
* @param [in]  minValidDisX         //!<x方向最小有效匹配距离
* @param [in]  minValidDisY         //!<y方向最小有效匹配距离
* @param [in]  minValidDisZ         //!<z方向最小有效匹配距离
* @return
*/
DLLEXPORT  bool conCatData(gc3d::GC3DMetaData&srcData,gc3d::GC3DMetaData&conData,gc3d::GC3DMetaData& resData,
                           float ax1,float ax2,float kx,float ky,float kz,float minValidDisX,float minValidDisY,float minValidDisZ);

/**
* @brief conCatDataByFixedPoint  对metadata进行拼接
* @param [in]  data1                //!<起始拼接图
* @param [in]  data2                //!<拼接图
* @param [inOut]  conData           //!<拼接好的图
* @param [in]  ax1                  //!<第一个坐标
* @param [in]  ax2                  //!<第二个坐标
* @param [in]  kx                   //!<x方向系数
* @param [in]  ky                   //!<y方向系数
* @param [in]  kz                   //!<z方向系数
* @param [in]  croIndX              //!<找到的交点坐标X
* @param [in]  croIndY              //!<找到的交点坐标Y
* @param [in]  minValidDisX         //!<x方向最小有效匹配距离
* @param [in]  minValidDisY         //!<y方向最小有效匹配距离
* @return
*/
DLLEXPORT  void conCatDataByFixedPoint(gc3d::GC3DMetaData&data1,gc3d::GC3DMetaData&data2,gc3d::GC3DMetaData& conData,
                                  float ax1,float ax2,float kx,float ky,float kz,int croIndX ,int croIndY,int minIndexX ,int minIndexY );


}





#endif // GC3DPOINTSTITCH_H
