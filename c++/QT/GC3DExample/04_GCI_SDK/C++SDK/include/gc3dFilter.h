/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#ifndef GC3DFILTER_H
#define GC3DFILTER_H
#include <vector>
#include<array>
#include "core/gc3dTypes.h"
#include "imgproc/gc3dImage.h"
#include "imgproc/gc3dImgproc.h"

namespace gc3d {

/**
* @brief rePtsForPlane  根据平面去掉杂点
* @param [inout] points            //!<输入的3D点数组，函数将用输入的点进行平面拟合
* @param [in]    planePoints       //!<输入的平面的点
* @param [in]    startper          //!<有效点的起始百分比，离拟合平面距离超过此百分比将去掉(0~1)
* @param [in]    endPer            //!<有效点的终止百分比，离拟合平面距离超过此百分比将去掉(0~1)
* @note startper需小于endPer
* @return
*/
DLLEXPORT void rePtsForPlane(std::vector<GPoint3f>& points,std::vector<GPoint3f>& planePoints,float startPer,float endPer);

/**
* @brief filterByMask   根据mask将非感兴趣区域置为0
* @param [inout] data              //!<输入的点云数据结构，根据mask将相应非感兴趣区域置为0
* @param [in]  mask                //!<是一个数组，大小为data.imgW*data.imgH，为0则将相应的data数据位置置为0，否则不变
* @note mask应该是一个data.imgW*data.imgH的unsigned char数组
* @return
*/
DLLEXPORT void filterByMask(GC3DMetaData& data,unsigned char* mask);

/**
* @brief erodeMetaData  会改变data的有效数据，使其有效数据变小，变化量有erodeSize决定
* @param [inout] data              //!<输入输出的数据
* @param [in]  erodeSize           //!<腐蚀的尺寸大小，为奇数3，5，7，...
* @return
*/
DLLEXPORT void erodeMetaData(GC3DMetaData& data,int erodeSize);


/**
* @brief getPointsFromRotationRect  获取旋转矩形里的点集
* @param [in] rect                 //!<输入的旋转矩形
* @param [out]  points             //!<输出的旋转矩形内的点集
* @return
*/
DLLEXPORT void getPointsFromRotationRect(gc3d::GRotationRect rect,std::vector<GPoint>& points);

/**
* @brief getPointsFromCircle  获取圆形区域的点集
* @param [in] circle               //!<输入的圆形区域
* @param [out]  points             //!<输出的圆形区域的点集
* @param [in]  flag                //!<指示获取圆上或者圆内，当flag=0,points获取的是圆上的点，当flag=0,points获取的是圆内的点
* @return
*/
DLLEXPORT void getPointsFromCircle(gc3d::GCircle circle,std::vector<GPoint>& points,int flag);


/**
* @brief getMaskImgFromContour  根据单个多边形顶点填充mask
* @param [in] vert                 //!<输入的轮廓顶点，轮廓点必须是有序点集
* @param [inout]  mask             //!<输入的mask，并在结果中会输出，最终填充完毕之后，多边形区域会置为255，否则置为0
* @param [in]  w                   //!<mask的宽
* @param [in]  h                   //!<mask的高
* @return
*/
DLLEXPORT   void getMaskImgFromContour(std::vector<GPoint>& vert,unsigned char* mask,const int w,const int h);

/**
* @brief getMaskImgFromMultiContour  根据多个多边形顶点填充mask
* @param [in] vert                 //!<输入的轮廓顶点集合，每个轮廓点必须是有序点集
* @param [inout]  mask             //!<输入的mask，并在结果中会输出，最终填充完毕之后，多边形区域会置为255，否则置为0
* @param [in]  w                   //!<mask的宽
* @param [in]  h                   //!<mask的高
* @return
*/
DLLEXPORT   void getMaskImgFromMultiContour(std::vector<std::vector<GPoint>>& verts,unsigned char* mask,const int w,const int h);


/**
* @brief getMaskImgFromMultiRect  根据多个矩形填充mask
* @param [in] rects                //!<输入的矩形集合
* @param [inout]  mask             //!<输入的mask，并在结果中会输出，最终填充完毕之后，多边形区域会置为255，否则置为0
* @param [in]  w                   //!<mask的宽
* @param [in]  h                   //!<mask的高
* @return
*/
DLLEXPORT  void getMaskImgFromMultiRect(const std::vector<GRect>& rects,unsigned char* mask,const int w,const int h);


/**
* @brief getMaskImgFromMultiCircle  根据多个圆形区域填充mask
* @param [in] circles              //!<输入的圆形区域集合
* @param [inout]  mask             //!<输入的mask，并在结果中会输出，最终填充完毕之后，多边形区域会置为255，否则置为0
* @param [in]  w                   //!<mask的宽
* @param [in]  h                   //!<mask的高
* @return
*/
DLLEXPORT   void getMaskImgFromMultiCircle(const std::vector<GCircle>& circles,unsigned char* mask,const int w,const int h);

/**
* @brief getMaskImgFromRegions  根据多个区域填充mask
* @param [in] regions:             //!<输入的区域集合
* @param [inout]  mask             //!<输入的mask，并在结果中会输出，最终填充完毕之后，多边形区域会置为255，否则置为0
* @param [in]  w                   //!<mask的宽
* @param [in]  h                   //!<mask的高
* @return
*/
DLLEXPORT   void getMaskImgFromRegions(const std::vector<GRegion>& regions,unsigned char* mask,const int w,const int h);

/**
* @brief zeroPlaneSet  零平面设置函数，该函数将改变3D数据Data模型，实际会对3D模型做刚体变换
* @param [in] regions              //!<用与做零平面的区域位置
* @param [inout]  data             //!<输入的3D数据
* @return
*/
DLLEXPORT    void zeroPlaneSet(GC3DMetaData& data,std::vector<GRegion>& regions,double&A,double& B,double& C,
                               gc3d::GPoint3d& centerP,bool useBase);

/**
* @brief zeroPlaneSetABC  零平面设置函数，该函数将改变3D数据Data模型，实际会对3D模型做刚体变换
* @param [in] A              //!<平面参数A
* @param [in] B              //!<平面参数B
* @param [in] C              //!<平面参数C
* @param [inout]  data             //!<输入的3D数据
* @return
*/
DLLEXPORT    void zeroPlaneSetABCP(GC3DMetaData& data,double A,double B,double C,gc3d::GPoint3d centerP,bool useBase);


/**
* @brief zeroPlaneSetMask  零平面设置函数，该函数将改变3D数据Data模型，实际会对3D模型做刚体变换
* @param [inout]  data             //!<输入的3D数据
* @param [in]  baseMask            //!<使用baseMask区域的点来做零平面，当baseMask[i]为真，则会选择相应的点做零平面
* @note mask应该是一个data.imgW*data.imgH的unsigned char数组
* @return
*/
DLLEXPORT  void zeroPlaneSetMask(GC3DMetaData& data,unsigned char* baseMask);

/**
* @brief gridMetaData  对metadata进行网格化
* @param [inout]  data             //!<输入的3D数据
* @param [in]  startX            //!<x起点
* @param [in]  startY            //!<y起点
* @param [in]  filedW            //!<标准视野宽度
* @param [in]  filedH            //!<标准视野高度
* @param [in]  sizePixel            //!<像素当量
* @note mask应该是一个data.imgW*data.imgH的unsigned char数组
* @return
*/
DLLEXPORT  void gridMetaData(GC3DMetaData& data,  float startX, float startY,float filedW, float filedH, float sizePixel);

/**
* @brief gridMetaData  对metadata进行网格化
* @param [inout]  data             //!<输入的3D数据
* @param [in]  startX            //!<x起点
* @param [in]  startY            //!<y起点
* @param [in]  filedW            //!<标准视野宽度
* @param [in]  filedH            //!<标准视野高度
* @param [in]  sizePixel          //!<像素当量
* @param [in]  expSize            //!<需要扩充的边缘宽度
* @note mask应该是一个data.imgW*data.imgH的unsigned char数组
* @return
*/
DLLEXPORT  void gridMetaData(GC3DMetaData& data,  float startX, float startY,float filedW, float filedH, float sizePixel, int expSize);






}


#endif // GC3DFILTER_H
