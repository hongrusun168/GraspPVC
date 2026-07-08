/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#ifndef GC3DMEASURE_H
#define GC3DMEASURE_H
#include <vector>
#include<array>
#include "core/gc3dTypes.h"
#include "imgproc/gc3dImage.h"
#include "imgproc/gc3dImgproc.h"
#include "imgproc/filestorage.h"

namespace gc3d {

/**
* @brief fitplaneUsePoint  用3D点拟合平面函数
* @param [in]   pts            //!<输入的3D点数组，函数将用输入的点进行平面拟合，至少需要4个点
* @param [inout]    A          //!<平面方程的系数
* @param [inout]    B          //!<平面方程的系数
* @param [inout]    C          //!<平面方程的系数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t fitplaneUsePoint(const std::vector<GPoint3f>& pts, double& A, double& B, double& C);


/**
* @brief ransacfitplaneUsePoint  用RANSAC的3D点拟合平面函数
* @param [in]   points            //!<输入的3D点数组，函数将用输入的点进行平面拟合，至少需要4个点
* @param [in]   threshold         //!<点到平面的阈值，在阈值类的点可以看成是在平面上
* @param [in]   validPtNumber     //!<有效点的个数大于validPtNumber时候停止迭代，一般根据平面所占有的点的个数的比例来设置
* @param [inout]    A             //!<平面方程的系数
* @param [inout]    B             //!<平面方程的系数
* @param [inout]    C             //!<平面方程的系数
* @param [in]    maxIter          //!<最大迭代次数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
extern "C" DLLEXPORT  uint32_t  ransacFitplaneUsePoint(std::vector<GPoint3f>& points, float threshold, int validPtNumber,double& A, double& B, double& C,const int maxIter=100);

/**
* @brief pointToPlaneDistance  计算点到平面的距离
* @param [in]   point              //!<输入的点
* @param [in]    A                 //!<平面方程的系数
* @param [in]    B                 //!<平面方程的系数
* @param [in]    C                 //!<平面方程的系数
* @param [in]    D                 //!<平面方程的系数
* @return 点到平面的距离
*/
DLLEXPORT    float pointToPlaneDistance(GPoint3f point, double A, double B, double C,double D);


/**
* @brief fitShpereUsePoint  用3D点拟合球
* @param [in]   pts                //!<输入的3D点数组，函数将用输入的点进行球拟合，至少需要4个点
* @param [inout]    A              //!<球面方程的系数
* @param [inout]    B              //!<球面方程的系数
* @param [inout]    C              //!<球面方程的系数
* @param [inout]    r              //!<球面方程的系数
* @note 输出球面方程  ： (x-A)²+(y-B)²+(z-C)²=r²
* @return
*/
DLLEXPORT  void fitShpereUsePoint( std::vector<GPoint3f>& pts,double& A, double& B, double& C,double& r);


/**
* @brief fitplaneUseRect  用矩形区域内的点拟合平面函数
* @param [in]   data              //!<输入的3D模型
* @param [in]   rect              //!<输入的矩形区域，函数将用data里的rect区域点进行平面拟合
* @param [inout]    A             //!<平面方程的系数
* @param [inout]    B             //!<平面方程的系数
* @param [inout]    C             //!<平面方程的系数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return
*/
DLLEXPORT   uint32_t fitplaneUseRect(GC3DMetaData& data,const GRect rect, double& A, double& B, double& C);


/**
* @brief fitplaneUseRotationRect  用旋转矩形区域内的点拟合平面函数
* @param [in]   data               //!<输入的3D模型
* @param [in]   rect               //!<输入的旋转矩形区域，函数将用data里的rect区域点进行平面拟合
* @param [inout]    A              //!<平面方程的系数
* @param [inout]    B              //!<平面方程的系数
* @param [inout]    C              平面方程的系数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t fitplaneUseRotationRect(GC3DMetaData& data,const GRotationRect rect, double& A, double& B, double& C);

/**
* @brief fitplaneUseCircle  用圆形区域内的点拟合平面函数
* @param [in]   data              //!<输入的3D模型
* @param [in]   circle            //!<输入的圆矩形区域，函数将用data里的circle区域点进行平面拟合
* @param [inout]    A             //!<平面方程的系数
* @param [inout]    B             //!<平面方程的系数
* @param [inout]    C             //!<平面方程的系数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t fitplaneUseCircle(GC3DMetaData& data,const GCircle circle, double& A, double& B, double& C);

/**
* @brief fitplane  用掩膜区域内的点拟合平面函数
* @param [in]   data               //!<输入的3D模型
* @param [in]   mask               //!<输入的掩膜区域，mask[i]为真时，将选中相应的点用于拟合平面
* @param [inout]    A              //!<平面方程的系数
* @param [inout]    B              //!<平面方程的系数
* @param [inout]    C              //!<平面方程的系数
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t fitplane(GC3DMetaData& data, unsigned char* mask, double& A, double& B, double& C);


/**
* @brief getBasePlaneData  获取零平面的相关信息
* @param [in]   data               //!<输入的3D模型
* @param [in]   mask               //!<输入的掩膜区域，mask[i]为真时，将选中相应的点用于拟合平面
* @param [inout]    A              //!< 平面方程的系数
* @param [inout]    B              //!<平面方程的系数
* @param [inout]    C              //!<平面方程的系数
* @param [inout]    aveHeight      //!<视野内相对于零平面的高度
* @note 输出平面方程  ： Ax+By+Cz+1 = 0
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t getBasePlaneData(GC3DMetaData& data, unsigned char* mask, double& A, double& B, double& C,double& aveHeight);

/**
* @brief flatnessRect  计算矩形区域内的3D点集的平面度
* @param [in]   data              //!<输入的3D模型
* @param [in]   rect              //!<输入的矩形区域，将会使用data里的rect区域点集
* @param [inout]    outputValue   //!<输出的最终的平面度的值
* @param [inout]    z_min         //!<离平面最下方的点到平面的距离
* @param [inout]    z_max         //!<离平面最上方的点到平面的距离
* @param [inout]    z_ave         //!<所有点到平面的平均距离
* @param [inout]    std_dev       //!<所有点到平面距离的标准差
* @param [inout]    zaveHeight    //!<用于拟合平面点的Z绝对值
* @param [in]    sigma            //!<遗留参数，无效
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t flatnessRect(GC3DMetaData& data, GRect rect, float& outputValue,float& z_min,float& z_max,float& z_ave,float& std_dev,float& zaveHeight,const float sigma);


/**
* @brief flatnessCircle  计算圆形区域内的3D点集的平面度
* @param [in]   data              //!<输入的3D模型
* @param [in]   circle            //!<输入的圆形区域，将会使用data里的circle区域点集
* @param [inout]    outputValue   //!<输出的最终的平面度的值
* @param [inout]    z_min         //!<离平面最下方的点到平面的距离
* @param [inout]    z_max         //!<离平面最上方的点到平面的距离
* @param [inout]    z_ave         //!<所有点到平面的平均距离
* @param [inout]    std_dev       //!<所有点到平面距离的标准差
* @param [inout]    zaveHeight    //!<用于拟合平面点的Z绝对值
* @param [in]    sigma            //!<遗留参数，无效
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT   uint32_t flatnessCircle(GC3DMetaData& data, GCircle circle, float& outputValue,float& z_min,float& z_max,float& z_ave,float& std_dev,float& zaveHeight,const float sigma);

/**
* @brief flatness  计算掩膜计算3D点集的平面度
* @param [in]   data              //!<输入的3D模型
* @param [in]   mask              //!<输入的掩膜区域，mask[i]为真时，将选中相应的点用于计算平面度
* @param [inout]    outputValue   //!<输出的最终的平面度的值
* @param [inout]    z_min         //!<离平面最下方的点到平面的距离
* @param [inout]    z_max         //!<离平面最上方的点到平面的距离
* @param [inout]    z_ave         //!<所有点到平面的平均距离
* @param [inout]    std_dev       //!<所有点到平面距离的标准差
* @param [inout]    zaveHeight    //!<用于拟合平面点的Z绝对值
* @param [in]    sigma            //!<遗留参数，无效
* @param [in]    isHeight         //!<是否需要拟合，当为true是不需要拟合平面，直接用Z高度计算平面度，否则需要拟合平面
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t flatness(GC3DMetaData& data, unsigned char* mask, float& outputValue,float& z_min,float& z_max,float& z_ave,float& std_dev,float& zaveHeight,const float sigma = 0.0f,const bool isHeight = false);


/**
* @brief flatnessUsePoints  使用3D点集计算平面度
* @param [in]   points            //!<输入的3D点集
* @param [in]   mask              //!<输入的掩膜区域，mask[i]为真时，将选中相应的点用于计算平面度
* @param [inout]    outputValue   //!<输出的最终的平面度的值
* @param [inout]    z_min         //!<离平面最下方的点到平面的距离
* @param [inout]    z_max         //!<离平面最上方的点到平面的距离
* @param [inout]    z_ave         //!<所有点到平面的平均距离
* @param [inout]    std_dev       //!<所有点到平面距离的标准差
* @param [inout]    zaveHeight    //!<用于拟合平面点的Z绝对值
* @param [in]    sigma            //!<遗留参数，无效
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t flatnessUsePoints(const std::vector<GPoint3f>& points, float& outputValue,float& z_min,float& z_max,float& z_ave,float& std_dev,float& zaveHeight,const float sigma = 0.0f);


/**
* @brief flatnessUseMultiRegion  使用点集的方式计算平面度，函数会对每一个点及其相邻区域进行分析，最终每一个点都会拟合出一个稳定的点参与最终的平面度计算
* @param [in]   data              //!<输入的3D模型
* @param [in]   pts               //!<输入的点集，每一个pts里的点都会寻找其周围_size*_size区域的点
* @param [inout]   planeDis       //!<输出的每个点到拟合平面的距离，即计算平面度拟合平面时的平面方程的距离
* @param [inout]   plane[]        //!<输出的拟合平面的方程，plane[0]x+plane[1]y+plane[2]z+1=0
* @param [inout]    outputValue   //!<输出的最终的平面度的值
* @param [inout]    z_min         //!<离平面最下方的点到平面的距离
* @param [inout]    z_max         //!<离平面最上方的点到平面的距离
* @param [inout]    z_ave         //!<所有点到平面的平均距离
* @param [inout]    std_dev       //!<所有点到平面距离的标准差
* @param [inout]    zaveHeight    //!<用于拟合平面点的Z绝对值
* @param [inout]    pn            //!<用于计算平面度点的数量
* @param [in]    _size            //!<计算单个点时的size,默认为7，即每个pts里的点都会计算周围领域7*7的点，计算一个平均的3D点
* @param [in]    startPercent     //!<计算单个点时并不是把_size*_size里的所有点用来拟合一个点，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent       //!<计算单个点时并不是把_size*_size里的所有点用来拟合一个点，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的终止百分比值(0~1)
* @return  成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t flatnessUseMultiRegion(GC3DMetaData& data,std::vector<GPoint>& pts,std::vector<float>& planeDis,double plane[],float& outputValue,float& z_min,float& z_max,float& z_ave,
                                                                   float& std_dev,float& zaveHeight,int& pn,const int _size = 7, const float startPercent = 0.1f,const float endPercent = 0.9f);


/**
* @brief getTolarence  计算两个多边形之间的断差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   basePlaneVert         //!<输入的基准面的多边形点集，必须是有序的，函数会自动寻找多边形内的3D点集
* @param [in]   measurePlaneVert      //!<输入的测量面的多边形点集，必须是有序的，函数会自动寻找多边形内的3D点集
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是先用basePlaneVert区域拟合平面，计算measurePlaneVert区域内点到平面的距离
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getTolarence(GC3DMetaData& data,std::vector<GPoint>& basePlaneVert,std::vector<GPoint>& measurePlaneVert,float& result,
                                                         float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);

/**
* @brief getTolarenceUseRect  计算两个矩形之间的断差
* @param [in]   data                 //!< 输入的3D模型
* @param [in]   basePlane            //!<输入的基准面的矩形，函数会自动寻找矩形内的3D点集
* @param [in]   measurePlane         //!<输入的测量面的矩形，函数会自动寻找矩形内的3D点集
* @param [inout]    result           //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是先用basePlane区域拟合平面，计算measurePlane区域内点到平面的距离
* @param [inout]    z_min            //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max            //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range            //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent        //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent          //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getTolarenceUseRect(GC3DMetaData& data,GRect basePlane,GRect measurePlane,float& result,
                                                                float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);

/**
* @brief getTolarenceUseCircle  计算两个圆形之间的断差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   basePlane             //!<输入的基准面的圆形，函数会自动寻找圆形内的3D点集
* @param [in]   measurePlane          //!<输入的测量面的圆形，函数会自动寻找圆形内的3D点集
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是先用basePlane区域拟合平面，计算measurePlane区域内点到平面的距离
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getTolarenceUseCircle(GC3DMetaData& data,GCircle basePlane,GCircle measurePlane,float& result,
                                                                  float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);

/**
* @brief getTolarenceMultiRegion  计算两个多边形集合之间的断差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   basePlaneVert         //!<输入的基准面的多边形集合点集，每个多边形必须是有序的，函数会自动寻找多个多边形内的3D点集
* @param [in]   measurePlaneVert      //!<输入的测量面的多边形集合点集，每个多边形必须是有序的，函数会自动寻找多个多边形内的3D点集
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是先用basePlaneVert区域拟合平面，计算measurePlaneVert区域内点到平面的距离
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getTolarenceMultiRegion(GC3DMetaData& data,std::vector<std::vector<GPoint>>& basePlaneVerts,std::vector<GPoint>& measurePlaneVert,float& result,
                                                                    float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);



/**
* @brief getTolarenceTwoRegion  计算两个区域之间的断差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   basePlane             //!<输入的基准面的区域，区域可以是圆形、矩形或者多边形
* @param [in]   measurePlane          //!<输入的测量面的的区域，区域可以是圆形、矩形或者多边形
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是先用basePlane区域拟合平面，计算measurePlane区域内点到平面的距离
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getTolarenceTwoRegion(GC3DMetaData& data,GRegion& basePlane,GRegion& measurePlane,float& result,
                                                                    float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);



/**
* @brief getHeightDifTwoRegion  计算两个区域之间的高度差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   basePlane             //!<输入的基准面的区域，区域可以是圆形、矩形或者多边形
* @param [in]   measurePlane          //!<输入的测量面的的区域，区域可以是圆形、矩形或者多边形
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是直接计算两个区域内Z平均值的差值
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getHeightDifTwoRegion(GC3DMetaData& data,GRegion& basePlane,GRegion& measurePlane,float& result,
                                                                    float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);


/**
* @brief getHeightDifTwoRegionFromfit  计算两个区域之间的高度差
* @param [in]   data                  //!<输入的3D模型
* @param [in]   mask                  //!<输入的基准面拟合区域
* @param [in]   basePlane             //!<输入的基准面的区域，区域可以是圆形、矩形或者多边形
* @param [in]   measurePlane          //!<输入的测量面的的区域，区域可以是圆形、矩形或者多边形
* @param [inout]    result            //!<输出最终的断差值，根据后续参数设定的不同，结果会有细微差异，计算的过程是直接计算两个区域内Z平均值的差值
* @param [inout]    z_min             //!<输出的测量面到基准面的最小高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    z_max             //!<输出的测量面到基准面的最大高度值，根据后续参数设定的不同，结果会有细微差异
* @param [inout]    range             //!<输出的测量面到基准面的高度范围
* @param [in]    startPercent         //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值(0~1)
* @param [in]    endPercent           //!<计算result时并不是计算测量面所有的点到基准面距离的平均值，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值(0~1)
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    uint32_t getHeightDifTwoRegionFromfit(GC3DMetaData& data,unsigned char* mask,GRegion& basePlane,GRegion& measurePlane,float& result,
                                                                    float& z_min,float& z_max,float& range,const float startPercent,const float endPercent);


/**
* @brief getLineHeightDTT  提取图像中一条线上的点的Z值，函数首先提取图像的两个点的连线点集，然后根据点集索引3D点的Z值，无效点会跳过
* @param [in]   data                  //!<输入的3D模型
* @param [in]   p1                    //!<输入的图像上的起始点
* @param [in]   p2                    //!<输入的图像上的终止点
* @param [inout]    result            //!<提取的P1-P2的连线点的Z值
* @param [inout]    z_min             //!<提取的P1-P2的连线点的Z最小值
* @param [inout]    z_max             //!<提取的P1-P2的连线点的Z最大值
* @return 成功返回 GC3D_SUCCESS 失败返回 GC3D_ALGORITHM_FITPLANE_FAIL
*/
DLLEXPORT    void getLineHeightDTT(GC3DMetaData& data,GPoint p1,GPoint p2,std::vector<float>& result,float& z_min,float& z_max);


/**
* @brief getLinePointsDTT  提取图像中一条线上的点的Z值，函数首先提取图像的两个点的连线点集，无效点会跳过
* @param [in]   data                   //!<输入的3D模型
* @param [in]   p1                     //!<输入的图像上的起始点
* @param [in]   p2                     //!<输入的图像上的终止点
* @param [inout]    pts                //!<提取的P1-P2的连线点的3D点集合
* @return
*/
DLLEXPORT    void getLinePointsDTT(GC3DMetaData& data,GPoint p1,GPoint p2,std::vector<GPoint3f>& pts);


/**
* @brief get3DPointsFromContours  获取轮廓区域内的3D点的集合
* @param [in]   data                   //!<输入的3D模型
* @param [in]   contours               //!<输入的轮廓点集，必须是有序的
* @param [inout]   outputPoints        //!<输出的3D点的集合
* @return
*/
DLLEXPORT  void get3DPointsFromContours(GC3DMetaData& data,std::vector<GPoint>& contours,std::vector<GPoint3f>& outputPoints);

/**
* @brief getColorMap  获取3D模型的伪彩色图像，伪彩色的计算规则是蓝->绿->红，z值越大越接近红色，越小越接近蓝色
* @param [in]   data                   //!<输入的3D模型
* @param [in]   zMin                   //!<手动输入的渲染范围z最小值，当z等于zMin，为绿色
* @param [in]   zMax                   //!<手动输入的渲染范围z最大值，当z等于zMin，为红色
* @param [inout]   img                 //!<输出的伪彩色图像的头指针，必须是已经分配好内存的，内存大小为sizeof(uchar)*data.imgH*data.imgW,像素排列规则是RGBRGBRGB...
* @return
*/
DLLEXPORT    void getColorMap(GC3DMetaData& data,unsigned char* img,const float zMin,const float zMax,float textureRatio = 0.0f,bool isPreView=false);


/**
* @brief getColorMapAuto  获取3D模型的伪彩色图像，伪彩色的计算规则是蓝->绿->红，z值越大越接近红色，越小越接近蓝色，函数自动计算渲染高度范围
* @param [in]   data                   //!<输入的3D模型
* @param [inout]   img                 //!<输出的伪彩色图像的头指针，必须是已经分配好内存的，内存大小为sizeof(uchar)*data.imgH*data.imgW,像素排列规则是RGBRGBRGB...
* @return
*/
DLLEXPORT    void getColorMapAuto(GC3DMetaData& data,unsigned char* img);


/**
* @brief lineFit3D  空间直线拟合函数
* @param [in]   p                      //!<输入的3D点集，用于拟合空间直线
* @param [inout]   A                   //!<空间直线方程系数
* @param [inout]   B                   //!<空间直线方程系数
* @param [inout]   C                   //!<空间直线方程系数
* @param [inout]   D                   //!<空间直线方程系数
* @param [inout]   E                   //!<空间直线方程系数
* @param [inout]   F                   //!<空间直线方程系数
* @param [inout]   G                   //!<空间直线方程系数
* @param [inout]   H                   //!<空间直线方程系数
* @param [inout]   err                 //!<空间直线拟合误差
* @note 空间直线输出方程式为：Ax+By+Cz+D=0，Ex+Fy+Gz+H=0
* @return 成功返回true  否则返回false
*/
DLLEXPORT  bool lineFit3D(const std::vector<GPoint3f>& p, float&A, float&B, float&C, float&D, float&E, float & F, float&G, float&H,double& err);


/**
* @brief lineness  空间直线度测量，函数首先会取得起始点到终止点上的所有3D点的集合用于计算
* @param [in]   data                     //!<输入的3D模型
* @param [in]   beginPoint               //!<空间直线起始的3D点
* @param [in]   endPoint                 //!<空间直线终止的3D点
* @param [inout]   straightness          //!<输出的直线度的值
* @return 成功返回true  否则返回false
*/
DLLEXPORT  bool lineness(GC3DMetaData& data,const GPoint& beginPoint,const GPoint& endPoint,double& straightness);

/**
* @brief lineness  空间圆度测量，函数首先会取得起始点到终止点上的所有3D点的集合用于计算
* @param [in]   data                     //!<输入的3D模型
* @param [in]   beginPoint               //!<空间直线起始的3D点
* @param [in]   endPoint                 //!<空间直线终止的3D点
* @param [inout]   straightness          //!<输出的圆度的值
* @return 成功返回true  否则返回false
*/
DLLEXPORT  bool circleness(GC3DMetaData& data,const GPoint& beginPoint,const GPoint& endPoint,double& roundness);


/**
* @brief lineCompensationon_Z  函数用于线性误差补偿
* @param [inout]   data                  //!<输入的3D模型
* @param [in]   Z0                       //!<高度补偿系数
* @param [in]   b                        //!<缩放补偿系数
* @note z=z+(z-Z0)*b
* @return 成功返回true  否则返回false
*/
DLLEXPORT  bool lineCompensationon_Z(GC3DMetaData& data,double Z0,double b);


/**
* @brief computeTwoLineDistance  函数计算两直线的距离，定位直线是基于灰度图的，计算距离是基于点云的
* @param [in]   data                       //!<输入的3D模型
* @param [in]   reg1                       //!<输入的包含了第一条线段的矩形
* @param [in]   reg2                       //!<输入的包含了第一条线段的矩形
* @param [in]   p11                        //!<输出的第一条直线的起点
* @param [in]   p12                        //!<输出的第一条直线的终点
* @param [in]   p21                        //!<输出的第二条直线的起点
* @param [in]   p22                        //!<输出的第二条直线的终点
* @param [in]   crossPoint1                //!<输出的用于计算两直线距离的第一个交点
* @param [in]   crossPoint2                //!<输出的用于计算两直线距离的第二个交点，两直线的距离即两个交点的3D空间距离
* @return 两直线的距离
*/
DLLEXPORT  float computeTwoLineDistance(GC3DMetaData& data, const GRegion& reg1, const GRegion& reg2,GPoint& p11,GPoint& p12,
                                        GPoint& p21,GPoint& p22,GPoint& crossPoint1,GPoint& crossPoint2,float pixelSize);

/**
* @brief getMostHightPointOfRegion  求取区域内最高若干个点云的平均值点
* @param [in]  data             //!<输入的3D数据
* @param [in]  region           //!<输入的测量区域，限定为GCI_SHAPE_RECTANGLE|GCI_SHAPE_CONTOURS|GCI_SHAPE_CIRCLE
* @param [inout]  avePoint3d    //!<求得的平均值3d点
* @param [in]  avePoint2d       //!<用于计算的点所有2D位置求平均
* @param [in]  selectNum        //!<用于计算的排序后，最高点的个数
* @param [in]  filterNum        //!<排除最高的点数，然后开始统计
* @note 测量区域有效点数小于selectNum + filterNum会直接返回false
* @return 是否计算成功
*/
DLLEXPORT bool getMostHightPointOfRegion(gc3d::GC3DMetaData& data, gc3d::GRegion region, gc3d::GPoint3f& avePoint3d, gc3d::GPoint& avePoint2d ,const int selectNum,const int filterNum);


/**
* @brief updataPixelLength  函数计算当前区域的像素当量
* @param [in]   data                       //!<输入的3D模型
* @param [in]   rect                       //!<输入的用于计算像素当量的矩形区域，即计算矩形区域内所有相邻点的距离的平均距离
* @return 像素当量，单位mm
*/
DLLEXPORT  float updataPixelLength(GC3DMetaData& data,GRegion& rect);

/**
* @brief getVolume  函数计算当前区域的体积
* @param [in]   data                       //!<输入的3D模型
* @param [in]   region                     //!<输入的用于体积的区域集合
* @param [in]   minHeight                  //!<输入的用于计算体积的起始高度，当高度小于这个值是不被计算
* @param [inout]   validPoints             //!<输出的用于计算体积的2D点集合
* @return 当前区域体积,单位mm³
*/
DLLEXPORT  float getVolume(GC3DMetaData& data,std::vector<gc3d::GRegion>& region,const float minHeight,
                           std::vector<gc3d::GPoint>& validPoints,float pixelSize);

/**
* @brief getVolumeUsePoints  函数用点计算体积，计算之前必须先设置零平面和像素当量！！！
* @param [in]   data                       //!<输入的3D模型
* @param [in]   pts                        //!<输入的用于体积的点集合，
* @return 当前区域体积,单位mm³
*/
DLLEXPORT  float getVolumeUsePoints(GC3DMetaData& data,std::vector<gc3d::GPoint>& pts,float pixelSize);

/**
* @brief iterateFitShpereUsePoint  迭代的方式拟合球面方程
* @param [in]   pts              //!<输入的3D点数组，函数将用输入的点进行球拟合
* @param [in]    percentage      //!<每次迭代之后选取的点云数量的百分比，范围0~1
* @param [in]    count           //!<迭代次数
* @param [inout]    A            //!<球面方程的系数
* @param [inout]    B            //!<球面方程的系数
* @param [inout]    C            //!<球面方程的系数
* @param [inout]    r            //!<球面方程的系数
* @note 输出球面方程  ： (x-A)²+(y-B)²+(z-C)²=r²
* @return
*/
DLLEXPORT  void iterateFitShpereUsePoint( std::vector<GPoint3f >& pts,float percentage,int count,double& A, double& B, double& C,double& r);

/**
* @brief getHeight  对一个区域进行分析
* @param [in]   data               //!<输入的3D模型
* @param [in]    region            //!<输入的区域
* @param [inout]    zmin           //!<区域内的最小值
* @param [inout]    minLocal       //!<区域内的最小值的具体位置
* @param [inout]    zmax           //!<区域内的最大值
* @param [inout]    maxLocal       //!<区域内的最大值的具体位置
* @param [inout]    zave           //!<区域内的平均值
* @param [in]    startPercent      //!<区域分析时并不是所有点都会计算，而是根据高度取其中间的一部分点，startPercent即根据高度的中间的起始百分比值
* @param [in]    endPercent        //!<区域分析时并不是所有点都会计算，而是根据高度取其中间的一部分点，endPercent即根据高度的中间的起始百分比值
* @return
*/
DLLEXPORT  bool getHeight(GC3DMetaData& data, std::vector<gc3d::GRegion>& region,float& zmin,GPoint& minLocal,float& zmax,GPoint& maxLocal,float& zave,const float startPercent,const float endPercent);

/**
* @brief  convexDetect 检测胶路，提取胶路宽、高和体积
* @param [in]   data                 //!<输入的3D模型
* @param [in]    rect                //!<包含胶路的矩形区域，需要超过胶路范围，胶路旁边必须是平面
* @param [in]    linePoints1         //!<输出的用来拟合第一条线的点
* @param [in]    linePoints2         //!<输出的用来拟合第二条线的点
* @param [inout]    glueWidth        //!<输出的胶路宽度
* @param [inout]    glueHeight       //!<输出的胶路高度
* @param [inout]    glueVolume       //!<输出的胶路体积
* @param [inout]    spliteHeight     //!<输入的胶路和平面的高度差分界线
* @return
*/
DLLEXPORT  bool convexDetect(gc3d::GC3DMetaData& data, gc3d::GRotationRect& rect,std::vector<GPoint>& linePoints1,std::vector<GPoint>& linePoints2,
                             float& glueWidth,float& glueHeight,float& glueVolume,float pixelSize,const float spliteHeight=0.05f);

/**
* @brief  calAngTwoVecs 计算两个向量之间的夹角
* @param [in]   A1                  //!<向量1 x
* @param [in]   B2                  //!<向量1 y
* @param [in]   C1                  //!<向量1 z
* @param [in]   A2                  //!<向量2 x
* @param [in]   B2                  //!<向量2 y
* @param [in]   C2                  //!<向量2 z
* @return 返回两个向量的夹角，单位角度
*/
DLLEXPORT double calAngTwoVecs(double A1, double B1, double C1, double A2, double B2, double C2);

}

#endif // GC3DMEASURE_H
