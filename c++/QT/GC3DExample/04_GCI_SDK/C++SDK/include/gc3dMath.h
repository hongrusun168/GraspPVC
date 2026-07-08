#pragma once
#ifndef GC3DMATH_H
#define GC3DMATH_H
#include <vector>
#include<array>
#include "core/gc3dTypes.h"
#include "imgproc/gc3dImage.h"
#include "imgproc/gc3dImgproc.h"
namespace gc3d {

/**
* @brief bestPolyFitting  曲线拟合
* @param [inout] pts               //!<输入的2D点数组，函数将用输入的点进行曲线拟合拟合
* @param [inout] coefficients      //!<输入的多项式系数，0是常数项，其中输入的大小-1是拟合的次数,输入的大小决定了拟合的次数
* @return
*/
extern "C" DLLEXPORT void bestPolyFitting(const std::vector<GPointf>& pts, std::vector<float>& coefficients);


/**
* @brief factorial  求阶乘
* @param [in] n:    //!<输入的待求的阶乘
* @return n！
*/
extern "C" DLLEXPORT int factorial(int n);
























}
#endif // GC3DMATH_H
