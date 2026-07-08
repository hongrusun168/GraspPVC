/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#include<vector>
#include "gc3dImage.h"
#include "../core/gc3dTypes.h"

namespace gc3d {

/**
 *  @defgroup GMatch
 *  @note 这个类是模板匹配类以及其相关的操作
 */
class DLLEXPORT GMatch {

public:

    /**
    * @brief 默认构造函数
    * @return
   */
    GMatch();

    /**
    * @brief 默认析构函数
    * @return
   */
    ~GMatch();

    /**
    * @brief setParams 用于设置匹配参数
    * @param [in] startAngle    //!<模板匹配的起始角度
    * @param [in] endAngle      //!<模板匹配的终止角度
    * @param [in] angleStep     //!<模板匹配的角度步长
    * @param [in] resizeValue   //!<为了加速，模板匹配时会缩放相应倍速
    * @return
   */
    void setParams(double startAngle, double endAngle, double angleStep,double resizeValue);

    /**
    * @brief makeTemplate 用于设置模板图像
    * @param [in] sourceImg     //!<做模板的原始图像
    * @param [in] roiRect       //!<将原始图像中的那一块感兴趣区域作为匹配图像
    * @return   成功返回true 失败返回false
   */
    bool makeTemplate(gc3d::GImage& sourceImg,const GRegion& roiRect);

    /**
    * @brief setAccuracyTemplate 用于设置更加精确的匹配位置的线特征，设置此参数可以使匹配更加精准
    * @param [in] reg1      //!<设置第一个区域，其中必须有一个线特征
    * @param [in] reg2      //!<设置第二个区域，其中必须有一个线特征，且不和第一条线平行
    * @return   成功返回true 失败返回false
   */
    bool setAccuracyTemplate(const GRegion& reg1, const GRegion& reg2);

    /**
    * @brief matches 用于设置开始匹配过程
    * @param [in] sourceImg         //!<需要定位的原始图像
    * @param [in] inputRegion       //!<需要定位的原始区域
    * @param [inout] outputRegion   //!<最终的输出的定位区域
    * @param [in] srchOffsetX       //!<允许的X方向偏差
    * @param [in] srchOffsetY       //!<允许的Y方向偏差
    * @param [in] grayThre          //!<允许的灰度偏差，当在匹配时两个像素差小于grayThre时为有效，大于grayThre为false
    * @param [in] countErrThre      //!<在快速判断是否为匹配区域时，countErrThre为跳过阈值，当错误点大于countErrThre时，则被快速跳过，否则匹配继续匹配，此参数为了加速匹配过程
    * @return   成功返回true 失败返回false
   */
    bool matches(gc3d::GImage& sourceImg,std::vector<GRegion>& inputRegion, std::vector<GRegion>& outputRegion,
                 const int srchOffsetX,const int srchOffsetY,const int grayThre=30,const int countErrThre=20);

    /**
    * @brief getMatches 用于设置获取和原始图像的角度和偏移
    * @param [in] sourceImg         //!<需要定位的原始图像
    * @param [in] srchOffsetX       //!<允许的X方向偏差
    * @param [in] srchOffsetY       //!<允许的Y方向偏差
    * @param [in] grayThre          //!<允许的灰度偏差，当在匹配时两个像素差小于grayThre时为有效，大于grayThre为false
    * @param [in] countErrThre      //!<在快速判断是否为匹配区域时，countErrThre为跳过阈值，当错误点大于countErrThre时，则被快速跳过，否则匹配继续匹配，此参数为了加速匹配过程
    * @return   成功返回true 失败返回false
   */
    bool getMatches(gc3d::GImage& sourceImg,const int srchOffsetX,const int srchOffsetY,const int grayThre,const int countErrThre);

    /**
    * @brief runMatch 用于设置输入一组区域，来获取定位区域，必须先调用getMatches函数
    * @param [in] inputRegion       //!<需要定位的原始图像需要定位的原始区域
    * @param [inout] outputRegion   //!<需要定位的原始图像最终的输出的定位区域
    * @return 无
   */
    void runMatch(std::vector<GRegion>& inputRegion, std::vector<GRegion>& outputRegion);

    /**
    * @brief drawMatches 用于设置绘制定位区域
    * @param [in] sourceImg     //!<需要定位的原始图像绘制的原始图
    * @param [in] regions       //!<需要定位的原始图像绘制的区域
    * @return 无
   */
    void drawMatches(gc3d::GImage& sourceImg, std::vector<GRegion>& regions);
};
}
