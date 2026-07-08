/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#include<vector>
#include "gc3dImage.h"
#include "../core/gc3dTypes.h"
#include<fstream>

namespace gc3d {

/**
 *  @defgroup GMatch 这个类包含了灰度模板匹配、轮廓匹配及其它相关操作
 */
class DLLEXPORT GMatchEx {
public:

    /**
    * @brief 默认构造函数
    * @return
   */
    GMatchEx();

    /**
    * @brief 默认析构函数
    * @return
   */
    ~GMatchEx();

    /**
    * @brief setPreProcImgType 用于设置图像预处理类型
    * @param [in] type    //!<需要定位的原始图像图像预处理类型 PRE_NOTHING 0 不做处理  PRE_MEDIAN 1 中值滤波 PRE_BINARY 2 二值化 PRE_MEDIANANDBINARY 3 中值滤波+二值化
    * @return
   */
    void setPreProcImgType(int type = PRE_NOTHING);

    /**
    * @brief setPreProcImgParams 用于设置图像预处理参数
    * @param [in] thresholdPre    //!<需要定位的原始图像二值化阈值
    * @param [in] medianRadius    //!<需要定位的原始图像中值滤波半径 可为 1 2
    * @return
   */
    void setPreProcImgParams(int thresholdPre,int medianRadius = 1);

    /**
    * @brief setCannyParams 用于设置canny的参数
    * @param [in] minThreshold      //!<需要定位的原始图像灰度低阈值
    * @param [in] maxThreshold      //!<需要定位的原始图像灰度高阈值
    * @param [in] apertureRadius    //!<需要定位的原始图像sobel掩模半径
    * @param [in] L2gradient        //!<需要定位的原始图像是否使用L2梯度
    * @return
   */
    void setCannyParams(int minThreshold,int maxThreshold, int apertureRadius = 1, bool L2gradient = true);

    /**
    * @brief setModelParams 用于设置模板通用参数
    * @param [in] angleBias    //!<需要定位的原始图像偏移角度(±)
    * @param [in] angleStep    //!<需要定位的原始图像角度步长
    * @param [in] maxBiasX     //!<需要定位的原始图像X方向最大偏移量
    * @param [in] maxBiasY     //!<需要定位的原始图像Y方向最大偏移量
    * @return
   */
    void setModelParams(double angleBias, double angleStep, int maxBiasX, int maxBiasY);

    /**
    * @brief setGrayModelParams 用于设置灰度模板额外参数
    * @param [in] grayThresholdValue    //!<需要定位的原始图像允许的灰度偏差，当在匹配时两个像素差小于grayThre时为有效，大于grayThre为false
    * @param [in] countErrValue         //!<需要定位的原始图像在快速判断是否为匹配区域时，countErrThre为跳过阈值，当错误点大于countErrThre时，则被快速跳过，否则匹配继续匹配，此参数为了加速匹配过程
    * @param [in] resizeValue           //!<需要定位的原始图像模板的缩放系数 0~1.0
    * @return
   */
    void setGrayModelParams(int grayThresholdValue,int countErrValue,double resizeValue);

    /**
    * @brief setModelType 用于设置模板匹配类型
    * @param [in] modelType         //!<需要定位的原始图像模板匹配的类型 MODELTYPE_GRAY 0 灰度匹配  MODELTYPE_BINARYEDGE 1 二值图像轮廓匹配 MODELTYPE_CANNY 2 canny边缘匹配
    * @return
   */
    void setModelType(int modelType = MODELTYPE_GRAY);

    /**
    * @brief getPreProcessImage 用于查看预处理图像预览
    * @param [in] imgSrc                //!<需要定位的原始图像输入的图像，必须是GCI_8UC1
    * @param [inout] imgPre             //!<需要定位的原始图像输出的图像，格式是GCI_8UC3
    * @return
   */
    void getPreProcessImage(gc3d::GImage &imgSrc,gc3d::GImage &imgPre);

    /**
    * @brief getModelPreviewImage 用于预览模板
    * @param [in] imgSrc                //!<需要定位的原始图像输入的图像，必须是GCI_8UC1
    * @param [inout] imgPre             //!<需要定位的原始图像输出的图像，格式是GCI_8UC3
    * @param [in] region                //!<需要定位的原始图像用来制作模板的区域
    * @return      成功返回true 失败返回false
   */
    bool getModelPreviewImage(gc3d::GImage &imgSrc,gc3d::GImage &imgPre,gc3d::GRegion region);

    /**
    * @brief createModel 用于设置图像预处理类型
    * @return   成功返回true 失败返回false
   */
    bool createModel();

    /**
    * @brief matchModel 匹配模板，得到最佳匹配的得分
    * @param [in] imgSrc                //!<需要定位的原始图像输入的图像，必须是GCI_8UC1
    * @param [inout] score              //!<需要定位的原始图像输出最佳匹配的得分
    * @return   成功返回true 失败返回false
   */
    bool matchModel(gc3d::GImage &imgSrc,double& score);

    /**
    * @brief affineTransRegions 匹配完成后，将区域仿射到目标位置
    * @param [in] inputRegions              //!<需要定位的原始图像输入的区域
    * @param [inout] outputRegions          //!<需要定位的原始图像输出仿射后的区域
    * @return   成功返回true 失败返回false
   */
    void affineTransRegions(std::vector<gc3d::GRegion>& inputRegions, std::vector<gc3d::GRegion>& outputRegions);

    /**
    * @brief resetMatchModel 重置模板类
    * @return   成功返回true 失败返回false
   */
    void resetMatchModel();

    /**
    * @brief getMatchedPositiondEdges 匹配完成后，获取匹配位置的模板点集(灰度匹配不适用)
    * @param [inout] mEdge              //!<需要定位的原始图像模板点集
    * @return
   */
    void getMatchedPositiondEdges(std::vector<gc3d::GPoint>& mEdge);

    /**
    * @brief getMatchRes 匹配完成后，获取匹配结果，包括角度，偏移量(灰度匹配不适用)
    * @param [inout] angel              //!<角度
    * @param [inout] tx                 //!<x偏移
    * @param [inout] ty                 //!<y偏移
    * @return
   */
    void getMatchRes(double& angel,int& tx,int& ty);

    /**
    * @brief saveModel 匹配完成后，获取匹配位置的模板点集(灰度匹配不适用)
    * @param [in] path              //!<保存路径
    * @return  成功返回true 失败返回false
   */
    bool saveModel(std::string path);

    /**
    * @brief loadModel 加载模板
    * @param [in] path              //!<加载路径
    * @return   成功返回true 失败返回false
   */
    bool loadModel(std::string path);

    /**
      @group 内部函数，不建议用户使用
      */
    /**
    * @brief serialize  序列化输出
    * @param [in] ofs      //!<标准文件流输出
    * @return
    * @note 该函数为内部应用函数，不建议使用
    */
    bool serialize(std::ofstream& ofs);

    /**
    * @brief deserialize  序列化读取
    * @param [in] ifs      //!<标准文件流输入
    * @return 成功返回true 失败返回false
    * @note 该函数为内部应用函数，不建议使用
    */
    bool deserialize(std::ifstream& ifs);

    /**
    * @brief getParams  参数读取
    * @param [inout] params             //!<读取的参数
    * @param [inout] paramsDescrib      //!<参数描述说明
    * @return 成功返回true 失败返回false
    * @note 该函数为内部应用函数，不建议使用
    */
    void getParams(std::vector<double>& params,std::vector<std::string>& paramsDescribes);

    /**
    * @brief setParams  参数读取
    * @param [inout] params             //!<设置的参数
    * @note 该函数为内部应用函数，不建议使用
    */
    void setParams(std::vector<double>& params);
};


}
