/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GCIIMGPROC_H
#define GCIIMGPROC_H

#include <vector>
#include "../core/gc3dTypes.h"
#include "gc3dImage.h"

namespace gc3d {

/**
* @brief getCurTime 当前时钟
* @return
*/
DLLEXPORT double getCurTime();

/**
* @brief getFreqTime 当前单个时钟长度，单位ms
* @return
*/
DLLEXPORT double getFreqTime();

/**
* @brief imread 读取GImage图像,仅支持读取8位单通道bmp图像
* @param [in] path              //!<图像路径
* @param [inout] image          //!<输出的图像
* @return
*/
DLLEXPORT GImage imread(std::string path);

/**
* @brief imwrite 写入GImage图像
* @param [in] path              //!<图像路径
* @param [inout] image          //!<输入的图像
* @return
*/
DLLEXPORT void imwrite( std::string path,GImage& image);

/**
* @brief imshow 将图像显示到窗口，当按下空格键继续
* @param [in] windowName        //!<窗口名字
* @param [in] image             //!<需要显示的图像
* @return
*/
DLLEXPORT void imshow(std::string windowName, gc3d::GImage& img);


/**
* @brief resize 上采样或下采样图像
* @param [in] inputImage        //!<输入图像
* @param [out] outputImage      //!<输出图像
* @param [in] dstW              //!<输出图像宽
* @param [in] dstH              //!<输出图像高
* @return
*/
DLLEXPORT void resize(const GImage& inputImage, GImage& outputImage, int dstW,int dstH);

/**
* @brief matchTemplate 模板匹配函数加速版实现
* @param [in] sourceImage       //!<原图像
* @param [in] tempImage         //!<模板图
* @param [inout] minLocal       //!<最佳匹配位置
* @param [inout] matchScore     //!<最佳匹配值
* @param [inout] scoreThre      //!<匹配的分数阈值
* @param [in] methd             //!<匹配方法
* @return
*/
DLLEXPORT void matchTemplate(GImage& sourceImage,GImage& tempImage,GPoint& minLocal, float& matchScore,float scoreThre, GCIMatchType methd=GCIMatchType::GCI_TM_SQDIFF);

/**
* @brief matchTemplate 模板匹配函数加速版实现
* @param [in] sourceImage       //!<原图像
* @param [in] tempImage         //!<模板图
* @param [inout] minLocal       //!<最佳匹配位置
* @param [inout] minValue       //!<最佳匹配值
* @return true  匹配成功，否则匹配失败
*/
DLLEXPORT bool matchTemplateAcc(const GImage& sourceImage,GImage& tempImage,GPoint& minLocal, double& minValue,
                                                    const int grayThre,const int countErrThre,const GPoint center,const int offsetX,const int offsetY);

/**
* @brief sobel函数 sobel算子
* @param [inout] image                  //!<输入图像
* @param [in] sobelThreshold:sobel      //!<阈值，X和Y方向边缘梯度之和要大于该值才被认为是边缘
* @return
*/
DLLEXPORT void sobel(const GImage& inputImage, GImage& outputImage, const int sobelThreshold=40);

/**
* @brief canny函数 canny
* @param [in] src                           //!<输入图像
* @param [out] dst                          //!<输入图像
* @param [in] lowThresh                     //!<边缘低阈值
* @param [in] highThresh                    //!<边缘高阈值
* @param [in] kRadius:sobel                 //!<滤波核的半径
* @param [in] L2gradient                    //!<是否使用L2梯度计算
* @return
*/
DLLEXPORT void canny(const gc3d::GImage& src, gc3d::GImage& dst, unsigned char lowThresh=50, unsigned char highThresh=100,
                 int kRadius = 1, bool L2gradient = true);

/**
* @brief blur函数 均值滤波
* @param [inout] image                  //!<输入图像
* @param [out] outputImage              //!<输出图像
* @param [in] size                      //!<滤波窗口大小，仅为:3,5,7,9...
* @return 是否成功
*/
DLLEXPORT bool blur(const GImage& inputImage, GImage& outputImage,const int size);

/**
* @brief medianBlur 中值滤波
* @param [inout] inputImage             //!<输入图像
* @param [inout] outputImage            //!<输入图像
* @param [in] kernelRadius              //!<滤波半径大小
* @return 是否成功
*/
DLLEXPORT bool medianBlur(const GImage& inputImage, GImage& outputImage,const int kernelRadius);

/**
* @brief ostu函数 大津法阈值分割
* @param  [inout] inputImage         //!<输入图像
* @param  [inout] outputImage        //!<输出图像
* @return 返回分割阈值
*/
DLLEXPORT int ostu(const GImage& inputImage, GImage& outputImage);

/**
* @brief erode 图像腐蚀
* @param  [inout] inputImage         //!<输入图像
* @param  [inout] outputImage        //!<输出图像
* @param [in] size                   //!<腐蚀窗口大小
* @return 是否成功
*/
DLLEXPORT bool erode(const gc3d::GImage& inImg,gc3d::GImage& outImg, const int size);

/**
* @brief dilate 图像膨胀
* @param  [inout] inputImage         //!<输入图像
* @param  [inout] outputImage        //!<输出图像
* @param [in] size                   //!<膨胀窗口大小
* @return 是否成功
*/
DLLEXPORT bool dilate(const gc3d::GImage& inImg,gc3d::GImage& outImg, const int size);

/**
* @brief threshold 快速阈值化
* @param  [inout] inputImage         //!<输入图像
* @param  [inout] outputImage        //!<输出图像
* @param  [inout] threValue          //!<阈值化分割值
* @param  [inout] setValue           //!<大于分割值设置为多少
* @param [in] mode                   //!<阈值化模式，是大于分割值设置为有效还是小于分割值设置为有效
* @return 是否成功
*/
DLLEXPORT void threshold(const gc3d::GImage& inImg,gc3d::GImage& outImg, const int threValue,const uchar setValue,GCIThresholdTypes mode);

/**
* @brief thinImage 骨架提取
* @param [in] srcImg                //!<输入图像，必须是二值的
* @param [out] dstImg               //!<输出图像，>0的点为骨架点
* @return 是否成功
*/
DLLEXPORT void thinImage(gc3d::GImage&  srcImg, gc3d::GImage& dstImg);

/**
* @brief fastDistanceTrans 快速距离变换
* @param [in] imgSrc                //!<输入二值图像
* @param [out] imgDst               //!<输出图像
* @return
*/
DLLEXPORT void fastDistanceTrans(gc3d::GImage& imgSrc,gc3d::GImage& imgDst);

/**
* @brief bilateralFilters -双边滤波
* @param  [inout] inputImage         //!<输入图像
* @param  [inout] outputImage        //!<输出图像
* @param [in] size                   //!<滤波窗口大小
* @param [in] sigma_s                //!<空域权重
* @param [in] sigma_r                //!<像素域权重
* @return
*/
DLLEXPORT void bilateralFilters(const gc3d::GImage& inImg,gc3d::GImage outImg,const int size,const float sigma_s,const float sigma_r);

/**
* @brief convertVaildMatToGImage 将metadata中的有效数组转化成Gimage
* @param [inout] gimage             //!<输出图像，有效点255，无效点0
* @param [in] maskflag              //!<有效位标志矩阵
* @return
*/
DLLEXPORT void convertVaildMatToGImage(gc3d::GImage& gimage,bool *maskflag);


/**
* @brief gciFindContours 轮廓查找函数
* @param [in] img                   //!<输入的要查找的轮廓图，必须是8位二值图像
* @param [out] contours             //!<输出的轮廓点
* @param [in] ground                //!<轮廓背景，可选黑色或白色，默认黑色位背景
* @return
*/
DLLEXPORT void findContours(const gc3d::GImage& img, std::vector<std::vector<gc3d::GPoint>> &contours,GCIContourGround ground=GCIContourGround::GCI_BLACK_BACKGROUND);

/**
* @brief drawContours 绘制轮廓
* @param [inout] img                //!<要绘制的轮廓图像
* @param [in] contours              //!<要绘制的轮廓
* @param [in] conIndex              //!<要绘制的轮廓索引，如果为<0，则全部绘制
* @param [in] color                 //!<绘制的轮廓颜色
* @param [in] thickness             //!<绘制时线条的宽度，若<0,则轮廓填充
* @return
*/
DLLEXPORT void drawContours(gc3d::GImage& img, std::vector<std::vector<gc3d::GPoint>> &contours,int conIndex,const Scalar& color,int thickness);


/**
* @brief drawConvexHull 绘制凸包
* @param [inout] img                //!<要绘制的图像
* @param [in] hull                  //!<要绘制的凸包
* @param [in] color                 //!<绘制的凸包颜色
* @param [in] thickness             //!<绘制时线条的宽度
* @return
*/
DLLEXPORT void drawConvexHull(gc3d::GImage& img, std::vector<gc3d::GPoint>& hull,const Scalar& color,int thickness);



/**
* @brief line 绘制直线
* @param [inout] img                //!<要绘制的轮廓图像
* @param [in] p1                    //!<要绘制的直线起点
* @param [in] p2                    //!<要绘制的直线终点
* @param [in] color                 //!<绘制的轮廓颜色
* @param [in] thickness             //!<绘制时线条的宽度
* @return
*/
DLLEXPORT void line(gc3d::GImage& img, gc3d::GPoint p1,gc3d::GPoint p2,const Scalar& color,int thickness);

/**
* @brief circle 绘制圆
* @param [inout] img                //!<要绘制的图像
* @param [in] center                //!<要绘制的圆中心点
* @param [in] radius                //!<要绘制的圆半径
* @param [in] color                 //!<绘制的圆颜色
* @param [in] thickness             //!<绘制时线条的宽度，若<0,则圆内填充
* @return
*/
DLLEXPORT void circle(gc3d::GImage& img, gc3d::GCircle cir,const Scalar& color,int thickness);

/**
* @brief rectancgle 绘制矩形
* @param [inout] img                //!<要绘制的图像
* @param [in] rect                  //!<要绘制的矩形
* @param [in] color                 //!<绘制的矩形颜色
* @param [in] thickness             //!<绘制时线条的宽度，若<0,则矩形内填充
* @return
*/
DLLEXPORT void rectangle(gc3d::GImage& img, gc3d::GRect rect,const Scalar& color,int thickness);

/**
* @brief rectangleRot 绘制旋转矩形
* @param [inout] img                //!<要绘制的图像
* @param [in] rect                  //!<要绘制的矩形
* @param [in] color                 //!<绘制的矩形颜色
* @param [in] thickness             //!<绘制时线条的宽度，若<0,则矩形内填充
* @return
*/
DLLEXPORT void rectangleRot(gc3d::GImage& img, gc3d::GRotationRect rect,const Scalar& color,int thickness);


/**
* @brief cvtColor 颜色转换函数
* @param [in] img                   //!<输入的要转换的图像
* @param [in] cvtCode               //!<颜色转换代码
* @return GImage 返回转换后的图像
*/
DLLEXPORT void cvtColor(gc3d::GImage& inImg,gc3d::GImage& outImg, GCIColorConversionCodes cvtCode);


/**
* @brief spliteImage 颜色通道分离，分为红绿蓝
* @param [in] inImg                 //!<输入的彩色图像，黑白图不做任何操作
* @param [in] outImgs               //!<红绿蓝三个通道的图像数据
* @return
*/
DLLEXPORT void spliteImage(gc3d::GImage& inImg,std::vector<gc3d::GImage>& outImgs);

/**
* @brief getPosition2D 用于筛选2D图像上的单个边缘点
* @param [in] imgSrc                //!<输入图像
* @param [in] pStart                //!<测量起点
* @param [in] pEnd                  //!<测量终点
* @param [inout] pRlt               //!<输出的结果位置
* @param [in] roiRadius             //!<垂直测量方向的平滑尺度半径
* @param [in] threshould            //!<边缘点最低阈值
* @param [in] isPositive            //!<true选择从低到高的边缘 false 选择从高到低的边缘
* @param [in] isFirst               //!<true选择第一个满足的条件的位置点 false 选择最后一个满足条件的位置点
* @return 是否成功
*/
DLLEXPORT bool getPosition2D(const gc3d::GImage& imgSrc,gc3d::GPointf pStart,gc3d::GPointf pEnd,gc3d::GPointf& pRlt,
                   int roiRadius,int threshould,bool isPositive = true,bool isFirst = true);

/**
* @brief getEdgesLinePositions 用于筛选2D图像上的边缘线
* @param [in] imgSrc                //!<输入图像
* @param [inout] pRlt               //!<输出的结果点集,无效值为(-1,-1)
* @param [in] pStart                //!<ROI中心线起点
* @param [in] pEnd                  //!<ROI中心线终点
* @param [in] numElement            //!<需要测量的单个点数量
* @param [in] roiW                  //!<单个点的平滑尺度直径
* @param [in] roiH                  //!<单个点的测量范围
* @param [in] threshould            //!<边缘点最低阈值
* @param [in] isPositive            //!<true 选择从低到高的边缘 false 选择从高到低的边缘
* @param [in] isFirst               //!<true 选择第一个满足的条件的位置点 false 选择最后一个满足条件的位置点
* @note 默认的测量方向为起点到终点方向的逆时针90度方向
* @return 是否成功
*/
DLLEXPORT bool getEdgesLinePositions(const gc3d::GImage& imgSrc,std::vector<gc3d::GPointf>& pRlts,gc3d::GPoint pStart,gc3d::GPoint pEnd,
                           int numElement,int roiW,int roiH,int threshould, bool isPositive = true,bool isFirst = true);

/**
* @brief putText函数 在图像上绘制文字
* @param [inout] imgSrc              //!<输入图像
* @param [in] str                   //!<需要绘制的文字
* @param [in] org                   //需要绘制的位置
* @param [in] color                 //绘制颜色
* @param [in] fontSize              //绘制字体大小
* @param [in] fn                    //字体类型
* @param [in] italic                //是否斜体
* @param [in] underline             //是否需要下划线
* @return
*/
DLLEXPORT void putText(gc3d::GImage &imgSrc, const char* str, gc3d::GPoint org, gc3d::Scalar color, int fontSize,
                       const char *fn="Arial", bool italic=false, bool underline=false);


/**
* @brief sobel函数 sobel算子 CUDA实现
* @param [inout] image              //!<输入图像
* @param [in] sobelThreshold        //!<sobel阈值
* @return
*/
DLLEXPORT void cuSobel(gc3d::GImage& image, const int sobelThreshold);

/**
* @brief cuBlur 均值滤波 CUDA实现
* @param [inout] image              //!<输入图像
* @param [in] size                  //!<滤波窗口大小，仅为:3,5,7,9...
* @return 是否成功
*/
DLLEXPORT bool cuBlur(gc3d::GImage& gimage,const int size);

/**
* @brief cuErode 图像腐蚀 CUDA实现
* @param [inout] image              //!<输入图像
* @param [in] size                  //!<腐蚀窗口大小
* @return 是否成功
*/
DLLEXPORT bool cuErode(gc3d::GImage& image, const int size);

/**
* @brief cuDilate 图像膨胀 CUDA实现
* @param [inout] image          //!<输入图像
* @param [in] size              //!<膨胀窗口大小
* @return 是否成功
*/
DLLEXPORT bool cuDilate(gc3d::GImage& image, const int size);

/**
* @brief cuBilateralFilters -双边滤波 CUDA实现
* @param [inout] gimage         //!<输入图像
* @param [in] size              //!<滤波窗口大小
* @param [in] sigma_s           //!<空域权重
* @param [in] sigma_r           //!<像素域权重
* @return
*/
DLLEXPORT void cuBilateralFilters(gc3d::GImage& gimage,const int size,const float sigma_s,const float sigma_r);





}

#endif // GCIIMGPROC_H
