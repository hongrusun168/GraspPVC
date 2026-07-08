/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GCIGIMAGE_H
#define GCIGIMAGE_H
#include "../core/gc3dTypes.h"

namespace gc3d {

/**
 *  @defgroup GImage
 *  @note 目前仅支持8位单通道（GCI_8UC1）和24位（GCI_8UC3）三通道图像，注意不要重复写入
 */
class DLLEXPORT GImage
{
public:

    /**
    * @brief 默认构造函数
    * @return
   */
    GImage();

    /**
    * @brief 默认析构函数
    * @return
   */
    ~GImage();

    /**
    * @brief 构造函数，通过指定输入的图像以及图像的宽高
    * @param [in] data          //!<单通道图像的输入
    * @param [in] width         //!<单通道图像的宽
    * @param [in] height        //!<单通道图像的高
    * @param [in] type          //!<图像数据类型
    * @return 构造好的Gimage 类
   */
    GImage(uchar* data,int width,int height,GCIImageDataType type);

    /**
    * @brief （）括号重载函数，通过指定图像的区域进行图像ROI裁切
    * @param [in] roi           //!<输入图像的ROI
    * @return ROI部分数据
   */
    GImage operator()(const GRect& roi) const;

    /**
    * @brief （）括号重载函数，通过指定图像的区域进行图像ROI裁切
    * @param [in] roi           //!<输入图像的ROI
    * @return ROI部分数据
   */
    GImage& operator = (const GImage& src);
    /**
    * @brief copyTo 函数，GImage的复值函数
    * @param [out] image        //!<输入图像
    * @return
   */
    void copyTo(GImage& image) const;

    /**
    * @brief setValue 函数，GImage设置为同一的值（仅用于灰度图，如类型不一致设置失败）
    * @param [in] value         //!<设置的值
    * @return
   */
    void setValue(uchar value);

    /**
    * @brief setValueAt 函数，设置GImage某个位置的像素值（仅用于灰度图，如类型不一致设置失败）
    * @param [in] row           //!<行
    * @param [in] col           //!<列
    * @param [in] value         //!<设置的值
    * @return
   */
    void setValueAt(int row,int col,uchar value);

    /**
    * @brief setValue 函数，GImage设置为同一的值（仅用于彩色图，如类型不一致设置失败）
    * @param [in] r             //!<设置的红色通道值
    * @param [in] g             //!<设置的绿色通道值
    * @param [in] b             //!<设置的蓝色通道值
    * @return
   */
    void setValue(uchar r,uchar g, uchar b);

    /**
    * @brief setValueAt 函数，设置GImage某个位置的像素值（仅用于彩色图，如类型不一致设置失败）
    * @param [in] row           //!<行
    * @param [in] col           //!<列
    * @param [in] r             //!<设置的红色通道值
    * @param [in] g             //!<设置的绿色通道值
    * @param [in] b             //!<设置的蓝色通道值
    * @return
   */
    void setValueAt(int row,int col,uchar r,uchar g, uchar b);

    /**
    * @brief type 函数，获取图像数据类型
    * @return 图像数据类型
   */
    GCIImageDataType type() const;

    /**
    * @brief setType 设置图像数据类型，通常用户无需调用，若调用，可能会导致数据错误
    * @param type               //!<设置的图像数据类型
    * @return
   */
    void setType(const GCIImageDataType type);

    /**
    * @brief size 设置图像数据类型，通常用户无需调用，若调用，可能会导致数据错误
    * @return 图像的size
   */
    Size size() const;

    /**
    * @return 图像是否为空
   */
    bool empty() const;

    /**
    * @brief release 释放数据
    * @return
   */
    void release();

    /**
    * @brief 静态函数zeros，通过指定图像的宽高
    * @param [in] width             //!<单通道图像的宽
    * @param [in] height            //!<单通道图像的高
    * @param [in] type              //!<图像数据类型
    * @return 初始化为0的Gimage
    * @note 注意为静态函数，调用Gimage::Zeros()
   */
    static GImage zeros(int width, int height,GCIImageDataType type);

public:
    uchar* data;                     //!<单通道图像的数据，行优先
    int cols;                        //!<单通道图像列数
    int rows;                        //!<单通道图像行数
private:
    GCIImageDataType mtype;          //!<图像数据类型

};

/**
* @brief & 重载函数，两幅图像中都不为0的位置才不为0
* @param [in] src1          //!<输入灰度图像1
* @param [in] src2          //!<输入灰度图像2
* @return 输出的二值图像
*/
DLLEXPORT GImage operator & (const GImage& src1,const GImage& src2);

/**
* @brief | 重载函数，两幅图像中只要有一幅不为0，即为255
* @param [in] src1          //!<输入灰度图像1
* @param [in] src2          //!<输入灰度图像2
* @return 输出的二值图像
*/
DLLEXPORT GImage operator | (const GImage& src1,const GImage& src2);

/**
* @brief >= 重载函数，两幅图像中前者大的即为255，否则为0
* @param [in] src1          //!<输入灰度图像1
* @param [in] src2          //!<输入灰度图像2
* @return 输出的二值图像
*/
DLLEXPORT GImage operator >= (const GImage& src1,const GImage& src2);

/**
* @brief >= 重载函数，两幅图像中后者大的即为255，否则为0
* @param [in] src1          //!<输入灰度图像1
* @param [in] src2          //!<输入灰度图像2
* @return 输出的二值图像
*/
DLLEXPORT GImage operator <= (const GImage& src1,const GImage& src2);


}
#endif
