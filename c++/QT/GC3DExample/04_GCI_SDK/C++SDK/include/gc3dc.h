#ifndef GC3DC_H
#define GC3DC_H
#include "gc3derror.h"
#include "gc3ddef.h"

//初始化系统，包含打开相机
extern "C" DLLEXPORT bool initialSystem();

//关闭相机，并反初始化系统
extern "C" DLLEXPORT void unInitialSystem();

//扫描 2D获取  2D灰度图
extern "C" DLLEXPORT bool snap2D(unsigned char* tex2D);

//获取3D 的灰度图
extern "C" DLLEXPORT void getTex(unsigned char* tex);

//扫描3D
extern "C" DLLEXPORT bool snap3D();

//计算3D数据 获取点云Z值
extern "C" DLLEXPORT bool compute3DZmap(float* zmap);

//计算3D数据 获取点云
extern "C" DLLEXPORT bool compute3D(float* xmap,float* ymap,float* zmap);

//获取图像宽
extern "C" DLLEXPORT int getWidth();

//获取图像高
extern "C" DLLEXPORT int getHeight();

//设置曝光时间1
extern "C" DLLEXPORT bool setExposureTime1(int exposureTime);

//设置曝光时间2
extern "C" DLLEXPORT bool setExposureTime2(int exposureTime);

//设置曝光时间3
extern "C" DLLEXPORT bool setExposureTime3(int exposureTime);

//设置白平衡系数
extern "C" DLLEXPORT bool setWhiteBalanceRatio(double redRatio,double greenRatio,double blueRatio);

//设置2D采集的曝光时间和增益
extern "C" DLLEXPORT bool setCamParam2D(int exposureTime,double gain);

//设置降噪指数
extern "C" DLLEXPORT bool setDenoiseParameters(int fmr,float den1,float den2,float den3);

//设置高度范围
extern "C" DLLEXPORT bool setHeightRange(float minH,float maxH);

//设置曝光次数
extern "C" DLLEXPORT bool setExposureNum(int time);

//保存点云
extern "C" DLLEXPORT void savePoints(char* savePath);

//获取像素大小
extern "C" DLLEXPORT float getPixelSize();

//保存图像接口
extern "C" DLLEXPORT void saveBmpImage(unsigned char* data,int width,int height,int channel);

//补洞
extern "C" DLLEXPORT void fillHole(int radius,int areaThreshold,int times);


#endif // GC3DC_H
