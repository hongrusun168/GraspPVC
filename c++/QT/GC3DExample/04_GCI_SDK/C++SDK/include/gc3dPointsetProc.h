/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#pragma once
#ifndef GC3DPOINTSETPROC_H
#define GC3DPOINTSETPROC_H
#include <vector>
#include<array>
#include "core/gc3dTypes.h"
#include "imgproc/gc3dImage.h"
#include "imgproc/gc3dImgproc.h"
#include "feature2D/gc2dalgorithm.h"
#include "rgs/rgsdef.h"
namespace gc3d {

/**
* @brief cuInitialDev  初始化显卡
* @return
*/
DLLEXPORT  void cuInitialDev();

/**
* @brief computeInv  求旋转矩阵的逆矩阵
* @param [in] R:            //!<旋转矩阵
* @note R*RInv=E
* @return
*/
DLLEXPORT  void computeInv( gc3d::GRotation& R);

/**
* @brief computeInvRT  A = R*B + T -->  B = R'*A+T'
* @param [inout] R:            //!<旋转矩阵
* @param [inout] T:            //!<平移向量
* @note R*RInv=E
* @return
*/
DLLEXPORT  void computeInvRT( gc3d::GRotation& R,gc3d::GTranslation& T);

/**
* @brief computeRT  有序点云的配准
* @param [in] srcPoints     //!<输入的源有序点云
* @param [in] dstPoints     //!<输入的目标有序点云
* @param [in] R             //!<配准计算得到的旋转矩阵
* @param [in] T             //!<配准计算得到的位移向量
* @note srcPoints=R*dstPoints+T
* @return 配准点的最大误差
*/
DLLEXPORT  double computeRT(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints, gc3d::GRotation& R,gc3d::GTranslation& T);

/**
* @brief computeRT_14  有序点云的配准（支持非正交坐标系）
* @param [in] srcPoints     //!<输入的源无序的点云
* @param [in] dstPoints     //!<输入的目标无序的点云
* @param [in] R             //!<配准计算得到的旋转矩阵
* @param [in] T             //!<配准计算得到的位移向量
* @note srcPoints=R*dstPoints+T 最多14个点
* @return 配准点的最大误差
*/
DLLEXPORT  double computeRT_14(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints, gc3d::GRotation& R,gc3d::GTranslation& T);

/**
* @brief ICPGetRT  无序点云的配准
* @param [in] srcPoints     //!<输入的源无序的点云
* @param [in] dstPoints     //!<输入的目标无序的点云
* @param [out] R            //!<配准计算得到的旋转向量
* @param [out] T            //!<配准计算得到的位移向量
* @param [out] rs           //!<配准计算得到的中间旋转变量数组
* @param [out] ts           //!<配准计算得到的中间位移向量数组
* @param [out] icpParam     //!<配准参数
* @note srcPoints=R*dstPoints+T
* @return 配准质量，越接近于1  配准质量越好
*/
DLLEXPORT  double icpGetRT(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints,gc3d::GRotation& R,gc3d::GTranslation& T,
                      std::vector<gc3d::GRotation>& rs,std::vector<gc3d::GTranslation>& ts,gc3d::ICPParam icpParam = gc3d::ICPParam()) ;

/**
* @brief ICP  无序点云的配准，无需rts  ts
* @param [in] srcPoints     //!<输入的源无序的点云
* @param [in] dstPoints     //!<输入的目标无序的点云
* @param [out] R            //!<配准计算得到的旋转向量
* @param [out] T            //!<配准计算得到的位移向量
* @param [out] icpParam     //!<配准参数
* @note srcPoints=R*dstPoints+T
* @return 配准质量，越接近于1  配准质量越好
*/
DLLEXPORT  double icp(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints,gc3d::GRotation& R,gc3d::GTranslation& T,
                      gc3d::ICPParam icpParam = gc3d::ICPParam()) ;


/**
* @brief icpForRecPlane  矩形平面配准
* @param [in] srcPoints     //!<输入的源无序的点云
* @param [in] dstPoints     //!<输入的目标无序的点云
* @param [out] R            //!<配准计算得到的旋转向量
* @param [out] T            //!<配准计算得到的位移向量
* @param [out] icpParam     //!<配准参数
* @note srcPoints=R*dstPoints+T
* @return 配准质量，越接近于1  配准质量越好
*/
DLLEXPORT  double icpForRecPlane(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints,gc3d::GRotation& R,gc3d::GTranslation& T,
                      gc3d::ICPParam icpParam = gc3d::ICPParam()) ;

/**
* @brief icpMove  中心平移配准
* @param [in] srcPoints     //!<输入的源无序的点云
* @param [in] dstPoints     //!<输入的目标无序的点云
* @param [out] T            //!<配准计算得到的位移向量
* @note srcPoints=R*dstPoints+T
* @return
*/
DLLEXPORT  void icpMove(std::vector<GPoint3f>& srcPoints, std::vector<GPoint3f>&  dstPoints,gc3d::GTranslation& T);


/**
* @brief mergeRT  当有多组RT需要合并时，调用该函数可以进行合并
* @param [in] rs     //!<输入的旋转矩形数组
* @param [in] ts     //!<输入的平移向量数组
* @param [out] R     //!<输出的旋转矩阵
* @param [out] T     //!<输出的平移向量
* @note rs和ts的大小必须一样
* @return
*/
DLLEXPORT  void mergeRT(std::vector<GRotation>& rs, std::vector<GTranslation>&  ts, gc3d::GRotation& R,gc3d::GTranslation& T);



/**
* @brief convertPointsRT  无序点云的配准
* @param [in] points           //!<输入的点云
* @param [in] R                //!<旋转向量
* @param [in] T                //!<位移向量
* @return 返回旋转平移后的点云
*/
DLLEXPORT std::vector<GPoint3f> convertPointsRT(std::vector<GPoint3f>& points, gc3d::GRotation& R,gc3d::GTranslation& T);

/**
* @brief pointPolygonTest  检测点是否在轮廓内
* @param [in] points               //!<输入的点
* @param [in] point                //!<测试的点
* @return true在轮廓内，false不在轮廓内
*/
DLLEXPORT bool pointPolygonTest(std::vector<GPoint>& points, gc3d::GPoint point);

/**
* @brief cuGFillHole   对称补洞操作（GPU操作，必须带英伟达独立显卡才能调用成功）
* @param [inout] meta          //!<补洞的输入数据，补洞之后会在该地方传回
* @param [in] radius           //!<补洞的默认半径，1，3，5，...
* @param [in] threshold        //!<补洞的阈值，只有当输入的灰度大于一定的给定的阈值才会进行补洞，这个是针对高光部分一般将该值设置为200，低光补洞可以设置为0
* @param [in] times            //!<补洞迭代的次数
* @param [in] areaThreshold    //!<只有当洞的面积小于areaThreshold才会对该区域进行补洞
* @param [in] addmask         //!<预分配内存参数 大小：meta.imgW*meta.imgH
* @return
*/
DLLEXPORT  void cuGFillHole(GC3DMetaData& meta,int radius,int threshold,int times,int areaThreshold,unsigned char* addmask=nullptr);

/**
* @brief cuGFillHole   对称补洞操作（GPU操作，必须带英伟达独立显卡才能调用成功）
* @param [inout] meta          //!<补洞的输入数据，补洞之后会在该地方传回
* @param [in] radius           //!<补洞的默认半径，1，3，5，...
* @param [in] threshold        //!<补洞的阈值，只有当输入的灰度大于一定的给定的阈值才会进行补洞，这个是针对高光部分一般将该值设置为200，低光补洞可以设置为0
* @param [in] times            //!<补洞迭代的次数
* @param [in] areaThreshold    //!<只有当洞的面积小于areaThreshold才会对该区域进行补洞
* @param [in] holeFlag         //!<预分配内存参数 大小：meta.imgW*meta.imgH
* @param [in] holeIndex        //!<预分配内存参数 大小：meta.imgW*meta.imgH
* @param [in] dataIndex        //!<预分配内存参数 大小：meta.imgW*meta.imgH
* @return
*/
DLLEXPORT  void fillHole(GC3DMetaData& meta,int radius,int threshold,int times,int areaThreshold,
                                  unsigned char* holeFlag,int* holeIndex,int* dataIndex);

/**
* @brief calDepthImg  对data中的深度图（depthImage）进行重新计算
* @param [inout] data          //!<输入输出的数据
* @return
*/
DLLEXPORT  void calDepthImg(GC3DMetaData& data);

/**
* @brief calDepthImgWithRange  对data中的深度图（depthImage）进行重新计算
* @param [inout] data         //!<输入输出的数据
* @param [in] zMin            //!<灰度分布的起始高度，低于此阈值的灰度分布为0
* @param [in] zMax            //!<灰度分布的终止高度，高于此阈值的灰度分布为255
* @param [in] isNeedTexture   //!<是否添加纹理图权重
* @param [in] textureRatio    //!<纹理权重的占比
* @return
*/
DLLEXPORT  void calDepthImgWithRange(GC3DMetaData& data,float zMin,float zMax,bool isNeedTexture = false ,float textureRatio = 0.01f);

/**
* @brief calDepthImgGData  对data中的深度图（depthImage）进行重新计算
* @param [ino] data             //!<输入的数据
* @param [inout] depthImage     //!<输入输出的灰度分布的深度图
* @return
*/
DLLEXPORT  void calDepthImgGData(GC3DGridData& data,unsigned char* depthImage);

/**
* @brief changeDataFromBasePlane  根据给定的基准面，对GC3DMetaData中的点云数据进行校正
* @param [in] basePlaneA       //!<平面方程参数A
* @param [in] basePlaneB       //!<平面方程参数B
* @param [in] basePlaneC       //!<平面方程参数C
* @param [in] aveHeight        //!<z方向的平移距离
* @note  基准面方程: basePlaneA*x+basePlaneB*y+basePlaneC*z+1=0
* @return
*/
DLLEXPORT  void changeDataFromBasePlane(GC3DMetaData& data,double basePlaneA,double basePlaneB,double basePlaneC,double aveHeight);

/**
* @brief get3DPointsFromContours  获取2D轮廓中的3D轮廓点
* @param [inout] data         //!<输入输出的数据
* @param [in] contours        //!<输入的2D轮廓点集
* @param [inout] outputPoints //!<输入输出的3D轮廓点集
* @return
*/
DLLEXPORT  void get3DPointsFromContours(GC3DMetaData& data,std::vector<GPoint>& contours,std::vector<GPoint3f>& outputPoints);

/**
* @brief gridData  数据的网格化(cpu版本)
* @param [inout] data          //!<输入输出的数据
* @param [inout] deviceInfo    //!<设备信息
* @param [in] mode             //!<选择网格化的模式
* @param [inout] width         //!<宽度
* @param [inout] height        //!<高度
* @param [inout] xmin          //!<x的最小值
* @param [inout] xmax          //!<x的最大值
* @param [inout] ymin          //!<y的最小值
* @param [inout] ymax          //!<y的最大值
* @param [inout] dx            //!<x方向的间距
* @param [inout] dy            //!<y方向的间距
* @param [inout] ZMat          //!<对应的高度
* @param [inout] ZMask         //!<是否有点的mask矩阵
* @param [inout] textureData   //!<网格化后的纹理图
* @note  mode==0是固定分辨率，设置好width ,height 从而自动得到xmin ,xm ax ymin, ymax ,dx,dy
* @note  mode==1是固定距离,设置好dx dy 自动得到xmin ,xmax ymin, ymax ,width ，height
* @note  mode==2是固定xmin ,xmax ymin, ymax ,dx,dy自动得到width，height
* @return
*/
DLLEXPORT  bool gridData(GC3DMetaData& data,gc3d::DeviceInformation&deviceInfo, int mode, int& width,
                         int &height,float& xmin,float& xmax,float& ymin,float& ymax, float &dx,
                         float &dy, float*&ZMat, bool *&ZMask,unsigned char*&textureData);

/**
* @brief rot2BaseByMask
* @param [inout] data      //!<要旋转的原数据
* @param [in] mask         //!<取的基准面
* @return
*/
DLLEXPORT  void rot2BaseByMask(GC3DMetaData& data,unsigned char* mask);
/**
* @brief getFixPlaneParams  获取设置固定平面的参数
* @param [inout]  data             //!<输入的3D数据
* @param [in] mask          //!<拟合平面用的区域
* @param [inout] plane_A    //!<平面方程系数A
* @param [inout] plane_B    //!<平面方程系数B
* @param [inout] plane_C    //!<平面方程系数C
* @param [inout] center_X    //!<旋转平面的中心点X
* @param [inout] center_Y    //!<旋转平面的中心点Y
* @param [inout] center_Z    //!<旋转平面的中心点Z
* @return 是否获取平面参数成功
*/
DLLEXPORT bool getFixPlaneParams(GC3DMetaData& data,unsigned char* mask,float& plane_A,
                                 float& plane_B,float& plane_C,float& center_X,float& center_Y,float& center_Z);

/**
* @brief setFixPlaneParams  设置固定平面为0平面，对点云校准
* @param [inout]  data             //!<输入的3D数据
* @param [in] plane_A    //!<平面方程系数A
* @param [in] plane_B    //!<平面方程系数B
* @param [in] plane_C    //!<平面方程系数C
* @param [in] center_X    //!<旋转平面的中心点X
* @param [in] center_Y    //!<旋转平面的中心点Y
* @param [in] center_Z    //!<旋转平面的中心点Z
* @return
*/
DLLEXPORT void setFixPlaneParams(GC3DMetaData& data,float plane_A,float plane_B,float plane_C,
                                 float center_X,float center_Y,float center_Z);


/**
* @brief setPlaneUsePlaneParams  使用目录下的"gPlaneParams.gp"文件更新点云
* @param [inout]  data             //!<输入的3D数据
* @return
*/
DLLEXPORT void setPlaneUsePlaneParams(GC3DMetaData& data);

/**
* @brief setPlaneUsePlaneParamsBySerial  使用目录下对应序列号的".gp"文件更新点云
* @param [inout]  data             //!<输入的3D数据
* @param [in]    camSerial         //!<相机序列号
* @return 若对应相机的平面不存在返回false,否则返回true
*/
DLLEXPORT bool setPlaneUsePlaneParamsBySerial(GC3DMetaData& data,std::string camSerial);


/**
* @brief writePCDFile  保存PCD格式的文件
* @param [inout] data      //!<要保存的数据
* @param [in] wpath       //!<保存的完整路径，包含文件名
* @return
*/
DLLEXPORT  void writePCDFile(GC3DMetaData& data,std::string wpath);
/**
* @brief writePLYFile  保存PLY格式的文件
* @param [inout] data      //!<要保存的数据
* @param [in] wpath       //!<保存的完整路径，包含文件名
* @return
*/
DLLEXPORT  void writePLYFile(GC3DMetaData& data,std::string wpath);

}

#endif // GC3DPOINTSETPROC_H
