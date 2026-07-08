/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3DRGS_H
#define GC3DRGS_H

#include<iostream>
#include<vector>
#include"../core/gc3dCoreTypes.h"
#include "rgsdef.h"


namespace gc3d {

/**
    * @brief calibrate 标定函数接口
    * @param [in] gripLocals        //!<机械手坐标数组，一般是九个
    * @param [in] cameraPoints      //!<相机坐标系下的点云数组，一般九个（必须和gripLocals大小一致）
    * @param [in] R                 //!<标定结果，相机坐标系到机械手坐标系的旋转矩阵
    * @param [in] T                 //!<标定结果，相机坐标系到机械手坐标系的平移向量
    * @param [in] error             //!<标定输出的误差结果，详细见结构体说明
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @param [in] calType           //!<标定类型，分为眼在手上，眼在手外
    * @param [in] ptType            //!<点云类型，分为有序和无序
    * @param [in] icpParam          //!<点云ICP参数，在无序点云配准时才会用到
    * @note 该手眼标定方法采用Daniilidis算法
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  calibrate(std::vector<gc3d::GripperLocal>& gripLocals,std::vector<std::vector<gc3d::GPoint3f>>& cameraPoints,
                                           gc3d::GRotation& R,gc3d::GTranslation& T,gc3d::CalibrateError& error,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER,
                                           GCICALTYPE calType=GCI_EYEINHAND,GCIPOINTTYPE ptType=GCI_POINT_UNORDER,ICPParam icpParam=ICPParam());
/**
    * @brief handToBasePoint    //!<用于将机械手坐标系下的点转换到基座坐标系
    * @param [in] srcp          //!<输入的点
    * @param [in] grip          //!<机械手的当前坐标
    * @param [inout] dstp       //!<输出的点
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  handToBasePoint(const GPoint3f srcp,GripperLocal& grip,GPoint3f& dstp,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);
/**
    * @brief convertToBasePoint //!<用于将相机坐标系下的点转换到基座坐标系
    * @param [in] srcp          //!<输入的点
    * @param [in] grip          //!<机械手的当前坐标
    * @param [inout] dstp       //!<输出的点
    * @param [in] R             //!<输入的相机到机械手的旋转矩阵
    * @param [in] T             //!<输入的相机到机械手的平移向量
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  convertToBasePoint(const GPoint3f srcp,GripperLocal& grip,GPoint3f& dstp,const gc3d::GRotation R,
                                                    const gc3d::GTranslation T,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);
/**
    * @brief convertToHandPoint //!<用于将相机坐标系下的点转换到法兰坐标系
    * @param [in] srcp          //!<输入的点
    * @param [inout] dstp       //!<输出的点
    * @param [in] R             //!<输入的相机到机械手的旋转矩阵
    * @param [in] T             //!<输入的相机到机械手的平移向量
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  convertToHandPoint(const GPoint3f srcp,GPoint3f& dstp,const gc3d::GRotation R,const gc3d::GTranslation T);
/**
    * @brief localGrips 用于定位的函数，该算法采用icp的方式进行配准，对模板点集和当前点集的大小无要求
    * @param [in] srcps             //!<输入的模板点集
    * @param [in] dstps             //!<输入的当前点集
    * @param [in] teachGrips        //!<需要定位的机械手坐标点数组
    * @param [inout] outGrips       //!<输出的定位的机械手坐标点数组
    * @param [inout] rot            //!<旋转角度参数
    * @param [in] icpThre           //!<允许的配准误差范围
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @note 该定位方法采用点云icp的方式计算
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  localGrips(std::vector<GPoint3f>& srcps,std::vector<GPoint3f>& dstps,std::vector<GripperLocal>& teachGrips,
                                            std::vector<GripperLocal>& outGrips,ICPParam icpParam,double& icpThre,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);
/**
    * @brief localGrips 用于定位的函数，该算法采用icp的方式进行配准，对模板点集和当前点集的大小无要求
    * @param [in] srcps             //!<输入的模板点集
    * @param [in] dstps             //!<输入的当前点集
    * @param [in] teachGrips        //!<需要定位的机械手坐标点数组
    * @param [inout] outGrips       //!<输出的定位的机械手坐标点数组
    * @param [in] icpThre           //!<允许的配准误差范围
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @note 该定位方法采用点云注册的方式计算，srcps和dstps的大小必须一致，否则调用失败
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS  localGripsNP(std::vector<GPoint3f>& srcps,std::vector<GPoint3f>& dstps,std::vector<GripperLocal>& teachGrips,
                                               std::vector<GripperLocal>& outGrips,const double icpThre = 2,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);


/**
    * @brief localGripsFromRT 用于定位的函数，已知旋转平移矩阵，定位最终的机械手坐标
    * @param [in] R                 //!<输入的旋转矩阵
    * @param [in] T                 //!<输入的平移向量
    * @param [in] teachGrips        //!<需要定位的机械手坐标点数组
    * @param [inout] outGrips       //!<输出的定位的机械手坐标点数组
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS localGripsFromRT(gc3d::GRotation& R,gc3d::GTranslation& T,std::vector<GripperLocal>& teachGrips,
                        std::vector<GripperLocal>& outGrips,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);

/**
    * @brief localGripFromRT 用于定位的函数，已知旋转平移矩阵，定位最终的机械手坐标
    * @param [in] R                 //!<输入的旋转矩阵
    * @param [in] T                 //!<输入的平移向量
    * @param [in] teachGrip        //!<需要定位的机械手坐标点数组
    * @param [inout] outGrips       //!<输出的定位的机械手坐标点数组
    * @param [in] rotOrd            //!<机械手的旋转顺序，分为内旋、外旋及顺序
    * @return GC3D_SUCCESS or GC3D_SOFTDOG_FAIL
   */
extern "C" DLLEXPORT RGS_STATUS localGripFromRT(gc3d::GRotation& R,gc3d::GTranslation& T,GripperLocal& teachGrip,
                                                GripperLocal& outGrip,GCIROBOTROTORD rotOrd =  GCI_XYZ_UNORDER);

}
#endif
