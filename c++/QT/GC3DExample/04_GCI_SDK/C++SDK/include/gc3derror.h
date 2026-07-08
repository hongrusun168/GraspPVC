/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3DERROR_H
#define GC3DERROR_H

//  GC3DErrorCode 错误码解析.
//
//返回的错误码是32位的int类型具体如下(前面的十六位一般用于的是GC3d的库，后面16位用于algorithm库)
//   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |S|DevId| partId|     Code      |S|LibId|        Err  Code      |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  where
//
//      S    - - indicates success/fail
//
//          0 - Success
//          1 - Fail (COERROR)
//
//      DevId - - 设备型号
//          0 - 单相机单光机
//          1 - 单相机二光机
//          2 - 2C1P
//          3 - 全部设备
//
//      PartId - - 部位
//          0 - 非实体部分
//          1 - 相机
//          2 - 光机
//      Code  - - 错误码
//      S    - - indicates success/fail
//
//          0 - Success
//          1 - Fail (COERROR)
//      LibId - - 库的名称
//          1 - algorithm库


#define GC3D_SUCCESS                         0x00000000  //成功
#define GC3D_PRODUCTTYPE_FAIL                0x70010000  //产品型号错误
#define GC3D_SOFTDOG_FAIL                    0x70020000  //加密狗错误
#define GC3D_1C1P_ERROR_EXPOSURE_NUMBER      0x80010000   //不支持的曝光次数
#define GC3D_1C1P_ERROR_CUDA_COMPUTE3D       0x80020000   //cuda解码错误
#define GC3D_1C1P_ERROR_CUDA_COMPUTE3D_HDR   0x80030000   //cuda多曝光融合错误
#define GC3D_1C1P_ERROR_METADATA_NONE        0x80040000   //数据为空，不可获取
#define GC3D_1C1P_ERROR_DEVICEINITIAL_REPEAT 0x80050000   //设备重复初始化
#define GC3D_1C1P_ERROR_REGISTER             0x80060000   //配准失败
#define GC3D_1C1P_ERROR_GRIDDATA_INVALID     0x80070000   //规则化网格数据无效
#define GC3D_1C1P_ERROR_REGISTED             0x80080000   //已经注册回调
#define GC3D_1C1P_ERROR_LOADPARAMS           0x80090000   //加载参数失败
#define GC3D_1C1P_ERROR_LOADFUNC             0x80011000   //加载函数失败
#define GC3D_1C1P_ERROR_OVERCAMERANUM        0x80012000   //超过相机数量
#define GC3D_1C1P_ERROR_OVERRECONTHRERANGE   0x80013000   //超过可设置的重建阈值范围，范围是0~256，且小阈值小于大阈值
#define GC3D_1C1P_ERROR_OVERDENOISERANGE     0x80014000   //超过可设置的降噪参数范围，1<=fmr<=3,0<=denoiseIndex1<=100,0.05<=denoiseIndex2<=50,0.05<=denoiseIndex3<=50
#define GC3D_1C1P_ERROR_HEIGHTRANGE          0x80015000   //高度范围设置错误，minHeight必须小于maxHeight
#define GC3D_1C1P_ERROR_METADATAERODE        0x80016000   //数据腐蚀错误，必须设置0,1,3,5,7,9...
#define GC3D_1C1P_ERROR_OVERGRIDSEARCHRANGE  0x80017000   //超过gridsearch范围，0<gridRange<100
#define GC3D_1C1P_ERROR_EXPOSURE_TIME        0x80018000     //不支持的曝光时间 100~maxExposureTime
#define GC3D_1C1P_ERROR_GAIN                 0x80019000     //不支持的增益 0~20
#define GC3D_1C1P_ERROR_CUDAMEMERROR         0x80020000     //显存分配错误
#define GC3D_1C1P_ERROR_DEVICEOPEN_REPEAT    0x80021000     //相机重复打开
#define GC3D_1C1P_ERROR_DEVICENOTOPEN        0x80022000     //相机未打开
#define GC3D_1C1P_ERROR_OVERHEARTBEATTIME    0x80023000     //超过相机心跳时间
#define GC3D_1C1P_ERROR_LOADCONFIG           0x80024000     //加载参数失败
#define GC3D_1C1P_ERROR_PROCESSLOCK          0x80025000     //进程锁初始化失败
#define GC3D_1C1P_ERROR_SCANMODE             0x80026000     //扫描模式设置错误
#define GC3D_1C1P_ERROR_MAJORCAM             0x80027000     //主相机设置错误
#define GC3D_1C1P_ERROR_RTSAVELOAD           0x80028000     //RT保存加载出错
#define GC3D_1C1P_ERROR_WHITEBALANCE         0x80029000     //不支持的白平衡参数
#define GC3D_1C1P_ERROR_OVERPARAMRANGE       0x80030000     //参数设置超过范围
#define  GC3D_1C1P_ERROR_PROJECT_TRIGER      0x82010000 //相机未触发
#define  GC3D_1C1P_ERROR_CAMERA_TRIGER       0x82020000 //相机掉帧
#define  GC3D_1C1P_ERROR_SET_TRIGER          0x82030000 //设置触发失败
#define  GC3D_1C1P_ERROR_CAMERA_LOSTPACKET   0x82040000 //相机丢包
#define GC3D_1C1P_CAM_CONNECT_FAIL           0x34300000  //相机连接失败
#define GC3D_1C1P_CAM_LIB_FAIL               0x34310000  //相机库匹配失败
#define GC3D_1C1P_CAM_INIT_FAIL              0x34320000  //相机初始化失败
#define GC3D_1C1P_ERROR_CAMERA_FRAME         0x34330000  //相机帧率不够
#define GC3D_1C1P_ERROR_CAMERA_NUM           0x34340000  //未查找到相机
#define GC3D_1C1P_ERROR_CAMERA_OPENED        0x34350000  //还有相机开启，请先关闭相机，再反初始化
#define GC3D_1C1P_CAM_UNCONNECTED            0x34360000  //相机掉线
#define GC3D_ALGORITHM_FITPLANE_FAIL         0x00009001  //平面拟合失败，矩阵奇异
#define GC3D_ALGORITHM_FITSPHERE_FAIL        0x00009002  //球拟合失败，矩阵奇异
#define GC3D_ALGORITHM_FITCIRCLE_FAIL        0x00009003  //圆拟合失败，矩阵奇异
#define GC3D_RGS_CALIBRATE_NUM_FAIL          0x00009004  //标定数据不够
#define GC3D_RGS_CALIBRATE_TYPE_FAIL         0x00009005  //机械手和标定类型设置不同


#endif // GC3DERROR_H
