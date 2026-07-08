/*///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2022, GCI Corporation, all rights reserved.
///////////////////////////////////////////////////////////////////////////////////////*/
#ifndef GC3D_H
#define GC3D_H
#include "gc3dc.h"

namespace gc3d {
/**
* @brief 函数initialDevice负责初始化设备
* @param [inout] infos 检测到的可用的设备信息
* @param [inout]  devSize 检测到的可用的设备数量
* @param [in]  updataGData 是否强制更新GData
* @return 返回是否初始化成功错误代码
*/
DLLEXPORT uint32_t initialDevice(DeviceInformation*& infos,size_t& devSize,bool updataGData=false);
/**
  * @brief 函数unInitialDevice负责反初始化设备，释放显存
  * @return 反初始化是否成功，若还有相机未关闭，反初始化失败
*/
DLLEXPORT uint32_t unInitialDevice();

/**
 *  @defgroup
 *  @note    相机的GC3DDevice相关参数设置、存图等相关操作
 */
class  DLLEXPORT GC3DDevice
{
public:
    GC3DDevice();
    ~GC3DDevice();

    /**
      * @brief 函数 openDeviceByIndex 通过索引号打开相机，索引号不能大于相机个数n-1
      * @param [in] ind 要打开的相机索引号
      * @return 打开成功 返回GC3D_SUCCESS
    */
    uint32_t openDeviceByIndex(const size_t ind);

    /**
      * @brief 函数 openDeviceBySerial 通过序列号打开相机
      * @param [in] serial 要打开的相机序列号
      * @return 打开成功 返回GC3D_SUCCESS
    */
    uint32_t openDeviceBySerial(const std::string serial);

    /**
      * @brief 函数 closeDevice 关闭相机
      * @return 关闭成功 返回GC3D_SUCCESS，如果未打开，关闭失败
    */
    uint32_t closeDevice();

    /**
      * @brief 函数snapShot3D是进行一次扫描
      * @return 返回是否成功拍摄的错误代码
    */
    uint32_t snapShot3D();
    /**
      * @brief 函数 getGC3DMetaData 获取当前相机重建数据
      * @param [inout] userMetaData重建后的数据
      * @return 是否获取数据成功的返回代码
      * @note 该函数一般是函数snapShot3D之后
    */
    uint32_t getGC3DMetaData(GC3DMetaData& userMetaData);
    /**
      * @brief 函数 getGC3DMetaDataUndistortion 获取去畸变后的数据
      * @param [inout] userMetaData重建后的数据
      * @return 是否获取数据成功的返回代码
      * @note 该函数一般是函数snapShot3D之后
    */
    uint32_t getGC3DMetaDataUndistortion(GC3DMetaData& userMetaData);
    /**
      * @brief 函数 snapShot2D 软触发获取一张2D图
      * @param [inout] image 获取得到的2D图像数据
      * @return
      * @note snapShot3D之后利用getGC3DMetaData也可以获取数据中也存在纹理数据，该函数的区别是软触发，可以简单看成利用2D相机拍照
    */
    uint32_t snapShot2D(GC3DImageData& image);
    /**
      * @brief 函数 snapShot2DUndistortion 软触发获取一张2D图
      * @param [inout] image 获取得到的去畸变的2D图像数据
      * @return
      * @note snapShot3D之后利用getGC3DMetaData也可以获取数据中也存在纹理数据，该函数的区别是软触发，可以简单看成利用2D相机拍照
    */
    uint32_t snapShot2DUndistortion(GC3DImageData& image);
    /**
      * @brief 函数registerEvent是注册回调函数用来处理相机拍摄得到的数据
      * @return
    */
    void registerEvent(gciCallBack func);
    /**
      * @brief 函数unRegisterEvent取消回调事件
      * @return
    */
    void unRegisterEvent();
    /**
      * @brief 函数 setCamParam2D 设置相机2D采图时的曝光时间和增益
      * @param [in] expTime 曝光时间，单位us
      * @param [in] gain 增益
      * @return 设置成功，返回GC3D_SUCCESS,否则返回错误
    */
    uint32_t setCamParam2D(const int expTime,const double gain);
    /**
      * @brief 函数 getCamParam2DByIndex 根据相机索引获取2D采图时的曝光时间和增益
      * @param [out] expTime 曝光时间，单位us
      * @param [out] gain 增益
      * @param [in] index 相机索引
      * @return 设置成功，返回GC3D_SUCCESS,否则返回错误
    */
    uint32_t getCamParam2D(int& expTime,double& gain);

    /**
      * @brief 函数getDevSerial获取当前设备序列号，设备未初始化或无有效相机，将会返回空
      * @return
    */
    std::string getDevSerial();

    /**
      * @brief 函数 getGC3DGridData 获取规整的重建数据
      * @param [inout] userGridData 规整的网格化数据
      * @return
    */
    uint32_t getGC3DGridData(GC3DGridData& userGridData);
    /**
      * @brief 函数 convertToDepthImage 手动调用去畸变后的数据
      * @param [inout] userGridData 去畸变后的等间距图像数据
      * @return 是否获取数据成功的返回代码
      * @note 该函数一般是函数getGC3DMetaData  获取metaData之后
    */
    uint32_t convertToDepthImage(GC3DGridData& userGridData);
    /**
      * @brief 函数setCameraParameters采用结构体 GC3DCameraParameters 一次性设置所有参数
      * @param [in] cameraParams是需要设置参数的结构体
      * @return 返回设置参数设置成功的错误代码
    */
    uint32_t setCameraParameters(GC3DCameraParameters cameraParams);
    /**
      * @brief 函数 getCameraParameters 获取3D相机的设置参数
      * @return 返回类型为GC3DCameraParameters的3D相机测试参数
    */
    GC3DCameraParameters getCameraParameters();
    /**
      * @brief 函数 setReconThreshold 设置重建阈值
      * @param [in] minThreshold 重建的最小亮度阈值
      * @param [in] maxThreshold 重建的最大亮度阈值
      * @return
    */
    uint32_t setReconThreshold(const int minThreshold,const int maxThreshold);
    /**
      * @brief 函数 getReconThreshold 获取重建阈值
      * @param [inout] minThreshold 重建的最小亮度阈值
      * @param [inout] maxThreshold 重建的最大亮度阈值
      * @return
    */
    uint32_t getReconThreshold(int& minThreshold,int& maxThreshold);
    /**
      * @brief 函数 setDenoiseParameters 设置降噪参数
      * @param [in] fmr 去噪半径，支持1-2-3，E.G. 3—7*7， 2—5*5， 1—3*3
      * @param [in] denoiseIndex1 全局降噪参数1：ft1，fmr半径内最少有效点数,根据FMR设置
      * @param [in] denoiseIndex2 全局降噪参数2：ft2, fmr半径内所有点的Z方差，用于快速去除雾状大面积噪音点，t2不易太小
      * @param [in] denoiseIndex3 局部降噪参数：fmr半径内，平局高度值相比，超过t3的点滤除
      * @return
      * @note 还要综合考虑，相机分辨率，相机视野大小物理分辨率，目标特征。
    */
    uint32_t setDenoiseParameters(const int fmr,const float denoiseIndex1,const float denoiseIndex2,const float denoiseIndex3);
    /**
      * @brief 函数 getDenoiseParameters 获取降噪参数
      * @param [inout] fmr 去噪半径，
      * @param [inout] denoiseIndex1 全局降噪参数1
      * @param [inout] denoiseIndex2 全局降噪参数2
      * @param [inout] denoiseIndex3 局部降噪参数
      * @return
    */
    uint32_t getDenoiseParameters(int& fmr,float& denoiseIndex1,float& denoiseIndex2,float& denoiseIndex3);
    /**
      * @brief 函数 setWhiteBalanceRatio 设置相机白平衡，仅对彩色相机有效
      * @param [in] redRatio 红色分量白平衡值
      * @param [in] greenRatio 绿色分量白平衡值
      * @param [in] blueRatio 蓝色分量白平衡值
      * @return 设置成功返回GC3D_SUCCESS
      * @note
    */
    uint32_t setWhiteBalanceRatio(double redRatio,double greenRatio,double blueRatio);
    /**
      * @brief 函数 getWhiteBalanceRatio 获取相机白平衡，仅对彩色相机有效
      * @param [in] redRatio 红色分量白平衡值
      * @param [in] greenRatio 绿色分量白平衡值
      * @param [in] blueRatio 蓝色分量白平衡值
      * @return 设置成功返回GC3D_SUCCESS
      * @note
    */
    uint32_t getWhiteBalanceRatio(double& redRatio,double& greenRatio,double& blueRatio);
    /**
      * @brief 函数 setSmoothParam 设置平滑参数
      * @param [in] smoothParam 0是无平滑，大于0则是平滑
      * @return
    */
    uint32_t setSmoothParam(const int smoothParam);
    /**
      * @brief 函数 getSmoothParam 获取平滑参数
      * @param [inout] smoothParam平滑参数
      * @return
    */
    int getSmoothParam();
    /**
      * @brief 函数 setHeightRange 设置高度范围
      * @param [in] minHeight 最小高度阈值
      * @param [in] maxHeight 最大高度阈值
      * @return
    */
    uint32_t setHeightRange(const float minHeight,const float maxHeight);
    /**
      * @brief 函数 getHeightRange 获取高度范围
      * @param [inout] minHeight 最小高度阈值
      * @param [inout] maxHeight 最大高度阈值
      * @return
    */
    void getHeightRange(float& minHeight,float& maxHeight);
    /**
      * @brief 函数 setScanMode 设置扫描模式  仅对多投设备有效
      * @param [in] mode  0:多头模式  1:第一台设备扫描  2:第二台设备扫描
      * @return
    */
    uint32_t setScanMode(const int mode);
    /**
      * @brief 函数 setMajorCam 设置多投相机融合方法  仅对多投设备多投模式有效
      * @param [in] majorInd  0:第一台相机为主相机  1:第二台相机为主相机 2:两台相机融合 3:选择第一台的左半数据和第二台的右半数据
      * @return
    */
    uint32_t setMajorCam(const int majorInd);
    /**
      * @brief 函数 getScanMode 获取扫描模式
      * @return 曝光模式  0:多头模式  1:第一台设备扫描  2:第二台设备扫描
    */
    int getScanMode();
    /**
      * @brief 函数 getMaxExposureTime 获取设备最大曝光时间
      * @return 设备的最大曝光时间
    */
    int getMaxExposureTime();
    /**
      * @brief 函数 getZeroPlaneHeight 获取零平面的高度
      * @return 零平面的高度
    */
    float getZeroPlaneHeight();
    /**
      * @brief 函数 getDeviceInfo 获得设备信息
      * @param [inout] devInfo 设备的信息数组
      * @return
    */
    uint32_t getDeviceInfo(DeviceInformation& devInfo);
    /**
      * @brief 函数 checkDeviceOnline 检查当前设备是否在线
      * @return true 在线  false 离线
    */
    bool checkDeviceOnline();
    /**
      * @brief 函数 getScanTime当前扫描时间
      * @return 获取函数的扫描时间，单位毫秒
    */
    double getScanTime();
    /**
      * @brief 函数 getTotalTime 当前扫描+计算的总时间
      * @return 获取扫描+计算的总时间，单位毫秒
    */
    double getTotalTime();

    /**
      * @brief 函数 isRGBDevice 是否为彩色相机
      * @return 是否为彩色相机
    */
    bool isRGBDevice();
    /**
      * @brief 函数 saveOptimizeData 保存优化数据
      * @return 是否成功获取
      * @note 内部开发函数，一般开发不需要该函数
    */
    bool saveOptimizeData();
    /**
      * @brief 函数 loadCameraConfig 加载相机配置文件
      * @param [in] fileFullPath 加载文件路径
      * @return
    */
    uint32_t loadCameraConfig(std::string fileFullPath);
    /**
      * @brief 函数 loadCameraConfig 保存相机配置文件
      * @param [in] fileFullPath 保存文件路径
      * @return
    */
    uint32_t saveCameraConfig(std::string fileFullPath);
    /**
      * @brief 函数 setErodeMetaDataSize 设置腐蚀范围
      * @param [in] eroSize 设置腐蚀大小
      * @return
    */
    uint32_t setErodeMetaDataSize(const int eroSize);
    /**
      * @brief 函数 setNeedGridData 设置是否需要规则化网格数据
      * @param [in] gridStatus 是否设置需要规则化网格数据
      * @return
    */
    uint32_t setNeedGridData(const bool gridStatus);
    /**
      * @brief 函数 getNeedGridData 获取当前生成网格化数据状态
      * @return 当前是否生成网格化数据状态
    */
    bool getNeedGridData();
    /**
      * @brief 函数 setGridSearchRange 设置是否需要规则化网格数据查找范围
      * @param [in] gridRange 查找范围（必须设置为>=0 && <=100的数，否则设置失败）
      * @return
      * @note 一般不需要独立设置，相机会默认设置
    */
    uint32_t setGridSearchRange(const int gridRange);
    /**
      * @brief 函数 setHeartBeatTimeout 设置网口相机心跳时间 初始化时默认3000ms
      * @param [in] timeout 心跳时间范围（1000ms~500000ms）
      * @return 设置成功返回GC3D_SUCCESS
      * @note
    */
    uint32_t setHeartBeatTimeout(unsigned int timeout);

    /**
      * @brief 函数 setUseBasePlane 是否使用基准面函数
      * @param [in] useBasePlane 为true则使用基准面，即点云中的所有Z值做计算：   data.z[index]=zeroPlaneHeigt-data.z[index]
      * @note
    */
    void setUseBasePlane(bool useBasePlane);
    /**
      * @brief 函数 getUseBasePlane 获取是否使用基准面函数
      * @return 设置成功返回GC3D_SUCCESS
      * @note
    */
    bool getUseBasePlane();
    /**
      * @brief 函数 setUpdataRT 设置双投相机时是否更新RT 仅双投有效
      * @param [in] updataRT 为true则根据扫描的点云数据更新RT否则使用上次的RT
      * @note   使用双投的相机时，首先设置为true,在测量高度对成像比较好的物体拍照计算一个好的RT，然后在设置为false;默认为true
    */
    uint32_t setUpdataRT(bool updataRT);
    /**
      * @brief 函数 saveRT 保存当前双投相机的RT 仅双投有效
      * @param [in] savePath 保存RT文件的路径
      * @note   当有一个好的匹配效果时，建议保存RT，下次在打开相机之后可以加载RT，并将setUpdataRT设置为false，即可用最佳的RT配准效果
    */
    uint32_t saveRT(std::string savePath);
    /**
      * @brief 函数 loadRT 加载双投相机的RT 仅双投有效
      * @param [in] loadPath 加载RT文件的路径
      * @note   当有一个好的RT参数时，建议加载RT，并将setUpdataRT设置为false，即可用最佳的RT配准效果
    */
    uint32_t loadRT(std::string loadPath);

    /**
      * @brief 函数 getExposureImage 获取第expTime次曝光的图像
      * @param [inout] image 图像信息
      * @param [in] expTime 第expTime次曝光的灰度图
      * @note
    */
    void getExposureImage(GC3DImageData& image,int expTime);

    /**
      * @brief 函数 fillHole 用于对数据进行填孔
      * @param [in] radius          填孔半径（一般设置为1~3）
      * @param [in] areaThreshold   需要填孔的面积阈值，单位像素（一般设置为100~10000）
      * @param [in] times           填孔次数（范围1~5）
      * @note
    */
    bool fillHole(const int radius,const int areaThreshold,const int times);

    /**
      * @brief 函数 getErrMsg 根据错误码返回错误信息
      * @param [in] errCode 错误码
      * @param [out]  错误信息
      * @return
    */
    static std::string getErrMsg(uint32_t errCode);
    /**
      * @brief 函数 readGCIP 读取.gci格式文件
      * @param [inout] data 内容读取后存放到结构体GC3DMetaData
      * @param [in] filename .gci所在的文件名
      * @return
      * @note 此函数为静态函数与设备不相关
    */
    static void readGCIP(GC3DMetaData& data,std::string filename);
    /**
      * @brief 函数 saveGCIP 保存.gci格式文件
      * @param [in] data 需要保存的结构体GC3DMetaData
      * @param [in] filename .gci需要保存的文件名
      * @return
      * @note 此函数为静态函数与设备不相关
    */
    static void saveGCIP(GC3DMetaData& data,std::string filename);
    /**
      * @brief 函数 savePoints 保存点云文件
      * @param [inout] data 需要保存的结构体GC3DMetaData
      * @param [in] filename 需要保存的文件名
      * @param [in] valid 其中valid = 0 保存数据包含无效点，valid = 1 仅有效点
      * @return
      * @note 此函数为静态函数与设备不相关
    */
    static void savePoints(GC3DMetaData& data,std::string filename,int valid =1);
    /**
      * @brief 函数 saveDepthData 保存网格化深度信息
      * @param [inout] data 需要保存的结构体GC3DGridData
      * @param [in] filename 需要保存的文件名
      * @return
      * @note 此函数为静态函数与设备不相关
    */
    static void saveDepthData(GC3DGridData& data,std::string filename);
    /**
      * @brief 函数 getGCIVersion 返回3D相机的版本
      * @return 3D相机的版本
    */
    static std::string getGCIVersion();
    /**
      * @brief 函数 parseParams 用于解析metaData数据成像时的相关设置参数信息
      * @param [in] data 要解析的数据
      * @param [inout] _camParam 解析出的相机参数
      * @param [inout] _minThre 解析出的最小成像阈值
      * @param [inout] _maxThre 解析出的最大成像阈值
      * @param [inout] _fmr 解析出的降噪半径
      * @param [inout] _den1 解析出的降噪指数1
      * @param [inout] _den2 解析出的降噪指数2
      * @param [inout] _den3 解析出的降噪指数3
      * @param [inout] _smoth 解析出的平滑参数
      * @param [inout] _minh 解析出的最小高度阈值
      * @param [inout] _maxh 解析出的最大高度阈值
      * @param [inout] _erode 解析出的腐蚀参数
      * @param [inout] _time 解析出的时间戳
      * @param [inout] _placeW 解析出的标准视野宽度
      * @param [inout] _placeH 解析出的标准视野高度
      * @return true 解析成功  false 解析失败（一般是因为老版本的metaData为包含相关参数信息）
      * @note 此函数为静态函数与设备不相关
    */
    static bool parseParams(GC3DMetaData& data,GC3DCameraParameters& _camParam,int& _minThre,int& _maxThre,int& _fmr,float& _den1,
                            float& _den2,float& _den3,int& _smoth,float& _minh,float& _maxh,int& _erode,double& _time,
                            float& _placeW,float& _placeH);
    /**
      * @brief 函数 meta2Grid 将原始的meta数据转化为规则化的gridData数据
      * @param [in] meta 待转话数据
      * @param [inout] gridData 转化后的数据
      * @return
    */
    static void meta2Grid(GC3DMetaData& meta,GC3DGridData& gridData);

private:
    /**
      * @brief 私有成员 相机的实现
    */
    class GC3DImp;
    /**
      * @brief 私有成员 相机的实现 指针
    */
    GC3DImp* impPtr = nullptr;
};
};

#endif

