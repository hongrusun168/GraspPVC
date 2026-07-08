#ifndef RGSDEF_H
#define RGSDEF_H
#include<vector>

#define RGS_SUCCESS                        0x00009003  //成功
#define RGS_CALIBRATE_NUM_FAIL             0x00009004  //标定数据不够
#define RGS_CALIBRATE_TYPE_FAIL            0x00009005  //机械手和标定类型设置不同
#define RGS_SOFTDOG_FAIL                   0x00009006  //加密狗错误
#define RGS_ICP_ERROR                      0x00009007  //icp错误
#define RGS_STATUS uint32_t

namespace gc3d {
/**
 *  @brief GCIROBOTROTORD是机械手的旋转表示
 * @note  机械手分为内旋和外旋方式
  */
enum GCIROBOTROTORD{
    GCI_ZYX_ORDER=0,       //!<旋转Z-Y-X
    GCI_XYZ_ORDER,        //!<旋转X-Y-Z  库卡
    GCI_XZY_ORDER,        //!<旋转X-Z-Y
    GCI_YXZ_ORDER,        //!<旋转Y-X-Z
    GCI_YZX_ORDER,        //!<旋转Y-Z-X
    GCI_ZXY_ORDER,        //!<旋转Z-X-Y
    GCI_XZX_ORDER,        //!<旋转X-Z-X
    GCI_XYX_ORDER,        //!<旋转X-Y-X
    GCI_YXY_ORDER,        //!<旋转Y-X-Y
    GCI_YZY_ORDER,        //!<旋转Y-Z-Y
    GCI_ZYZ_ORDER,        //!<旋转Z-Y-Z
    GCI_ZXZ_ORDER,         //!<旋转Z-X-Z  柴孚
    GCI_ZYX_UNORDER,       //!<旋转Z-Y-X  川崎
    GCI_XYZ_UNORDER,        //!<旋转X-Y-Z  ABB 发那科 越疆等常见的一种旋转方式
    GCI_XZY_UNORDER,        //!<旋转X-Z-Y
    GCI_YXZ_UNORDER,        //!<旋转Y-X-Z
    GCI_YZX_UNORDER,        //!<旋转Y-Z-X
    GCI_ZXY_UNORDER,        //!<旋转Z-X-Y
    GCI_XZX_UNORDER,        //!<旋转X-Z-X
    GCI_XYX_UNORDER,        //!<旋转X-Y-X
    GCI_YXY_UNORDER,        //!<旋转Y-X-Y
    GCI_YZY_UNORDER,        //!<旋转Y-Z-Y
    GCI_ZYZ_UNORDER,        //!<旋转Z-Y-Z
    GCI_ZXZ_UNORDER         //!<旋转Z-X-Z
};

/**
 *  @brief GCICALTYPE是安装类型，分为眼在手上和眼在手外
 * @note  不同的安装类型在会在标定时调用不同的标定算法
  */
enum GCICALTYPE{
    GCI_EYEINHAND=0,        //!<眼在手上的安装方式
    GCI_EYETOHAND           //!<眼在手外的安装方式
};

/**
 *  @brief GCIPOINTTYPE表示在标定或定位时的点云是
 * 有序的还是无序的，它们的主要区别是采用的配准算法不同
 * GCI_POINT_ORDER 当点云是无序点云时，此时采用点云注册的方式
 * 对点云进行配准
 * GCI_POINT_UNORDER 当点云是无序点云时，此时采用icp的方式
 * 对点云进行配准
 * @note
  */
enum GCIPOINTTYPE{
    GCI_POINT_ORDER=0,      //!<有序点云
    GCI_POINT_UNORDER       //!<无序点云
};

/**
 *  @brief 类ICPParam是icp配准需要设置的参数
 * maxIter 迭代终止条件： 最大迭代次数，迭代次数达到该值时迭代终止
 * maxPointNum 最大计算点数，当大于这个点数时会进行采样
 * needMove 是否需要移动做为粗配,一般当目标比较平坦，3D特征不明显时，设置为true，若目标弧面较多，设置为false
 * isRotation 是否需要适应旋转匹配,当可能的旋转大小不确定时，设置为true
 * angels 是否需要适应旋转匹配,一般每间隔15度设置一个旋转角，仅适应RZ旋转，isRotation设置为true时有效
 * @note
  */
class ICPParam
{
public:
    ICPParam() {}
    ICPParam(int _maxIter, double _minDisIter, int _maxPointNum,bool _needMove,bool _isRotation):maxIter(_maxIter),
             minDisIter(_minDisIter),  maxPointNum(_maxPointNum),needMove(_needMove),isRotation(_isRotation) {}
    int maxIter = 3;          //!<最大迭代次数
    double minDisIter = 0.2;    //!<最近点误差阈值
    int maxPointNum = 50000;    //!<最大计算点数，用降采样
    bool needMove = false;      //!<是否需要包围盒粗配
    bool isRotation = false;    //!<是否需要适应旋转匹配
    std::vector<double>angels;  //!指定旋转角度,当isRotation为true时有效
};

/**
 *  @brief 类GripperLocal用来存储机械手坐标
 * @note 最多能存储六轴机械手坐标值，当机械手为3轴
 * 或者4轴时，多余的位置无效
  */
class GripperLocal{
public:
    GripperLocal(){}
    GripperLocal(double _x,double _y,double _z,double _rx,double _ry,double _rz):
        x(_x),y(_y),z(_z),rx(_rx),ry(_ry),rz(_rz){}
    double x=0;             //!<机械手x坐标
    double y=0;             //!<机械手y坐标
    double z=0;             //!<机械手z坐标
    double rx=0;            //!<机械手rx坐标
    double ry=0;            //!<机械手ry坐标
    double rz=0;            //!<机械手rz坐标
};
/**
 *  @brief 类GRotation用来存储旋转矩阵
 * @note 旋转矩阵是一个3*3的矩阵分布如下
 *       r11------r12------r13
 *        |        |        |
 *        |        |        |
 *       r21------r22------r23
 *        |        |        |
 *        |        |        |
 *       r31------r32------r33
  */
class GRotation{
public:
    GRotation(){}
    GRotation(double _r11,double _r12,double _r13,
              double _r21,double _r22,double _r23,
              double _r31,double _r32,double _r33):
              r11(_r11),r12(_r12),r13(_r13),
              r21(_r21),r22(_r22),r23(_r23),
              r31(_r31),r32(_r32),r33(_r33){}
    double r11=1;
    double r12=0;
    double r13=0;
    double r21=0;
    double r22=1;
    double r23=0;
    double r31=0;
    double r32=0;
    double r33=1;
};
/**
 *  @brief 类GTranslation用来存储平移向量
 * @note 旋转矩阵是一个3*3的矩阵分布如下
  */
class GTranslation{
public:
    GTranslation(){}
    GTranslation(double _tx,double _ty,double _tz):
              tx(_tx),ty(_ty),tz(_tz){}
    double tx=0;            //!<x方向平移
    double ty=0;            //!<y方向平移
    double tz=0;            //!<z方向平移
};


/**
 *  @brief 类CalibrateError用来存储标定误差
 * 标定误差的计算是用第一个拍照位作为定位点位，用其他位置
 * 来算第一个拍照定位，和其做差值
 * @note
  */
struct CalibrateError{
    std::vector<GripperLocal> gripError;            //!<每个点位的坐标误差
    GripperLocal errorMax,errorMin,errorSTD;        //!<每个点位的各个值的最大值，最小值，以及方差
};
}


#endif // RGSDEF_H
