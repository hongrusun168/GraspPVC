 该代码仓库仅仅包含了通用抓取模型和在 Aubo 机械臂上的推理测试代码, 其中 Sim_GraspNet 中包含了网络模型和一些库的编译源码.
 
 在 SimGraspNetAubo 文件夹中,有结合 Mecheye 相机的推理代码文件 SimGrasp_Aubo_Mecheye.py;
 
 以及通过 ROS2 通信结合 Orbbec 相机的推理代码文件 SimGrasp_Aubo_Orbbec.py;
 
 新加上了结合 GCI 3D 相机的推理代码文件 SimGrasp_Server.py;
 
 其中基于 Mecheye 相机的推理代码比较完善,基于 Orbbec 相机的推理代码性能需要进一步优化.
 
 GCI 3D 相机在 ubuntu 系统上的安装和使用都不完善,其公司只提供了一些 API 和一个简单的测试用例,实际上配置有非常多的坑


## 环境配置

conda create -n simgrasp python=3.10

conda activate simgrasp3


requirements:

    - numpy 1.26.4 (numpy 库的版本一定维持在 2.0 版本以下,其他库的安装要以 numpy 库为基础)
    - torch 2.5.0   -- mkl==2024.0.0(有时候会出现 import torch 报错问题,需要降低 mkl 库的版本)    
    - pyaubo_sdk 0.26.0rc2    
    - opencv-python 4.8.0.74     
    - open3d 0.18.0    
    - scipy 1.15.3    
    - matplotlib 3.10.8    
    - h5py 3.15.1    
    - scikit-learn 1.7.2    
    - pointnet2._ext    
        | -- cd ./Sim_GraspNet/pointnet2        
        | -- python setup.py install        
        | -- 编译之后，需要将路径 ./Sim_GraspNet/pointnet2/pointnet2 下的 *.so 编译文件复制到 ./Sim_GraspNet/pointnet2 路径下（有时候需要，具体视情况而定）        
    - pn2_ext    
        | -- cd ./Sim_GraspNet/models/sim_suction_model/utils/pn2_utils        
        | -- python setup.py install        
    - knn_pytorch    
        | -- cd ./Sim_GraspNet/knn        
        | -- python setup.py install        
    - MecheyeAPI    
        | -- 参考链接 https://github.com/MechMindRobotics/mecheye_python_samples/tree/master/area_scan_3d_camera        
    - 如果需要使用 ROS2 连接 Orbbec 相机,需要在系统中安装 ros2    
    - rclpy    
    - OrbbecSDK_ROS2    
        | -- 参考链接 https://github.com/orbbec/OrbbecSDK_ROS2


## 代码仓库说明
    | -- SimGraspNetServer                                                      // 存放无序抓取的推理代码
        | -- SimGrasp_server_PVC.py                                             // PVC 套管项目中的所用主函数，本机测试可用
        | -- main.py                                                            // 基于 gRPC 通信协议的服务端启动程序
        | -- utils.py                                                           // 包含各种功能函数
        | -- Slerp_utils.py                                                     // 路径插值的一些功能函数
        | -- sim_grasp_policy_utils.py                                          // 包含抓取模型的加载、数据处理和抓取可视化等功能函数
        | -- collision_detect_utils.py                                          // 碰撞检测类的实现
        | -- LSM.py                                                             // 拟合平面点云，在后续推理过程中可以依据平面方程去除地板点云
        | -- params                                                             // 存放代码参数，相机参数
            | -- params_GCI3D_in_hand.json                                      // 眼在手上相机参数以及 homepose（空调套管参数）
            | -- params_GCI3D_to_hand.json                                      // 眼在手外相机参数（空调套管相机参数）
            | -- camera_PVC.json                                                // PVC 套管项目中的相机参数
            | -- config.json                                                    // 代码中的一些参数
                | -- hose_*_*                                                   // 专门用于框定软管抓取的工作范围
                | -- pipe_*_*                                                   // 专门用于框定铝管抓取的工作范围
                | -- camera_server_url                                          // 相机图像采集的服务端口
                | -- hose_params                                                // 拍摄软管的相机参数
                | -- pipe_params                                                // 拍摄铝管的相机参数
                | -- voxel_size                                                 // 点云的体素下采样参数
                | -- simgrasp_checkpoint_path                                   // 网络模型权重路径
                | -- hose_robot_ip                                              // 抓取软管机械臂的 ip 地址
                | -- pipe_robot_ip                                              // 抓取铝管机械臂的 ip 地址
                | -- hose_robot_port                                            // 对应端口
                | -- pipe_robot_port                                            // 对应端口
                | -- hose_pcd_xrange                                            // 软管抓取时对点云的 x 轴的限制范围
                | -- hose_pcd_yrange                                            // 软管抓取时对点云的 y 轴的限制范围
                | -- hose_image_visualize                                       // 软管抓取时是否可视化拍摄图像
                | -- hose_pcd_visualize                                         // 软管抓取时是否可视化点云
                | -- hose_collision_visualize                                   // 软管抓取时是否可视化碰撞检测示例
                | -- hose_Grasp_visualize                                       // 软管抓取时是否可视化抓取姿态
                | -- hose_score_threshold                                       // 软管抓取时的分数筛选阈值
                | -- hose_degree_threshold                                      // 软管抓取时的抓取姿态角度筛选阈值
                | -- hose_pose_xdelta                                           // 软管抓取时对应的相机标定在 x 轴方向的误差
                | -- hose_pose_ydelta                                           // 软管抓取时对应的相机标定在 y 轴方向的误差
                | -- hose_pose_distance                                         // 软管抓取时对应的机械臂法兰盘到夹爪末端的距离（约，需要调整）
                | -- hose_topk                                                  // 软管抓取时对于每一个采样点的抓取姿态预测数量
                | -- pipe_*                                                     // 这些参数与 hose 的对应参数含义是一致的
        | -- weights                                                            // 存放无序抓取模型权重
        | -- grasp_py                                                           // 关于 gRPC 通信的实现细节
    | -- Sim_GraspNet                                                           // 存放关于模型框架的代码
    ### 待解决问题
    （1）代码过于凌乱，需要重新整理一下，尤其是对参数的输入进行整理，写入 config.json 文件中，便于调试         完成
    （2）代码的命名规则需要修改，                                                                    完成
