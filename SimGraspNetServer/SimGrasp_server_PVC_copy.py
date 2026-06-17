# 代码中所使用的路径均为相对路径
import warnings
warnings.filterwarnings("ignore", category = FutureWarning)
warnings.filterwarnings("ignore", category = UserWarning)

import os
import cv2
import sys
import copy
import json
import time
import torch
import base64
import random
import requests
import threading
import pyaubo_sdk
import numpy as np
import open3d as o3d
from flask import Flask, request, jsonify

current_dir = os.path.abspath(os.path.dirname(__file__))
desired_dir = os.path.abspath(os.path.join(current_dir, "..", "Sim_GraspNet"))
sys.path.insert(0, desired_dir)

# 导入SimGraspNet相关模块
from utils import visualize_pcd
from utils import depth_image2pcd
from utils import pose_6d_to_matrix
from utils import flip_rotation_matrix
from utils import Visualize_Masked_Image
from utils import filter_pointcloud_by_xy
from utils import convert_grasp_pose_to_6d
from utils import adjust_gripper_orientation
from utils import rotate_grasp_matrix_90_deg
from utils import filter_vertical_grasps_simple
from utils import translate_grasp_point_along_direction
from sim_grasp_policy_utils import visualize_grasps
from sim_grasp_policy_utils import sim_grasp_net_model
from sim_grasp_policy_utils import get_and_process_SimData
from models.SimGraspNet_cluster import pred_decode_topk
from collision_detect_utils import CollisionDetector
from Slerp import plan_full_6dof_path

from pyDHgripper import AG95
from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import *

device = "cuda" if torch.cuda.is_available() else "cpu"


class AuboRobotController:                                                              # 专用于机械臂控制的类
    def __init__(self, robot_ip, robot_port):
        self._M_PI = 3.14159265358979323846
        self._robot_ip = robot_ip
        self._robot_port = robot_port
        self.robot_rpc_client = None
        self._Connect_to_AuboArm()
        self._robot_name = self.robot_rpc_client.getRobotNames()[0]
        self._robot = self.robot_rpc_client.getRobotInterface(self._robot_name)
        
    
    def _Connect_to_AuboArm(self):
        """
            连接到机械臂，建立 RPC 客户端
        """
        self.robot_rpc_client = pyaubo_sdk.RpcClient()
        self.robot_rpc_client.connect(self._robot_ip, self._robot_port)
        if self.robot_rpc_client.hasConnected():
            print ("1. ----------------------连接到机械臂成功----------------------")
            self._Load_to_AuboArm()
        else:
            print ("1. ----------------------连接到机械臂失败----------------------")
    
    def _Load_to_AuboArm(self):
        """
            登陆到机械臂，获取机械臂状态
        """
        self.robot_rpc_client.login("aubo", "123456")
        if self.robot_rpc_client.hasLogined():
            print ("2. ----------------------登陆到机械臂成功----------------------")
        else:
            print ("2. ----------------------登陆到机械臂失败----------------------")
    
    def _waitArrival(self):
        cnt = 0
        while self._robot.getMotionControl().getExecId() == -1:
            cnt += 1
            if cnt > 5:
                print ("Motion fail!")
                return -1
            time.sleep(0.05)
            # print("getExecId: ", self._robot.getMotionControl().getExecId())
        
        id = self._robot.getMotionControl().getExecId()
        while True:
            idl = self._robot.getMotionControl().getExecId()
            if id != idl:
                break
            time.sleep(0.05)
    
    def Move_to_Pose(self, pose):
        """
            控制机械臂移动到指定位姿
        """
        # print ("Moving to Grasp ... ...")
        # time.sleep(0.05)
        self._robot.getMotionControl() \
            .moveLine(pose, 45 * (self._M_PI / 180), 1000 * (self._M_PI / 180), 0, 0)
        self._waitArrival()


class ConnectAndCaptureImages(object):
    def __init__(self):
        self.camera = Camera()
        self.ConnectToCamera()

    def ConnectToCamera(self):
        """
            默认连接到 0 号相机
        """
        camera_infos = Camera.discover_cameras()
        error_status = self.camera.connect(camera_infos[0])
        while not error_status.is_ok():
            show_error(error_status)
            time.sleep(5)
            error_status = self.camera.connect(camera_infos[0])
        print("3. ----------------------连接到相机成功----------------------")

    def Capture(self):
        """
            采集 RGB 图像、深度图和点云数据
        """

        # # 采集 RGB 图像
        # frame2d = Frame2D()
        # show_error(self.camera.capture_2d(frame2d))
        # color_map = frame2d.get_color_image()
        # img = color_map.data()

        # 采集深度图
        frame3d = Frame3D()
        show_error(self.camera.capture_3d(frame3d))
        depth_map = frame3d.get_depth_map()
        depth_img = depth_map.data()

        return None, depth_img


class RealTimeGraspDetector:
    def __init__(self) -> None:
        self._configs = "./params/configs.json"                                         # 配置文件路径
        self._camera_params = None                                                      # 相机参数文件路径
        self._factor_depth = 1000.0                                                     # 相机的深度缩放因子
        self._voxel_size = None                                                         # 体素下采样参数
        self._simgrasp_checkpoint_path = None                                           # 推理模型路径
        self._robot_ip = None                                                           # 机械臂 IP 地址
        self._robot_port = None                                                         # 机械臂端口号
        self._M_PI = None                                                               # 圆周率
        self._CameraMatrix = None                                                       # 相机内参矩阵
        self._CalibMatrix = None                                                        # 相机外参矩阵
        self._PVC_inner_processing = False                                              # PVC 内管处理状态
        self._PVC_outer_processing = False                                              # PVC 外管处理状态

        self._PVC_inner_homePose = None                                                 # 拍摄 PVC 内管时机械臂的法兰盘的位姿
        self._PVC_inner_x_min_1 = None                                                  # PVC 内管 x 坐标最小值（碰撞场景范围）
        self._PVC_inner_x_max_1 = None                                                  # PVC 内管 x 坐标最大值
        self._PVC_inner_y_min_1 = None                                                  # PVC 内管 y 坐标最小值
        self._PVC_inner_y_max_1 = None                                                  # PVC 内管 y 坐标最大值
        self._PVC_inner_x_min_2 = None                                                  # PVC 内管 x 坐标最小值（点云采样范围）
        self._PVC_inner_x_max_2 = None                                                  # PVC 内管 x 坐标最大值
        self._PVC_inner_y_min_2 = None                                                  # PVC 内管 y 坐标最小值
        self._PVC_inner_y_max_2 = None                                                  # PVC 内管 y 坐标最大值
        self._PVC_inner_processing = False                                              # PVC 内管处理状态
        self.inner_grasp_pose = None                                                    # PVC 内管抓取姿态点位
        self.inner_desPose1 = None                                                      # PVC 内管预设放置点位 1
        self.inner_desPose2 = None                                                      # PVC 内管预设放置点位 2

        self._PVC_outer_homePose = None                                                 # 拍摄 PVC 外管时机械臂的法兰盘的位姿
        self._PVC_outer_x_min_1 = None                                                  # PVC 外管 x 坐标最小值（碰撞场景范围）
        self._PVC_outer_x_max_1 = None                                                  # PVC 外管 x 坐标最大值
        self._PVC_outer_y_min_1 = None                                                  # PVC 外管 y 坐标最小值
        self._PVC_outer_y_max_1 = None                                                  # PVC 外管 y 坐标最大值
        self._PVC_outer_x_min_2 = None                                                  # PVC 外管 x 坐标最小值（点云采样范围）
        self._PVC_outer_x_max_2 = None                                                  # PVC 外管 x 坐标最大值
        self._PVC_outer_y_min_2 = None                                                  # PVC 外管 y 坐标最小值
        self._PVC_outer_y_max_2 = None                                                  # PVC 外管 y 坐标最大值
        self._PVC_outer_processing = False                                              # PVC 外管处理状态
        self.outer_grasp_pose = None                                                    # PVC 外管抓取姿态点位
        self.outer_desPose1 = None                                                      # PVC 外管预设放置点位 1
        self.outer_desPose2 = None                                                      # PVC 外管预设放置点位 2

        self._PVC_inner_image_visualize = False                                         # PVC 内管图像 mask 可视化开关
        self._PVC_outer_image_visualize = False                                         # PVC 外管图像 mask 可视化开关
        self._PVC_inner_pcd_visualize = False                                           # PVC 内管点云可视化开关
        self._PVC_outer_pcd_visualize = False                                           # PVC 外管点云可视化开关
        self._PVC_inner_Forcast_visualize = False                                       # PVC 内管抓取预测结果可视化开关
        self._PVC_outer_Forcast_visualize = False                                       # PVC 外管抓取预测结果可视化开关
        self._PVC_inner_collision_visualize = False                                     # PVC 内管碰撞检测结果可视化开关
        self._PVC_outer_collision_visualize = False                                     # PVC 外管碰撞检测结果可视化开关
        self._PVC_inner_Grasp_visualize = False                                         # PVC 内管抓取位姿可视化开关
        self._PVC_outer_Grasp_visualize = False                                         # PVC 外管抓取位姿可视化开关
        self._PVC_inner_score_threshold = None                                          # PVC 内管抓取评分过滤阈值
        self._PVC_outer_score_threshold = None                                          # PVC 外管抓取评分过滤阈值
        self._PVC_inner_degree_threshold = None                                         # PVC 内管抓取垂直度过滤阈值
        self._PVC_outer_degree_threshold = None                                         # PVC 外管抓取垂直度过滤阈值
        self._PVC_inner_pose_distance = None                                            # PVC 内管抓取点位沿着抓取方向的平移距离
        self._PVC_outer_pose_distance = None                                            # PVC 外管抓取点位沿着抓取方向的平移距离
        self._PVC_inner_topk = None                                                     # PVC 内管抓取预测 topk
        self._PVC_outer_topk = None                                                     # PVC 外管抓取预测 topk
        self._PVC_inner_pose_xdelta = None                                              # PVC 内管抓取点位 x 轴方向偏移量
        self._PVC_inner_pose_ydelta = None                                              # PVC 内管抓取点位 y 轴方向偏移量
        self._PVC_outer_pose_xdelta = None                                              # PVC 外管抓取点位 x 轴方向偏移量
        self._PVC_outer_pose_ydelta = None                                              # PVC 外管抓取点位 y 轴方向偏移量

        self._Load_Params()                                                                         # 加载参数
        self._PVC_inner_w2e_matrix = pose_6d_to_matrix(self._PVC_inner_homePose)                    # 6D 位姿转化为外参矩阵
        self._PVC_outer_w2e_matrix = pose_6d_to_matrix(self._PVC_outer_homePose)                    # 6D 位姿转化为外参矩阵
        self._SimGraspNet = sim_grasp_net_model(self._simgrasp_checkpoint_path)                     # 加载 simgrasp 模型
        self._ARC = AuboRobotController(self._robot_ip, self._robot_port)                           # 初始化机械臂控制接口
        self._Gripper = AG95("COM3")                                                                # 初始化 DHgripper 控制接口
        self._CC = ConnectAndCaptureImages()                                                        # 初始化 Mecheye 相机控制接口


    def _Load_Params(self) -> None:
        """
            加载各种参数
        """
        with open(self._configs, "r") as f:                                             # 加载 config 配置文件中的参数
            params = json.load(f)
        self._PVC_inner_x_min_1 = params["PVC_inner_x_min_1"]
        self._PVC_inner_x_max_1 = params["PVC_inner_x_max_1"]
        self._PVC_inner_y_min_1 = params["PVC_inner_y_min_1"]
        self._PVC_inner_y_max_1 = params["PVC_inner_y_max_1"]
        self._PVC_inner_x_min_2 = params["PVC_inner_x_min_2"]
        self._PVC_inner_x_max_2 = params["PVC_inner_x_max_2"]
        self._PVC_inner_y_min_2 = params["PVC_inner_y_min_2"]
        self._PVC_inner_y_max_2 = params["PVC_inner_y_max_2"]
        self._PVC_outer_x_min_1 = params["PVC_outer_x_min_1"]
        self._PVC_outer_x_max_1 = params["PVC_outer_x_max_1"]
        self._PVC_outer_y_min_1 = params["PVC_outer_y_min_1"]
        self._PVC_outer_y_max_1 = params["PVC_outer_y_max_1"]
        self._PVC_outer_x_min_2 = params["PVC_outer_x_min_2"]
        self._PVC_outer_x_max_2 = params["PVC_outer_x_max_2"]
        self._PVC_outer_y_min_2 = params["PVC_outer_y_min_2"]
        self._PVC_outer_y_max_2 = params["PVC_outer_y_max_2"]
        self._camera_server_url = params["camera_server_url"]
        self._camera_params = params["camera_params"]
        self._voxel_size = params["voxel_size"]
        self._simgrasp_checkpoint_path = params["simgrasp_checkpoint_path"]
        self._robot_ip = params["robot_ip"]
        self._robot_port = params["robot_port"]
        self._M_PI = params["M_PI"]

        PVC_inner_homePose_array = np.asarray(params["PVC_inner_homePose"])             # 加载机械臂初始位姿并将欧拉角从度转化为弧度
        PVC_inner_homePose_array[3:] = np.deg2rad(PVC_inner_homePose_array[3:])
        self._PVC_inner_homePose = PVC_inner_homePose_array
        PVC_outer_homePose_array = np.asarray(params["PVC_outer_homePose"])
        PVC_outer_homePose_array[3:] = np.deg2rad(PVC_outer_homePose_array[3:])
        self._PVC_outer_homePose = PVC_outer_homePose_array
        

        self._PVC_inner_image_visualize = params["PVC_inner_image_visualize"]
        self._PVC_outer_image_visualize = params["PVC_outer_image_visualize"]
        self._PVC_inner_pcd_visualize = params["PVC_inner_pcd_visualize"]
        self._PVC_outer_pcd_visualize = params["PVC_outer_pcd_visualize"]
        self._PVC_inner_Forcast_visualize = params["PVC_inner_Forcast_visualize"]
        self._PVC_outer_Forcast_visualize = params["PVC_outer_Forcast_visualize"]
        self._PVC_inner_collision_visualize = params["PVC_inner_collision_visualize"]
        self._PVC_outer_collision_visualize = params["PVC_outer_collision_visualize"]
        self._PVC_inner_Grasp_visualize = params["PVC_inner_Grasp_visualize"]
        self._PVC_outer_Grasp_visualize = params["PVC_outer_Grasp_visualize"]
        self._PVC_inner_score_threshold = params["PVC_inner_score_threshold"]
        self._PVC_outer_score_threshold = params["PVC_outer_score_threshold"]
        self._PVC_inner_degree_threshold = params["PVC_inner_degree_threshold"]
        self._PVC_outer_degree_threshold = params["PVC_outer_degree_threshold"]
        self._PVC_inner_pose_distance = params["PVC_inner_pose_distance"]
        self._PVC_outer_pose_distance = params["PVC_outer_pose_distance"]
        self._PVC_inner_topk = params["PVC_inner_topk"]
        self._PVC_outer_topk = params["PVC_outer_topk"]
        self._PVC_inner_pose_xdelta = params["PVC_inner_pose_xdelta"]
        self._PVC_inner_pose_ydelta = params["PVC_inner_pose_ydelta"]
        self._PVC_outer_pose_xdelta = params["PVC_outer_pose_xdelta"]
        self._PVC_outer_pose_ydelta = params["PVC_outer_pose_ydelta"]

        outer_desPose1 = np.asarray(params["PVC_outer_desPose1"])
        outer_desPose1[3:] = np.deg2rad(outer_desPose1[3:])
        self.outer_desPose1 = outer_desPose1
        outer_desPose2 = np.asarray(params["PVC_outer_desPose2"])
        outer_desPose2[3:] = np.deg2rad(outer_desPose2[3:])
        self.outer_desPose2 = outer_desPose2

        inner_desPose1 = np.asarray(params["PVC_inner_desPose1"])
        inner_desPose1[3:] = np.deg2rad(inner_desPose1[3:])
        self.inner_desPose1 = inner_desPose1
        inner_desPose2 = np.asarray(params["PVC_inner_desPose2"])
        inner_desPose2[3:] = np.deg2rad(inner_desPose2[3:])
        self.inner_desPose2 = inner_desPose2

        f.close()

        with open(self._camera_params, "r") as f:                                       # 加载相机内参和外参
            params = json.load(f)
        self._CameraMatrix = np.asarray(params["CameraMatrix"])
        self._CalibMatrix = np.asarray(params["CalibMatrix"])
        self._CalibMatrix[:3, 3] = self._CalibMatrix[:3, 3] / self._factor_depth
        f.close()


    def perform_grasp_detect_inner(self):                                                           # 执行 PVC 内管的抓取任务
        try:
            self._PVC_inner_processing = True                                                       # 标定为正在执行
            print("\n[INFO]: PVC_inner, 开始执行抓取检测流程 ...")


            start_time = time.time()
            self._ARC.Move_to_Pose(self._PVC_inner_homePose)                                        # 移动机械臂到拍摄 PVC 内管的初始位姿
            self._Gripper.set_pos(val = 100)                                                        # 调整夹爪到合适的宽度,以防止与 PVC 内管发生碰撞
            end_time = time.time()
            print("[INFO]: PVC_inner, 机器复位花费时间 ", end_time - start_time, "s")

            time.sleep(0.01)

            start_time = time.time()
            img, depth = self._CC.Capture()                                                         # 采集 RGB 图像和深度图
            end_time = time.time()
            print("[INFO]: PVC_inner, 采集图像花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            # print("[INFO]: 正在生成点云 ...")
            scene_pcd, PVC_inner_pcd = self._Depth_to_Pcd(img, depth, which_side = "PVC_inner")     # 反投影点云
            if len(scene_pcd.points) == 0 or len(PVC_inner_pcd.points) == 0:                        # 点云未空,提前结束
                print("[INFO]: PVC_inner, 生成点云为空,检查相机是否被占用")
                self._PVC_inner_processing = False
                return False, "点云为空,检查相机设备"
            end_time = time.time()
            print("[INFO]: PVC_inner, 点云投影花费时间 ", end_time - start_time, "s")
            

            start_time = time.time()
            scene_pcd.transform(self._CalibMatrix)                                                  # 将点云从相机坐标系变换到夹爪坐标系
            PVC_inner_pcd.transform(self._CalibMatrix)
            scene_pcd.transform(self._PVC_inner_w2e_matrix)                                         # 将点云从夹爪坐标系变换到世界坐标系
            PVC_inner_pcd.transform(self._PVC_inner_w2e_matrix)
            PVC_inner_pcd, _ = PVC_inner_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)
            # print("[INFO]: 点云生成完成,点云数量:", len(PVC_inner_pcd.points))                       # 去除离群点云
            if self._PVC_inner_pcd_visualize == True:                                               # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(PVC_inner_pcd)
            end_time = time.time()
            print("[INFO]: PVC_inner, 点云处理花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            gg_array = self._Forecast_Grasp(PVC_inner_pcd, top_k = self._PVC_inner_topk, which_side = "PVC_inner", visualize = self._PVC_inner_Forcast_visualize)
            # print("[INFO]: PVC_inner, 预测抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC_inner, 预测抓取花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            gg_array = gg_array[gg_array[:, 0] > self._PVC_inner_score_threshold]                   # 根据评分过滤抓取姿态
            # print("[INFO]: PVC_inner, 评分过滤后抓取位姿数量:", len(gg_array))

            gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._PVC_inner_degree_threshold)
                                                                                                    # 尽可能保证垂直抓取
            # print("[INFO]: PVC_inner, 垂直抓取过滤后抓取位姿数量:", len(gg_array))

            gg_array = self._Collsion_Detect(gg_array, "PVC_inner", scene_pcd, visualize = self._PVC_inner_collision_visualize)
                                                                                                    # 碰撞检测
            print("[INFO]: PVC_inner, 碰撞检测后抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC_inner, 抓取处理花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            if len(gg_array) == 0:
                print("[INFO]: PVC_inner, 无有效抓取位姿,抓取失败")
                self._PVC_inner_processing = False  
                return False, "无有效抓取位姿,抓取失败"
            
            random_index = np.random.randint(0, len(gg_array))                                      # 从候选抓取姿态中随即选取一个
            best_gg = gg_array[random_index].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
            # best_grasp_fliped = rotate_grasp_matrix_90_deg(best_grasp_fliped)                       # 注意法兰盘和夹爪坐标系之间的关系,这里是两者安装时有 90 度的夹角,因此需要将抓取姿态沿着抓取方向旋转 90 度
            best_gg[4:13] = best_grasp_fliped.flatten()
            best_gg[13] += self._PVC_inner_pose_xdelta                                              # 相机标定 x 轴没有误差
            best_gg[14] += self._PVC_inner_pose_ydelta                                              # 相机标定 y 轴误差约5.8 cm

            best_gg_grasp1 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_inner_pose_distance - 0.05 + best_gg[1] * 0.01)
            pose1 = convert_grasp_pose_to_6d(best_gg_grasp1, "pose1")                               # 抓取点位(法兰盘)
            best_gg_grasp2 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_inner_pose_distance + best_gg[1] * 0.01)
            pose2 = convert_grasp_pose_to_6d(best_gg_grasp2, "pose2")                               # 抓取点位(夹爪)

            pose1[3:] = np.rad2deg(pose1[3:])                                                       # 这里是为了防止机械臂法兰盘过度旋转
            pose1[3], pose1[4], pose1[5] = adjust_gripper_orientation(pose1[3], pose1[4], pose1[5])
            pose1[3:] = np.deg2rad(pose1[3:])

            pose2[3:] = np.rad2deg(pose2[3:])
            pose2[3], pose2[4], pose2[5] = adjust_gripper_orientation(pose2[3], pose2[4], pose2[5])
            pose2[3:] = np.deg2rad(pose2[3:])

            pose3 = copy.deepcopy(pose2)                                                            # 抓取后回到抓取点位上方
            pose3[2] = pose2[2] + 0.20
            end_time = time.time()
            print("[INFO]: PVC_inner, 处理点位花费时间 ", end_time - start_time, "s")


            # opt_start_time = time.time()
            self._ARC.Move_to_Pose(pose1)                                                           # 移动到抓取点位上方
            self._ARC.Move_to_Pose(pose2)                                                           # 移动到抓取点位
            self._Gripper.set_pos(val = 30)                                                         # 执行抓取动作
            self._ARC.Move_to_Pose(pose3)                                                           # 抓取后移动到抓取点位上方
            # self._ARC.Move_to_Pose(self.inner_desPose1)                                             # 移动到预设放置点位 1
            # self._ARC.Move_to_Pose(self.inner_desPose2)                                             # 移动到预设放置点位 2
            self._Gripper.set_pos(val = 100)                                                        # 张开夹爪,完成放置
            # self._ARC.Move_to_Pose(self.inner_desPose1)                                             # 移动机械臂回初始位姿
            # opt_end_time = time.time()
            # print(f"[INFO]: PVC_inner, 抓取执行时间: {opt_end_time - opt_start_time:.2f} 秒")


            if self._PVC_inner_Grasp_visualize:
            #     vis_gg = []
            #     vis_gg.append(gg_array[random_index].copy())
            #     self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
                self._Vis_Grasps(gg_array, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)

            self.inner_grasp_pose = pose2
            self._PVC_inner_processing = False
            return True, "PVC 内管检测抓取成功"

        except Exception as e:
            error_msg = f"抓取检测过程中发生错误: {str(e)}"
            print("[INFO]:", error_msg)
            self._PVC_inner_processing = False
            return False, error_msg


    def perform_grasp_detect_outer(self):                                                           # 执行 PVC 外管的抓取任务
        try:
            self._PVC_outer_processing = True                                                       # 标定为正在执行
            print("[INFO]: PVC_outer, 开始执行抓取检测流程 ...")

            start_time = time.time()
            self._ARC.Move_to_Pose(self._PVC_outer_homePose)                                        # 移动机械臂到拍摄 PVC 外管的初始位姿
            self._Gripper.set_pos(val = 250)                                                        # 张开夹爪
            end_time = time.time()
            print("[INFO]: PVC_outer, 机器复位花费时间 ", end_time - start_time, "s")

            time.sleep(0.01)

            # img, depth, success = self._Get_outer_image()                                           # 从本地加载 PVC 外管图像数据,用于调试
            start_time = time.time()
            img, depth = self._CC.Capture()
            end_time = time.time()
            print("[INFO]: PVC_outer, 采集图像花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            # print("[INFO]: 正在生成点云 ...")
            scene_pcd, PVC_outer_pcd = self._Depth_to_Pcd(img, depth, which_side = "PVC_outer")     # 反投影点云
            if len(scene_pcd.points) == 0 or len(PVC_outer_pcd.points) == 0:                        # 点云为空,提前结束
                print("[INFO]: PVC_outer, 生成点云为空,检查相机是否被占用")
                self._PVC_outer_processing = False
                return False, "点云为空,检查相机设备"
            end_time = time.time()
            print("[INFO]: PVC_outer, 点云投影花费时间 ", end_time - start_time, "s")
            

            start_time = time.time()
            scene_pcd.transform(self._CalibMatrix)                                                  # 将点云从相机坐标系变换到夹爪坐标系
            PVC_outer_pcd.transform(self._CalibMatrix)
            scene_pcd.transform(self._PVC_outer_w2e_matrix)                                         # 将点云从夹爪坐标系变换到世界坐标系
            PVC_outer_pcd.transform(self._PVC_outer_w2e_matrix)
            PVC_outer_pcd, _ = PVC_outer_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)
            # print("[INFO]: 点云生成完成,点云数量:", len(PVC_outer_pcd.points))                        # 去除离群点云
            if self._PVC_outer_pcd_visualize == True:                                               # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(PVC_outer_pcd)
            end_time = time.time()
            print("[INFO]: PVC_outer, 点云处理花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            gg_array = self._Forecast_Grasp(PVC_outer_pcd, top_k = self._PVC_outer_topk, which_side = "PVC_outer", visualize = self._PVC_outer_Forcast_visualize)
            # print("[INFO]: PVC_outer, 预测抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC_outer, 预测抓取花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            gg_array = gg_array[gg_array[:, 0] > self._PVC_outer_score_threshold]                   # 根据评分过滤抓取姿态
            # print("[INFO]: PVC_outer, 评分过滤后抓取位姿数量:", len(gg_array))

            gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._PVC_outer_degree_threshold)
                                                                                                    # 尽可能垂直向下抓取
            # print("[INFO]: PVC_outer, 垂直抓取过滤后抓取位姿数量:", len(gg_array))

            gg_array = self._Collsion_Detect(gg_array, "PVC_outer", scene_pcd, visualize = self._PVC_outer_collision_visualize)
                                                                                                    # 碰撞检测
            print("[INFO]: PVC_outer, 碰撞检测后抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC_outer, 抓取处理花费时间 ", end_time - start_time, "s")


            start_time = time.time()
            if len(gg_array) == 0:
                print("[INFO]: PVC_outer, 无有效抓取位姿,抓取失败")
                self._PVC_outer_processing = False
                return False, "无有效抓取位姿,抓取失败"
            
            random_index = np.random.randint(0, len(gg_array))                                      # 从候选抓取姿态中随即选取一个
            best_gg = gg_array[random_index].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
            best_gg[4:13] = best_grasp_fliped.flatten()
            best_gg[13] += self._PVC_outer_pose_xdelta                                              # 相机标定 x 轴误差
            best_gg[14] += self._PVC_outer_pose_ydelta                                              # 相机标定 y 轴误差

            best_gg_grasp1 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_outer_pose_distance - 0.05 + best_gg[1] * 0.01)
            pose1 = convert_grasp_pose_to_6d(best_gg_grasp1, "pose1")                                # 抓取点位(法兰盘)
            best_gg_grasp2 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_outer_pose_distance + best_gg[1] * 0.01)
            pose2 = convert_grasp_pose_to_6d(best_gg_grasp2, "pose2")                               # 抓取点位(夹爪)

            pose1[3:] = np.rad2deg(pose1[3:])                                                       # 这里是为了防止机械臂法兰盘过度旋转
            pose1[3], pose1[4], pose1[5] = adjust_gripper_orientation(pose1[3], pose1[4], pose1[5])
            pose1[3:] = np.deg2rad(pose1[3:])

            pose2[3:] = np.rad2deg(pose2[3:])
            pose2[3], pose2[4], pose2[5] = adjust_gripper_orientation(pose2[3], pose2[4], pose2[5])
            pose2[3:] = np.deg2rad(pose2[3:])

            pose3 = copy.deepcopy(pose2)                                                            # 抓取后回到抓取点位上方
            pose3[2] = pose2[2] + 0.20
            end_time = time.time()
            print("[INFO]: PVC_outer, 处理点位花费时间 ", end_time - start_time, "s")
            

            # opt_start_time = time.time()
            self._ARC.Move_to_Pose(pose1)                                                           # 移动到抓取点位上方
            self._ARC.Move_to_Pose(pose2)                                                           # 移动到抓取点位
            self._Gripper.set_pos(val = 120)                                                         # 执行抓取动作
            self._ARC.Move_to_Pose(pose3)                                                           # 抓取后移动到抓取点位上方
            # self._ARC.Move_to_Pose(self.outer_desPose1)                                             # 移动到预设放置点位 1
            # self._ARC.Move_to_Pose(self.outer_desPose2)                                             # 移动到预设放置点位 2
            self._Gripper.set_pos(val = 250)                                                        # 张开夹爪,完成放置
            # self._ARC.Move_to_Pose(self.outer_desPose1)                                             # 移动机械臂回初始位姿
            # opt_end_time = time.time()
            # print(f"[INFO]: PVC_outer, 抓取执行时间: {opt_end_time - opt_start_time:.2f} 秒")

            if self._PVC_outer_Grasp_visualize:
                vis_gg = []
                vis_gg.append(gg_array[random_index].copy())
                self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
            
            self.outer_grasp_pose = pose2
            self._PVC_outer_processing = False
            return True, " PVC 外管检测抓取成功"

        except Exception as e:
            error_msg = f"抓取检测过程中发生错误: {str(e)}"
            print("[INFO]:", error_msg)
            self._PVC_outer_processing = False
            return False, error_msg


    def _Get_inner_image(self):                                                                     # 从本地加载 PVC 内管图像数据,用于调试
        img = cv2.imread("./collected_images/raw_color/image_PVC_inner.png")
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        depth_array = np.load("./collected_images/raw_depth/depth_PVC_inner.npy")
        return img_rgb, depth_array, True


    def _Get_outer_image(self):                                                                     # 从本地加载 PVC 外管图像数据,用于调试
        img = cv2.imread("./collected_images/raw_color/image_PVC_inner.png")
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        depth_array = np.load("./collected_images/raw_depth/depth_PVC_inner.npy")
        return img_rgb, depth_array, True


    def _capture_camera_right(self):                                                                # 执行右侧相机数据采集任务
        try:
            endpoint = f"{self._camera_server_url}/capture_right"
            response = requests.get(endpoint, timeout = 30)

            if response.status_code != 200:                                             # 若请求失败提前返回
                print(f"[INFO]: 请求失败，状态码：{response.status_code}")
                return None, None, False

            try:
                data = response.json()
                if "color_image" in data and data["color_image"] != None:               # 确认彩色图像是否存在并解析
                    img_data = base64.b64decode(result["color_image"])
                    nparr = np.frombuffer(img_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                else:                                                                   # 没有彩色图像
                    print("[INFO]: 扫描成功,但未获取到彩色图像")
                    return None, None, False

                metadata = result.get("metadata", {})
                if "raw_depth_data" in result and result["raw_depth_data"] != None:     # 确认深度图像是否存在并解析
                    raw_data = base64.b64decode(result["raw_depth_data"])
                    depth_array = np.frombuffer(raw_data, dtype = np.float32)
                    if "metadata" in result:
                        height = metadata.get("height", img_rgb.shape[0])
                        width = metadata.get("width", img_rgb.shape[1])
                        depth_array = depth_array.reshape((height, width))
                    else:
                        depth_array = depth_array.reshape((img_rgb.shape[0], img_rgb.shape[1]))
                else:                                                                   # 没有深度图像             
                    print("[INFO]: 扫描成功,但未获取到深度图")
                    return None, None, False

                return img_rgb, depth_array, True

            except json.JSONDecodeError:
                print("[INFO]: 错误：解析Socket服务器返回的JSON数据失败")
                return None, None, False

        except requests.exceptions.ConnectionError:
            print("[INFO]: 错误：无法连接到Socket服务器，请确保C++程序正在运行")
            return {"success": False, "message": "连接失败"}

        except requests.exceptions.Timeout:
            print("[INFO]: 错误：请求超时")
            return {"success": False, "message": "请求超时"}

        except Exception as e:
            print(f"[INFO]: 错误：请求过程中发生异常 - {str(e)}")
            return {"success": False, "message": str(e)}


    def _capture_camera_left(self):                                                                 # 执行左侧相机数据采集任务
        try:
            endpoint = f"{self._camera_server_url}/capture_left"
            response = requests.get(endpoint, timeout = 30)

            if response.status_code != 200:                                             # 若请求失败提前返回
                print(f"[INFO]: 请求失败，状态码：{response.status_code}")
                return None, None, False

            try:
                data = response.json()
                if "color_image" in data and data["color_image"] != None:               # 确认彩色图像是否存在并解析
                    img_data = base64.b64decode(result["color_image"])
                    nparr = np.frombuffer(img_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                else:                                                                   # 没有彩色图像
                    print("[INFO]: 扫描成功,但未获取到彩色图像")
                    return None, None, False

                metadata = result.get("metadata", {})
                if "raw_depth_data" in result and result["raw_depth_data"] != None:     # 确认深度图像是否存在并解析
                    raw_data = base64.b64decode(result["raw_depth_data"])
                    depth_array = np.frombuffer(raw_data, dtype = np.float32)
                    if "metadata" in result:
                        height = metadata.get("height", img_rgb.shape[0])
                        width = metadata.get("width", img_rgb.shape[1])
                        depth_array = depth_array.reshape((height, width))
                    else:
                        depth_array = depth_array.reshape((img_rgb.shape[0], img_rgb.shape[1]))
                else:                                                                   # 没有深度图像             
                    print("[INFO]: 扫描成功,但未获取到深度图")
                    return None, None, False

                return img_rgb, depth_array, True

            except json.JSONDecodeError:
                print("[INFO]: 错误：解析Socket服务器返回的JSON数据失败")
                return None, None, False

        except requests.exceptions.ConnectionError:
            print("[INFO]: 错误：无法连接到Socket服务器，请确保C++程序正在运行")
            return {"success": False, "message": "连接失败"}

        except requests.exceptions.Timeout:
            print("[INFO]: 错误：请求超时")
            return {"success": False, "message": "请求超时"}

        except Exception as e:
            print(f"[INFO]: 错误：请求过程中发生异常 - {str(e)}")
            return {"success": False, "message": str(e)}


    def _Depth_to_Pcd(self, img, depth_img, which_side):
        h, w = depth_img.shape[:2]
        scene_mask = np.zeros((h, w), dtype = bool)                                     # 场景点云 mask 
        PVC_inner_mask = np.zeros((h, w), dtype = bool)                                 # PVC 内管点云 mask
        PVC_outer_mask = np.zeros((h, w), dtype = bool)                                 # PVC 外管点云 mask

        if which_side == "PVC_inner":                                                   # PVC_inner 图像 mask
            scene_mask[self._PVC_inner_y_min_1:self._PVC_inner_y_max_1, self._PVC_inner_x_min_1:self._PVC_inner_x_max_1] = True
            PVC_inner_mask[self._PVC_inner_y_min_2:self._PVC_inner_y_max_2, self._PVC_inner_x_min_2:self._PVC_inner_x_max_2] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = PVC_inner_mask)
            if self._PVC_inner_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                 # 用于可视化场景 mask
                Visualize_Masked_Image(img, PVC_inner_mask)                             # 用于可视化抓取区域 mask

        elif which_side == "PVC_outer":                                                 # PVC_outer 图像 mask
            scene_mask[self._PVC_outer_y_min_1:self._PVC_outer_y_max_1, self._PVC_outer_x_min_1:self._PVC_outer_x_max_1] = True
            PVC_outer_mask[self._PVC_outer_y_min_2:self._PVC_outer_y_max_2, self._PVC_outer_x_min_2:self._PVC_outer_x_max_2] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = PVC_outer_mask)
            if self._PVC_outer_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                 # 用于可视化场景 mask
                Visualize_Masked_Image(img, PVC_outer_mask)                             # 用于可视化抓取区域 mask

        return scene_pcd, pipes_pcd
    

    def _Forecast_Grasp(self, pipes_pcd, top_k = 10, visualize = False, which_side = None):
         # 确保不计算梯度
        with torch.no_grad():
            end_points = get_and_process_SimData(pipes_pcd, visualize=visualize)
            
            # 移动数据到设备
            for key in end_points:
                if isinstance(end_points[key], torch.Tensor):
                    end_points[key] = end_points[key].to(device)
            
            # 推理
            end_points = self._SimGraspNet(end_points, which_side = which_side)
            grasp_preds = pred_decode_topk(end_points, top_k = top_k)
            
            # 立即移动到CPU并转换为numpy
            gg_array = grasp_preds[0].detach().cpu().numpy()
            
            # 清理中间变量
            del end_points
            del grasp_preds
            
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            return gg_array


    def _Collsion_Detect(self, gg_array, which_side, pcd, visualize = False):                       # 碰撞检测接口
        mfcdetector = CollisionDetector(pcd, voxel_size = self._voxel_size)
        if which_side == "PVC_outer":
            collision_mask = mfcdetector.detect_PVC_outer(gg_array, visualize = visualize)
        else:
            collision_mask = mfcdetector.detect_PVC_inner(gg_array, visualize = visualize)
        gg_array = gg_array[collision_mask]
        return gg_array


    def _Vis_Grasps(self, gg_array, scene_pcd, scale_factor = 20.0, top_percent = 4, top1_only = True, sort_or_not = True, show_grasp_points = True, show_grasp_directions = True):
        """
        可视化抓取姿态
        
        参数:
            gg_array: 抓取姿态数组
            scene_pcd: 场景点云
            scale_factor: 夹爪模型缩放因子
            top_percent: 显示的抓取姿态比例
            top1_only: 是否只显示一个抓取姿态
            sort_or_not: 是否排序抓取姿态
            show_grasp_points: 是否显示抓取点（红色球体=调整后位置，黄色球体=原始中心）
            show_grasp_directions: 是否显示抓取方向（蓝色箭头）
        """
        pcd = scene_pcd.uniform_down_sample(int(len(scene_pcd.points) / 100_000))
        visualize_grasps(gg_array, pcd, scale_factor = scale_factor, 
                        top_percent = top_percent, top1_only = top1_only, sort_or_not = sort_or_not,
                        show_grasp_points = show_grasp_points, 
                        show_grasp_directions = show_grasp_directions)


app = Flask(__name__)                                                                               # Flask app 启动
grasp_detector = RealTimeGraspDetector()


@app.route("/process_inner", methods = ["POST", "GET"])
def process_right():
    if request.method == "POST":                                                # 解析请求数据
        data = request.get_json(silent = True) or request.data.decode() if request.data else None
    else:
        data = request.args.get("data", None)
    print(f"[INFO]: 收到{request.method}请求，数据: {data}")

    success, message = grasp_detector.perform_grasp_detect_inner()              # 右臂执行抓取

    if success == True:                                                         # 抓取成功返回
        response = {
            "success": True,
            "message": message,
        }
        return jsonify(response), 200
    else:                                                                       # 抓取失败返回
        response = {
            "success": False,
            "message": message,
        }
        return jsonify(response), 500


@app.route("/process_outer", methods = ["POST", "GET"])
def process_left():
    if request.method == "POST":                                                # 解析请求数据
        data = request.get_json(silent = True) or request.data.decode() if request.data else None
    else:
        data = request.args.get("data", None)
    print(f"[INFO]: 收到{request.method}请求，数据: {data}")

    success, message = grasp_detector.perform_grasp_detect_outer()              # 右臂执行抓取

    if success == True:                                                         # 抓取成功返回
        response = {
            "success": True,
            "message": message,
        }
        return jsonify(response), 200
    else:                                                                       # 抓取失败返回
        response = {
            "success": False,
            "message": message,
        }
        return jsonify(response), 500


def run_flask_server() -> None:
    print("Flask服务器启动...")
    print(f"无序抓取上料服务器地址: http://127.0.0.1:5302")
    print("可用端点:")
    print("  /process_right  - 右臂抓取检测")
    print("  /process_left   - 左臂抓取检测")
    app.run(host = "127.0.0.1", port = 5302, debug = False)


if __name__ == "__main__":
    grasp_detector._ARC.robot_rpc_client.getRuntimeMachine().start()
    # for i in range(1):
    #     # grasp_detector.perform_grasp_detect_outer()
    #     start_time = time.time()
    #     grasp_detector.perform_grasp_detect_inner()
    #     grasp_detector.perform_grasp_detect_outer()
    #     end_time = time.time()
    #     print(f"[INFO]: 抓取检测执行时间: {end_time - start_time:.2f} 秒")
    run_flask_server()
    grasp_detector._ARC.robot_rpc_client.getRuntimeMachine().stop()


