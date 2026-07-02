# Grasp Service - 算法人员修改此文件
# 算法人员只需实现 business_logic 中的三个函数

import os
import cv2
import sys
import copy
import json
import math
import time
import torch

import numpy as np
import open3d as o3d

from pyDHgripper import AG95


import threading

# 将依赖的路径导入到环境中
current_dir = os.path.abspath(os.path.dirname(__file__))
desired_dir1 = os.path.abspath(os.path.join(current_dir, "..", ".."))
desired_dir2 = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "Sim_GraspNet"))
sys.path.insert(0, desired_dir1)
sys.path.insert(0, desired_dir2)


import grpc
from generated import grasp_pb2, grasp_pb2_grpc
from Slerp_utils import *
from collision_detect_utils import CollisionDetector
from models.SimGraspNet_cluster import pred_decode_topk
from utils import (
    is_same_pose,
    visualize_pcd,
    depth_image2pcd,
    pose_6d_to_matrix,
    rotate_grasp_matrix,
    remove_floor_points,
    flip_rotation_matrix,
    Visualize_Masked_Image,
    filter_pointcloud_by_xy,
    convert_grasp_pose_to_6d,
    adjust_gripper_orientation,
    filter_vertical_grasps_simple,
    translate_grasp_point_along_direction
)
from sim_grasp_policy_utils import (
    visualize_grasps,
    sim_grasp_net_model,
    get_and_process_SimData
)

from AuboUtils import *
from CameraUtils import *

device = "cuda" if torch.cuda.is_available() else "cpu"


class RealTimeGraspDetector:
    def __init__(self) -> None:
        self._configs = "./params/configs.json"                                                     # 配置文件路径
        self._camera_params = None                                                                  # 相机参数文件路径
        self._factor_depth = 1000.0                                                                 # 相机的深度缩放因子
        self._voxel_size = None                                                                     # 体素下采样参数
        self._simgrasp_checkpoint_path = None                                                       # 推理模型路径
        self._robot_ip = None                                                                       # 机械臂 IP 地址
        self._robot_port = None                                                                     # 机械臂端口号
        self._M_PI = None                                                                           # 圆周率
        self._CameraMatrix = None                                                                   # 相机内参矩阵
        self._CalibMatrix = None                                                                    # 相机外参矩阵
        self._PVC_processing = False                                                                # PVC 管处理状态
        self._EVA_processing = False                                                                # EVA 管处理状态

        self._PVC_homePose = None                                                                   # 拍摄 PVC 管时机械臂的法兰盘的位姿
        self._PVC_scene_range = None                                                                # PVC 管的碰撞范围
        self._PVC_pcd_range = None                                                                  # PVC 管的采样范围
        self.PVC_grasp_pose = None                                                                  # PVC 管抓取姿态点位
        self.PVC_desPose = None                                                                     # PVC 管预设放置点位

        self._EVA_homePose = None                                                                   # 拍摄 EVA 管时机械臂的法兰盘的位姿
        self._EVA_scene_range = None                                                                # EVA 管的碰撞范围
        self._EVA_pcd_range = None                                                                  # EVA 管的采样范围
        self.EVA_grasp_pose = None                                                                  # EVA 管抓取姿态点位
        self.EVA_desPose = None                                                                     # EVA 管预设放置点位

        self._PVC_image_visualize = False                                                           # PVC 管图像 mask 可视化开关
        self._EVA_image_visualize = False                                                           # EVA 管图像 mask 可视化开关
        self._PVC_pcd_visualize = False                                                             # PVC 管点云可视化开关
        self._EVA_pcd_visualize = False                                                             # EVA 管点云可视化开关
        self._PVC_Forcast_visualize = False                                                         # PVC 管抓取预测结果可视化开关
        self._EVA_Forcast_visualize = False                                                         # EVA 管抓取预测结果可视化开关
        self._PVC_collision_visualize = False                                                       # PVC 管碰撞检测结果可视化开关
        self._EVA_collision_visualize = False                                                       # EVA 管碰撞检测结果可视化开关
        self._PVC_Grasp_visualize = False                                                           # PVC 管抓取位姿可视化开关
        self._EVA_Grasp_visualize = False                                                           # EVA 管抓取位姿可视化开关
        self._PVC_score_threshold = None                                                            # PVC 管抓取评分过滤阈值
        self._EVA_score_threshold = None                                                            # EVA 管抓取评分过滤阈值
        self._PVC_degree_threshold = None                                                           # PVC 管抓取垂直度过滤阈值
        self._EVA_degree_threshold = None                                                           # EVA 管抓取垂直度过滤阈值
        self._PVC_pose_distance = None                                                              # PVC 管抓取点位沿着抓取方向的平移距离
        self._EVA_pose_distance = None                                                              # EVA 管抓取点位沿着抓取方向的平移距离
        self._PVC_topk = None                                                                       # PVC 管抓取预测 topk
        self._EVA_topk = None                                                                       # EVA 管抓取预测 topk
        self._PVC_pose_xdelta = None                                                                # PVC 管抓取点位 x 轴方向偏移量
        self._PVC_pose_ydelta = None                                                                # PVC 管抓取点位 y 轴方向偏移量
        self._EVA_pose_xdelta = None                                                                # EVA 管抓取点位 x 轴方向偏移量
        self._EVA_pose_ydelta = None                                                                # EVA 管抓取点位 y 轴方向偏移量

        self._Capture_homePose = None                                                               # 拍照的时候机械臂的位姿
        self._PVC_waypoint = None
        self._EVA_waypoint = None

        self._Load_Params()                                                                         # 加载参数
        self._PVC_w2e_matrix = pose_6d_to_matrix(self._PVC_homePose)                                # 6D 位姿转化为外参矩阵
        self._EVA_w2e_matrix = pose_6d_to_matrix(self._EVA_homePose)                                # 6D 位姿转化为外参矩阵
        self._SimGraspNet = sim_grasp_net_model(self._simgrasp_checkpoint_path)                     # 加载 simgrasp 模型
        self._ARC = AuboRobotController(self._robot_ip, self._robot_port)                           # 初始化机械臂控制接口
        self._Gripper = AG95("COM3")                                                                # 初始化 DHgripper 控制接口，并设置夹爪参数
        self._Gripper.set_vel(1000)
        self._Gripper.set_rot_vel(100)                                                              
        self._CC = ConnectAndCaptureImages()                                                        # 初始化 Mecheye 相机控制接口
        
        self._PVC_pose1 = None                                                                      # PVC 抓取点位 1
        self._PVC_pose2 = None                                                                      # PVC 抓取点位 2
        self._PVC_pose3 = None                                                                      # PVC 抓取点位 3
        self._EVA_pose1 = None                                                                      # EVA 抓取点位 1
        self._EVA_pose2 = None                                                                      # EVA 抓取点位 2
        self._EVA_pose3 = None                                                                      # EVA 抓取点位 3
        self._PVC_image = None                                                                      # PVC 上料图片数据
        self._PVC_depth = None                                                                      # PVC 上料深度数据
        self._EVA_image = None                                                                      # EVA 上料图片数据
        self._EVA_depth = None                                                                      # EVA 上料深度数据


    def _Load_Params(self) -> None:
        """
            加载各种参数
        """
        with open(self._configs, "r") as f:                                                         # 加载 config 配置文件中的参数
            params = json.load(f)
        self._PVC_scene_range = params["PVC_scene_range"]
        self._PVC_pcd_range = params["PVC_pcd_range"]
        self._EVA_scene_range = params["EVA_scene_range"]
        self._EVA_pcd_range = params["EVA_pcd_range"]
        self._camera_params = params["camera_params"]
        self._voxel_size = params["voxel_size"]
        self._simgrasp_checkpoint_path = params["simgrasp_checkpoint_path"]
        self._robot_ip = params["robot_ip"]
        self._robot_port = params["robot_port"]
        self._M_PI = params["M_PI"]

        PVC_homePose_array = np.asarray(params["PVC_homePose"])                                     # 加载机械臂初始位姿并将欧拉角从度转化为弧度
        PVC_homePose_array[3:] = np.deg2rad(PVC_homePose_array[3:])
        self._PVC_homePose = PVC_homePose_array
        EVA_homePose_array = np.asarray(params["EVA_homePose"])
        EVA_homePose_array[3:] = np.deg2rad(EVA_homePose_array[3:])
        self._EVA_homePose = EVA_homePose_array

        Capture_homePose_array = np.asarray(params["Capture_homePose"])
        Capture_homePose_array[3:] = np.deg2rad(Capture_homePose_array[3:])
        self._Capture_homePose = Capture_homePose_array
        PVC_waypoint_array = np.asarray(params["PVC_waypoint"])
        PVC_waypoint_array[3:] = np.deg2rad(PVC_waypoint_array[3:])
        self._PVC_waypoint = PVC_waypoint_array
        EVA_waypoint_array = np.asarray(params["EVA_waypoint"])
        EVA_waypoint_array[3:] = np.deg2rad(EVA_waypoint_array[3:])
        self._EVA_waypoint = EVA_waypoint_array
        

        self._PVC_image_visualize = params["PVC_image_visualize"]
        self._EVA_image_visualize = params["EVA_image_visualize"]
        self._PVC_pcd_visualize = params["PVC_pcd_visualize"]
        self._EVA_pcd_visualize = params["EVA_pcd_visualize"]
        self._PVC_Forcast_visualize = params["PVC_Forcast_visualize"]
        self._EVA_Forcast_visualize = params["EVA_Forcast_visualize"]
        self._PVC_collision_visualize = params["PVC_collision_visualize"]
        self._EVA_collision_visualize = params["EVA_collision_visualize"]
        self._PVC_Grasp_visualize = params["PVC_Grasp_visualize"]
        self._EVA_Grasp_visualize = params["EVA_Grasp_visualize"]
        self._PVC_score_threshold = params["PVC_score_threshold"]
        self._EVA_score_threshold = params["EVA_score_threshold"]
        self._PVC_degree_threshold = params["PVC_degree_threshold"]
        self._EVA_degree_threshold = params["EVA_degree_threshold"]
        self._PVC_pose_distance = params["PVC_pose_distance"]
        self._EVA_pose_distance = params["EVA_pose_distance"]
        self._PVC_topk = params["PVC_topk"]
        self._EVA_topk = params["EVA_topk"]
        self._PVC_pose_xdelta = params["PVC_pose_xdelta"]
        self._PVC_pose_ydelta = params["PVC_pose_ydelta"]
        self._EVA_pose_xdelta = params["EVA_pose_xdelta"]
        self._EVA_pose_ydelta = params["EVA_pose_ydelta"]

        EVA_desPose = np.asarray(params["EVA_desPose"])                                             # EVA 管预设放置位置
        EVA_desPose[3:] = np.deg2rad(EVA_desPose[3:])
        self.EVA_desPose = EVA_desPose

        PVC_desPose = np.asarray(params["PVC_desPose"])                                             # PVA 管预设放置位置
        PVC_desPose[3:] = np.deg2rad(PVC_desPose[3:])
        self.PVC_desPose = PVC_desPose

        f.close()

        with open(self._camera_params, "r") as f:                                                   # 加载相机内参和外参
            params = json.load(f)
        self._CameraMatrix = np.asarray(params["CameraMatrix"])
        self._CalibMatrix = np.asarray(params["CalibMatrix"])
        self._CalibMatrix[:3, 3] = self._CalibMatrix[:3, 3] / self._factor_depth
        f.close()


    def _Capture_image_and_depth(self):
        print("[INFO]: 采集图像和深度图 ......")
        # try:
        img, depth = self._CC.Capture(which_side = "PVC", save = False)                            # 采集 RGB 图像和深度图
        if depth is None:
            self._Clear_EVA_images()
            self._Clear_PVC_images()
            return 1, "采集数据失败"
        self._PVC_image = copy.deepcopy(img)
        self._PVC_depth = copy.deepcopy(depth)
        self._EVA_image = copy.deepcopy(img)
        self._EVA_depth = copy.deepcopy(depth)
        # except Exception as e:
        #     self._Clear_EVA_images()
        #     self._Clear_PVC_images()
        #     return 2, e
        return 0, "采集数据成功"
    

    def generate_poses_from_o3d_cloud(self, pcd, max_points=5120, num_rotations=4):
        """
        直接接收 Open3D PointCloud 对象的姿态生成函数（结合 180 度抓取对称性）
        
        :param pcd: 原始 Open3D PointCloud 对象
        :param max_points: 限制最终参与计算的最大点云数量，默认 5120
        :param num_rotations: 在 180 度半圆周期内生成的旋转姿态数量，默认 4
        :return: 16维抓取姿态矩阵, Numpy 数组 (M * num_rotations, 16)
        """
        # 检查点云是否为空
        if pcd.is_empty():
            return np.empty((0, 16), dtype=np.float32)

        # ==================== 1. 统计学滤波去噪 ====================
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        # ==================== 2. 动态体素下采样 (<= 5120) ====================
        # Open3D 获取点数使用的是 len(pcd.points)
        if len(pcd.points) <= max_points:
            processed_pc_np = np.asarray(pcd.points)
        else:
            voxel_size = 0.002  # 初始体素 2mm
            while True:
                downsampled_pcd = pcd.voxel_down_sample(voxel_size)
                current_points = len(downsampled_pcd.points)
                
                if current_points <= max_points:
                    break
                voxel_size *= 1.15  # 点数超标时，逐步放大体素使点云变稀疏
                
            processed_pc_np = np.asarray(downsampled_pcd.points)

        print(f"[姿态采集] 下采样后点数: {processed_pc_np.shape[0]}")

        # ==================== 3. 16维抓取姿态生成 (基于180°对称) ====================
        num_points = processed_pc_np.shape[0]
        if num_points == 0:
            return np.empty((0, 16), dtype=np.float32)

        # 旋转周期改为 np.pi (180度)，endpoint=False 避免包含重复的 180° 边界
        angles = np.linspace(0, np.pi, num_rotations, endpoint=False)
        
        rotation_matrices = []
        for theta in angles:
            cos_t = np.cos(theta)
            sin_t = np.sin(theta)
            R = np.array([
                [cos_t, -sin_t,  0.0],
                [sin_t,  cos_t,  0.0],
                [0.0,    0.0,    1.0]
            ], dtype=np.float32)
            rotation_matrices.append(R.flatten())

        # 预分配 16 维矩阵内存
        total_poses = num_points * num_rotations
        grasp_poses = np.ones((total_poses, 16), dtype=np.float32)

        # 双重循环填充数据
        idx = 0
        for point in processed_pc_np:
            x, y, z = point
            for r_flatten in rotation_matrices:
                # v[0:4] 保持为 0
                grasp_poses[idx, 4:13] = r_flatten    # v[4:13] 旋转矩阵
                grasp_poses[idx, 13:16] = [x, y, z]   # v[13:16] 抓取中心点坐标
                idx += 1

        print(f"[姿态采集] 最终生成的 16 维向量总数: {grasp_poses.shape[0]}")
        return grasp_poses


    def _Process_PVC(self):
        # try:
            # 每轮计算上料点位开始前都要清空对应的上料点位
            self._Clear_PVC_poses()
            # 点云反投影
            scene_pcd, PVC_pcd = self._Depth_to_Pcd(self._PVC_image, self._PVC_depth, which_side = "PVC")
            self._Clear_PVC_images()

            # 点云变换和裁剪
            scene_pcd.transform(self._CalibMatrix)
            PVC_pcd.transform(self._CalibMatrix)
            PVC_pcd, _ = PVC_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)
            plane_params = (0.004949, 0.014015, 0.999890, -0.210453)                                # 去除平面点云
            PVC_pcd = remove_floor_points(PVC_pcd, plane_params, threshold = 0.0050)
            # filter_pointcloud_by_xy(scene_pcd, x_range = (-0.255000, 0.090000), y_range = (-0.973493, -0.550379), z_range = (-0.10, 0.30))
            # filter_pointcloud_by_xy(PVC_pcd, z_range = (-0.10, 0.30))
            # o3d.io.write_point_cloud("pvc_plane.ply", scene_pcd)

            if len(scene_pcd.points) == 0 or len(PVC_pcd.points) < 5120:                            # 点云未空,提前结束
                print("[INFO]: PVC, 点云为空或场景点云数量不足,检查相机是否被占用")
                self._PVC_processing = False
                return 3, "点云为空或场景点云数量不足,检查相机设备"
            
            if self._PVC_pcd_visualize == True:                                                     # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(PVC_pcd)

            # 抓取姿态预测
            # gg_array = self._Forecast_Grasp(PVC_pcd, top_k = self._PVC_topk, which_side = "PVC", visualize = self._PVC_Forcast_visualize)
            gg_array = self.generate_poses_from_o3d_cloud(PVC_pcd, max_points = 5120, num_rotations = 18)
            # 抓取姿态后处理
            gg_array = gg_array[gg_array[:, 0] > self._PVC_score_threshold]                         # 根据评分过滤抓取姿态
            # gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._PVC_degree_threshold)
                                                                                                    # 尽可能保证垂直抓取
            gg_array = self._Collsion_Detect(gg_array, "PVC", scene_pcd, visualize = self._PVC_collision_visualize)
                                                                                                    # 碰撞检测
            
            # 保证存在可抓取姿态
            if len(gg_array) == 0:
                print("[INFO]: PVC, 无有效抓取位姿,抓取失败")
                return 4, "无有效抓取位姿,抓取失败"
            
            # 挑选可抓取姿态
            best_gg = gg_array[0].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
            best_grasp_fliped = rotate_grasp_matrix(best_grasp_fliped, -45)
            best_gg[4:13] = best_grasp_fliped.flatten()
            best_gg[13] += self._PVC_pose_xdelta                                                    # 相机标定 x 轴没有误差
            best_gg[14] += self._PVC_pose_ydelta                                                    # 相机标定 y 轴误差约5.8 cm

            best_gg_grasp1 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_pose_distance - 0.10 + best_gg[1] * 0.01)
            pose1 = convert_grasp_pose_to_6d(best_gg_grasp1, "pose1")                               # 抓取点位(法兰盘)
            best_gg_grasp2 = translate_grasp_point_along_direction(best_gg, distance = self._PVC_pose_distance + best_gg[1] * 0.01)
            pose2 = convert_grasp_pose_to_6d(best_gg_grasp2, "pose2")                               # 抓取点位(夹爪)

            pose1[3:] = np.rad2deg(pose1[3:])                                                       # 预抓取点位
            pose1[3], pose1[4], pose1[5] = adjust_gripper_orientation(pose1[3], pose1[4], pose1[5])
            pose1[3:] = np.deg2rad(pose1[3:])

            pose2[3:] = np.rad2deg(pose2[3:])                                                       # 抓取点位
            pose2[3], pose2[4], pose2[5] = adjust_gripper_orientation(pose2[3], pose2[4], pose2[5])
            pose2[3:] = np.deg2rad(pose2[3:])

        
            pose3 = copy.deepcopy(pose2)                                                            # 抓取完成后的后撤点位
            pose3[2] += 0.25

            self._PVC_pose1 = pose1
            self._PVC_pose2 = pose2
            self._PVC_pose3 = pose3

            if self._PVC_Grasp_visualize:
                vis_gg = []
                vis_gg.append(gg_array[0].copy())
                self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
                self._Vis_Grasps(gg_array, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)

            return 0, "PVC 管上料点计算结束"
        # except Exception as e:
        #     self._Clear_PVC_images()
        #     return 5, e


    def _Process_EVA(self):
        # try:
            self._Clear_EVA_poses()
            # 点云反投影
            scene_pcd, EVA_pcd = self._Depth_to_Pcd(self._EVA_image, self._EVA_depth, which_side = "EVA")   # 反投影点云
            self._Clear_EVA_images()

            # 点云变换和裁剪
            scene_pcd.transform(self._CalibMatrix)                                                  # 将点云从相机坐标系变换到夹爪坐标系
            EVA_pcd.transform(self._CalibMatrix)
            EVA_pcd, _ = EVA_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)
            plane_params = (0.008391, 0.014485, 0.999860, -0.212847)
            EVA_pcd = remove_floor_points(EVA_pcd, plane_params, threshold = 0.0045)
            # filter_pointcloud_by_xy(scene_pcd, x_range = (0.189594, 0.539492), y_range = (-0.971354, -0.553787),  z_range = (-0.10, 0.30))
            # filter_pointcloud_by_xy(EVA_pcd, z_range = (-0.10, 0.30))
            # o3d.io.write_point_cloud("eva_plane.ply", scene_pcd)
            
            if self._EVA_pcd_visualize == True:                                                     # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(EVA_pcd)

            if len(scene_pcd.points) == 0 or len(EVA_pcd.points) < 5120:                            # 点云为空,提前结束
                print("[INFO]: EVA, 生成点云为空,检查相机是否被占用")
                return 6, "点云为空,检查相机设备"
            
            
            # 抓取姿态预测
            # gg_array = self._Forecast_Grasp(EVA_pcd, top_k = self._EVA_topk, which_side = "EVA", visualize = self._EVA_Forcast_visualize)
            gg_array = self.generate_poses_from_o3d_cloud(EVA_pcd, max_points = 5120, num_rotations = 18)
            # 抓取姿态的后处理
            gg_array = gg_array[gg_array[:, 0] > self._EVA_score_threshold]                         # 根据评分过滤抓取姿态
            # gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._EVA_degree_threshold)
                                                                                                    # 尽可能垂直向下抓取
            gg_array = self._Collsion_Detect(gg_array, "EVA", scene_pcd, visualize = self._EVA_collision_visualize)
                                                                                                    # 碰撞检测

            # 保证存在可抓取姿态
            if len(gg_array) == 0:
                print("[INFO]: PVC, 无有效抓取位姿,抓取失败")
                return 7, "无有效抓取位姿, 抓取失败"
            
            # 挑选可抓取姿态
            best_gg = gg_array[0].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
            best_grasp_fliped = rotate_grasp_matrix(best_grasp_fliped, -45)
            best_gg[4:13] = best_grasp_fliped.flatten()
            best_gg[13] += self._EVA_pose_xdelta                                                    # 相机标定 x 轴误差
            best_gg[14] += self._EVA_pose_ydelta                                                    # 相机标定 y 轴误差

            best_gg_grasp1 = translate_grasp_point_along_direction(best_gg, distance = self._EVA_pose_distance - 0.10 + best_gg[1] * 0.01)
            pose1 = convert_grasp_pose_to_6d(best_gg_grasp1, "pose1")                               # 抓取点位(法兰盘)
            best_gg_grasp2 = translate_grasp_point_along_direction(best_gg, distance = self._EVA_pose_distance + best_gg[1] * 0.01)
            pose2 = convert_grasp_pose_to_6d(best_gg_grasp2, "pose2")                               # 抓取点位(夹爪)

            pose1[3:] = np.rad2deg(pose1[3:])                                                       # 预抓取点位
            pose1[3], pose1[4], pose1[5] = adjust_gripper_orientation(pose1[3], pose1[4], pose1[5])
            pose1[3:] = np.deg2rad(pose1[3:])

            pose2[3:] = np.rad2deg(pose2[3:])                                                       # 抓取点位
            pose2[3], pose2[4], pose2[5] = adjust_gripper_orientation(pose2[3], pose2[4], pose2[5])
            pose2[3:] = np.deg2rad(pose2[3:])

            pose3 = copy.deepcopy(pose2)                                                            # 抓取后安全点位
            pose3[2] += 0.25

            self._EVA_pose1 = pose1
            self._EVA_pose2 = pose2
            self._EVA_pose3 = pose3

            if self._EVA_Grasp_visualize:
                self._Vis_Grasps(gg_array, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)                
                vis_gg = []
                vis_gg.append(gg_array[0].copy())
                self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
            
            return 0, "EVA 管上料点计算结束"
        # except Exception as e:
        #     self._Clear_EVA_images()
        #     return 5, e


    def _Load_PVC(self, stop_event = None):
        # try:
            if self._PVC_pose1 is None or self._PVC_pose2 is None or self._PVC_pose3 is None:
                return 8, "PVC 上料点位存在空值"

            # 调整夹爪开口大小 =====================================================================
            print("[INFO]: 抓取 PVC 管前设定夹爪宽度 ......")
            self._Gripper.set_pos(val = 100)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_PVC_poses()
                return -1, "客户端主动关闭"

            # 机械臂复位 ==========================================================================
            print("[INFO]: 机械臂运动到 PVC 管的预抓取点位 ......")
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            if is_same_pose(tcp_pose, self._PVC_homePose) == False:
                mid_pos = get_middle_pose(tcp_pose, self._PVC_homePose)
                mid_pos[2] += 0.10
                self._ARC.MoveC_to_Pose(mid_pos, self._PVC_homePose)
            # 等待复位
            while(is_same_pose(tcp_pose, self._PVC_homePose) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)


            # 移动到 PVC 的抓取点位 1 ==============================================================
            print("[INFO]: 机械臂运动到 PVC 管的中间抓取点位 1 ......")
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            start_pos = tcp_pose[:3]
            _, mid_6dof, _ = generate_robust_trajectory(start_pos, self._PVC_pose1, num_points = 1200)
            print("[INFO]: ", mid_6dof)
            self._ARC.MoveC_to_Pose(mid_6dof, self._PVC_pose1)
            # 等待到达抓取点位 1
            while(is_same_pose(tcp_pose, self._PVC_pose1) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)

            # 移动到 PVC 的抓取点位 2 ===============================================================
            mid_pos = copy.deepcopy(self._PVC_pose1)
            for i in range(3):
                mid_pos[i] = (mid_pos[i] + self._PVC_pose2[i]) / 2.0
            self._ARC.MoveC_to_Pose(mid_pos, self._PVC_pose2)
            # 等待到达抓取点位 2
            while(is_same_pose(tcp_pose, self._PVC_pose2) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            
            # 闭合夹爪 =============================================================================
            time.sleep(0.10)
            self._Gripper.set_pos(val = 10)
            time.sleep(0.05)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_PVC_poses()
                return -1, "客户端主动关闭"

            # 上升到安全点位 3 =====================================================================
            mid_pos = copy.deepcopy(self._PVC_pose2)
            for i in range(3):
                mid_pos[i] = (mid_pos[i] + self._PVC_pose3[i]) / 2.0
            self._ARC.MoveC_to_Pose(mid_pos, self._PVC_pose3)
            # 等待到达抓取点位 3
            while(is_same_pose(tcp_pose, self._PVC_pose3) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)

            # 圆弧运动到途径点 ======================================================================
            mid_pos = get_middle_pose(self._PVC_pose3, self._PVC_waypoint)
            mid_pos[2] += 0.05
            self._ARC.MoveC_to_Pose(mid_pos, self._PVC_waypoint)
            # 等待到达 PVC 上料点位 =================================================================
            while(is_same_pose(tcp_pose, self._PVC_waypoint) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)

            # 圆弧运动到上料点 ======================================================================
            mid_pos = get_middle_pose(self._PVC_waypoint, self.PVC_desPose)
            mid_pos[2] += 0.05
            self._ARC.MoveC_to_Pose(mid_pos, self.PVC_desPose)
            # 等待到达 PVC 上料点位 =================================================================
            while(is_same_pose(tcp_pose, self.PVC_desPose) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            
            # 张开夹爪 =============================================================================
            time.sleep(0.10)
            self._Gripper.set_pos(val = 100)
            time.sleep(0.05)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_PVC_poses()
                return -1, "客户端主动关闭"


            # 返回到 homepose ======================================================================
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            mid_pos = get_middle_pose(tcp_pose, self._PVC_waypoint)
            mid_pos[2] += 0.10
            self._ARC.MoveC_to_Pose(mid_pos, self._PVC_waypoint)
            # 等待到达 EVA 预抓取点位 ===============================================================
            while(is_same_pose(tcp_pose, self._PVC_waypoint) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_PVC_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            self._Clear_PVC_poses()
            return 0, "PVC 上料动作完成"
        
        # except Exception as e:
        #     self._Clear_PVC_poses()
        #     return 5, e
    

    def _Load_EVA(self, stop_event = None):
        # try:
            if self._EVA_pose1 is None or self._EVA_pose2 is None or self._EVA_pose3 is None:
                return 1, "EVA 上料点位存在空值"

            # 调整夹爪开口大小 =====================================================================
            self._Gripper.set_pos(val = 225)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_EVA_poses()
                return -1, "客户端主动关闭"

            # 机械臂复位 ==========================================================================
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            if is_same_pose(tcp_pose, self._EVA_homePose) == False:
                mid_pos = get_middle_pose(tcp_pose, self._EVA_homePose)
                mid_pos[2] += 0.10
                self._ARC.MoveC_to_Pose(mid_pos, self._EVA_homePose)
            # 等待复位
            while(is_same_pose(tcp_pose, self._EVA_homePose) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            

            # 移动到 EVA 的抓取点位 1 ==============================================================
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            start_pos = tcp_pose[:3]
            _, mid_6dof, _ = generate_robust_trajectory(start_pos, self._EVA_pose1, num_points = 1200)
            self._ARC.MoveC_to_Pose(mid_6dof, self._EVA_pose1)
            # 等待到达抓取点位 1
            while(is_same_pose(tcp_pose, self._EVA_pose1) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            # 移动到 EVA 的抓取点位 2 ===============================================================
            mid_pos = copy.deepcopy(self._EVA_pose1)
            for i in range(3):
                mid_pos[i] = (mid_pos[i] + self._EVA_pose2[i]) / 2.0
            self._ARC.MoveC_to_Pose(mid_pos, self._EVA_pose2)
            # 等待到达抓取点位 2
            while(is_same_pose(tcp_pose, self._EVA_pose2) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            # 闭合夹爪 =============================================================================
            time.sleep(0.10)
            self._Gripper.set_pos(val = 120)
            time.sleep(0.05)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_EVA_poses()
                return -1, "客户端主动关闭"

            # 上升到安全点位 3 =====================================================================
            mid_pos = copy.deepcopy(self._EVA_pose2)
            for i in range(3):
                mid_pos[i] = (mid_pos[i] + self._EVA_pose3[i]) / 2.0
            self._ARC.MoveC_to_Pose(mid_pos, self._EVA_pose3)
            # 等待到达抓取点位 3
            while(is_same_pose(tcp_pose, self._EVA_pose3) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            # 圆弧运动到途径点 ======================================================================
            mid_pos = get_middle_pose(self._EVA_pose3, self._EVA_waypoint)
            mid_pos[2] += 0.05
            self._ARC.MoveC_to_Pose(mid_pos, self._EVA_waypoint)
            # 等待到达 PVC 上料点位 =================================================================
            while(is_same_pose(tcp_pose, self._EVA_waypoint) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            # 圆弧运动到上料 ======================================================================
            mid_pos = get_middle_pose(self._EVA_waypoint, self.EVA_desPose)
            mid_pos[2] += 0.10
            self._ARC.MoveC_to_Pose(mid_pos, self.EVA_desPose)
            # 等待到达 PVC 上料点位 =================================================================
            while(is_same_pose(tcp_pose, self.EVA_desPose) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)


            
            # 张开夹爪 =============================================================================
            self._Gripper.set_pos(val = 225)
            time.sleep(0.10)
            if stop_event is not None and stop_event.is_set() == True:
                self._Clear_EVA_poses()
                return -1, "客户端主动关闭"

            # 返回到 homepose ======================================================================
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            mid_pos = get_middle_pose(tcp_pose, self._EVA_waypoint)
            mid_pos[2] += 0.10
            self._ARC.MoveC_to_Pose(mid_pos, self._EVA_waypoint)
            # 等待到达 EVA 预抓取点位 ===============================================================
            while(is_same_pose(tcp_pose, self._EVA_waypoint) == False):
                if stop_event is not None and stop_event.is_set() == True:
                    self._Clear_EVA_poses()
                    return -1, "客户端主动关闭"
                tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
                time.sleep(0.01)
            
            self._Clear_EVA_poses()
            return 0, "泡棉管上料动作完成"

        # except Exception as e:
        #     self._Clear_EVA_poses()
        #     return 5, e

        
    def _Depth_to_Pcd(self, img, depth_img, which_side):
        h, w = depth_img.shape[:2]
        scene_mask = np.zeros((h, w), dtype = bool)                                                 # 场景点云 mask 
        PVC_mask = np.zeros((h, w), dtype = bool)                                                   # PVC 内管点云 mask
        EVA_mask = np.zeros((h, w), dtype = bool)                                                   # PVC 外管点云 mask

        if which_side == "PVC":                                                               # PVC 图像 mask
            scene_mask[self._PVC_scene_range[2]:self._PVC_scene_range[3], self._PVC_scene_range[0]:self._PVC_scene_range[1]] = True
            PVC_mask[self._PVC_pcd_range[2]:self._PVC_pcd_range[3], self._PVC_pcd_range[0]:self._PVC_pcd_range[1]] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = PVC_mask)
            if self._PVC_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                             # 用于可视化场景 mask
                Visualize_Masked_Image(img, PVC_mask)                                               # 用于可视化抓取区域 mask

        elif which_side == "EVA":                                                                   # EVA 图像 mask
            scene_mask[self._EVA_scene_range[2]:self._EVA_scene_range[3], self._EVA_scene_range[0]:self._EVA_scene_range[1]] = True
            EVA_mask[self._EVA_pcd_range[2]:self._EVA_pcd_range[3], self._EVA_pcd_range[0]:self._EVA_pcd_range[1]] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = EVA_mask)
            if self._EVA_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                             # 用于可视化场景 mask
                Visualize_Masked_Image(img, EVA_mask)                                               # 用于可视化抓取区域 mask

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
        if which_side == "EVA":
            collision_mask = mfcdetector.detect_EVA(gg_array, visualize = visualize)
        else:
            collision_mask = mfcdetector.detect_PVC(gg_array, visualize = visualize)
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


    def _emergency_stop(self):
        self._ARC.Stop_Move()
        return 0

    
    def _Move_to_PVC_homePose(self):
        tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                    # 获取机械臂当前 TCP 位姿
        mid_pose = get_middle_pose(tcp_pose, self._PVC_homePose)                                    # 计算当前位姿和拍摄 PVC 管初始位姿的中间位姿
        mid_pose[2] += 0.05
        self._ARC.MoveC_to_Pose(mid_pose, self._PVC_homePose)                                       # 移动机械臂到拍摄 PVC 管的初始位姿
        

    def _Move_to_EVA_homePose(self):
        tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                    # 获取机械臂当前 TCP 位姿
        mid_pos = get_middle_pose(tcp_pose, self._EVA_homePose)                                     # 计算当前位姿和拍摄 EVA 管位姿的中间位姿
        mid_pos[2] += 0.10                                                                          # 中间位姿在 z 轴上抬高 5 cm,以防止机械臂运动过程中与 PVC 外管发生碰撞
        self._ARC.MoveC_to_Pose(mid_pos, self._EVA_homePose)                                        # 机械臂沿着圆弧轨迹移动到拍摄 EVA 管的初始位姿
    

    def _Move_to_PVC_waypoint(self):
        tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                    # 获取机械臂当前 TCP 位姿
        if is_same_pose(tcp_pose, self._PVC_waypoint) == False:
            mid_pos = get_middle_pose(tcp_pose, self._PVC_waypoint)                                 # 计算当前位姿和拍摄位姿的中间位姿
            mid_pos[2] += 0.10                                                                      # 中间位姿在 z 轴上抬高 10 cm,以防止机械臂运动过程中与 PVC 外管发生碰撞
            self._ARC.MoveC_to_Pose(mid_pos, self._PVC_waypoint)                                    # 机械臂沿着圆弧轨迹移动到拍摄 EVA 管的初始位姿
        while(is_same_pose(tcp_pose, self._PVC_waypoint) == False):
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            time.sleep(0.01)


    def _Move_to_EVA_waypoint(self):
        tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                    # 获取机械臂当前 TCP 位姿
        if is_same_pose(tcp_pose, self._EVA_waypoint) == False:
            mid_pos = get_middle_pose(tcp_pose, self._EVA_waypoint)                                 # 计算当前位姿和拍摄位姿的中间位姿
            mid_pos[2] += 0.10                                                                      # 中间位姿在 z 轴上抬高 10 cm,以防止机械臂运动过程中与 PVC 外管发生碰撞
            self._ARC.MoveC_to_Pose(mid_pos, self._EVA_waypoint)                                    # 机械臂沿着圆弧轨迹移动到拍摄 EVA 管的初始位姿
        while(is_same_pose(tcp_pose, self._EVA_waypoint) == False):
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            time.sleep(0.01)


    def _Clear_PVC_images(self):
        self._PVC_depth = None
        self._PVC_image = None
    

    def _Clear_EVA_images(self):
        self._EVA_depth = None
        self._EVA_image = None
    

    def _Clear_PVC_poses(self):
        self._PVC_pose1 = None
        self._PVC_pose2 = None
        self._PVC_pose3 = None


    def _Clear_EVA_poses(self):
        self._EVA_pose1 = None
        self._EVA_pose2 = None
        self._EVA_pose3 = None


    def _zero_back(self):
        def _distance(pos1, pos2):
            return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)
        tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                    # 获取机械臂当前 TCP 位姿

        d_pvc = _distance(tcp_pose, self._PVC_waypoint)                                             # 计算当前点到 PVC 途径点的距离
        d_eva = _distance(tcp_pose, self._EVA_waypoint)                                             # 计算当前点到 EVA 途径点的距离

        des_pose = copy.deepcopy(tcp_pose)
        des_pose[2] = self._PVC_waypoint[2]
        if is_same_pose(tcp_pose, des_pose) == False:
            mid_pos = get_middle_pose(tcp_pose, des_pose)
            self._ARC.MoveC_to_Pose(mid_pos, des_pose)
        
        while(is_same_pose(tcp_pose, des_pose) == False):
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()
            time.sleep(0.01)

        if d_pvc < d_eva:
            self._Move_to_PVC_waypoint()
        else:
            self._Move_to_EVA_waypoint()


grasp_detector = RealTimeGraspDetector()


def grasp_pvc_and_eva(context):
    """
    抓取 PVC 和泡棉的业务逻辑
    Returns:
        tuple: (errorcode, message)
    """

    # 1. 创建线程停止事件（初始未触发）
    stop_event = threading.Event()

    # 2. RPC 生命周期回调：RPC 结束时触发
    def on_rpc_done():
        stop_event.set()  # 标记「停止信号已触发」

    # 3. 注册回调：只要RPC结束（取消/完成/断开），必执行 on_rpc_done
    context.add_callback(on_rpc_done)

    # =========================================================
    # TODO: 算法人员在这里实现具体的业务逻辑
    # =========================================================
    print("[业务逻辑] 执行抓取 PVC 和泡棉")
    error_code, message = grasp_detector._Capture_image_and_depth()
    if error_code != 0:
        return error_code, message
    
    error_code, message = grasp_detector._Process_PVC()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Load_PVC(stop_event)
    if error_code != 0:
        return error_code, message

    error_code, message = grasp_detector._Process_EVA()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Load_EVA(stop_event)

    return error_code, message


def grasp_pvc(context):
    """
    单抓 PVC 的业务逻辑

    Returns:
        tuple: (errorcode, message)
    """
    # 1. 创建线程停止事件（初始未触发）
    stop_event = threading.Event()
    # 2. RPC 生命周期回调：RPC 结束时触发
    def on_rpc_done():
        stop_event.set()  # 标记「停止信号已触发」
    # 3. 注册回调：只要RPC结束（取消/完成/断开），必执行 on_rpc_done
    context.add_callback(on_rpc_done)

    # =========================================================
    # TODO: 算法人员在这里实现具体的业务逻辑
    # =========================================================
    print("[业务逻辑] 执行单抓 PVC")
    error_code, message = grasp_detector._Capture_image_and_depth()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Process_PVC()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Load_PVC(stop_event)
    return error_code, message


def grasp_eva(context):
    """
    单抓泡棉的业务逻辑

    Returns:
        tuple: (errorcode, message)
    """
    # 1. 创建线程停止事件（初始未触发）
    stop_event = threading.Event()
    # 2. RPC 生命周期回调：RPC 结束时触发
    def on_rpc_done():
        stop_event.set()  # 标记「停止信号已触发」
    # 3. 注册回调：只要RPC结束（取消/完成/断开），必执行 on_rpc_done
    context.add_callback(on_rpc_done)

    # =========================================================
    # TODO: 算法人员在这里实现具体的业务逻辑
    # =========================================================
    print("[业务逻辑] 执行单抓泡棉")
    error_code, message = grasp_detector._Capture_image_and_depth()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Process_EVA()
    if error_code != 0:
        return error_code, message
    error_code, message = grasp_detector._Load_EVA(stop_event)
    return error_code, message
    

def emergency_stop():
    grasp_detector._emergency_stop()
    return 0, "急停成功"


def zero_back():
    grasp_detector._zero_back()
    return 0, "机械臂复位"


def process_pvc(context):
    print("[业务逻辑] 执行 PVC 上料点位计算流程")
    error_code, message = grasp_detector._Process_PVC()
    return error_code, message


def process_eva(context):
    print("[业务逻辑] 执行 泡棉管 上料点位计算流程")
    error_code, message = grasp_detector._Process_EVA()
    return error_code, message


def capture_image():
    print("[业务逻辑] 拍摄上料场景图")
    error_code, message = grasp_detector._Capture_image_and_depth()
    return error_code, message


def load_pvc(context):
    """
    单抓 PVC 的业务逻辑

    Returns:
        tuple: (errorcode, message)
    """
    # 1. 创建线程停止事件（初始未触发）
    stop_event = threading.Event()
    # 2. RPC 生命周期回调：RPC 结束时触发
    def on_rpc_done():
        stop_event.set()  # 标记「停止信号已触发」
    # 3. 注册回调：只要RPC结束（取消/完成/断开），必执行 on_rpc_done
    context.add_callback(on_rpc_done)

    # =========================================================
    # TODO: 算法人员在这里实现具体的业务逻辑
    # =========================================================
    print("[业务逻辑] 执行单抓 PVC")
    success, message = grasp_detector._Load_PVC(stop_event)
    return success, message


def load_eva(context):
    """
    单抓泡棉的业务逻辑

    Returns:
        tuple: (errorcode, message)
    """
    # 1. 创建线程停止事件（初始未触发）
    stop_event = threading.Event()
    # 2. RPC 生命周期回调：RPC 结束时触发
    def on_rpc_done():
        stop_event.set()  # 标记「停止信号已触发」
    # 3. 注册回调：只要RPC结束（取消/完成/断开），必执行 on_rpc_done
    context.add_callback(on_rpc_done)

    # =========================================================
    # TODO: 算法人员在这里实现具体的业务逻辑
    # =========================================================
    print("[业务逻辑] 执行单抓泡棉")
    success, message = grasp_detector._Load_EVA(stop_event)
    return success, message


class GraspServiceServicer(grasp_pb2_grpc.GraspServiceServicer):
    """gRPC 服务实现类"""
    def __init__(self):
        super().__init__()
        # 2. 初始化一个互斥锁
        self._lock = threading.Lock()

    def GraspPVCandEVA(self, request, context):
        print(f"[gRPC] GraspPVCandEVA 收到请求，正在等待锁... request: {request.defaultReq}")
        
        # 3. 使用 with 语句。如果锁被占用，线程会在这里静静等待，直到前一个线程释放
        with self._lock:
            print(f"[gRPC] GraspPVCandEVA 获取到锁，开始执行...")
            errorcode, message = grasp_pvc_and_eva(context)
            
        # 离开 with 作用域后，锁会自动释放，排队中的下一个线程会被唤醒
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)


    def GraspPVC(self, request, context):
        print(f"[gRPC] GraspPVC 收到请求，正在等待锁... request: {request.defaultReq}")
        
        with self._lock:
            print(f"[gRPC] GraspPVC 获取到锁，开始执行...")
            errorcode, message = grasp_pvc(context)
            
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)


    def GraspEVA(self, request, context):
        print(f"[gRPC] GraspEVA 收到请求，正在等待锁... request: {request.defaultReq}")
        
        with self._lock:
            print(f"[gRPC] GraspEVA 获取到锁，开始执行...")
            errorcode, message = grasp_eva(context)
            
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    

    def EmergencyStop(self, request, context):
        # 急停依然不加锁，即使别的线程在排队或执行上料，急停也能瞬间插队执行
        print(f"[gRPC] EmergencyStop 急停被调用, request: {request.defaultReq}")
        errorcode, message = emergency_stop()
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    

    def ZeroBack(self, request, context):
        print(f"[gRPC] ZeroBack 机械臂回零, request: {request.defaultReq}")
        errorcode, message = zero_back()
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    

    def ProcessPVC(self, request, context):
        print(f"[gRPC] ProcessPVC 计算 PVC 管上料点位被调用, request: {request.defaultReq}")
        errorcode, message = process_pvc(context)
        print(errorcode, message)
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    

    def ProcessEVA(self, request, context):
        print(f"[gRPC] ProcessEVA 计算泡棉管上料点位被调用, request: {request.defaultReq}")
        errorcode, message = process_eva(context)
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    
    def CaptureImage(self, request, context):
        print(f"[gRPC] CaptureImage 计算泡棉管上料点位被调用, request: {request.defaultReq}")
        errorcode, message = capture_image()
        print(errorcode, message)
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)

    def LoadPVC(self, request, context):
        print(f"[gRPC] LoadPVC 被调用, request: {request.defaultReq}")
        errorcode, message = load_pvc(context)
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)
    
    def LoadEVA(self, request, context):
        print(f"[gRPC] LoadEVA 被调用, request: {request.defaultReq}")
        errorcode, message = load_eva(context)
        return grasp_pb2.DefaultResponse(errorcode=errorcode, message=message)