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
from datetime import datetime
import random
import pyaubo_sdk
import numpy as np
import open3d as o3d

current_dir = os.path.abspath(os.path.dirname(__file__))
desired_dir = os.path.abspath(os.path.join(current_dir, "..", "Sim_GraspNet"))
sys.path.insert(0, desired_dir)

# 导入SimGraspNet相关模块
from utils import visualize_pcd
from utils import depth_image2pcd
from utils import pose_6d_to_matrix
from utils import remove_floor_points
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
from Slerp_utils import *

from pyDHgripper import AG95
from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import *

device = "cuda" if torch.cuda.is_available() else "cpu"


class AuboRobotController:                                                                          # 专用于机械臂控制的类
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
        self._robot.getMotionControl() \
            .moveLine(pose, 50 * (self._M_PI / 180), 1000 * (self._M_PI / 180), 0, 0)
        self._waitArrival()
    

    def MoveC_to_Pose(self, pose1, pose2):
        ret = self._robot.getMotionControl()\
            .moveCircle(pose1, pose2, 60 * (self._M_PI / 180), 1000000 * (self._M_PI / 180), 0, 0) # 接口调用: 圆弧运动
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

    def Capture(self, which_side = None, save = False):
        """
            采集 RGB 图像、深度图和点云数据，并保存为文件
        """
        img = None 

        # 采集 RGB 图像
        frame2d = Frame2D()
        show_error(self.camera.capture_2d(frame2d))
        color_map = frame2d.get_color_image()
        img = color_map.data()

        # 采集深度图
        frame3d = Frame3D()
        show_error(self.camera.capture_3d(frame3d))
        depth_map = frame3d.get_depth_map()
        depth_img = depth_map.data()

        if save == True:
            # 生成时间戳作为文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]                            # 格式: 20260417_150530_123
            
            # 创建保存目录（如果不存在）
            save_dir = "./captured_images"
            os.makedirs(save_dir, exist_ok=True)
            
            # 保存 RGB 图像为 PNG
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)                                          # 转换为BGR用于cv2保存
            if which_side == "PVC":
                img_path = os.path.join(save_dir, "PVC", "images", f"{timestamp}.png")
                depth_npy_path = os.path.join(save_dir, "PVC", "depth_npys", f"{timestamp}.npy")
                depth_png_path = os.path.join(save_dir, "PVC", "depth_imgs", f"{timestamp}.png")

            elif which_side == "CB":
                img_path = os.path.join(save_dir, "CB", "images", f"{timestamp}.png")
                depth_npy_path = os.path.join(save_dir, "CB", "depth_npys", f"{timestamp}.npy")
                depth_png_path = os.path.join(save_dir, "CB", "depth_imgs", f"{timestamp}.png")

            cv2.imwrite(img_path, img_bgr)

            # 保存深度图为 NPY
            np.save(depth_npy_path, depth_img)
            
            # 将深度图值归一化到 0-255 范围并保存为 PNG
            # 获取深度图的最小和最大值
            depth_min = np.min(depth_img)
            depth_max = np.max(depth_img)
            
            if depth_max > depth_min:
                # 归一化到 0-255
                depth_normalized = ((depth_img - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
            else:
                depth_normalized = np.zeros_like(depth_img, dtype=np.uint8)
            
            cv2.imwrite(depth_png_path, depth_normalized)

        return img, depth_img


class RealTimeGraspDetector:
    def __init__(self) -> None:
        self._configs = "./params/configs_CB.json"                                                  # 配置文件路径
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
        self._CB_processing = False                                                                # CB 管处理状态

        self._PVC_homePose = None                                                                   # 拍摄 PVC 管时机械臂的法兰盘的位姿
        self._PVC_scene_range = None                                                                # PVC 管的碰撞范围
        self._PVC_pcd_range = None                                                                  # PVC 管的采样范围
        self.PVC_grasp_pose = None                                                                  # PVC 管抓取姿态点位
        self.PVC_desPose = None                                                                     # PVC 管预设放置点位

        self._CB_homePose = None                                                                   # 拍摄 CB 管时机械臂的法兰盘的位姿
        self._CB_scene_range = None                                                                # CB 管的碰撞范围
        self._CB_pcd_range = None                                                                  # CB 管的采样范围
        self.CB_grasp_pose = None                                                                  # CB 管抓取姿态点位
        self.CB_desPose = None                                                                     # CB 管预设放置点位

        self._PVC_image_visualize = False                                                           # PVC 管图像 mask 可视化开关
        self._CB_image_visualize = False                                                           # CB 管图像 mask 可视化开关
        self._PVC_pcd_visualize = False                                                             # PVC 管点云可视化开关
        self._CB_pcd_visualize = False                                                             # CB 管点云可视化开关
        self._PVC_Forcast_visualize = False                                                         # PVC 管抓取预测结果可视化开关
        self._CB_Forcast_visualize = False                                                         # CB 管抓取预测结果可视化开关
        self._PVC_collision_visualize = False                                                       # PVC 管碰撞检测结果可视化开关
        self._CB_collision_visualize = False                                                       # CB 管碰撞检测结果可视化开关
        self._PVC_Grasp_visualize = False                                                           # PVC 管抓取位姿可视化开关
        self._CB_Grasp_visualize = False                                                           # CB 管抓取位姿可视化开关
        self._PVC_score_threshold = None                                                            # PVC 管抓取评分过滤阈值
        self._CB_score_threshold = None                                                            # CB 管抓取评分过滤阈值
        self._PVC_degree_threshold = None                                                           # PVC 管抓取垂直度过滤阈值
        self._CB_degree_threshold = None                                                           # CB 管抓取垂直度过滤阈值
        self._PVC_pose_distance = None                                                              # PVC 管抓取点位沿着抓取方向的平移距离
        self._CB_pose_distance = None                                                              # CB 管抓取点位沿着抓取方向的平移距离
        self._PVC_topk = None                                                                       # PVC 管抓取预测 topk
        self._CB_topk = None                                                                       # CB 管抓取预测 topk
        self._PVC_pose_xdelta = None                                                                # PVC 管抓取点位 x 轴方向偏移量
        self._PVC_pose_ydelta = None                                                                # PVC 管抓取点位 y 轴方向偏移量
        self._CB_pose_xdelta = None                                                                # CB 管抓取点位 x 轴方向偏移量
        self._CB_pose_ydelta = None                                                                # CB 管抓取点位 y 轴方向偏移量

        self._Load_Params()                                                                         # 加载参数
        self._PVC_w2e_matrix = pose_6d_to_matrix(self._PVC_homePose)                                # 6D 位姿转化为外参矩阵
        self._CB_w2e_matrix = pose_6d_to_matrix(self._CB_homePose)                                # 6D 位姿转化为外参矩阵
        self._SimGraspNet = sim_grasp_net_model(self._simgrasp_checkpoint_path)                     # 加载 simgrasp 模型
        self._ARC = AuboRobotController(self._robot_ip, self._robot_port)                           # 初始化机械臂控制接口
        self._Gripper = AG95("COM3")                                                                # 初始化 DHgripper 控制接口，并设置夹爪参数
        self._Gripper.set_vel(1000)
        self._Gripper.set_rot_vel(100)                                                              
        self._CC = ConnectAndCaptureImages()                                                        # 初始化 Mecheye 相机控制接口


    def _Load_Params(self) -> None:
        """
            加载各种参数
        """
        with open(self._configs, "r") as f:                                                         # 加载 config 配置文件中的参数
            params = json.load(f)
        self._PVC_scene_range = params["PVC_scene_range"]
        self._PVC_pcd_range = params["PVC_pcd_range"]
        self._CB_scene_range = params["CB_scene_range"]
        self._CB_pcd_range = params["CB_pcd_range"]
        self._camera_server_url = params["camera_server_url"]
        self._camera_params = params["camera_params"]
        self._voxel_size = params["voxel_size"]
        self._simgrasp_checkpoint_path = params["simgrasp_checkpoint_path"]
        self._robot_ip = params["robot_ip"]
        self._robot_port = params["robot_port"]
        self._M_PI = params["M_PI"]

        PVC_homePose_array = np.asarray(params["PVC_homePose"])                                     # 加载机械臂初始位姿并将欧拉角从度转化为弧度
        PVC_homePose_array[3:] = np.deg2rad(PVC_homePose_array[3:])
        self._PVC_homePose = PVC_homePose_array
        CB_homePose_array = np.asarray(params["CB_homePose"])
        CB_homePose_array[3:] = np.deg2rad(CB_homePose_array[3:])
        self._CB_homePose = CB_homePose_array
        

        self._PVC_image_visualize = params["PVC_image_visualize"]
        self._CB_image_visualize = params["CB_image_visualize"]
        self._PVC_pcd_visualize = params["PVC_pcd_visualize"]
        self._CB_pcd_visualize = params["CB_pcd_visualize"]
        self._PVC_Forcast_visualize = params["PVC_Forcast_visualize"]
        self._CB_Forcast_visualize = params["CB_Forcast_visualize"]
        self._PVC_collision_visualize = params["PVC_collision_visualize"]
        self._CB_collision_visualize = params["CB_collision_visualize"]
        self._PVC_Grasp_visualize = params["PVC_Grasp_visualize"]
        self._CB_Grasp_visualize = params["CB_Grasp_visualize"]
        self._PVC_score_threshold = params["PVC_score_threshold"]
        self._CB_score_threshold = params["CB_score_threshold"]
        self._PVC_degree_threshold = params["PVC_degree_threshold"]
        self._CB_degree_threshold = params["CB_degree_threshold"]
        self._PVC_pose_distance = params["PVC_pose_distance"]
        self._CB_pose_distance = params["CB_pose_distance"]
        self._PVC_topk = params["PVC_topk"]
        self._CB_topk = params["CB_topk"]
        self._PVC_pose_xdelta = params["PVC_pose_xdelta"]
        self._PVC_pose_ydelta = params["PVC_pose_ydelta"]
        self._CB_pose_xdelta = params["CB_pose_xdelta"]
        self._CB_pose_ydelta = params["CB_pose_ydelta"]

        CB_desPose = np.asarray(params["CB_desPose"])                                             # CB 管预设放置位置
        CB_desPose[3:] = np.deg2rad(CB_desPose[3:])
        self.CB_desPose = CB_desPose

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


    def perform_grasp_detect_PVC(self):                                                             # 执行 PVC 管的抓取任务
        try:
            self._PVC_processing = True                                                             # 标定为正在执行
            print("[INFO]: PVC, 开始执行抓取检测流程 ...")

            # ------------------------- 机械臂和夹爪复位流程 -------------------------
            start_time = time.time()
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                # 获取机械臂当前 TCP 位姿
            mid_pose = get_middle_pose(tcp_pose, self._PVC_homePose)                                # 计算当前位姿和拍摄 PVC 管初始位姿的中间位姿
            mid_pose[2] += 0.05
            self._ARC.MoveC_to_Pose(mid_pose, self._PVC_homePose)                                   # 移动机械臂到拍摄 PVC 管的初始位姿
            time.sleep(1.25 * 3)
            self._Gripper.set_pos(val = 100)                                                        # 调整夹爪到合适的宽度,以防止与 PVC 内管发生碰撞
            end_time = time.time()
            print("[INFO]: PVC, 机器复位花费时间 ", end_time - start_time, "s")
            # ----------------------------------------------------------------------

            # ------------------------- 深度图采集流程 -------------------------
            start_time = time.time()
            img, depth = self._CC.Capture(which_side = "PVC", save = True)                          # 采集 RGB 图像和深度图
            end_time = time.time()
            print("[INFO]: PVC, 采集图像花费时间 ", end_time - start_time, "s")
            # -----------------------------------------------------------------

            # ------------------------- 点云反投影流程 -------------------------
            start_time = time.time()
            scene_pcd, PVC_pcd = self._Depth_to_Pcd(img, depth, which_side = "PVC")                 # 反投影点云
            if len(scene_pcd.points) == 0 or len(PVC_pcd.points) == 0:                              # 点云未空,提前结束
                print("[INFO]: PVC, 生成点云为空,检查相机是否被占用")
                self._PVC_processing = False
                return False, "点云为空,检查相机设备"
            end_time = time.time()
            print("[INFO]: PVC, 点云投影花费时间 ", end_time - start_time, "s")
            # -----------------------------------------------------------------
            
            # ------------------------- 点云预处理流程 -------------------------
            start_time = time.time()
            scene_pcd.transform(self._CalibMatrix)                                                  # 将点云从相机坐标系变换到夹爪坐标系
            PVC_pcd.transform(self._CalibMatrix)
            scene_pcd.transform(self._PVC_w2e_matrix)                                               # 将点云从夹爪坐标系变换到世界坐标系
            PVC_pcd.transform(self._PVC_w2e_matrix)
            PVC_pcd, _ = PVC_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)

            plane_params = (0.005351, -0.002177, 0.999983, -0.011734)                               # 去除平面点云
            PVC_pcd = remove_floor_points(PVC_pcd, plane_params, threshold = 0.0050)
            filter_pointcloud_by_xy(scene_pcd, x_range = (-0.135453, 0.126521), y_range = (-0.981123, -0.626720))

            # o3d.io.write_point_cloud("plane.ply", scene_pcd)

            if self._PVC_pcd_visualize == True:                                                     # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(PVC_pcd)
            end_time = time.time()
            print("[INFO]: PVC, 点云处理花费时间 ", end_time - start_time, "s")
            # -----------------------------------------------------------------

            # ------------------------- 预测抓取姿态流程 -------------------------
            start_time = time.time()
            gg_array = self._Forecast_Grasp(PVC_pcd, top_k = self._PVC_topk, which_side = "PVC", visualize = self._PVC_Forcast_visualize)
            print("[INFO]: PVC, 预测抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC, 预测抓取花费时间 ", end_time - start_time, "s")
            # -------------------------------------------------------------------

            # ------------------------- 抓取姿态后处理流程 -------------------------
            start_time = time.time()
            gg_array = gg_array[gg_array[:, 0] > self._PVC_score_threshold]                         # 根据评分过滤抓取姿态
            print("[INFO]: PVC, 评分过滤后抓取位姿数量:", len(gg_array))

            gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._PVC_degree_threshold)
                                                                                                    # 尽可能保证垂直抓取
            print("[INFO]: PVC, 垂直抓取过滤后抓取位姿数量:", len(gg_array))


            gg_array = self._Collsion_Detect(gg_array, "PVC", scene_pcd, visualize = self._PVC_collision_visualize)
                                                                                                    # 碰撞检测
            print("[INFO]: PVC, 碰撞检测后抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: PVC, 抓取处理花费时间 ", end_time - start_time, "s")

            if len(gg_array) == 0:
                print("[INFO]: PVC, 无有效抓取位姿,抓取失败")
                self._PVC_processing = False  
                return False, "无有效抓取位姿,抓取失败"
            # --------------------------------------------------------------------

            # ------------------------- 挑选抓取姿态处理流程 -------------------------
            gg_array = gg_array[np.argsort(-gg_array[:, 15])]
            random_index = np.random.randint(0, len(gg_array))                                      # 从候选抓取姿态中随即选取一个
            # random_index = 0
            best_gg = gg_array[random_index].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
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
            # ----------------------------------------------------------------------

            if self._PVC_Grasp_visualize:
                vis_gg = []
                vis_gg.append(gg_array[random_index].copy())
                self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
                self._Vis_Grasps(gg_array, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)


            # ------------------------- 上料处理流程 -------------------------
            # 圆弧运动到预抓取点位，然后执行抓取
            opt_start_time = time.time()
            start_pos = self._PVC_homePose[:3]
            end_6DoF = pose1
            trajs, mid_6DoF, _ = generate_robust_trajectory(start_pos, end_6DoF, num_points = 1200)
            self._ARC.MoveC_to_Pose(mid_6DoF, end_6DoF)                                             # 沿着圆弧轨迹移动到抓取点位
            time.sleep(0.75 * 2)
            self._ARC.Move_to_Pose(pose2)                                                           # 移动到抓取点位
            self._Gripper.set_pos(val = 15)                                                         # 执行抓取动作
            time.sleep(0.05)                                                                        # 等待夹爪稳定闭合
            # 上升到安全点位，然后圆弧运动到 PVC 管的放置点位
            self._ARC.Move_to_Pose(pose3)
            mid_pos = get_middle_pose(pose3, self.PVC_desPose)                                      # 计算抓取点位和放置点位的中间位姿
            mid_pos[2] += 0.20
            self._ARC.MoveC_to_Pose(mid_pos, self.PVC_desPose)                                      # 沿着圆弧轨迹移动到放置点位上方
            time.sleep(1.25 * 2)
            self._Gripper.set_pos(val = 100)                                                        # 张开夹爪,完成放置
            time.sleep(0.05)                                                                        # 等待夹爪稳定张开
            opt_end_time = time.time()
            print(f"[INFO]: PVC, 抓取执行时间: {opt_end_time - opt_start_time:.2f} 秒")
            # ---------------------------------------------------------------

            self.PVC_grasp_pose = pose2
            self._PVC_processing = False
            return True, "PVC 管检测抓取成功"

        except Exception as e:
            error_msg = f"抓取检测过程中发生错误: {str(e)}"
            print("[INFO]:", error_msg)
            self._PVC_processing = False
            return False, error_msg


    def perform_grasp_detect_CB(self):                                                             # 执行 CB 管的抓取任务
        try:
            self._CB_processing = True                                                             # 标定为正在执行
            print("[INFO]: CB, 开始执行抓取检测流程 ...")

            # ------------------------- 机械臂和夹爪复位流程 -------------------------
            start_time = time.time()
            tcp_pose = self._ARC._robot.getRobotState().getTcpPose()                                # 获取机械臂当前 TCP 位姿
            mid_pos = get_middle_pose(tcp_pose, self._CB_homePose)                                 # 计算当前位姿和拍摄 CB 管位姿的中间位姿
            mid_pos[2] += 0.05                                                                      # 中间位姿在 z 轴上抬高 5 cm,以防止机械臂运动过程中与 PVC 外管发生碰撞
            self._ARC.MoveC_to_Pose(mid_pos, self._CB_homePose)                                    # 机械臂沿着圆弧轨迹移动到拍摄 CB 管的初始位姿
            time.sleep(1.25 * 3)
            self._Gripper.set_pos(val = 250)                                                        # 张开夹爪
            end_time = time.time()
            print("[INFO]: CB, 机器复位花费时间 ", end_time - start_time, "s")

            start_time = time.time()
            img, depth = self._CC.Capture(which_side = "CB", save = False)                          # 采集 RGB 图像和深度图
            end_time = time.time()
            print("[INFO]: CB, 采集图像花费时间 ", end_time - start_time, "s")
            # ----------------------------------------------------------------------

            # ------------------------- 深度图采集流程 -------------------------
            start_time = time.time()
            scene_pcd, CB_pcd = self._Depth_to_Pcd(img, depth, which_side = "CB")                 # 反投影点云
            if len(scene_pcd.points) == 0 or len(CB_pcd.points) == 0:                              # 点云为空,提前结束
                print("[INFO]: CB, 生成点云为空,检查相机是否被占用")
                self._CB_processing = False
                return False, "点云为空,检查相机设备"
            end_time = time.time()
            print("[INFO]: CB, 点云投影花费时间 ", end_time - start_time, "s")
            # ------------------------------------------------------------------
            
            # ------------------------- 点云预处理流程 -------------------------
            start_time = time.time()
            scene_pcd.transform(self._CalibMatrix)                                                  # 将点云从相机坐标系变换到夹爪坐标系
            CB_pcd.transform(self._CalibMatrix)
            scene_pcd.transform(self._CB_w2e_matrix)                                               # 将点云从夹爪坐标系变换到世界坐标系
            CB_pcd.transform(self._CB_w2e_matrix)
            CB_pcd, _ = CB_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 2.0)
            plane_params = (0.005351, -0.002177, 0.999983, -0.011734)
            CB_pcd = remove_floor_points(CB_pcd, plane_params, threshold = 0.0040)
            filter_pointcloud_by_xy(scene_pcd, x_range = (-0.135951, 0.413263), y_range = (-0.982722, -0.629318))

            # o3d.io.write_point_cloud("CB_scene.ply", scene_pcd)

            if self._CB_pcd_visualize == True:                                                     # 可视化点云
                visualize_pcd(scene_pcd)
                visualize_pcd(CB_pcd)
            end_time = time.time()
            print("[INFO]: CB, 点云处理花费时间 ", end_time - start_time, "s")
            # -----------------------------------------------------------------

            # ------------------------- 抓取姿态预测处理流程 -------------------------
            start_time = time.time()
            gg_array = self._Forecast_Grasp(CB_pcd, top_k = self._CB_topk, which_side = "CB", visualize = self._CB_Forcast_visualize)
            # print("[INFO]: CB, 预测抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: CB, 预测抓取花费时间 ", end_time - start_time, "s")
            # ----------------------------------------------------------------------

            # ------------------------- 抓取姿态后处理流程 -------------------------
            start_time = time.time()
            gg_array = gg_array[gg_array[:, 0] > self._CB_score_threshold]                         # 根据评分过滤抓取姿态
            # print("[INFO]: CB, 评分过滤后抓取位姿数量:", len(gg_array))

            gg_array = filter_vertical_grasps_simple(gg_array, max_angle_degrees = self._CB_degree_threshold)
                                                                                                    # 尽可能垂直向下抓取
            # print("[INFO]: CB, 垂直抓取过滤后抓取位姿数量:", len(gg_array))

            gg_array = self._Collsion_Detect(gg_array, "CB", scene_pcd, visualize = self._CB_collision_visualize)
                                                                                                    # 碰撞检测
            print("[INFO]: CB, 碰撞检测后抓取位姿数量:", len(gg_array))
            end_time = time.time()
            print("[INFO]: CB, 抓取处理花费时间 ", end_time - start_time, "s")

            if len(gg_array) == 0:
                print("[INFO]: CB, 无有效抓取位姿,抓取失败")
                self._CB_processing = False
                return False, "无有效抓取位姿,抓取失败"
            # ----------------------------------------------------------------------

            # ------------------------- 挑选抓取姿态处理流程 -------------------------
            gg_array = gg_array[np.argsort(-gg_array[:, 15])]
            random_index = np.random.randint(0, len(gg_array))                                      # 从候选抓取姿态中随即选取一个
            best_gg = gg_array[random_index].copy()
            best_grasp = best_gg.copy()[4:13].reshape(3, 3)
            best_grasp_fliped = flip_rotation_matrix(best_grasp)
            best_gg[4:13] = best_grasp_fliped.flatten()
            best_gg[13] += self._CB_pose_xdelta                                                    # 相机标定 x 轴误差
            best_gg[14] += self._CB_pose_ydelta                                                    # 相机标定 y 轴误差

            best_gg_grasp1 = translate_grasp_point_along_direction(best_gg, distance = self._CB_pose_distance - 0.10 + best_gg[1] * 0.01)
            pose1 = convert_grasp_pose_to_6d(best_gg_grasp1, "pose1")                               # 抓取点位(法兰盘)
            best_gg_grasp2 = translate_grasp_point_along_direction(best_gg, distance = self._CB_pose_distance + best_gg[1] * 0.01)
            pose2 = convert_grasp_pose_to_6d(best_gg_grasp2, "pose2")                               # 抓取点位(夹爪)

            pose1[3:] = np.rad2deg(pose1[3:])                                                       # 预抓取点位
            pose1[3], pose1[4], pose1[5] = adjust_gripper_orientation(pose1[3], pose1[4], pose1[5])
            pose1[3:] = np.deg2rad(pose1[3:])

            pose2[3:] = np.rad2deg(pose2[3:])                                                       # 抓取点位
            pose2[3], pose2[4], pose2[5] = adjust_gripper_orientation(pose2[3], pose2[4], pose2[5])
            pose2[3:] = np.deg2rad(pose2[3:])

            pose3 = copy.deepcopy(pose2)                                                            # 抓取后安全点位
            pose3[2] += 0.25
            # ----------------------------------------------------------------------

            if self._CB_Grasp_visualize:
                self._Vis_Grasps(gg_array, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)
                vis_gg = []
                vis_gg.append(gg_array[random_index].copy())
                self._Vis_Grasps(vis_gg, scene_pcd, scale_factor = 1.0, top_percent = 100.0, top1_only = False, sort_or_not = False)

            # ------------------------- 抓取处理流程 -------------------------
            # opt_start_time = time.time()
            # start_pos = self._CB_homePose[:3]
            # end_6DoF = pose1
            # trajs, mid_6DoF, _ = generate_robust_trajectory(start_pos, end_6DoF, num_points = 1200)
            # self._ARC.MoveC_to_Pose(mid_6DoF, end_6DoF)                                             # 先沿着圆弧轨迹移动到抓取点位上方
            # time.sleep(0.75 * 2)
            # self._ARC.Move_to_Pose(pose2)                                                           # 移动到抓取点位上方
            # self._Gripper.set_pos(val = 120)                                                        # 执行抓取动作
            # time.sleep(0.05)
            
            # self._ARC.Move_to_Pose(pose3)
            # mid_pos = get_middle_pose(pose3, self.CB_desPose)
            # mid_pos[2] += 0.20
            # self._ARC.MoveC_to_Pose(mid_pos, self.CB_desPose)                                      # 抓取后沿着圆弧轨迹移动到抓取点位上方
            # time.sleep(1.25 * 2)

            # self._Gripper.set_pos(val = 250)                                                        # 张开夹爪,完成放置
            # time.sleep(0.05)
            # opt_end_time = time.time()
            # print(f"[INFO]: CB, 抓取执行时间: {opt_end_time - opt_start_time:.2f} 秒")
            # ---------------------------------------------------------------

            
            self.CB_grasp_pose = pose2
            self._CB_processing = False
            return True, "CB 管检测抓取成功"

        except Exception as e:
            error_msg = f"抓取检测过程中发生错误: {str(e)}"
            print("[INFO]:", error_msg)
            self._CB_processing = False
            return False, error_msg


    def _Depth_to_Pcd(self, img, depth_img, which_side):
        h, w = depth_img.shape[:2]
        scene_mask = np.zeros((h, w), dtype = bool)                                                 # 场景点云 mask 
        PVC_mask = np.zeros((h, w), dtype = bool)                                                   # PVC 内管点云 mask
        CB_mask = np.zeros((h, w), dtype = bool)                                                   # PVC 外管点云 mask

        if which_side == "PVC":                                                               # PVC 图像 mask
            scene_mask[self._PVC_scene_range[2]:self._PVC_scene_range[3], self._PVC_scene_range[0]:self._PVC_scene_range[1]] = True
            PVC_mask[self._PVC_pcd_range[2]:self._PVC_pcd_range[3], self._PVC_pcd_range[0]:self._PVC_pcd_range[1]] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = PVC_mask)
            if self._PVC_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                             # 用于可视化场景 mask
                Visualize_Masked_Image(img, PVC_mask)                                               # 用于可视化抓取区域 mask

        elif which_side == "CB":                                                                   # CB 图像 mask
            scene_mask[self._CB_scene_range[2]:self._CB_scene_range[3], self._CB_scene_range[0]:self._CB_scene_range[1]] = True
            CB_mask[self._CB_pcd_range[2]:self._CB_pcd_range[3], self._CB_pcd_range[0]:self._CB_pcd_range[1]] = True
            scene_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = scene_mask)
            pipes_pcd = depth_image2pcd(img, depth_img, self._CameraMatrix, self._factor_depth, mask = CB_mask)
            if self._CB_image_visualize == True:
                Visualize_Masked_Image(img, scene_mask)                                             # 用于可视化场景 mask
                Visualize_Masked_Image(img, CB_mask)                                               # 用于可视化抓取区域 mask

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
        if which_side == "CB":
            collision_mask = mfcdetector.detect_CB(gg_array, visualize = visualize)
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


grasp_detector = RealTimeGraspDetector()


if __name__ == "__main__":
    homepose = [0.10792, -0.71082, 0.73372, 179.998, 0.002, -179.990]
    homepose[3:] = np.deg2rad(homepose[3:])
    grasp_detector._ARC.robot_rpc_client.getRuntimeMachine().start()
    time.sleep(2)
    for i in range(1):
        start_time = time.time()
        grasp_detector.perform_grasp_detect_CB()
        # grasp_detector.perform_grasp_detect_PVC()
        end_time = time.time()
        print(f"[INFO]: 抓取检测执行时间: {end_time - start_time:.2f} 秒 \n")
    time.sleep(2)
    # grasp_detector._ARC.Move_to_Pose(homepose)
    grasp_detector._ARC.robot_rpc_client.getRuntimeMachine().stop()



# [0.14351, -0.70683, 0.73370, 180.00, 0.001, -179.992]                     # 能够拍摄全完整的框
# "PVC_scene_range": [730, 1870, 40, 1700],