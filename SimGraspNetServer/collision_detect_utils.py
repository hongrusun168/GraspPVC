import copy
import json
import time
import numpy as np
import open3d as o3d
from utils import draw_gripper
from utils import visualize_pcd
from utils import collision_checker


class CollisionDetector():
    def __init__(self, pcd, voxel_size = 0.005):
        self.voxel_size = voxel_size
        self.pcd = copy.deepcopy(pcd)
        self.pcd_sampled = self.pcd.voxel_down_sample(voxel_size = self.voxel_size)
        self._EVA_width = None                                  # 抓取 EVA 管时夹爪的张开距离
        self._EVA_FINGER_WIDTH = None                           # 抓取 EVA 管时夹爪的指厚（沿着 x 轴方向的夹爪大小）
        self._EVA_FINGER_HEIGHT = None                          # 抓取 EVA 管时夹爪的指宽（沿着 y 轴方向的夹爪大小）
        self._EVA_FINGER_LENGTH = None                          # 抓取 EVA 管时夹爪的指长（沿着 z 轴方向的夹爪大小）
        self._EVA_MIDDLE_WIDTH = None                           # 抓取 EVA 管时夹爪的中间接触块厚度
        self._EVA_MIDDLE_HEIGHT = None                          # 抓取 EVA 管时夹爪的中间接触块宽度
        self._EVA_MIDDLE_LENGTH = None                          # 抓取 EVA 管时夹爪的中间接触块长度
        self._EVA_MIDDLE_OFFSET = None                          # 抓取 EVA 管时夹爪的中间接触块起始 Z
        self._EVA_BASE_WIDTH = None                             # 抓取 EVA 管时夹爪基座的厚度
        self._EVA_BASE_HEIGHT = None                            # 抓取 EVA 管时夹爪基座的宽度
        self._EVA_BASE_LENGTH = None                            # 抓取 EVA 管时夹爪基座的长度

        self._PVC_width = None                                  # 抓取 PVC 管时夹爪的张开距离
        self._PVC_FINGER_WIDTH = None                           # 抓取 PVC 管时夹爪的指厚（沿着 x 轴方向的夹爪大小）
        self._PVC_FINGER_HEIGHT = None                          # 抓取 PVC 管时夹爪的指宽（沿着 y 轴方向的夹爪大小）
        self._PVC_FINGER_LENGTH = None                          # 抓取 PVC 管时夹爪的指长（沿着 z 轴方向的夹爪大小）
        self._PVC_MIDDLE_WIDTH = None                           # 抓取 PVC 管时夹爪的中间接触块厚度
        self._PVC_MIDDLE_HEIGHT = None                          # 抓取 PVC 管时夹爪的中间接触块宽度
        self._PVC_MIDDLE_LENGTH = None                          # 抓取 PVC 管时夹爪的中间接触块长度
        self._PVC_MIDDLE_OFFSET = None                          # 抓取 PVC 管时夹爪的中间接触块起始 Z
        self._PVC_BASE_WIDTH = None                             # 抓取 PVC 管时夹爪基座的厚度
        self._PVC_BASE_HEIGHT = None                            # 抓取 PVC 管时夹爪基座的宽度
        self._PVC_BASE_LENGTH = None                            # 抓取 PVC 管时夹爪基座的长度

    
    def load_params_from_json(self, json_path):
        with open(json_path, 'r') as f:
            params = json.load(f)
        
        self._EVA_width = params["EVA_width"]
        self._EVA_FINGER_WIDTH = params["EVA_FINGER_WIDTH"]
        self._EVA_FINGER_HEIGHT = params["EVA_FINGER_HEIGHT"]
        self._EVA_FINGER_LENGTH = params["EVA_FINGER_LENGTH"]
        self._EVA_MIDDLE_WIDTH = params["EVA_MIDDLE_WIDTH"]
        self._EVA_MIDDLE_HEIGHT = params["EVA_MIDDLE_HEIGHT"]
        self._EVA_MIDDLE_LENGTH = params["EVA_MIDDLE_LENGTH"]
        self._EVA_MIDDLE_OFFSET = params["EVA_MIDDLE_OFFSET"]
        self._EVA_BASE_WIDTH = params["EVA_BASE_WIDTH"]
        self._EVA_BASE_HEIGHT = params["EVA_BASE_HEIGHT"]
        self._EVA_BASE_LENGTH = params["EVA_BASE_LENGTH"]

        self._PVC_width = params["PVC_width"]
        self._PVC_FINGER_WIDTH = params["PVC_FINGER_WIDTH"]
        self._PVC_FINGER_HEIGHT = params["PVC_FINGER_HEIGHT"]
        self._PVC_FINGER_LENGTH = params["PVC_FINGER_LENGTH"]
        self._PVC_MIDDLE_WIDTH = params["PVC_MIDDLE_WIDTH"]
        self._PVC_MIDDLE_HEIGHT = params["PVC_MIDDLE_HEIGHT"]
        self._PVC_MIDDLE_LENGTH = params["PVC_MIDDLE_LENGTH"]
        self._PVC_MIDDLE_OFFSET = params["PVC_MIDDLE_OFFSET"]
        self._PVC_BASE_WIDTH = params["PVC_BASE_WIDTH"]
        self._PVC_BASE_HEIGHT = params["PVC_BASE_HEIGHT"]
        self._PVC_BASE_LENGTH = params["PVC_BASE_LENGTH"]


    def _crop_by_depth(self, depth_min = None, depth_max = None):
        """
        通过深度过滤掉点云中的点，减少计算开销
        """
        pcd_to_crop = copy.deepcopy(self.pcd)
        points = np.asarray(pcd_to_crop.points)
        colors = np.asarray(pcd_to_crop.colors) if pcd_to_crop.has_colors() else None
        normals = np.asarray(pcd_to_crop.normals) if pcd_to_crop.has_normals() else None

        depth_mask = np.ones((points.shape[0],), dtype = bool)
        if depth_min is not None:
            depth_mask = depth_mask & (points[:, 2] >= depth_min)
        if depth_max is not None:
            depth_mask = depth_mask & (points[:, 2] <= depth_max)

        points_cropped = points[depth_mask]
        pcd_cropped = o3d.geometry.PointCloud()
        pcd_cropped.points = o3d.utility.Vector3dVector(points_cropped)
        if colors is not None:
            colors_cropped = colors[depth_mask]
            pcd_cropped.colors = o3d.utility.Vector3dVector(colors_cropped)
        if normals is not None:
            normals_cropped = normals[depth_mask]
            pcd_cropped.normals = o3d.utility.Vector3dVector(normals_cropped)
        
        self.pcd = pcd_cropped

    
    def detect_EVA(self, gg_array, visualize = False):

        gripper = draw_gripper(width = 0.0170,                                              # 夹爪张开距离
                               FINGER_WIDTH = 0.002,                                        # 夹爪的厚度，沿着 x 轴方向的夹爪大小
                               FINGER_HEIGHT = 0.025,                                       # 夹爪的宽度，沿着 y 轴方向的夹爪大小
                               FINGER_LENGTH = 0.100,                                       # 夹爪的长度，沿着 z 轴方向的夹爪大小
                               MIDDLE_WIDTH = 0.015,                                        # 中心接触手指的厚度
                               MIDDLE_HEIGHT = 0.025,                                       # 中心接触手指的宽度
                               MIDDLE_LENGTH = 0.010,                                       # 中心接触手指的长度
                               MIDDLE_BEGIN_z = 0.0880,                                     # 中心接触手指的起始 z 点
                               BASE_WIDTH = 0.050,                                          # 夹爪基部的厚度
                               BASE_HEIGHT = 0.020,                                         # 夹爪基部的宽度
                               BASE_LENGTH = 0.050)                                         # 夹爪基部的长度

        gripper_params = {
            'width': 0.0170,                                    # 夹爪张开距离
            'FINGER_WIDTH': 0.002,                              # 指厚 (X)
            'FINGER_HEIGHT': 0.025,                             # 指宽 (Y)
            'FINGER_LENGTH': 0.100,                             # 指长 (Z)
            'MIDDLE_WIDTH': 0.015,                              # 中间接触块厚度
            'MIDDLE_HEIGHT': 0.025,                             # 中间接触块宽度
            'MIDDLE_LENGTH': 0.010,                             # 中间接触块长度
            'MIDDLE_OFFSET': 0.0885,                            # 中间接触块起始 Z
            'BASE_WIDTH': 0.0500,                               # 基座厚度
            'BASE_HEIGHT': 0.020,                               # 基座宽度
            'BASE_LENGTH': 0.050,                               # 基座长度
        } 
    
        collision_results = collision_checker(
            self.pcd_sampled,                                   # 经过预处理的点云
            gg_array,                                           # 抓取姿态阵列
            gripper = gripper,                                  # 碰撞检测的夹爪模型
            **gripper_params,                                   # 解包几何参数
            collision_point_threshold = 5,                      # 避障：超过此点数认为碰撞
            min_points_per_region = 25,                         # 抓取：每个半区最少需要的点数
            depth_scale = -0.008,                               # 内抓深度单位 1mm
            visualize = visualize,                              # 是否可视化
        )
        return collision_results
    

    def detect_PVC(self, gg_array, visualize = False):

        gripper = draw_gripper(width = 0.00675,                                             # 夹爪张开距离
                               FINGER_WIDTH = 0.002,                                        # 夹爪的厚度，沿着 x 轴方向的夹爪大小
                               FINGER_HEIGHT = 0.020,                                       # 夹爪的宽度，沿着 y 轴方向的夹爪大小
                               FINGER_LENGTH = 0.200,                                       # 夹爪的长度，沿着 z 轴方向的夹爪大小
                               MIDDLE_WIDTH = 0.004,                                        # 中心接触手指的厚度
                               MIDDLE_HEIGHT = 0.020,                                       # 中心接触手指的宽度
                               MIDDLE_LENGTH = 0.008,                                       # 中心接触手指的长度
                               MIDDLE_BEGIN_z = 0.191,                                      # 中心接触手指的起始 z 点
                               BASE_WIDTH = 0.030,                                          # 夹爪基部的厚度
                               BASE_HEIGHT = 0.020,                                         # 夹爪基部的宽度
                               BASE_LENGTH = 0.030)                                         # 夹爪基部的长度

        gripper_params = {
            'width': 0.00675,           # 夹爪张开距离
            'FINGER_WIDTH': 0.002,      # 指厚 (X)
            'FINGER_HEIGHT': 0.020,     # 指宽 (Y)
            'FINGER_LENGTH': 0.200,     # 指长 (Z)
            'MIDDLE_WIDTH': 0.004,      # 中间接触块厚度
            'MIDDLE_HEIGHT': 0.020,     # 中间接触块宽度
            'MIDDLE_LENGTH': 0.008,     # 中间接触块长度
            'MIDDLE_OFFSET': 0.191,     # 中间接触块起始 Z
            'BASE_WIDTH': 0.060,        # 基座厚度
            'BASE_HEIGHT': 0.040,       # 基座宽度
            'BASE_LENGTH': 0.060,       # 基座长度
        }

        collision_results = collision_checker(
            self.pcd_sampled,                                   # 经过预处理的点云
            gg_array,                                           # 抓取姿态阵列
            gripper = gripper,                                  # 碰撞检测的夹爪模型
            **gripper_params,                                   # 解包几何参数
            collision_point_threshold = 3,                      # 避障：超过此点数认为碰撞
            min_points_per_region = 10,                         # 抓取：每个半区最少需要的点数
            depth_scale = -0.003,                               # 内抓深度单位 1mm
            visualize = visualize,                              # 是否可视化
        )

        return collision_results


    def detect_AL(self, gg_array, visualize = False):

        gripper = draw_gripper(width = 0.022,                                               # 夹爪张开距离
                               FINGER_WIDTH = 0.001,                                        # 夹爪的厚度，沿着 x 轴方向的夹爪大小
                               FINGER_HEIGHT = 0.050,                                       # 夹爪的宽度，沿着 y 轴方向的夹爪大小
                               FINGER_LENGTH = 0.100,                                       # 夹爪的长度，沿着 z 轴方向的夹爪大小
                               MIDDLE_WIDTH = 0.015,                                        # 中心接触手指的厚度
                               MIDDLE_HEIGHT = 0.050,                                       # 中心接触手指的宽度
                               MIDDLE_LENGTH = 0.010,                                       # 中心接触手指的长度
                               MIDDLE_BEGIN_z = 0.089,                                      # 中心接触手指的起始 z 点
                               BASE_WIDTH = 0.080,                                          # 夹爪基部的厚度
                               BASE_HEIGHT = 0.050,                                         # 夹爪基部的宽度
                               BASE_LENGTH = 0.080)                                         # 夹爪基部的长度

        gripper_params = {
            'width': 0.022,                                     # 夹爪张开距离
            'FINGER_WIDTH': 0.001,                              # 指厚 (X)
            'FINGER_HEIGHT': 0.050,                             # 指宽 (Y)
            'FINGER_LENGTH': 0.100,                             # 指长 (Z)
            'MIDDLE_WIDTH': 0.015,                              # 中间接触块厚度
            'MIDDLE_HEIGHT': 0.050,                             # 中间接触块宽度
            'MIDDLE_LENGTH': 0.010,                             # 中间接触块长度
            'MIDDLE_OFFSET': 0.089,                             # 中间接触块起始 Z
            'BASE_WIDTH': 0.080,                                # 基座厚度
            'BASE_HEIGHT': 0.050,                               # 基座宽度
            'BASE_LENGTH': 0.080,                               # 基座长度
        }

        collision_results = collision_checker(
            self.pcd_sampled,                                   # 经过预处理的点云
            gg_array,                                           # 抓取姿态阵列
            gripper = gripper,                                  # 碰撞检测的夹爪模型
            **gripper_params,                                   # 解包几何参数
            collision_point_threshold = 3,                      # 避障：超过此点数认为碰撞
            min_points_per_region = 50,                         # 抓取：每个半区最少需要的点数
            depth_scale = -0.004,                               # 内抓深度单位 1mm
            visualize = visualize,                              # 是否可视化
        )

        return collision_results
    

    def detect_CB(self, gg_array, visualize = False):

        gripper = draw_gripper(width = 0.055,                                               # 夹爪张开距离
                               FINGER_WIDTH = 0.015,                                        # 夹爪的厚度，沿着 x 轴方向的夹爪大小
                               FINGER_HEIGHT = 0.050,                                       # 夹爪的宽度，沿着 y 轴方向的夹爪大小
                               FINGER_LENGTH = 0.100,                                       # 夹爪的长度，沿着 z 轴方向的夹爪大小
                               MIDDLE_WIDTH = 0.030,                                        # 中心接触手指的厚度
                               MIDDLE_HEIGHT = 0.050,                                       # 中心接触手指的宽度
                               MIDDLE_LENGTH = 0.010,                                       # 中心接触手指的长度
                               MIDDLE_BEGIN_z = 0.085,                                      # 中心接触手指的起始 z 点
                               BASE_WIDTH = 0.080,                                          # 夹爪基部的厚度
                               BASE_HEIGHT = 0.050,                                         # 夹爪基部的宽度
                               BASE_LENGTH = 0.080)                                         # 夹爪基部的长度

        gripper_params = {
            'width': 0.055,                                     # 夹爪张开距离
            'FINGER_WIDTH': 0.015,                              # 指厚 (X)
            'FINGER_HEIGHT': 0.050,                             # 指宽 (Y)
            'FINGER_LENGTH': 0.100,                             # 指长 (Z)
            'MIDDLE_WIDTH': 0.030,                              # 中间接触块厚度
            'MIDDLE_HEIGHT': 0.050,                             # 中间接触块宽度
            'MIDDLE_LENGTH': 0.010,                             # 中间接触块长度
            'MIDDLE_OFFSET': 0.085,                             # 中间接触块起始 Z
            'BASE_WIDTH': 0.080,                                # 基座厚度
            'BASE_HEIGHT': 0.050,                               # 基座宽度
            'BASE_LENGTH': 0.080,                               # 基座长度
        }

        collision_results = collision_checker(
            self.pcd_sampled,                                   # 经过预处理的点云
            gg_array,                                           # 抓取姿态阵列
            gripper = gripper,                                  # 碰撞检测的夹爪模型
            **gripper_params,                                   # 解包几何参数
            collision_point_threshold = 3,                      # 避障：超过此点数认为碰撞
            min_points_per_region = 30,                         # 抓取：每个半区最少需要的点数
            depth_scale = -0.002,                               # 内抓深度单位 1mm
            visualize = visualize,                              # 是否可视化
            show_num = 5,                                       # 可视化夹爪个数
        )

        return collision_results
