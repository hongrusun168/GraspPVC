#import open3d as o3d
import glob
import pickle
import numpy as np
import sys
import os
import numpy as np

from tqdm import tqdm
from torch.utils.data import Dataset
import open3d as o3d
import provider
import torch

import os
import numpy as np

import torch
import collections.abc as container_abcs
from torch.utils.data import Dataset
from tqdm import tqdm
#import MinkowskiEngine as ME
import h5py
from collections import defaultdict, Counter
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from sklearn.mixture import GaussianMixture
import matplotlib.cm
from scipy.spatial import cKDTree
from collections import defaultdict
import copy


def pc_normalize(pc):
    centroid = np.mean(pc, axis=0)
    #centroid=0
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc ** 2, axis=1)))
    pc = pc / m
    #pc=pc
   # m=0
    return pc, centroid, m


def rotate_point_cloud_with_normal(xyz_normal):
    ''' Randomly rotate XYZ, normal point cloud.
        Input:
            xyz_normal: N,6, first three channels are XYZ, last 3 all normal
        Output:
            N,6, rotated XYZ, normal point cloud
    '''
    rot_angle = (np.random.random() * np.pi / 3) - np.pi / 6  # -30 ~ +30 degree
    #rot_angle = 0  # -30 ~ +30 degree

    c, s = np.cos(rot_angle), np.sin(rot_angle)
    rotation_matrix = np.array([[1, 0, 0],
                                [0, c, -s],
                                [0, s, c]])
    
    shape_pc = xyz_normal[:, 0:3]
    shape_normal = xyz_normal[:, 3:6]
    
    xyz_normal[:, 0:3] = np.dot(shape_pc, rotation_matrix)
    xyz_normal[:, 3:6] = np.dot(shape_normal, rotation_matrix)

    return xyz_normal,rotation_matrix


def random_scale_point_cloud(data, scale_low=0.9, scale_high=1.2):
    """ Randomly scale the point cloud. Scale is per point cloud.
        Input:
            BxNx3 array, original batch of point clouds
        Return:
            BxNx3 array, scaled batch of point clouds
    """
    N, C = data.shape
    scales = np.random.uniform(scale_low, scale_high)
    #for batch_index in range(n):
    data[:,:] *= scales
    return data,scales


def visualize_point_clouds(scored_positions, scores, points_with_features):
    # 可视化带分数的点云，并可视化带特征的点云

    # Create a point cloud object for scored_positions
    scored_pcd = o3d.geometry.PointCloud()
    scored_pcd.points = o3d.utility.Vector3dVector(scored_positions)
    
    # Map scores to colors using a colormap (e.g., hot, which goes from blue to red)
    max_score = scores.max()
    min_score = scores.min()
    scores_normalized = (scores - min_score) / (max_score - min_score)
    colors = plt.get_cmap('hot')(scores_normalized)[:, :3]  # Get the RGB values from the colormap
    scored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # Create a point cloud object for points_with_features
    features_pcd = o3d.geometry.PointCloud()
    features_pcd.points = o3d.utility.Vector3dVector(points_with_features[:, :3])
    
    # Assuming the last three columns are RGB values for visualization
    # Normalize feature values to [0, 1] for color mapping if necessary
    features_pcd.paint_uniform_color([0.5, 0.5, 0.5])

    # Visualize both point clouds in the same window
    o3d.visualization.draw_geometries([scored_pcd, features_pcd], 
                                      window_name="Scored Positions and Point Cloud with Features",
                                      point_show_normal=False)


def visualize_dense_scores_heatmap(points, dense_scores):
    """
    Visualize a heatmap of dense scores on a point cloud.

    :param points: Nx3 numpy array of point cloud positions.
    :param dense_scores: N-length numpy array of scores for each point.
    """
    # Normalize the dense scores to [0, 1] for colormap mapping
    scores_normalized = (dense_scores - np.min(dense_scores)) / (np.max(dense_scores) - np.min(dense_scores))
    
    # Apply a colormap (e.g., 'hot') to the normalized scores to get RGB colors
    cmap = plt.get_cmap('hot')
    colors = cmap(scores_normalized.flatten())[:, :3]  # Get the RGB values, ignore alpha

    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Visualize the point cloud with the heatmap
    o3d.visualization.draw_geometries([pcd], window_name="Dense Scores Heatmap")


def visualize_binary_grasp_quality_heatmap(points, binary_scores):
    # 可视化二值抓取质量热力图，显示哪些位置可以抓取，哪些不能抓取
    # Create a custom color map for binary scores: 0s to red, 1s to green
    colors = np.zeros((len(binary_scores), 3))
    colors[binary_scores == 0, :] = [1, 0, 0]  # Red for non-graspable points
    colors[binary_scores == 1, :] = [0, 1, 0]  # Green for graspable points

    # Create an Open3D point cloud for visualization
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud with the heatmap
    o3d.visualization.draw_geometries([pcd], window_name="Binary Grasp Quality Heatmap")





def visualize_vectors_with_scores(unique_points, view_scores_matrix, views, fixed_length=0.1):
    """
    Visualizes unique points with vectors indicating significant views.
    Vectors have a fixed length and are colored from red (low score) to green (high score).

    Parameters:
    - unique_points: Nx3 numpy array of unique points' coordinates.
    - view_scores_matrix: Nx300 matrix with scores for each view of each unique point.
    - views: 300x3 array of view direction vectors from Fibonacci lattice.
    - fixed_length: Fixed length for each vector.
    """
    # Initialize point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(unique_points)

    # Initialize lists for lines and colors
    lines = []
    colors = []

    # Determine score range for color mapping
    min_score = np.min(view_scores_matrix[view_scores_matrix > 0])
    max_score = np.max(view_scores_matrix)

    # Function to map scores to colors
    def score_to_color(score, min_score, max_score):
        normalized_score = (score - min_score) / (max_score - min_score)
        return [1-normalized_score, normalized_score, 0]  # RGB color

    # Process each unique point and its scores
    N = len(unique_points)
    for i in range(N):
        for view_idx, score in enumerate(view_scores_matrix[i]):
            if score > 0:
                direction = views[view_idx]
                direction_normalized = direction / np.linalg.norm(direction)
                end_point = unique_points[i] + direction_normalized * fixed_length
                lines.append([unique_points[i], end_point])
                colors.append(score_to_color(score, min_score, max_score))

    # Preparing line set for visualization
    lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.vstack(lines)),
        lines=o3d.utility.Vector2iVector(lines_idx)
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Visualize
    o3d.visualization.draw_geometries([pcd, line_set])


def print_grasp_structure_info(grasp_data_structure):
    # Top Level - Grasp Points
    print(f"Total number of unique grasp points (t_ori): {len(grasp_data_structure)}")

    for point_key, approach_vectors in grasp_data_structure.items():
        print(f"\nGrasp Point {point_key}:")
        print(f"  Number of unique approach vectors: {len(approach_vectors)}")

        for approach_key, rotation_matrices in approach_vectors.items():
            print(f"    Approach Vector {approach_key}:")
            print(f"      Number of unique suction rotation matrices: {len(rotation_matrices)}")

            for rotation_key, depths in rotation_matrices.items():
                print(f"        Rotation Matrix {rotation_key}:")
                print(f"          Number of stand-off depths: {len(depths)}")



class simgraspdata(Dataset):
    def __init__(self, data_root, label_root, stage_range=None, max_stages=None):
        """
        初始化数据集
        
        参数:
            data_root: 点云数据根目录
            label_root: 标签数据根目录  
            stage_range: 场景范围，格式为(start, end)或None表示全部
            max_stages: 最大场景数量，None表示无限制
        """
        self.data_root = data_root
        self.label_root = label_root
        #print(data_root)
        #stage_root = os.listdir(data_root)
        
        self.room_points={}
        self.room_coord_min, self.room_coord_max = [], []
        
        block_size=1000
        self.block_size=block_size
        self.num_point=40000
        num_point=self.num_point

        # Initialize a dictionary to store the count of subframes for each stage
        stage_frame_count = {}
        self.data_list = []
        
        # 获取所有可用的场景文件
        available_stages = []
        for file_name in os.listdir(self.data_root):
            if file_name.endswith('.npz'):
                parts = file_name.split('_')
                if len(parts) == 2:
                    # This is a frame point cloud file
                    stage, frame_with_ext = parts
                    frame = frame_with_ext.split('.')[0]
                    stage = int(stage)
                    available_stages.append((stage, int(frame)))
                elif len(parts) == 1:
                    # This is a multiview point cloud file
                    stage_with_ext = parts[0]
                    stage = int(stage_with_ext.split('.')[0])
                    available_stages.append((stage, None))
        
        # 按场景编号排序
        available_stages.sort(key=lambda x: x[0])
        
        # 应用场景范围过滤
        if stage_range is not None:
            start_stage, end_stage = stage_range
            available_stages = [(stage, frame) for stage, frame in available_stages 
                               if start_stage <= stage <= end_stage]
        
        # 应用最大场景数量限制
        if max_stages is not None:
            available_stages = available_stages[:max_stages]
        
        # 加载所有符合条件的场景（不再排除任何场景）
        for stage, frame in available_stages:
            self.data_list.append([stage, frame])
            if stage not in stage_frame_count:
                stage_frame_count[stage] = 1
            else:
                stage_frame_count[stage] += 1

        # Calculate the total length as the sum of all stages and their respective subframes
        self.total_length = sum(stage_frame_count.values())
        print("Total length:", self.total_length)
        #print(self.room_idxs)

        # Compute the rotation matrices for aligning the view vectors with the z-axis


    def __len__(self):
        return (self.total_length)

    def __getitem__(self, index):
        """
        数据集的__getitem__方法 - 返回一个训练样本
        
        Args:
            index: 样本索引
            
        Returns:
            ret_dict: 包含训练数据的字典，这就是网络输入的原始end_points
        """
        
        # ==================== 1. 获取场景和帧信息 ====================
        room_idx, frame = self.data_list[index]  # 获取场景索引和帧号

        # 构建点云文件路径
        if frame is not None:
            file_path = f"{self.data_root}/{room_idx}_{frame}.npz"  # 单帧点云文件
        else:
            file_path = f"{self.data_root}/{room_idx}.npz"  # 多视角点云文件

        # ==================== 2. 加载原始点云数据 ====================
        points = np.load(file_path, allow_pickle=True)['arr_0']  # 加载点云数据
        # points形状: (N_original, 10) - 包含xyz坐标(3) + 法向量(3) + RGB颜色(3) + 分割标签(1)
        N_points = points.shape[0]  # 原始点云的点数

        # ==================== 3. 随机选择区域中心点 ====================
        center = points[np.random.choice(N_points)][:3]  # 随机选择一个点作为区域中心
        block_min = center - [self.block_size / 2.0, self.block_size / 2.0, 0]  # 区域最小坐标
        block_max = center + [self.block_size / 2.0, self.block_size / 2.0, 0]  # 区域最大坐标
        # block_size = 1000，创建一个1000x1000的区域
        
        # ==================== 4. 加载预处理标签数据 ====================
        preprocessed_data_path = self.label_root + f"/stage_{room_idx}/stage_{room_idx}_preprocessed_data.pkl"
        with open(preprocessed_data_path, 'rb') as f:
            preprocessed_data = pickle.load(f)  # 加载预处理的抓取标签数据

        # 解包预处理数据
        unique_t_ori_points = copy.deepcopy(preprocessed_data['unique_t_ori_points'])  # (N, 3) - 唯一抓取点坐标
        approach_directions = copy.deepcopy(preprocessed_data['approach_directions'])  # (N, 3, 3) - 接近方向向量
        N = len(unique_t_ori_points)  # 唯一抓取点的数量

        # ==================== 5. 区域点云筛选 ====================
        # 筛选在指定区域内的点云
        point_idxs = np.where((points[:, 0] >= block_min[0]) & (points[:, 0] <= block_max[0]) & 
                             (points[:, 1] >= block_min[1]) & (points[:, 1] <= block_max[1]))[0]
        
        # 从区域内的点中随机选择self.num_point个点
        if point_idxs.size >= self.num_point:
            selected_point_idxs = np.random.choice(point_idxs, self.num_point, replace=False)  # 不重复采样
        else:
            selected_point_idxs = np.random.choice(point_idxs, self.num_point, replace=True)   # 重复采样

        # 根据索引选择点云
        selected_points = points[selected_point_idxs, :]  # (40000, 10) - 选择的点云数据
        
        # ==================== 6. 数据增强：随机缩放 ====================
        # scale_factor: 缩放因子，范围[0.9, 1.2]
        scaled_points, scale_factor = random_scale_point_cloud(selected_points[:, 0:3])  # 随机缩放xyz坐标
        
        # 将缩放后的坐标与原始特征组合
        # points_w_feature: (40000, 6) - 缩放后的点云数据，包含缩放后的xyz坐标和原始法向量特征
        points_w_feature = np.zeros((self.num_point, 6))  # (40000, 6)
        points_w_feature[:, 0:3] = scaled_points  # 缩放后的xyz坐标
        points_w_feature[:, 3:6] = selected_points[:, 3:6]  # 原始法向量特征
        
        # ==================== 7. 数据增强：随机旋转 ====================
        # rotation_matrix: (3, 3) - 旋转矩阵，绕x轴旋转-30°到+30°
        rotated_points_w_feature, rotation_matrix = rotate_point_cloud_with_normal(points_w_feature)
        
        # ==================== 8. 点云归一化 ====================
        normalized_points, centroid, max_distance = pc_normalize(rotated_points_w_feature[:, 0:3])
        # normalized_points: (40000, 3) - 归一化后的坐标
        # centroid: (3,) - 点云中心
        # max_distance: 标量 - 最大距离，用于归一化
        
        # ==================== 9. 唯一抓取点处理 ====================
        sparse_points_xyz = unique_t_ori_points  # (N, 3) - 原始唯一抓取点坐标
        sparse_points_xyz *= scale_factor  # 应用相同的缩放因子
        sparse_points_xyz_rotated = np.dot(unique_t_ori_points, rotation_matrix)  # 应用相同的旋转矩阵
        # 应用相同的归一化
        sparse_points_xyz_rotated -= centroid  # 减去中心
        sparse_points_xyz_rotated /= max_distance  # 除以最大距离
        sparse_points_xyz_normalized = sparse_points_xyz_rotated  # (N, 3) - 归一化后的唯一抓取点坐标
        
        # ==================== 10. 接近方向处理 ====================
        # 将接近方向重塑为2D格式进行旋转
        approach_directions_reshaped = approach_directions.reshape(-1, 3)  # (N*3, 3)
        rotated_approach_directions_reshaped = np.dot(approach_directions_reshaped, rotation_matrix)  # (N*3, 3)
        rotated_approach_directions = rotated_approach_directions_reshaped.reshape(-1, 3, 3)  # (N, 3, 3)
        
        # ==================== 11. 最终点云数据组合 ====================
        final_points_w_feature = np.zeros_like(points_w_feature)  # (40000, 6)
        final_points_w_feature[:, 0:3] = normalized_points  # 归一化后的 xyz 坐标
        final_points_w_feature[:, 3:6] = rotated_points_w_feature[:, 3:6]  # 旋转后的法向量特征

        print (f"final_points_w_feature shape: {final_points_w_feature.shape}")
        # =======================================================================================================================================================================================
        temp_pcd = o3d.geometry.PointCloud()
        # print (normalized_points)
        temp_pcd.points = o3d.utility.Vector3dVector(normalized_points.astype(np.float64))
        temp_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 1.0, max_nn = 30))
        final_points_w_feature[:, 3:6] = np.asarray(temp_pcd.normals)
        # =======================================================================================================================================================================================
        
        
        # ==================== 12. 视图分数二值化 ====================
        # 每个唯一抓取点有 3 个接近方向，对这些方向的视图分数进行二值化，>0.9 为 True，否则为 False
        binary_view_scores = (preprocessed_data['normalized_view_score'] > 0.9)  # (N, 3)
        
        # ==================== 13. 构建返回字典 ====================
        ret_dict = {
            # 完整点云数据 (40000, 6) - xyz坐标 + 法向量
            'point_clouds': final_points_w_feature.astype(np.float64),
            
            # 点云坐标 (40000, 3) - 仅xyz坐标
            "coors": final_points_w_feature[:, 0:3].astype(np.float64),
            
            # 点云特征 (40000, 3) - 仅法向量
            "feats": final_points_w_feature[:, 3:6].astype(np.float64),
            
            # 唯一抓取点坐标 (N, 3) - 归一化后的唯一抓取点位置
            'sparse_points': sparse_points_xyz_normalized.astype(np.float64),
            
            # 归一化抓取分数 (N,) - 每个唯一抓取点的质量分数
            'normalized_scores': preprocessed_data['normalized_scores'].astype(np.float64),
            
            # 旋转后的接近方向 (N, 3, 3) - 每个唯一抓取点的3个接近方向矩阵
            'rotated_approach_directions': rotated_approach_directions.astype(np.float64),
            
            # 二值化视图分数 (N, 3) - 每个唯一抓取点的3个方向的二值化分数，>0.9为1，否则为0
            'normalized_view_score': binary_view_scores.astype(np.float64),
            
            # 归一化抓取分数 (N, 3, 12, 7) - 每个唯一抓取点每个方向的抓取质量分数
            'normalized_grasp_score': preprocessed_data['normalized_grasp_score'].astype(np.float64),
            
            # 数据增强旋转矩阵 (3, 3) - 用于数据增强的旋转矩阵
            "augument_matrix": rotation_matrix,
        }

        return ret_dict













