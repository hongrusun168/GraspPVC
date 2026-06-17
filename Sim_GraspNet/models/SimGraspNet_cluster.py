import os
import sys
import numpy as np
import torch
import torch.nn as nn

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(ROOT_DIR)
from models.SimGraspDataset import simgraspdata
from models.modules import ApproachVecNet, GraspAffordanceNet,GroupNet, PoseNet
from pointnet2.pointnet2_utils import furthest_point_sample, gather_operation
from torch.utils.data import  DataLoader
from sim_suction_model.utils.pointnet2_model import Pointnet2_scorenet
from loss_utils import generate_grasp_views,batch_viewpoint_params_to_matrix,batch_rot_matrix,batch_viewpoint_params_to_matrix_data
from knn.knn_modules import knn
GRASPNESS_THRESHOLD = 0.05
NUM_VIEW = 800
M_POINT = 2048
import open3d as o3d    
import matplotlib
import copy

import torch.nn.functional as F

def debug_end_points(end_points, stage_name="Unknown"):
    """
    调试函数：详细打印end_points的内容
    
    Args:
        end_points: 要调试的字典
        stage_name: 调试阶段名称
    """
    print(f"\n{'='*50}")
    print(f"🔍 DEBUG: {stage_name}")
    print(f"{'='*50}")
    
    for key, value in end_points.items():
        if isinstance(value, torch.Tensor):
            print(f"📊 {key}:")
            print(f"   Shape: {value.shape}")
            if value.numel() < 20:  # 如果元素数量少，打印具体值
                print(f"   Values: {value}")
            else:
                if value.dtype in [torch.float32, torch.float64, torch.float16]:
                    print(f"   Min: {value.min():.4f}, Max: {value.max():.4f}, Mean: {value.mean():.4f}")
                else:
                    print(f"   Min: {value.min()}, Max: {value.max()}")
        elif isinstance(value, (list, tuple)):
            print(f"📋 {key}: {type(value)} with {len(value)} items")
        else:
            print(f"📝 {key}: {type(value)} = {value}")
    print(f"{'='*50}\n")

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

    # Function to map scores to colors
    def score_to_color(score):
        # 处理二值化数据的颜色映射
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:  # score == 1
            return [0.0, 0.0, 1.0]  # Blue for valid grasps

    # Process each unique point and its scores
    N = len(unique_points)
    total_lines = 0
    
    # 只可视化非零分数的方向（保持原有逻辑）
    for i in range(N):
        point_lines = 0
        for view_idx, score in enumerate(view_scores_matrix[i]):
            if score > 0:  # 只可视化有效方向
                direction = views[view_idx]
                direction_normalized = direction / np.linalg.norm(direction)
                end_point = unique_points[i] + direction_normalized * fixed_length
                lines.append([unique_points[i], end_point])
                colors.append(score_to_color(score))
                point_lines += 1
        total_lines += point_lines
        if i < 5:  # 打印前5个点的线条数量
            non_zero_scores = view_scores_matrix[i][view_scores_matrix[i] > 0]
            print(f"Point {i}: {point_lines} lines (valid directions: {non_zero_scores})")


    # Preparing line set for visualization
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        # Visualize
        o3d.visualization.draw_geometries([pcd, line_set])
    else:
        print("No valid directions to visualize")
        o3d.visualization.draw_geometries([pcd])

def visualize_all_directions(unique_points, valid_normalized_view_score, approach_directions, fixed_length=0.05):
    """
    可视化每个点的所有3个方向，包括有效和无效的方向
    
    Parameters:
    - unique_points: Nx3 numpy array of unique points' coordinates.
    - valid_normalized_view_score: Nx3 array with scores for each of the 3 directions.
    - approach_directions: Nx3x3 array of approach direction vectors.
    - fixed_length: Fixed length for each vector.
    """
    # Initialize point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(unique_points)

    # Initialize lists for lines and colors
    lines = []
    colors = []

    # Function to map scores to colors
    def score_to_color(score):
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:  # score == 1
            return [0.0, 0.0, 1.0]  # Blue for valid grasps

    # Process each unique point and its 3 directions
    N = len(unique_points)
    total_lines = 0
    
    for i in range(N):
        point_lines = 0
        # 可视化所有3个方向
        for dir_idx in range(3):
            score = valid_normalized_view_score[i, dir_idx]
            direction = approach_directions[i, dir_idx]  # 使用实际的approach direction
            
            # 归一化方向向量
            direction_normalized = direction / np.linalg.norm(direction)
            end_point = unique_points[i] + direction_normalized * fixed_length
            
            lines.append([unique_points[i], end_point])
            colors.append(score_to_color(score))
            point_lines += 1
        
        total_lines += point_lines
        if i < 5:  # 打印前5个点的线条数量
            print(f"Point {i}: {point_lines} lines (all 3 directions: {valid_normalized_view_score[i]})")
    
    print(f"Total lines: {total_lines}, Total points: {N}, Expected: {N * 3}")

    # Preparing line set for visualization
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        # Visualize
        o3d.visualization.draw_geometries([pcd, line_set])
    else:
        print("No directions to visualize")
        o3d.visualization.draw_geometries([pcd])

def visualize_best_directions(unique_points, top_view_inds, template_views, view_scores, fixed_length=0.02, save_path=None):
    """
    可视化每个点分数最高的方向（最终grasp使用的方向）
    
    Parameters:
    - unique_points: Nx3 numpy array of unique points' coordinates.
    - top_view_inds: N array of indices indicating the best direction for each point.
    - template_views: 800x3 array of template view direction vectors.
    - view_scores: Nx800 matrix with scores for each view of each unique point.
    - fixed_length: Fixed length for each vector.
    - save_path: Path to save the visualization results.
    """
    # Initialize point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(unique_points)
    
    # Initialize lists for lines and colors
    lines = []
    colors = []
    
    # Process each unique point and its best direction
    N = len(unique_points)
    
    for i in range(N):
        # 获取该点分数最高的方向索引
        best_view_idx = top_view_inds[i]
        
        # 获取对应的方向向量
        direction = template_views[best_view_idx]
        
        # 获取该方向的分数
        best_score = view_scores[i, best_view_idx]
        
        # 归一化方向向量
        direction_normalized = direction / np.linalg.norm(direction)
        end_point = unique_points[i] + direction_normalized * fixed_length
        
        lines.append([unique_points[i], end_point])
        
        # 所有最佳方向都使用统一的蓝色
        colors.append([0.0, 0.0, 1.0])  # 蓝色
        
    # Preparing line set for visualization
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        # 保存可视化结果
        if save_path is not None:
            # 保存点云
            o3d.io.write_point_cloud(f"{save_path}_best_directions_points.ply", pcd)
            # 保存线条
            o3d.io.write_line_set(f"{save_path}_best_directions.ply", line_set)
        
        # Visualize
        o3d.visualization.draw_geometries([pcd, line_set])
    else:
        if save_path is not None:
            o3d.io.write_point_cloud(f"{save_path}_best_directions_points.ply", pcd)
        o3d.visualization.draw_geometries([pcd])

def visualize_training_directions(unique_points, view_scores_matrix, template_views, fixed_length=0.1, save_path=None):
    """
    可视化训练时的有效方向（NUM_VIEW个方向中的有效方向）
    
    Parameters:
    - unique_points: Nx3 numpy array of unique points' coordinates.
    - view_scores_matrix: NxNUM_VIEW matrix with scores for each of the NUM_VIEW directions.
    - template_views: 300x3 array of template view direction vectors.
    - fixed_length: Fixed length for each vector.
    - save_path: Path to save the visualization results.
    """
    # Initialize point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(unique_points)

    # Initialize lists for lines and colors
    lines = []
    colors = []

    # Function to map scores to colors
    def score_to_color(score):
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:  # score == 1
            return [0.0, 0.0, 1.0]  # Blue for valid grasps

    # Process each unique point and its NUM_VIEW directions
    N = len(unique_points)
    total_lines = 0
    
    for i in range(N):
        point_lines = 0
        # 可视化所有NUM_VIEW个方向中的有效方向
        for view_idx, score in enumerate(view_scores_matrix[i]):
            if score > 0:  # 只显示有效方向
                direction = template_views[view_idx]  # 使用NUM_VIEW个预定义方向
                
                # 归一化方向向量
                direction_normalized = direction / np.linalg.norm(direction)
                end_point = unique_points[i] + direction_normalized * fixed_length
                
                lines.append([unique_points[i], end_point])
                colors.append(score_to_color(score))
                point_lines += 1
        
        total_lines += point_lines
        if i < 5:  # 打印前5个点的线条数量
            valid_indices = np.where(view_scores_matrix[i] > 0)[0]
            print(f"Point {i}: {point_lines} lines (valid direction indices: {valid_indices[:10]}...)")  # 只显示前10个索引
    
    print(f"Total lines: {total_lines}, Total points: {N}")

    # Preparing line set for visualization
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        # 保存可视化结果
        if save_path is not None:
            # 保存点云
            o3d.io.write_point_cloud(f"{save_path}_points.ply", pcd)
            # 保存线条
            o3d.io.write_line_set(f"{save_path}_directions.ply", line_set)
            print(f"Visualization saved to {save_path}_points.ply and {save_path}_directions.ply")
        
        # Visualize
        o3d.visualization.draw_geometries([pcd, line_set])
    else:
        print("No valid directions to visualize")
        if save_path is not None:
            o3d.io.write_point_cloud(f"{save_path}_points.ply", pcd)
            print(f"Point cloud saved to {save_path}_points.ply")
        o3d.visualization.draw_geometries([pcd])

def fibonacci_sphere(samples=NUM_VIEW):
    points = []
    phi = np.pi * (3. - np.sqrt(5.))  # Golden angle in radians
    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)  # radius at y
        theta = phi * i  # golden angle increment
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        points.append([x, y, z])
    return np.array(points)

class Sim_Grasp_Net(nn.Module):

    def __init__(self, cylinder_radius=0.05, seed_feat_dim=256, is_training=True):
        super().__init__()
        self.is_training = is_training
        self.seed_feature_dim = seed_feat_dim
        self.M_points = M_POINT
        self.num_view = NUM_VIEW
        self.num_angle=12
        self.num_depth=7
        self.backbone  = Pointnet2_scorenet(input_chann=6, k_score=1)


        self.graspable = GraspAffordanceNet(seed_feature_dim=self.seed_feature_dim)
        self.rotation = ApproachVecNet(self.num_view, seed_feature_dim=self.seed_feature_dim, is_training=self.is_training)
        self.group = GroupNet(nsample=64, cylinder_radius=cylinder_radius, seed_feature_dim=self.seed_feature_dim)
        self.posead = PoseNet(num_angle=self.num_angle, num_depth=self.num_depth)

    def get_visualization_data(self, end_points, batch_idx=0):
        """
        提取可视化所需的数据
        
        Args:
            end_points: 网络输出
            batch_idx: batch索引
            
        Returns:
            包含可视化数据的字典
        """
        viz_data = {}

        if self.is_training:
            # 训练模式数据
            if 'coors' in end_points and 'pointwise_label' in end_points:
                coors_data = end_points['coors'][batch_idx]
                viz_data['dense_points'] = coors_data.detach().cpu().numpy() if hasattr(coors_data, 'detach') else coors_data
                
                label_data = end_points['pointwise_label'][batch_idx]
                viz_data['affordance_scores'] = label_data.detach().cpu().numpy() if hasattr(label_data, 'detach') else label_data
            
            # 训练模式下的唯一抓取点云数据
            if 'sparse_points' in end_points:
                sparse_data = end_points['sparse_points'][batch_idx]
                viz_data['sparse_points'] = sparse_data.detach().cpu().numpy() if hasattr(sparse_data, 'detach') else sparse_data
            
            # 训练模式下的原始3个方向数据
            if 'normalized_view_score' in end_points:
                view_data = end_points['normalized_view_score'][batch_idx]
                viz_data['view_scores_3dir'] = view_data.detach().cpu().numpy() if hasattr(view_data, 'detach') else view_data
            
            # 训练模式下的完整800个方向数据（映射后的）
            if 'batch_grasp_view_graspness' in end_points:
                grasp_data = end_points['batch_grasp_view_graspness'][batch_idx]
                viz_data['view_scores_800dir'] = grasp_data.detach().cpu().numpy() if hasattr(grasp_data, 'detach') else grasp_data
            
            # 实际的方向向量数据
            if 'rotated_approach_directions' in end_points:
                dir_data = end_points['rotated_approach_directions'][batch_idx]
                viz_data['approach_directions'] = dir_data.detach().cpu().numpy() if hasattr(dir_data, 'detach') else dir_data
            
            # 方向映射索引数据（真实方向→模板方向的映射）
            if 'view_inds_mapping' in end_points and len(end_points['view_inds_mapping']) > batch_idx:
                mapping_data = end_points['view_inds_mapping'][batch_idx]
                viz_data['view_inds_mapping'] = mapping_data.detach().cpu().numpy() if hasattr(mapping_data, 'detach') else mapping_data
            
            # ==================== 网络预测结果 ====================
            # 网络预测的抓取可行性分数
            if 'graspness_score' in end_points:
                graspness_data = end_points['graspness_score'][batch_idx]
                graspness_np = graspness_data.detach().cpu().numpy() if hasattr(graspness_data, 'detach') else graspness_data
                # graspness_score的形状是(B, N, 1)，需要squeeze掉最后一维
                if graspness_np.ndim > 1:
                    graspness_np = graspness_np.squeeze()
                viz_data['pred_affordance_scores'] = graspness_np
            
            # 网络选取的点（FPS采样后的点）
            if 'xyz_graspable' in end_points:
                xyz_data = end_points['xyz_graspable'][batch_idx]
                viz_data['pred_sparse_points'] = xyz_data.detach().cpu().numpy() if hasattr(xyz_data, 'detach') else xyz_data
            
            # 网络预测的方向分数
            if 'view_score' in end_points:
                view_score_data = end_points['view_score'][batch_idx]
                viz_data['pred_view_scores'] = view_score_data.detach().cpu().numpy() if hasattr(view_score_data, 'detach') else view_score_data
            
            # 生成template_views
            from .loss_utils import generate_grasp_views
            viz_data['template_views'] = generate_grasp_views(NUM_VIEW).cpu().numpy()
        else:
            # 测试模式数据
            if 'xyz_graspable' in end_points:
                xyz_data = end_points['xyz_graspable'][batch_idx]
                viz_data['sparse_points'] = xyz_data.detach().cpu().numpy() if hasattr(xyz_data, 'detach') else xyz_data
            
            if 'grasp_top_view_inds' in end_points:
                inds_data = end_points['grasp_top_view_inds'][batch_idx]
                viz_data['top_view_inds'] = inds_data.detach().cpu().numpy() if hasattr(inds_data, 'detach') else inds_data
            
            if 'view_score' in end_points:
                score_data = end_points['view_score'][batch_idx]
                viz_data['view_scores'] = score_data.detach().cpu().numpy() if hasattr(score_data, 'detach') else score_data
                
                # 生成template_views
                from .loss_utils import generate_grasp_views
                viz_data['template_views'] = generate_grasp_views(NUM_VIEW).cpu().numpy()
        
        return viz_data

    def forward(self, end_points, which_side = "EVA"):

        if which_side == "EVA":
            self.M_points = 4096
        elif which_side == "PVC":
            self.M_points = 8192
        elif which_side == "CB":
            self.M_points = 4096 * 2
        
        # ==================== 1. 点云特征提取 ====================
        # 获取输入点云数据
        pointcloud = end_points['point_clouds']  # (B, N, 6) - 6个特征(xyz + normal) - (8, 40000, 6)
        B, _, _ = pointcloud.shape
        
        # 调整维度顺序：从(B, N, 6)变为(B, 6, N) - 适配PointNet++输入格式
        pointcloud = pointcloud.permute(0, 2, 1)  # (B, 6, N) -(8, 6, 40000)
        
        # 通过PointNet++骨干网络提取特征
        # seed_features: 种子点特征 (B, feat_dim, N) - (8, 256, 40000)
        # seed_xyz: 种子点坐标 (B, 3, N) - (8, 3, 40000)
        # end_points: 新增以下键值对：
        #   - 'input_xyz': (B, 3, N) - 原始XYZ坐标 - (8, 3, 40000)
        #   - 'input_features': (B, 3, N) - 原始法向量特征 - (8, 3, 40000)
        #   - 'fp3_features': (B, 256, N) - 最终提取的特征 - (8, 256, 40000)
        #   - 'fp3_xyz': (B, 3, N) - 最终坐标（与input_xyz相同） - (8, 3, 40000)
        seed_features, seed_xyz, end_points = self.backbone(pointcloud, end_points)

        # ==================== 2. 抓取可行性预测 ====================
        # 通过GraspAffordanceNet预测每个点的抓取可行性分数
        # end_points新增键值对['graspness_score']: (B, N, 1) - 每个点的抓取可行性分数 - (8, 40000, 1)
        # GraspAffordanceNet最后一层没有使用激活函数，输出范围是(-inf, inf)
        end_points = self.graspable(seed_features, end_points)
        
        # 调整特征维度：从(B, feat_dim, N)变为(B, N, feat_dim)
        seed_features_flipped = seed_features.transpose(1, 2)  # (B, N, feat_dim) - (8, 40000, 256)

        # 获取抓取可行性分数并压缩最后一维
        graspness_score = end_points['graspness_score'].squeeze(2)  # (B, N) - (8, 40000)

        # ==================== 3. 抓取候选点筛选 ====================
        # 创建抓取可行性掩码：分数大于阈值的点被认为是可抓取的
        graspable_mask = graspness_score > GRASPNESS_THRESHOLD  # (B, N) - 布尔掩码 - (8, 40000)

        # 初始化存储可抓取点特征和坐标的列表
        seed_features_graspable = []
        seed_xyz_graspable = []

        graspable_num_batch = 0.  # 统计总的可抓取点数
        
        # 调整种子点坐标维度：从(B, 3, N)变为(B, N, 3)
        seed_xyz=seed_xyz.permute(0, 2, 1)  # (B, N, 3) - (8, 40000, 3)

        # ==================== 4. 逐batch处理可抓取点 ====================
        for i in range(B):
            # 获取当前batch的抓取可行性掩码
            cur_mask = graspable_mask[i]  # (N,) - 当前batch的布尔掩码 - (40000,)
            graspable_num_batch += cur_mask.sum()  # 累加可抓取点数，只计算True的点数 N_graspable
            
            # 根据掩码筛选可抓取点的特征和坐标
            cur_feat = seed_features_flipped[i][cur_mask]  # (N_graspable, feat_dim) - (N_graspable, 256)
            cur_seed_xyz = seed_xyz[i][cur_mask]  # (N_graspable, 3)
            
            # 如果没有可抓取点，随机选择一个点作为 fallback
            if cur_seed_xyz.shape[0] == 0:
                print(f"Warning: No graspable points in batch {i}, selecting random point")
                random_idx = torch.randint(0, seed_xyz[i].shape[0], (1,))
                cur_seed_xyz = seed_xyz[i][random_idx]  # (1, 3)
                cur_feat = seed_features_flipped[i][random_idx]  # (1, feat_dim)
                print(f"cur_seed_xyz: {cur_seed_xyz}")
                print(f"cur_seed_xyz.shape: {cur_seed_xyz.shape}")
                print(f"cur_feat: {cur_feat}")
                print(f"cur_feat.shape: {cur_feat.shape}")
            
            
            # ==================== 5. 最远点采样(FPS) ====================
            # 将坐标调整为FPS所需格式
            cur_seed_xyz = cur_seed_xyz.unsqueeze(0)  # (1, N_graspable, 3)

            fps_idxs = furthest_point_sample(cur_seed_xyz, self.M_points)
            
            # 调整坐标格式用于gather操作
            cur_seed_xyz_flipped = cur_seed_xyz.transpose(1, 2).contiguous()  # (1, 3, N_graspable)
            # 根据FPS索引收集坐标
            cur_seed_xyz = gather_operation(cur_seed_xyz_flipped, fps_idxs).transpose(1, 2).squeeze(0).contiguous()  # (M_POINT, 3) - (2048, 3)

            # 调整特征格式用于gather操作
            cur_feat_flipped = cur_feat.unsqueeze(0).transpose(1, 2).contiguous()  # (1, feat_dim, N_graspable) - (1, 256, N_graspable)
            # 根据FPS索引收集特征
            cur_feat = gather_operation(cur_feat_flipped, fps_idxs).squeeze(0).contiguous()  # (feat_dim, M_POINT) - (256, 2048)
            # 将处理后的特征和坐标添加到列表
            seed_features_graspable.append(cur_feat)
            seed_xyz_graspable.append(cur_seed_xyz)

        # ==================== 6. 堆叠batch数据 ====================
        # 将所有batch的数据堆叠成张量
        seed_xyz_graspable = torch.stack(seed_xyz_graspable, 0)  # (B, M_POINT, 3) - (8, 2048, 3)
        seed_features_graspable = torch.stack(seed_features_graspable)  # (B, feat_dim, M_POINT) - (8, 256, 2048)
        
        # 将结果存储到end_points中
        # end_points新增键值对['xyz_graspable']: (B, M_POINT, 3) - (8, 2048, 3)
        # end_points新增键值对['graspable_count_stage1']: 平均每个batch的可抓取点数
        end_points['xyz_graspable'] = seed_xyz_graspable
        end_points['graspable_count_stage1'] = graspable_num_batch / B  # 平均每个batch的可抓取点数
        # ==================== 7. 接近方向预测 ====================
        # 通过ApproachVecNet预测接近方向
        # 输入: seed_features_graspable (B, feat_dim, M_POINT) - (8, 256, 2048)
        # 输出: 
        #   - end_points: 更新后的数据字典，新增以下键值对：
        #     * 'view_score': (B, M_POINT, 800) - 每个点对800个模板方向的分数 - (8, 2048, 800)
        #     * 'grasp_top_view_inds': (B, M_POINT) - 每个点的最佳方向索引 - (8, 2048)
        #     * 'grasp_top_view_xyz': (B, M_POINT, 3) - 最佳方向向量（仅测试模式）
        #     * 'grasp_top_view_rot': (B, M_POINT, 3, 3) - 对应旋转矩阵（仅测试模式）
        #   - res_feat: (B, feat_dim, M_POINT) - 残差特征，用于后续网络 - (8, 256, 2048)
        end_points, res_feat = self.rotation(seed_features_graspable, end_points)

        # 特征融合：原始特征 + 残差特征
        seed_features_graspable = seed_features_graspable + res_feat  # (B, feat_dim, M_POINT) - (8, 256, 2048)

        # ==================== 8. 训练模式：方向映射和标签生成 ====================
        if self.is_training:
            
            # 获取训练数据
            dense_points = end_points['coors']  # (B, 40000, 3) - 训练点云坐标 - (8, 40000, 3)
            batch_size, dense_N, _ = dense_points.size() # batch_size=8, dense_N=40000

            # 获取唯一抓取点数据
            unique_t_ori_points = end_points['sparse_points']  # (B, N, 3) - 唯一抓取点坐标 - (8, N, 3)
            normalized_scores = end_points['normalized_scores']  # (B, N) - 归一化抓取分数 - (8, N)
            approach_directions = end_points['rotated_approach_directions']  # (B, N, 3, 3) - 旋转后的接近方向 - (8, N, 3, 3)
            normalized_view_score= end_points['normalized_view_score']  # (B, N, 3) - 二值化的视图分数 - (8, N, 3)
            normalized_grasp_score=end_points['normalized_grasp_score']  # (B, N, 3, 12, 7) - 归一化抓取分数 - (8, N, 3, 12, 7)

            # 生成800个模板方向
            template_views = generate_grasp_views(NUM_VIEW).to(seed_xyz_graspable.device)  # (800, 3)
            
            # 初始化存储列表
            end_points_view_score_list = []      # 存储每个batch的视图分数
            batch_grasp_point_list = []          # 存储每个batch的抓取点
            end_points_grasp_score_list = []     # 存储每个batch的抓取分数
            end_points_view_rot_list = []        # 存储每个batch的视图旋转矩阵
            pointwise_label_list=[]              # 存储每个batch的点级标签

            # 初始化存储张量
            # 存储视图分数，每个种子点对800个模板方向的分数标签 - (B, M_POINT, NUM_VIEW) - (8, 2048, 800)
            if which_side == "EVA":
                end_points_view_score = torch.zeros(batch_size, 4096, NUM_VIEW, device=seed_xyz_graspable.device)
                # 存储抓取点坐标，每个种子点的抓取点坐标 - (B, M_POINT, 3) - (8, 2048, 3)
                batch_grasp_point_tensor = torch.zeros(batch_size, 4096, 3, device=seed_xyz_graspable.device)
                # 存储抓取分数，每个种子点对800个模板方向的抓取分数标签 - (B, M_POINT, NUM_VIEW, 12, 7) - (8, 2048, 800, 12, 7)
                end_points_grasp_score = torch.zeros(batch_size, 4096, NUM_VIEW, 12, 7, device=seed_xyz_graspable.device)
                # 存储视图旋转矩阵，每个种子点对800个模板方向的旋转矩阵 - (B, M_POINT, NUM_VIEW, 3, 3) - (8, 2048, 800, 3, 3)
                end_points_view_rot = torch.zeros(batch_size, 4096, NUM_VIEW, 3, 3, device=seed_xyz_graspable.device)
            elif which_side == "PVC":
                end_points_view_score = torch.zeros(batch_size, 8192 * 2, NUM_VIEW, device=seed_xyz_graspable.device)
                # 存储抓取点坐标，每个种子点的抓取点坐标 - (B, M_POINT, 3) - (8, 2048, 3)
                batch_grasp_point_tensor = torch.zeros(batch_size, 8192 * 2, 3, device=seed_xyz_graspable.device)
                # 存储抓取分数，每个种子点对800个模板方向的抓取分数标签 - (B, M_POINT, NUM_VIEW, 12, 7) - (8, 2048, 800, 12, 7)
                end_points_grasp_score = torch.zeros(batch_size, 8192 * 2, NUM_VIEW, 12, 7, device=seed_xyz_graspable.device)
                # 存储视图旋转矩阵，每个种子点对800个模板方向的旋转矩阵 - (B, M_POINT, NUM_VIEW, 3, 3) - (8, 2048, 800, 3, 3)
                end_points_view_rot = torch.zeros(batch_size, 8192 * 2, NUM_VIEW, 3, 3, device=seed_xyz_graspable.device)
            # 存储点级标签，训练点云中每个点的抓取分数标签 - (B, 40000) - (8, 40000)
            end_points_affordance=torch.zeros(batch_size, dense_N, device=seed_xyz_graspable.device)

            # ==================== 9. 逐batch处理训练数据 ====================
            for i in range(batch_size):

                # 筛选有效的唯一抓取点（非零坐标的点） 为什么会有零坐标的点，存疑，可能是数据增强导致的
                valid_mask = torch.any(unique_t_ori_points[i] != 0, dim=1)  # (N,) - 布尔掩码
                
                # 根据掩码获取有效数据
                valid_sparse_points = unique_t_ori_points[i][valid_mask]  # (N_valid, 3) - 有效唯一抓取点坐标
                valid_normalized_scores = normalized_scores[i][valid_mask]  # (N_valid,) - 有效归一化分数
                valid_approach_directions = approach_directions[i][valid_mask]  # (N_valid, 3, 3) - 有效接近方向
                valid_normalized_view_score = normalized_view_score[i][valid_mask]  # (N_valid, 3) - 有效视图分数
                valid_normalized_grasp_score= normalized_grasp_score[i][valid_mask]  # (N_valid, 3, 12, 7) - 有效抓取分数
   
                num_valid_points, _ = valid_sparse_points.size()  # N_valid

                # ==================== 10. 原始训练点云标签生成 ====================
                # 计算原始训练点云与有效唯一抓取点的距离
                distances = torch.cdist(dense_points[i], valid_sparse_points)  # (40000, N_valid)

                # 找到原始训练点云每个点与有效唯一抓取点的k个最近邻点
                _, indices = torch.topk(distances, k=2, largest=False, dim=1)  # (40000, 2)

                # 基于最近邻有效唯一抓取点的分数为原始训练点云分配标签
                # 原始训练点云中的一个点，在有效唯一抓取点中找到2个最近邻点，然后取这2个最近邻点的分数的平均值作为原始训练点云的分数
                affordance_score = torch.mean(valid_normalized_scores[indices], dim=1)  # (40000,)
                
                # ==================== 11. KNN方向映射 ====================
                # 将接近方向重塑为2D格式用于KNN计算
                approach_directions_reshaped = valid_approach_directions.view(-1, 3)  # (N_valid*3, 3)

                # 准备KNN计算的输入格式
                grasp_views_ = template_views.transpose(0, 1).contiguous().unsqueeze(0)  # (1, 3, 800)
                grasp_views_trans_ = approach_directions_reshaped.transpose(0, 1).contiguous().unsqueeze(0)  # (1, 3, N_valid*3)

                # 执行KNN搜索：为每个真实方向找到最接近的模板方向
                view_inds = knn(grasp_views_, grasp_views_trans_, k=1).squeeze() - 1  # (N_valid*3,) - 模板方向索引

                # 重塑KNN结果为(N_valid, 3)格式
                view_inds_reshaped = view_inds.view(-1, 3)  # (N_valid, 3)
                
                # 存储view_inds到end_points中用于可视化
                if 'view_inds_mapping' not in end_points:
                    end_points['view_inds_mapping'] = []
                end_points['view_inds_mapping'].append(view_inds_reshaped)  # 存储GPU张量，与其他数据保持一致

                # ==================== 12. 视图分数映射 ====================
                # 创建行索引用于scatter操作
                row_indices = torch.arange(num_valid_points).unsqueeze(1).expand(-1, 3)  # (N_valid, 3)
                
                # 初始化视图分数张量
                view_scores = torch.zeros(num_valid_points, NUM_VIEW).to(view_inds_reshaped.device)  # (N_valid, 800)
                print(f"Initial view_scores shape: {view_scores.shape}")

                # 将真实方向的分数映射到对应的模板方向
                view_scores[row_indices, view_inds_reshaped] = valid_normalized_view_score  # (N_valid, 800)
                print(f"Non-zero view scores: {(view_scores > 0).sum()}")

                # ==================== 13. 抓取分数映射 ====================
                # 初始化抓取分数张量
                marked_tensor = torch.zeros(num_valid_points, NUM_VIEW, 12, 7).to(view_inds_reshaped.device)  # (N_valid, 800, 12, 7)
                print(f"Initial marked_tensor shape: {marked_tensor.shape}")
                
                # 将真实方向的抓取分数映射到对应的模板方向
                marked_tensor[row_indices, view_inds_reshaped] = valid_normalized_grasp_score  # (N_valid, 800, 12, 7)
                final_grasp_scores = marked_tensor
                print(f"Non-zero grasp scores: {(final_grasp_scores > 0).sum()}")
                
                # ==================== 14. 种子点与稀疏点匹配 ====================
                # 计算种子点与稀疏点的距离
                distances = torch.cdist(seed_xyz_graspable[i], valid_sparse_points)  # (M_POINT, N_valid)
                print(f"Seed-sparse distance matrix shape: {distances.shape}")
                
                # 找到每个种子点的最近邻稀疏点
                nn_inds = torch.argmin(distances, dim=1)  # (M_POINT,) - 最近邻索引
                print(f"Nearest neighbor indices shape: {nn_inds.shape}")

                # ==================== 15. 旋转矩阵生成 ====================
                # 为模板方向生成旋转矩阵
                angles = torch.zeros(template_views.size(0), dtype=template_views.dtype, device=template_views.device)  # (800,)
                print(f"Angles shape: {angles.shape}")

                # 将模板方向转换为旋转矩阵
                grasp_views_rot = batch_viewpoint_params_to_matrix_data(template_views, angles)  # (800, 3, 3)
                print(f"Template rotation matrices shape: {grasp_views_rot.shape}")
                
                # 应用数据增强矩阵
                grasp_views_rot_trans = torch.matmul(end_points["augument_matrix"][i], grasp_views_rot)  # (800, 3, 3)
                print(f"Augmented rotation matrices shape: {grasp_views_rot_trans.shape}")

                # 扩展旋转矩阵到所有种子点
                grasp_views_rot_trans = grasp_views_rot_trans.unsqueeze(0).expand(self.M_points, -1, -1, -1)  # (M_POINT, 800, 3, 3)
                print(f"Expanded rotation matrices shape: {grasp_views_rot_trans.shape}")

                # ==================== 16. 数据分配 ====================
                # 将稀疏点的数据分配给对应的种子点
                end_points_view_score[i] = view_scores[nn_inds]  # (M_POINT, 800) - 视图分数
                batch_grasp_point_tensor[i] = valid_sparse_points[nn_inds]  # (M_POINT, 3) - 抓取点坐标
                end_points_grasp_score[i] = final_grasp_scores[nn_inds]  # (M_POINT, 800, 12, 7) - 抓取分数
                end_points_view_rot[i] = grasp_views_rot_trans  # (M_POINT, 800, 3, 3) - 旋转矩阵
                end_points_affordance[i] = affordance_score  # (20000,) - 密集点标签
                print(f"Assigned data shapes:")
                print(f"  end_points_view_score[{i}]: {end_points_view_score[i].shape}")
                print(f"  batch_grasp_point_tensor[{i}]: {batch_grasp_point_tensor[i].shape}")
                print(f"  end_points_grasp_score[{i}]: {end_points_grasp_score[i].shape}")
                print(f"  end_points_view_rot[{i}]: {end_points_view_rot[i].shape}")
                print(f"  end_points_affordance[{i}]: {end_points_affordance[i].shape}")

                # 将当前batch的数据添加到列表中
                end_points_view_score_list.append(end_points_view_score[i])
                batch_grasp_point_list.append(batch_grasp_point_tensor[i])
                end_points_grasp_score_list.append(end_points_grasp_score[i])
                end_points_view_rot_list.append(end_points_view_rot[i])
                pointwise_label_list.append(end_points_affordance[i])
                # input("Press Enter to continue...")
                
            # ==================== 17. 堆叠batch数据 ====================
            print("\n=== Stacking Batch Data ===")
            
            # 将所有batch的数据堆叠成张量
            batch_grasp_view_graspness = torch.stack(end_points_view_score_list)  # (B, M_POINT, 800)
            batch_grasp_point = torch.stack(batch_grasp_point_list)  # (B, M_POINT, 3)
            batch_grasp_score = torch.stack(end_points_grasp_score_list)  # (B, M_POINT, 800, 12, 7)
            batch_view_rot = torch.stack(end_points_view_rot_list)  # (B, M_POINT, 800, 3, 3)
            pointwise_label = torch.stack(pointwise_label_list)  # (B, 20000)
            print(f"Stacked batch data shapes:")
            print(f"  batch_grasp_view_graspness: {batch_grasp_view_graspness.shape}")
            print(f"  batch_grasp_point: {batch_grasp_point.shape}")
            print(f"  batch_grasp_score: {batch_grasp_score.shape}")
            print(f"  batch_view_rot: {batch_view_rot.shape}")
            print(f"  pointwise_label: {pointwise_label.shape}")
            
            # 将堆叠的数据存储到end_points字典中
            end_points['batch_grasp_view_graspness'] = batch_grasp_view_graspness
            end_points['batch_grasp_point'] = batch_grasp_point
            end_points['batch_grasp_score'] = batch_grasp_score
            end_points['batch_grasp_view_rot'] = batch_view_rot
            end_points['pointwise_label'] = pointwise_label

            # ==================== 18. 最佳方向选择 ====================
            print("\n=== Best Direction Selection ===")
            
            # 获取最佳方向索引（由ApproachVecNet预测）
            top_view_inds = end_points['grasp_top_view_inds']  # (B, M_POINT) - 每个点的最佳方向索引
            template_views_rot = end_points['batch_grasp_view_rot']  # (B, M_POINT, 800, 3, 3) - 所有方向的旋转矩阵
            print(f"Top view indices shape: {top_view_inds.shape}")
            print(f"Template views rotation shape: {template_views_rot.shape}")
            
            # 复制抓取分数用于处理
            grasp_scores = copy.deepcopy(end_points['batch_grasp_score'])  # (B, M_POINT, 800, 12, 7)
            print(f"Grasp scores shape: {grasp_scores.shape}")
            
            # 获取张量维度
            B, Ns, V, A, D = grasp_scores.size()
            print(f"Grasp scores dimensions: B={B}, Ns={Ns}, V={V}, A={A}, D={D}")
            
            # 准备索引用于gather操作
            top_view_inds_ = top_view_inds.view(B, Ns, 1, 1, 1).expand(-1, -1, -1, 3, 3)  # (B, M_POINT, 1, 3, 3)
            print(f"Expanded top_view_inds for rotation: {top_view_inds_.shape}")
            
            # 根据最佳方向索引选择对应的旋转矩阵
            top_template_views_rot = torch.gather(template_views_rot, 2, top_view_inds_).squeeze(2)  # (B, M_POINT, 3, 3)
            print(f"Selected top template views rotation: {top_template_views_rot.shape}")
            
            # 准备索引用于抓取分数选择
            top_view_inds_ = top_view_inds.view(B, Ns, 1, 1, 1).expand(-1, -1, -1, A, D)  # (B, M_POINT, 1, 12, 7)
            print(f"Expanded top_view_inds for scores: {top_view_inds_.shape}")
            
            # 根据最佳方向索引选择对应的抓取分数
            top_view_grasp_scores = torch.gather(grasp_scores, 2, top_view_inds_).squeeze(2)  # (B, M_POINT, 12, 7)
            print(f"Selected top view grasp scores: {top_view_grasp_scores.shape}")

            # 更新end_points中的抓取分数为最佳方向的分数
            end_points['batch_grasp_score'] = top_view_grasp_scores  # (B, M_POINT, 12, 7)
            print(f"Updated batch_grasp_score shape: {end_points['batch_grasp_score'].shape}")
                 
            # 设置最终使用的旋转矩阵
            grasp_top_views_rot = top_template_views_rot  # (B, M_POINT, 3, 3)
            print(f"Final grasp_top_views_rot shape: {grasp_top_views_rot.shape}")
        else:
            # ==================== 19. 测试模式 ====================
            # 测试模式下直接使用预计算的旋转矩阵
            grasp_top_views_rot = end_points['grasp_top_view_rot']  # (B, M_POINT, 3, 3)

        # ==================== 20. 局部特征聚合 ====================
        # 通过GroupNet进行局部特征聚合
        # 输入：种子点坐标、种子点特征、最佳方向的旋转矩阵
        # 输出：聚合后的局部特征
        group_features = self.group(seed_xyz_graspable.contiguous(), seed_features_graspable.contiguous(), grasp_top_views_rot)

        # ==================== 21. 抓取姿态预测 ====================
        # 通过PoseNet预测最终的抓取姿态
        # 输入：聚合后的局部特征
        # 输出：抓取角度、深度等参数
        end_points = self.posead(group_features, end_points)

        return end_points
    


def visualize_with_score_mask(segmented_point_cloud, dense_scores, rgb_colors=None):
    # Normalize the scores for colormap
    normalized_scores = (dense_scores - np.min(dense_scores)) / (np.max(dense_scores) - np.min(dense_scores))
    
    # Get color map from Matplotlib
    cmap = matplotlib.cm.get_cmap('plasma')
    score_colors = cmap(normalized_scores)[:, :3]  # RGB colors based on scores
    
    # Set alpha for blending
    alpha = 0.7
    
    # If rgb_colors is not provided, use uniform gray for all points
    if rgb_colors is None:
        rgb_colors = np.full_like(segmented_point_cloud, fill_value=127)  # Gray
    
    # Blend the score colors with the rgb_colors
    blended_colors = alpha * score_colors + (1 - alpha) * rgb_colors / 255.0
    
    # Create a point cloud object for the dense points
    dense_pcd = o3d.geometry.PointCloud()
    dense_pcd.points = o3d.utility.Vector3dVector(segmented_point_cloud)
    dense_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Uniform gray color
    
    # Create another point cloud object for the blended colors (score mask)
    score_mask_pcd = o3d.geometry.PointCloud()
    score_mask_pcd.points = o3d.utility.Vector3dVector(segmented_point_cloud)
    score_mask_pcd.colors = o3d.utility.Vector3dVector(blended_colors)
    
    # Display the dense points first, then overlay the score mask
    #o3d.visualization.draw_geometries([dense_pcd], window_name="Dense Point Cloud")
    o3d.visualization.draw_geometries([score_mask_pcd], window_name="Score Mask Overlay")

    return dense_pcd, score_mask_pcd


def normalize_grasp_score(grasp_score):
    min_val = torch.min(grasp_score)
    max_val = torch.max(grasp_score)
    grasp_score_normalized = (grasp_score - min_val) / (max_val - min_val)
    grasp_score_normalized = grasp_score_normalized.view(-1, 1)
    return grasp_score_normalized


def combine_scores(grasp_score, closest_scores):
    # Geometric mean combination

        combined_scores = torch.sigmoid(closest_scores) * torch.log1p(grasp_score)

        return combined_scores


def pred_decode(end_points):
    """
    解码网络输出，生成抓取姿态预测
    
    Args:
        end_points: 网络输出字典，包含各种预测结果
        
    Returns:
        grasp_preds: 抓取姿态预测列表，每个元素为 [batch_size, M_POINT, 17] 的张量
    """
    # 定义抓取参数离散化数量
    NUM_ANGLE = 12  # 角度离散化数量 (0-π，12个角度)
    NUM_DEPTH = 7   # 深度离散化数量 (1-7，7个深度级别)
    
    # 获取批次大小
    batch_size = len(end_points["point_clouds"])
    
    grasp_preds = []
    
    # 遍历每个批次
    for b in range(batch_size):
        
        # 提取抓取中心点坐标 (1, M_POINT, 3) - (1, 2048, 3)
        grasp_center = end_points['xyz_graspable'][b].float()
        
        # 提取抓取分数预测 (1, M_POINT, 12, 7) - (1, 2048, 12, 7)
        grasp_score = end_points['grasp_score_pred'][b].float()
        
        # 重塑分数张量，分离角度和深度维度 (M_POINT, NUM_ANGLE * NUM_DEPTH) - (2048, 84)
        grasp_score = grasp_score.view(M_POINT, NUM_ANGLE * NUM_DEPTH)
        
        # 选择每个点的最佳抓取配置 (M_POINT,) - (2048,)
        grasp_score, grasp_score_inds = torch.max(grasp_score, -1)  # 对每个点找到配置中分数最高的索引 (0 - NUM_ANGLE*NUM_DEPTH-1)
        grasp_score = grasp_score.view(-1, 1)  # (M_POINT, 1) - (2048, 1)
        
        # 归一化抓取分数 (M_POINT, 1) - (2048, 1)
        grasp_score = normalize_grasp_score(grasp_score)
        
        # 解析角度和深度索引 (M_POINT,) - (2048,)
        angle_index = grasp_score_inds // NUM_DEPTH  # 角度索引 (0 - NUM_ANGLE-1)
        depth_index = grasp_score_inds % NUM_DEPTH   # 深度索引 (0 - NUM_DEPTH-1)
        
        # 恢复实际角度和深度值
        grasp_angle = (angle_index.float()) * np.pi / NUM_ANGLE
        # 将深度减半，让抓取更浅（原来是1-7cm，现在是0.5-3.5cm）
        grasp_depth = (depth_index.float() + 1) * 0.5
        grasp_depth = grasp_depth.view(-1, 1)
        
        # 提取最佳接近方向向量 (M_POINT, 3) - (2048, 3)
        approaching = end_points['grasp_top_view_xyz'][b].float()
                
        # 将方向向量和角度转换为抓取旋转矩阵 (M_POINT, 3, 3) - (2048, 3, 3)
        grasp_rot = batch_viewpoint_params_to_matrix_data(approaching, grasp_angle)
        
        # 获取原始点云坐标 (N, 3)
        dense_coordinates = end_points['point_clouds'][0, :, 0:3]

        # 计算抓取中心点到原始点云的距离 (M_POINT, N) - (2048, N)
        distances = torch.cdist(grasp_center, dense_coordinates)
        
        # 找到最接近的原始点云点 (M_POINT,) - (2048,)
        closest_indices = torch.argmin(distances, dim=1)
        
        # 转换为NumPy数组用于后续处理
        grasp_center_np = grasp_center.cpu().numpy().astype(np.float64) # (M_POINT, 3) - (2048, 3)
        point_cloud_np = end_points['point_clouds'].cpu().numpy().astype(np.float64) 
        point_cloud_np = point_cloud_np.squeeze(0) # (N, 3)
        
        # 创建Open3D点云对象 (用于可视化)
        point_cloud_pcd = o3d.geometry.PointCloud()
        point_cloud_pcd.points = o3d.utility.Vector3dVector(point_cloud_np[:, :3])
        
        # 生成抓取视图模板 (800, 3)
        template_views = generate_grasp_views(NUM_VIEW).to(grasp_center.device) # (800, 3)
        selected_views = template_views[end_points['grasp_top_view_inds'][b]] # (M_POINT, 3) - (2048, 3)
        
        # 计算抓取位置（考虑深度偏移） (M_POINT, 3) - (2048, 3)
        grasp_locations = grasp_center - selected_views * grasp_depth * 0.01
        
        # 加载夹爪模型（用于可视化）
        hand_mesh = o3d.io.read_triangle_mesh("Props/gripper.ply")
        
        # 创建夹爪姿态列表（用于可视化）
        gripper_poses = []
        for rotation, location in zip(grasp_rot.cpu().numpy(), grasp_locations.cpu().numpy()):
            transformation = np.eye(4)
            transformation[:3, :3] = rotation
            transformation[:3, 3] = location
            transformed_mesh = copy.deepcopy(hand_mesh).transform(transformation)
            gripper_poses.append(transformed_mesh)
        
        # 提取graspness分数
        graspness_score = end_points['graspness_score'][0].cpu().numpy().astype(np.float64) # (N, 1)
        closest_scores = end_points['graspness_score'][0][closest_indices] # (M_POINT, 1) - (2048, 1)

        # 归一化最近点分数
        closest_scores = normalize_grasp_score(closest_scores) # (M_POINT, 1) - (2048, 1)
        
        # 融合两种分数
        combine_score = combine_scores(grasp_score, closest_scores) # (M_POINT, 1) - (2048, 1)
        
        # 归一化融合分数
        combine_score = normalize_grasp_score(combine_score) # (M_POINT, 1) - (2048, 1)
        
        # 重塑旋转矩阵 (M_POINT, 9) - (2048, 9)
        grasp_rot = grasp_rot.view(M_POINT, 9)
        
        # 初始化物体ID (M_POINT, 1) - (2048, 1)
        obj_ids = -1 * torch.ones_like(grasp_score)
        
        # 获取归一化参数
        m = end_points['m']          # 归一化缩放因子
        centroid = end_points['centroid']  # 归一化中心点
        device = end_points['graspness_score'].device
        
        # 坐标反归一化
        grasp_center = torch.tensor((grasp_center_np * m) + centroid, dtype=torch.float32).to(device)
        
        # 设置夹爪尺寸参数
        grasp_width = 0.02 * torch.ones_like(grasp_score)   # 2cm宽度
        grasp_height = 0.02 * torch.ones_like(grasp_score)  # 2cm高度
        
        # 构建最终抓取姿态
        final_grasp = torch.cat([
            combine_score,      # 融合分数 [2048, 1]
            grasp_width,        # 夹爪宽度 [2048, 1] 
            grasp_height,       # 夹爪高度 [2048, 1]
            grasp_depth,        # 抓取深度 [2048, 1]
            grasp_rot,          # 旋转矩阵 [2048, 9]
            grasp_center,       # 抓取中心 [2048, 3]
            obj_ids             # 物体ID [2048, 1]
        ], axis=-1)
        
        grasp_preds.append(final_grasp)
    
    return grasp_preds


def pred_decode_topk(end_points, top_k=10):
    """
    完全保持原始功能的优化版本（修复GPU->CPU转换问题）
    """
    NUM_ANGLE = 12
    NUM_DEPTH = 7
    NUM_VIEW = 800
    
    batch_size = len(end_points['point_clouds'])
    device = end_points['graspness_score'].device
    
    grasp_preds = []
    template_views = generate_grasp_views(NUM_VIEW).to(device)
    
    for b in range(batch_size):
        # 获取数据
        grasp_center = end_points['xyz_graspable'][b].float()
        grasp_score = end_points['grasp_score_pred'][b].float()
        
        M_POINT = grasp_center.shape[0]
        total_configs = NUM_ANGLE * NUM_DEPTH
        
        # 1. Top-k选择
        grasp_score_flat = grasp_score.view(M_POINT, total_configs)
        grasp_score_topk, grasp_score_inds = torch.topk(
            grasp_score_flat, k=top_k, dim=-1, largest=True, sorted=True
        )
        
        # 2. 解析索引
        angle_indices = (grasp_score_inds // NUM_DEPTH).reshape(-1)
        depth_indices = (grasp_score_inds % NUM_DEPTH).reshape(-1)
        
        # 3. 计算角度和深度
        grasp_angles = angle_indices.float() * (np.pi / NUM_ANGLE)
        grasp_depths = (depth_indices.float() + 1) * 0.5
        
        # 4. 获取中心点
        point_indices = torch.arange(M_POINT, device=device)
        point_indices = point_indices.repeat_interleave(top_k)
        grasp_center_selected = grasp_center[point_indices]
        
        # 5. 获取接近方向
        if 'grasp_top_view_inds' in end_points:
            view_inds_all = end_points['grasp_top_view_inds'][b]
            if view_inds_all.dim() == 1:
                view_inds_all = view_inds_all.unsqueeze(1).repeat(1, NUM_ANGLE)
            view_inds_selected = view_inds_all[point_indices, angle_indices]
            approaching_selected = template_views[view_inds_selected]
        else:
            approaching_selected = torch.zeros(M_POINT * top_k, 3, device=device)
            approaching_selected[:, 2] = -1
        
        # 6. 计算抓取位置
        grasp_locations = grasp_center_selected - approaching_selected * grasp_depths.unsqueeze(-1) * 0.01
        
        # 7. 分块处理距离计算（完全在GPU上，避免CPU转换）
        dense_coordinates = end_points['point_clouds'][b, :, :3]
        graspness_scores = end_points['graspness_score'][b]
        if graspness_scores.dim() == 1:
            graspness_scores = graspness_scores.unsqueeze(-1)
        
        total_queries = grasp_center_selected.shape[0]
        chunk_size = 1000  # 每次处理1000个查询点
        
        closest_scores = torch.zeros(total_queries, 1, device=device)
        
        # 完全在GPU上计算，不转到CPU
        for i in range(0, total_queries, chunk_size):
            end_idx = min(i + chunk_size, total_queries)
            chunk_centers = grasp_center_selected[i:end_idx]
            
            # 精确计算距离（全在GPU上）
            chunk_distances = torch.cdist(chunk_centers, dense_coordinates)
            chunk_closest_indices = torch.argmin(chunk_distances, dim=1)
            chunk_closest_scores = graspness_scores[chunk_closest_indices]
            closest_scores[i:end_idx] = chunk_closest_scores
            
            # 清理
            del chunk_distances, chunk_closest_indices, chunk_closest_scores
        
        # 8. 归一化分数
        grasp_score_topk_flat = grasp_score_topk.reshape(-1, 1)
        grasp_score_norm = normalize_grasp_score(grasp_score_topk_flat)
        closest_scores_norm = normalize_grasp_score(closest_scores)
        combine_score = combine_scores(grasp_score_norm, closest_scores_norm)
        combine_score = normalize_grasp_score(combine_score)
        
        # 9. 坐标反归一化（修复版本：确保所有变量都是GPU tensor）
        if 'm' in end_points and 'centroid' in end_points:
            m = end_points['m']
            centroid = end_points['centroid']
            
            # 获取当前批次的m和centroid
            if isinstance(m, (list, torch.Tensor)):
                m = m[b] if len(m) > b else m
            if isinstance(centroid, (list, torch.Tensor)):
                centroid = centroid[b] if len(centroid) > b else centroid
            
            # === 关键修复：统一转换为GPU tensor ===
            # 处理 m
            if m is not None:
                if isinstance(m, np.ndarray):
                    m = torch.from_numpy(m).float().to(device)
                elif isinstance(m, torch.Tensor):
                    m = m.to(device)
                elif isinstance(m, (int, float)):
                    m = torch.tensor(m, dtype=torch.float32, device=device)
                elif isinstance(m, list):
                    m = torch.tensor(m, dtype=torch.float32, device=device)
                
                # 如果m是标量tensor，提取值用于标量乘法
                if m.numel() == 1:
                    m_scalar = m.item()
                    grasp_center_denorm = grasp_locations * m_scalar
                else:
                    # 确保维度匹配
                    if m.dim() == 1 and m.shape[0] == 3:
                        m = m.unsqueeze(0)  # (3,) -> (1, 3)
                    grasp_center_denorm = grasp_locations * m
            else:
                grasp_center_denorm = grasp_locations
            
            # 处理 centroid
            if centroid is not None:
                if isinstance(centroid, np.ndarray):
                    centroid = torch.from_numpy(centroid).float().to(device)
                elif isinstance(centroid, torch.Tensor):
                    centroid = centroid.to(device)
                elif isinstance(centroid, (int, float)):
                    centroid = torch.tensor(centroid, dtype=torch.float32, device=device)
                elif isinstance(centroid, list):
                    centroid = torch.tensor(centroid, dtype=torch.float32, device=device)
                
                # 添加centroid
                if centroid.numel() == 1:
                    grasp_center_denorm = grasp_center_denorm + centroid.item()
                else:
                    if centroid.dim() == 1 and centroid.shape[0] == 3:
                        centroid = centroid.unsqueeze(0)
                    grasp_center_denorm = grasp_center_denorm + centroid
        else:
            grasp_center_denorm = grasp_locations
        
        # 10. 构建最终结果
        grasp_rot = batch_viewpoint_params_to_matrix_data(approaching_selected, grasp_angles)
        grasp_rot_flat = grasp_rot.reshape(M_POINT * top_k, 9)
        
        final_grasp = torch.cat([
            combine_score,
            0.02 * torch.ones_like(combine_score),
            0.02 * torch.ones_like(combine_score),
            grasp_depths.unsqueeze(-1),
            grasp_rot_flat,
            grasp_center_denorm,
            -1 * torch.ones_like(combine_score)
        ], dim=-1)
        
        grasp_preds.append(final_grasp)
        
        # 清理
        del grasp_center, grasp_score, grasp_score_flat, grasp_score_topk, grasp_score_inds
        del angle_indices, depth_indices, grasp_angles, grasp_depths
        del point_indices, grasp_center_selected, approaching_selected
        del grasp_locations, grasp_rot, closest_scores
        
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    
    # 最后才转到CPU（在函数返回前）
    grasp_preds_cpu = []
    for pred in grasp_preds:
        grasp_preds_cpu.append(pred.cpu())
    
    return grasp_preds_cpu