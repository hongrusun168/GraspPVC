import os
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
from mlp import SharedMLP

import pointnet2.pytorch_utils as pt_utils
from pointnet2.pointnet2_utils import CylinderQueryAndGroup
from loss_utils import generate_grasp_views, batch_viewpoint_params_to_matrix,batch_viewpoint_params_to_matrix_data


class GraspAffordanceNet(nn.Module):
    """
    抓取可行性预测网络
    功能：为每个点预测抓取可行性分数，判断该点是否适合进行抓取操作
    """
    def __init__(self, seed_feature_dim):
        super(GraspAffordanceNet, self).__init__()
        # 第一层：特征提取和变换
        self.conv1 = nn.Conv1d(seed_feature_dim, 256, 1)  # 输入特征维度 -> 256维特征
        self.bn1 = nn.BatchNorm1d(256)  # 批归一化层
        
        # 第二层：输出抓取可行性分数
        self.conv2 = nn.Conv1d(256, 1, 1)  # 256维特征 -> 1维分数

    def forward(self, seed_features, end_points):
        """
        前向传播
        输入: seed_features (B, feat_dim, N) - (8, 256, 40000) PointNet++提取的特征
        输出: end_points (更新后的数据字典)
        """
        # ==================== 1. 特征提取 ====================
        # 第一层：特征变换和批归一化
        # conv1: (B, feat_dim, N) -> (B, 256, N) - 特征变换 - (8, 256, 40000) -> (8, 256, 40000)
        # bn1: (B, 256, N) -> (B, 256, N) - 批归一化 - (8, 256, 40000) -> (8, 256, 40000)
        # relu: (B, 256, N) -> (B, 256, N) - ReLU非线性激活，负值被置为0 - (8, 256, 40000) -> (8, 256, 40000)
        features = F.relu(self.bn1(self.conv1(seed_features)))  # (B, 256, N) - 激活后的特征 - (8, 256, 40000)
        
        # ==================== 2. 抓取可行性预测 ====================
        # 第二层：预测抓取可行性分数（无激活函数，输出原始分数）
        # conv2: (B, 256, N) -> (B, 1, N) - 预测抓取可行性分数 - (8, 256, 40000) -> (8, 1, 40000)
        graspable_score = self.conv2(features)  # (B, 1, N) - 每个点的抓取可行性分数 - (8, 1, 40000)
        
        # ==================== 3. 维度调整 ====================
        # 调整维度：从(B, 1, N)变为(B, N, 1)以适配后续处理
        graspable_score = graspable_score.transpose(2, 1).contiguous()  # (B, N, 1) - 调整后的抓取可行性分数 - (8, 40000, 1)
        
        # ==================== 4. 存储结果 ====================
        # 将预测分数存储到end_points中
        end_points['graspness_score'] = graspable_score  # (B, N, 1) - 抓取可行性分数 - (8, 40000, 1)

        return end_points
class ApproachVecNet(nn.Module):
    """
    接近方向预测网络
    功能：为每个抓取点预测最佳的接近方向
    """
    def __init__(self, num_view, seed_feature_dim, is_training=True):
        super().__init__()
        self.num_view = num_view          # 模板方向数量 - 800
        self.in_dim = seed_feature_dim    # 输入特征维度 - 256
        self.is_training = is_training    # 是否为训练模式
        
        # 网络层定义
        self.conv1 = nn.Conv1d(self.in_dim, self.in_dim, 1)  # 特征提取层
        self.conv2 = nn.Conv1d(self.in_dim, self.num_view, 1)  # 方向预测层

    def forward(self, seed_features, end_points):
        """
        前向传播
        输入: seed_features (B, feat_dim, M_POINT) - (8, 256, 2048)
        输出: end_points (更新后的数据字典), res_features (残差特征)
        """
        B, _, num_seed = seed_features.size()  # B=8, num_seed=2048
        
        # ==================== 1. 特征提取 ====================
        # 第一层卷积：特征提取和变换
        # conv1: (B, feat_dim, M_POINT) -> (B, 256, M_POINT) - 特征变换 - (8, 256, 2048) -> (8, 256, 2048)
        # bn1: (B, 256, M_POINT) -> (B, 256, M_POINT) - 批归一化 - (8, 256, 2048) -> (8, 256, 2048)
        # relu: (B, 256, M_POINT) -> (B, 256, M_POINT) - ReLU非线性激活，负值被置为0 - (8, 256, 2048) -> (8, 256, 2048)
        res_features = F.relu(self.conv1(seed_features), inplace=True)  # (B, 256, M_POINT) - 激活后的特征 - (8, 256, 2048)
        
        # 第二层卷积：预测每个模板方向的分数
        # conv2: (B, 256, M_POINT) -> (B, 800, M_POINT) - 预测每个模板方向的分数 - (8, 256, 2048) -> (8, 800, 2048)
        features = self.conv2(res_features)  # (B, 800, M_POINT) - 每个点的模板方向分数 - (8, 800, 2048)
        
        # 调整维度：从(B, num_view, M_POINT)变为(B, M_POINT, num_view)
        view_score = features.transpose(1, 2).contiguous()  # (B, M_POINT, 800) - 调整后的模板方向分数 - (8, 2048, 800)
        end_points['view_score'] = view_score
        
        # ==================== 2. 最佳方向选择 ====================
        if self.is_training:
            # 训练模式：使用多项式采样增加随机性
            # 归一化分数到[0,1]范围
            view_score_ = view_score.clone().detach()  # (B, M_POINT, 800) - (8, 2048, 800)
            view_score_max, _ = torch.max(view_score_, dim=2)  # (B, M_POINT) - 每个点的最大分数 - (8, 2048)
            view_score_min, _ = torch.min(view_score_, dim=2)  # (B, M_POINT) - 每个点的最小分数 - (8, 2048)
            
            # 扩展维度用于广播
            view_score_max = view_score_max.unsqueeze(-1).expand(-1, -1, self.num_view)  # (B, M_POINT, 800) - (8, 2048, 800)
            view_score_min = view_score_min.unsqueeze(-1).expand(-1, -1, self.num_view)  # (B, M_POINT, 800) - (8, 2048, 800)
            
            # 归一化：[0,1]范围
            view_score_ = (view_score_ - view_score_min) / (view_score_max - view_score_min + 1e-8)  # (B, M_POINT, 800) - (8, 2048, 800)

            # 多项式采样：根据分数概率分布采样方向
            top_view_inds = []
            for i in range(B):
                # multinomial采样：
                # 对每个点，根据其800个方向的分数分布进行采样
                # 结果：2048个点，每个点选择1个方向
                top_view_inds_batch = torch.multinomial(view_score_[i], 1, replacement=False)  # (M_POINT, 1) - (2048, 1)
                top_view_inds.append(top_view_inds_batch)
            top_view_inds = torch.stack(top_view_inds, dim=0).squeeze(-1)  # (B, M_POINT) - 最佳方向索引 - (8, 2048)
            
        else:
            # 测试模式：选择分数最高的方向
            _, top_view_inds = torch.max(view_score, dim=2)  # (B, M_POINT) - 最高分数方向的索引 - (8, 2048)

            # ==================== 3. 生成旋转矩阵 ====================
            # 扩展索引维度用于gather操作
            top_view_inds_ = top_view_inds.view(B, num_seed, 1, 1).expand(-1, -1, -1, 3).contiguous()  # (B, M_POINT, 1, 3)
            
            # 生成模板方向向量
            template_views = generate_grasp_views(self.num_view).to(features.device)  # (800, 3) - Fibonacci球面采样
            template_views = template_views.view(1, 1, self.num_view, 3).expand(B, num_seed, -1, -1).contiguous()  # (B, M_POINT, 800, 3)
            
            # 根据索引收集最佳方向向量
            vp_xyz = torch.gather(template_views, 2, top_view_inds_).squeeze(2)  # (B, M_POINT, 3) - 最佳方向向量
            vp_xyz_ = vp_xyz.view(-1, 3)  # (B*M_POINT, 3) - 展平用于批量处理
            
            # 生成旋转矩阵（角度为0，只考虑方向）
            batch_angle = torch.zeros(vp_xyz_.size(0), dtype=vp_xyz.dtype, device=vp_xyz.device)  # (B*M_POINT,)
            vp_rot = batch_viewpoint_params_to_matrix_data(vp_xyz_, batch_angle).view(B, num_seed, 3, 3)  # (B, M_POINT, 3, 3)
            
            # 存储结果
            end_points['grasp_top_view_xyz'] = vp_xyz      # 最佳方向向量
            end_points['grasp_top_view_rot'] = vp_rot      # 对应的旋转矩阵

        # 存储最佳方向索引
        end_points['grasp_top_view_inds'] = top_view_inds  # (B, M_POINT)
        
        return end_points, res_features  # 返回更新后的end_points和残差特征


class GroupNet(nn.Module):
    #def __init__(self, nsample, seed_feature_dim, cylinder_radius=0.05, hmin=-0.2, hmax=0.4):
    def __init__(self, nsample, seed_feature_dim, cylinder_radius=0.05, hmin=-0.02, hmax=0.04):    
        super().__init__()
        self.nsample = nsample
        self.in_dim = seed_feature_dim
        self.cylinder_radius = cylinder_radius
        mlps = [3 + self.in_dim, 256, 256]   # use xyz, so plus 3

        self.grouper = CylinderQueryAndGroup(radius=cylinder_radius, hmin=hmin, hmax=hmax, nsample=nsample,
                                             use_xyz=True, normalize_xyz=True,rotate_xyz=True)
        self.mlps = pt_utils.SharedMLP(mlps, bn=True)

    def forward(self, seed_xyz_graspable, seed_features_graspable, vp_rot):
        grouped_feature = self.grouper(seed_xyz_graspable, seed_xyz_graspable, vp_rot,
                                       seed_features_graspable)  # B*3 + feat_dim*M*K
        new_features = self.mlps(grouped_feature)  # (batch_size, mlps[-1], M, K)
        new_features = F.max_pool2d(new_features, kernel_size=[1, new_features.size(3)])  # (batch_size, mlps[-1], M, 1)
        new_features = new_features.squeeze(-1)   # (batch_size, mlps[-1], M)
        return new_features


class PoseNet(nn.Module):
    def __init__(self, num_angle, num_depth):
        super().__init__()
        self.num_angle = num_angle
        self.num_depth = num_depth

        self.conv1 = nn.Conv1d(256, 256, 1)  # input feat dim need to be consistent with CloudCrop module
        self.conv_swad = nn.Conv1d(256, num_angle*num_depth, 1)

    def forward(self, vp_features, end_points):
        B, _, num_seed = vp_features.size()
        vp_features = F.relu(self.conv1(vp_features), inplace=True)
        vp_features = self.conv_swad(vp_features)
        vp_features = vp_features.view(B, 1, self.num_angle, self.num_depth, num_seed)
        vp_features = vp_features.permute(0, 1, 4, 2, 3)

        # split prediction
        end_points['grasp_score_pred'] = vp_features[:, 0]  # B * num_seed * num angle * num_depth
        #end_points['grasp_width_pred'] = vp_features[:, 1]
        return end_points
