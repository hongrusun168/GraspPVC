"""
Sim-Grasp 数据预处理模块
功能：处理抓取仿真数据，提取抓取特征和质量分数，为神经网络训练准备数据

主要功能：
- 在单位球面上生成均匀分布的观察视角
- 处理抓取候选数据，提取抓取位置、接近方向、质量分数
- 数据标准化和批量处理
- 保存预处理后的训练数据
"""

import numpy as np
import pickle
from tqdm import tqdm
import os
from collections import defaultdict


def generate_grasp_views(N=300, phi=(np.sqrt(5) - 1) / 2, center=np.zeros(3), r=1):
    """
    在单位球面上使用斐波那契网格采样观察视角
    参考论文: https://arxiv.org/abs/0912.4540
    
    输入参数:
        N: [int] 采样的视角数量
        phi: [float] 视角坐标计算的常数，不同phi值产生不同分布，默认: (sqrt(5)-1)/2
        center: [np.ndarray, (3,), np.float32] 球心坐标
        r: [float] 球面半径
    
    输出:
        views: [np.ndarray, (N,3), np.float32] 采样的视角坐标
    """
    views = []
    for i in range(N):
        zi = (2 * i + 1) / N - 1
        xi = np.sqrt(1 - zi ** 2) * np.cos(2 * i * np.pi * phi)
        yi = np.sqrt(1 - zi ** 2) * np.sin(2 * i * np.pi * phi)
        views.append([xi, yi, zi])
    views = r * np.array(views) + center
    return views.astype(np.float32)


def preprocess_grasp_data(candidate_simulation):
    """
    预处理抓取仿真数据，提取抓取特征和质量分数
    
    输入参数:
        candidate_simulation: [dict] 抓取仿真候选数据字典
    
    输出:
        unique_t_ori_points: [np.ndarray] 唯一抓取位置点
        normalized_scores: [np.ndarray] 归一化的抓取质量分数
        approach_directions: [np.ndarray] 接近方向向量
        normalized_summed_view_score: [np.ndarray] 归一化的视图分数
        affordance_scores: [np.ndarray] 抓取可行性分数
    """
    
    # ==================== 1. 计算唯一抓取点的数量 ====================
    total_grasp_poses = sum(len(grasp_data["grasp_samples"]) for grasp_data in candidate_simulation.values())
    num_unique_points = total_grasp_poses // (3 * 12 * 7)  # 唯一抓取点数量，每个抓取点有3个接近方向，12个旋转角度，7个深度

    # ==================== 2. 设置抓取参数的最大值 ====================
    max_approach_directions = 3  # 最大接近方向数
    max_rotation_angles = 12     # 最大旋转角度数
    max_depths = 7              # 最大深度数

    # ==================== 3. 初始化数据数组 ====================
    approach_directions = np.zeros((num_unique_points, max_approach_directions, 3))  # (num_unique_points, 3, 3) - 接近方向向量
    affordance_scores = np.zeros((num_unique_points, max_approach_directions, max_rotation_angles, max_depths))  # (num_unique_points, 3, 12, 7) - 抓取可行性分数
    view_score = np.zeros((num_unique_points, max_approach_directions, max_rotation_angles, max_depths))  # (num_unique_points, 3, 12, 7) - 视图分数

    # ==================== 4. 提取唯一的抓取位置点及其对应的样本 ====================
    # t_ori_dict: {tuple(x, y, z): [sample1, sample2, ...]}
    # 字典结构: {抓取位置坐标: 该位置的所有抓取样本列表}
    # 每个抓取位置有252个样本 (3个接近方向 × 12个旋转角度 × 7个深度)
    t_ori_dict = defaultdict(list)  # 创建一个默认值为空列表的字典
    for object_index, grasp_data in candidate_simulation.items():
        # grasp_data: 包含某个物体的所有抓取数据
        # grasp_data["grasp_samples"]: 该物体的所有抓取样本列表
        for sample in grasp_data["grasp_samples"]:
            # sample: 单个抓取样本，包含10个字段:
            # - 't_ori': 抓取位置坐标 [x, y, z]
            # - 'approach_direction': 接近方向向量 [x, y, z]
            # - 'grasp_translation': 抓取器平移向量
            # - 'grasp_rotation_matrix': 抓取旋转矩阵 (3x3)
            # - 'stand_off': 抓取深度
            # - 'rotation_angle': 旋转角度 (0-11)
            # - 'collision_quality': 碰撞质量分数 (0.25, 0.5, 0.75, 1.0)
            # - 'simulation_quality': 仿真质量分数 (-1, 0, 1)
            # - 'segmentation_id': 分割ID
            # - 'object_name': 物体名称
            t_ori = tuple(sample["t_ori"][:3])  # 提取抓取位置的xyz坐标作为唯一标识
            t_ori_dict[t_ori].append(sample)    # 将样本添加到对应位置的列表中

    # ==================== 5. 填充数据数组 ====================

    simulation_quality_1_count = 0  # 统计仿真成功的样本数量
    collision_quality_count = 0     # 统计有碰撞质量的样本数量
    
    # 外层循环: 遍历每个唯一抓取位置
    # i: 抓取位置索引 (0 到 num_unique_points-1)
    # t_ori: 抓取位置坐标 (x, y, z)
    # samples: 该位置的所有抓取样本列表 (252个样本)
    for i, (t_ori, samples) in enumerate(t_ori_dict.items()):
        # 内层循环: 遍历该位置的所有抓取样本
        # j: 样本在列表中的索引 (0 到 251)
        # sample: 单个抓取样本字典
        for j, sample in enumerate(samples):
            # 将一维索引j转换为多维索引，对应抓取参数组合
            # 数据结构: 3个接近方向 × 12个旋转角度 × 7个深度 = 252个组合
            
            # 计算接近方向索引 (0, 1, 2)
            approach_dir_index = j // (max_rotation_angles * max_depths) % max_approach_directions
            
            # 计算旋转角度索引 (0 到 11)
            rotation_index = (j // max_depths) % max_rotation_angles
            
            # 计算深度索引 (0 到 6)
            stand_off_index = j % max_depths

            # 存储接近方向向量到对应位置
            # approach_directions[i, approach_dir_index] = [x, y, z] 向量
            approach_directions[i, approach_dir_index] = sample['approach_direction']

            # 根据抓取质量设置不同的分数值           
            # 情况1: 仿真成功的抓取 (simulation_quality = 1)
            if sample.get("simulation_quality", 0) == 1:
                # 设置抓取可行性分数为1 (表示该抓取是可行的)
                affordance_scores[i, approach_dir_index, rotation_index, stand_off_index] = 1
                # 设置视图分数为10 (高分数，表示该方向视图质量很好)
                view_score[i, approach_dir_index, rotation_index, stand_off_index] = 10
                simulation_quality_1_count += 1
            # 情况2: 有碰撞质量的抓取 (collision_quality > 0)
            elif sample.get("collision_quality", 0) != 0:
                # 设置视图分数为碰撞质量值 (0.25, 0.5, 0.75, 1.0)
                # 不设置affordance_scores (保持0，表示不可行)
                view_score[i, approach_dir_index, rotation_index, stand_off_index] = sample.get("collision_quality", 0)
                collision_quality_count += 1
            # 情况3: 完全失败的抓取 (simulation_quality != 1 且 collision_quality = 0)
            # 不设置任何分数，保持默认值0

    # ==================== 6. 计算抓取可行性分数 ====================
    # 每个抓取点有3个接近方向，每个接近方向有12个旋转角度，每个旋转角度有7个深度，对所有维度求和得到每个抓取点的总可行性分数
    summed_scores = np.sum(affordance_scores, axis=(1, 2, 3))  # (num_unique_points,) - 每个唯一抓取点的总可行性分数

    # ==================== 7. 对抓取分数进行归一化 ====================
    min_score = np.min(summed_scores)
    max_score = np.max(summed_scores)
    range_score = max_score - min_score
    normalized_scores = (summed_scores - min_score) / range_score  # (num_unique_points,) - 归一化的抓取质量分数
    unique_t_ori_points = np.array(list(t_ori_dict.keys()))  # (num_unique_points, 3) - 唯一抓取位置点

    # ==================== 8. 对视图分数进行归一化 ====================
    # 对每个抓取点的每个接近方向，将旋转角度和深度维度求和
    summed_view_score = np.sum(view_score, axis=(2, 3))  # (num_unique_points, 3) - 每个抓取点每个接近方向的总视图分数
    min_values_per_channel = np.min(summed_view_score, axis=1, keepdims=True)  # (num_unique_points, 1) - 每个抓取点的最小视图分数
    max_values_per_channel = np.max(summed_view_score, axis=1, keepdims=True)  # (num_unique_points, 1) - 每个抓取点的最大视图分数
    range_values_per_channel = max_values_per_channel - min_values_per_channel  # (num_unique_points, 1) - 每个抓取点的视图分数范围

    # 避免除零错误
    epsilon = 1e-8
    normalized_summed_view_score = (summed_view_score - min_values_per_channel) / (range_values_per_channel + epsilon)  # (num_unique_points, 3) - 归一化的视图分数
    
    # 返回唯一抓取位置点、归一化的抓取质量分数、接近方向向量、归一化的视图分数、抓取可行性分数
    return unique_t_ori_points, normalized_scores, approach_directions, normalized_summed_view_score, affordance_scores


class SimGraspDataPreprocessor:
    """
    Sim-Grasp 数据预处理器
    功能：批量处理抓取仿真数据，提取特征并保存为训练数据
    """
    
    def __init__(self, data_root, label_root, block_size=150, num_points=20000):
        """
        初始化数据预处理器
        
        输入参数:
            data_root: [str] 点云数据根目录路径
            label_root: [str] 标签数据根目录路径
            block_size: [int] 数据块大小，默认150
            num_points: [int] 点云点数，默认20000
        """
        self.data_root = data_root
        self.label_root = label_root
        self.block_size = block_size
        self.num_points = num_points
        self.views = generate_grasp_views(N=300)  # 生成观察视角

    def preprocess_and_save_stage_data(self):
        """
        批量预处理并保存场景数据
        处理指定范围内的所有场景，提取抓取特征并保存为pickle文件
        """
        for room_idx in tqdm(range(0, 1)):
            # 加载抓取仿真候选数据
            candidate_simulation_path = self.label_root + f"/stage_{room_idx}" + f"/stage_{room_idx}_grasp_simulation_candidates_test.pkl"
            with open(candidate_simulation_path, 'rb') as f:
                candidate_simulation = pickle.load(f)

            # 预处理抓取数据
            unique_t_ori_points, normalized_scores, approach_directions, normalized_view_score, normalized_grasp_score = preprocess_grasp_data(candidate_simulation)

            # 创建预处理数据字典
            preprocessed_data = {
                'unique_t_ori_points': unique_t_ori_points,
                'normalized_scores': normalized_scores,
                'approach_directions': approach_directions,
                'normalized_view_score': normalized_view_score,
                'normalized_grasp_score': normalized_grasp_score
            }

            # 创建场景目录
            stage_dir = os.path.join(self.label_root, f"stage_{room_idx}")
            os.makedirs(stage_dir, exist_ok=True)

            # 保存预处理数据为pickle文件
            with open(os.path.join(stage_dir, f'stage_{room_idx}_preprocessed_data.pkl'), 'wb') as f:
                pickle.dump(preprocessed_data, f)

# 使用示例
if __name__ == "__main__":
    # 设置数据路径
    data_root = '/home/a/shr/simgrasp-sxy/Sim-Grasp/datasets/real/train/pointcloud_grasp_train_pipe13'
    label_root = '/home/a/shr/simgrasp-sxy/Sim-Grasp/datasets/real/train/real_data_grasp_train_pipe13'
    
    # 创建预处理器并执行批量处理
    preprocessor = SimGraspDataPreprocessor(data_root, label_root)
    preprocessor.preprocess_and_save_stage_data()