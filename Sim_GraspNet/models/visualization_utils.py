"""
独立的可视化工具模块
将可视化和保存逻辑从网络类中分离出来
"""

import os
import time
import numpy as np
import open3d as o3d
import matplotlib
from typing import Optional, Dict, Any

def visualize_training_results(dense_points: np.ndarray,
                             affordance_scores: np.ndarray,
                             sparse_points: np.ndarray,
                             view_scores: np.ndarray,
                             template_views: np.ndarray,
                             approach_directions: Optional[np.ndarray] = None,
                             show_windows: bool = False,
                             view_scores_3dir: Optional[np.ndarray] = None,
                             view_scores_800dir: Optional[np.ndarray] = None,
                             view_inds_mapping: Optional[np.ndarray] = None,
                             # 网络预测结果
                             pred_affordance_scores: Optional[np.ndarray] = None,
                             pred_sparse_points: Optional[np.ndarray] = None,
                             pred_view_scores: Optional[np.ndarray] = None,
                             # xyz_graspable点的标签对比
                             graspable_point_labels: Optional[np.ndarray] = None) -> Dict[str, Any]:
    """
    可视化训练结果
    
    Args:
        dense_points: 密集点云 (N, 3)
        affordance_scores: 抓取可行性分数标签 (N,)
        sparse_points: 稀疏点云标签 (M, 3)
        view_scores: 视图分数标签 (M, 3) 或 (M, NUM_VIEW)
        template_views: 模板视图 (NUM_VIEW, 3)
        approach_directions: 实际的方向向量 (M, 3, 3)
        show_windows: 是否显示窗口
        view_scores_3dir: 原始3个方向的视图分数标签 (M, 3)
        view_scores_800dir: 映射到800个方向的视图分数标签 (M, NUM_VIEW)
        view_inds_mapping: 方向映射索引 (M, 3) - 每个有效抓取点的3个接近方向对应的模板方向索引
        # 网络预测结果
        pred_affordance_scores: 网络预测的抓取可行性分数 (N,)
        pred_sparse_points: 网络选取的点（FPS采样后的点）(M_pred, 3)
        pred_view_scores: 网络预测的方向分数 (M_pred, NUM_VIEW)
        graspable_point_labels: xyz_graspable点的真实标签 (M_pred,)
    
    Returns:
        包含可视化结果的字典
    """
    results = {}
    
    # 1. 可视化分数掩码
    if affordance_scores is not None:
        score_mask_pcd = _create_score_mask_visualization(dense_points, affordance_scores)
        results['score_mask'] = score_mask_pcd
        
        if show_windows:
            o3d.visualization.draw_geometries([score_mask_pcd], window_name="Score Mask")
    
    # 2. 可视化训练方向
    # if sparse_points is not None and template_views is not None:
    #     # 1. 可视化真实的3个接近方向
    #     if view_scores_3dir is not None and approach_directions is not None:
    #         real_directions = _create_training_3directions_visualization(
    #             sparse_points, view_scores_3dir, approach_directions
    #         )
    #         if real_directions is not None:
    #             results['real_directions'] = real_directions
    #             if show_windows:
    #                 pcd = o3d.geometry.PointCloud()
    #                 pcd.points = o3d.utility.Vector3dVector(sparse_points)
                    # o3d.visualization.draw_geometries([pcd, real_directions], window_name="Real 3 Directions")
        
        # 2. 可视化映射后的800个方向中的有效方向（使用模板方向）
        if view_scores_800dir is not None:
            mapped_directions = _create_training_directions_visualization(
                sparse_points, view_scores_800dir, template_views
            )
            if mapped_directions is not None:
                results['mapped_directions'] = mapped_directions
                if show_windows:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(sparse_points)
                    o3d.visualization.draw_geometries([pcd, mapped_directions], window_name="Mapped Template Directions")
        
        # 3. 可视化所有映射后的方向（包括无效的）
        # if view_scores_800dir is not None:
        #     all_directions = _create_all_directions_visualization(
        #         sparse_points, view_scores_800dir, template_views
        #     )
        #     if all_directions is not None:
        #         results['all_directions'] = all_directions
        #         if show_windows:
        #             pcd = o3d.geometry.PointCloud()
        #             pcd.points = o3d.utility.Vector3dVector(sparse_points)
                    # o3d.visualization.draw_geometries([pcd, all_directions], window_name="All Mapped Directions")
        
        # 4. 可视化template_views的分布
        # if template_views is not None:
        #     template_pcd = _create_template_views_visualization(template_views)
        #     if template_pcd is not None:
        #         results['template_views'] = template_pcd
        #         if show_windows:
        #             o3d.visualization.draw_geometries([template_pcd], window_name="Template Views Distribution")
        
        # 5. 可视化方向映射关系（真实方向→模板方向）
        if view_inds_mapping is not None and template_views is not None and approach_directions is not None:
            mapping_visualization = _create_direction_mapping_visualization(
                sparse_points, view_inds_mapping, template_views, approach_directions
            )
            if mapping_visualization is not None:
                results['direction_mapping'] = mapping_visualization
                if show_windows:
                    o3d.visualization.draw_geometries([mapping_visualization], window_name="Direction Mapping")
    
    # ==================== 网络预测结果可视化 ====================
    
    # 6. 可视化网络预测的抓取可行性分数
    if pred_affordance_scores is not None and dense_points is not None:
        print(f"Debug: pred_affordance_scores shape: {pred_affordance_scores.shape}")
        print(f"Debug: dense_points shape: {dense_points.shape}")
        
        # 检查形状是否匹配
        if len(pred_affordance_scores) != len(dense_points):
            print(f"Warning: Shape mismatch - pred_affordance_scores: {pred_affordance_scores.shape}, dense_points: {dense_points.shape}")
            # 如果形状不匹配，跳过预测分数可视化
        else:
            try:
                pred_score_mask_pcd = _create_score_mask_visualization(dense_points, pred_affordance_scores)
                results['pred_score_mask'] = pred_score_mask_pcd
                
                if show_windows:
                    o3d.visualization.draw_geometries([pred_score_mask_pcd], window_name="Predicted Score Mask")
            except Exception as e:
                print(f"Error creating predicted score mask: {e}")
                print(f"pred_affordance_scores type: {type(pred_affordance_scores)}")
                print(f"pred_affordance_scores dtype: {pred_affordance_scores.dtype if hasattr(pred_affordance_scores, 'dtype') else 'N/A'}")
    
    # 7. 可视化网络选取的点（FPS采样后的点）
    if pred_sparse_points is not None:
        pred_points_pcd = o3d.geometry.PointCloud()
        pred_points_pcd.points = o3d.utility.Vector3dVector(pred_sparse_points)
        pred_points_pcd.paint_uniform_color([1, 0, 1])  # 紫色表示网络选取的点
        results['pred_sparse_points'] = pred_points_pcd
        
        if show_windows:
            o3d.visualization.draw_geometries([pred_points_pcd], window_name="Network Selected Points (FPS)")
    
    # 8. 可视化网络预测的方向分数
    if pred_view_scores is not None and pred_sparse_points is not None and template_views is not None:
        pred_directions = _create_predicted_directions_visualization(
            pred_sparse_points, pred_view_scores, template_views
        )
        if pred_directions is not None:
            results['pred_directions'] = pred_directions
            if show_windows:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pred_sparse_points)
                pcd.paint_uniform_color([1, 0, 1])  # 紫色点
                o3d.visualization.draw_geometries([pcd, pred_directions], window_name="Predicted Directions")
    
    return results

def visualize_test_results(sparse_points: np.ndarray,
                         top_view_inds: np.ndarray,
                         view_scores: np.ndarray,
                         template_views: np.ndarray,
                         show_windows: bool = False,
                         show_best_directions: bool = True,
                         show_all_directions: bool = False,
                         show_score_visualization: bool = True) -> Dict[str, Any]:
    """
    可视化测试结果
    
    Args:
        sparse_points: 稀疏点云 (M, 3)
        top_view_inds: 最佳视图索引 (M,)
        view_scores: 视图分数 (M, NUM_VIEW)
        template_views: 模板视图 (NUM_VIEW, 3)
        show_windows: 是否显示窗口
        show_best_directions: 是否显示最佳方向
        show_all_directions: 是否显示所有方向
        show_score_visualization: 是否显示网络预测分数可视化
    
    Returns:
        包含可视化结果的字典
    """
    results = {}
    
    # 1. 可视化最佳方向
    if show_best_directions:
        best_directions = _create_best_directions_visualization(
            sparse_points, top_view_inds, template_views, view_scores
        )
        results['best_directions'] = best_directions
        
        if show_windows:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(sparse_points)
            o3d.visualization.draw_geometries([pcd, best_directions], window_name="Best Directions")
    
    # 2. 可视化所有方向
    if show_all_directions:
        all_directions = _create_all_directions_visualization(
            sparse_points, view_scores, template_views
        )
        results['all_directions'] = all_directions
        
        if show_windows:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(sparse_points)
            o3d.visualization.draw_geometries([pcd, all_directions], window_name="All Directions")

    # 3. 可视化网络预测分数
    if show_score_visualization:
        # 计算每个点的最高分数作为该点的抓取可行性分数
        point_scores = view_scores.max(axis=1)  # (M,)
        
        # 使用分数掩码可视化
        score_visualization = _create_score_mask_visualization(sparse_points, point_scores)
        results['score_visualization'] = score_visualization
        
        if show_windows:
            o3d.visualization.draw_geometries([score_visualization], window_name="Network Predicted Grasp Scores")
    
    return results

def _create_score_mask_visualization(point_cloud: np.ndarray, scores: np.ndarray):
    """创建分数掩码可视化"""
    # 确保scores是一维数组
    if scores.ndim > 1:
        scores = scores.squeeze()
    
    # 归一化分数
    normalized_scores = (scores - np.min(scores)) / (np.max(scores) - np.min(scores) + 1e-8)
    
    # 获取颜色映射
    cmap = matplotlib.cm.get_cmap('plasma')
    score_colors = cmap(normalized_scores)[:, :3]  # RGB colors based on scores
    
    # 创建点云对象
    score_mask_pcd = o3d.geometry.PointCloud()
    score_mask_pcd.points = o3d.utility.Vector3dVector(point_cloud)
    score_mask_pcd.colors = o3d.utility.Vector3dVector(score_colors)
    
    return score_mask_pcd

def _create_predicted_directions_visualization(points: np.ndarray, view_scores: np.ndarray, template_views: np.ndarray):
    """创建网络预测方向的可视化"""
    lines = []
    colors = []
    
    def score_to_color(score):
        # 根据分数强度设置颜色（从红色到绿色）
        if score <= 0:
            return [1.0, 0.0, 0.0]  # 红色：低分数
        elif score <= 0.5:
            return [1.0, 1.0, 0.0]  # 黄色：中等分数
        else:
            return [0.0, 1.0, 0.0]  # 绿色：高分数
    
    N = len(points)
    for i in range(N):
        for view_idx, score in enumerate(view_scores[i]):
            if score > 0.1:  # 只显示分数较高的方向
                direction = template_views[view_idx]
                norm = np.linalg.norm(direction)
                if norm > 1e-8:  # 避免除零
                    direction_normalized = direction / norm
                    end_point = points[i] + direction_normalized * 0.01
                    lines.append([points[i], end_point])
                    colors.append(score_to_color(score))
    
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None

def _create_training_3directions_visualization(points: np.ndarray, view_scores: np.ndarray, approach_directions: np.ndarray):
    """创建训练模式下的3个方向可视化"""
    lines = []
    colors = []
    
    def score_to_color(score):
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:
            return [0.0, 0.0, 1.0]  # Blue for valid grasps
    
    N = len(points)
    for i in range(N):
        for dir_idx in range(3):  # 每个点有3个方向
            score = view_scores[i, dir_idx]
            
            # 直接可视化所有方向（包括有效和无效的）
            # 使用实际的方向向量
            direction = approach_directions[i, dir_idx]  # (3,) 向量
            
            norm = np.linalg.norm(direction)
            if norm > 1e-8:  # 避免除零
                direction_normalized = direction / norm
            else:
                continue  # 跳过零向量
            end_point = points[i] + direction_normalized * 0.01
            lines.append([points[i], end_point])
            colors.append(score_to_color(score))
    
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None

def _create_training_directions_visualization(points: np.ndarray, view_scores: np.ndarray, template_views: np.ndarray):
    """创建训练方向可视化"""
    lines = []
    colors = []
    
    def score_to_color(score):
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:
            return [0.0, 0.0, 1.0]  # Blue for valid grasps
    
    N = len(points)
    for i in range(N):
        for view_idx, score in enumerate(view_scores[i]):
            if score > 0:  # 只可视化有效方向
                direction = template_views[view_idx]
                norm = np.linalg.norm(direction)
                if norm > 1e-8:  # 避免除零
                    direction_normalized = direction / norm
                    end_point = points[i] + direction_normalized * 0.01
                    lines.append([points[i], end_point])
                    colors.append(score_to_color(score))
    
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None

def _create_best_directions_visualization(points: np.ndarray, top_view_inds: np.ndarray, 
                                        template_views: np.ndarray, view_scores: np.ndarray):
    """创建最佳方向可视化"""
    lines = []
    colors = []
    
    N = len(points)
    for i in range(N):
        best_view_idx = top_view_inds[i]
        direction = template_views[best_view_idx]
        norm = np.linalg.norm(direction)
        if norm > 1e-8:  # 避免除零
            direction_normalized = direction / norm
        else:
            continue  # 跳过零向量
        end_point = points[i] + direction_normalized * 0.02
        lines.append([points[i], end_point])
        colors.append([0.0, 0.0, 1.0])  # 蓝色
    
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None

def _create_all_directions_visualization(points: np.ndarray, view_scores: np.ndarray, template_views: np.ndarray):
    """创建所有方向可视化"""
    # 二值化分数
    view_scores_normalized = (view_scores - view_scores.min()) / (view_scores.max() - view_scores.min() + 1e-8)
    view_scores_binary = (view_scores_normalized > 0.5).astype(np.float32)
    
    lines = []
    colors = []
    
    def score_to_color(score):
        if score == 0:
            return [1.0, 0.0, 0.0]  # Red for invalid grasps
        else:
            return [0.0, 0.0, 1.0]  # Blue for valid grasps
    
    N = len(points)
    for i in range(N):
        for view_idx, score in enumerate(view_scores_binary[i]):
            if score > 0:  # 只可视化有效方向
                direction = template_views[view_idx]
                norm = np.linalg.norm(direction)
                if norm > 1e-8:  # 避免除零
                    direction_normalized = direction / norm
                    end_point = points[i] + direction_normalized * 0.01
                    lines.append([points[i], end_point])
                    colors.append(score_to_color(score))
    
    if len(lines) > 0:
        lines_idx = [[i, i+1] for i in range(0, len(lines)*2, 2)]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack(lines)),
            lines=o3d.utility.Vector2iVector(lines_idx)
        )
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None

def _create_template_views_visualization(template_views: np.ndarray):
    """
    可视化template_views的分布
    
    Args:
        template_views: (NUM_VIEW, 3) - Fibonacci球面采样的方向向量
    
    Returns:
        o3d.geometry.PointCloud - 可视化template_views分布的点云
    """
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(template_views)
    
    # 设置颜色：使用彩虹色映射显示方向分布
    num_views = len(template_views)
    colors = np.zeros((num_views, 3))
    
    # 根据z坐标（高度）设置颜色
    z_coords = template_views[:, 2]
    z_normalized = (z_coords - z_coords.min()) / (z_coords.max() - z_coords.min() + 1e-8)
    
    for i in range(num_views):
        # 使用matplotlib的viridis颜色映射
        cmap = matplotlib.cm.get_cmap('viridis')
        color = cmap(z_normalized[i])[:3]  # 取RGB，忽略alpha
        colors[i] = color
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd

def _create_direction_mapping_visualization(sparse_points: np.ndarray, 
                                          view_inds_mapping: np.ndarray, 
                                          template_views: np.ndarray, 
                                          approach_directions: np.ndarray):
    """
    可视化方向映射关系（真实方向→模板方向）
    
    Args:
        sparse_points: 稀疏点云 (M, 3)
        view_inds_mapping: 方向映射索引 (M, 3) - 每个有效抓取点的3个接近方向对应的模板方向索引
        template_views: 模板视图 (NUM_VIEW, 3)
        approach_directions: 实际的方向向量 (M, 3, 3)
    
    Returns:
        o3d.geometry.LineSet - 可视化方向映射关系的线集
    """
    if sparse_points is None or view_inds_mapping is None or template_views is None or approach_directions is None:
        return None
    
    lines = []
    colors = []
    
    # 确保数据大小匹配
    N_sparse = len(sparse_points)
    N_mapping = len(view_inds_mapping)
    N_approach = len(approach_directions)
    
    # 使用最小的长度来避免索引越界
    N = min(N_sparse, N_mapping, N_approach)
    
    if N == 0:
        return None
    
    for i in range(N):
        # 获取第i个抓取点的3个真实方向
        real_directions = approach_directions[i]  # (3, 3) - 3个真实方向向量
        mapping_indices = view_inds_mapping[i]    # (3,) - 对应的模板方向索引
        
        for j in range(3):
            # 真实方向（从抓取点出发）
            real_direction = real_directions[j]  # (3,) - 第j个真实方向
            real_direction_normalized = real_direction / np.linalg.norm(real_direction)
            real_end_point = sparse_points[i] + real_direction_normalized * 0.015
            
            # 对应的模板方向（从抓取点出发）
            template_idx = mapping_indices[j]
            if 0 <= template_idx < len(template_views):
                template_direction = template_views[template_idx]  # (3,) - 对应的模板方向
                template_direction_normalized = template_direction / np.linalg.norm(template_direction)
                template_end_point = sparse_points[i] + template_direction_normalized * 0.015
                
                # 添加真实方向线（红色）
                lines.append([sparse_points[i], real_end_point])
                colors.append([1.0, 0.0, 0.0])  # 红色
                
                # 添加模板方向线（蓝色）
                lines.append([sparse_points[i], template_end_point])
                colors.append([0.0, 0.0, 1.0])  # 蓝色
                
                # 添加连接线（绿色，显示映射关系）
                lines.append([real_end_point, template_end_point])
                colors.append([0.0, 1.0, 0.0])  # 绿色
    
    if lines:
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array(lines).reshape(-1, 3))
        line_set.lines = o3d.utility.Vector2iVector(np.arange(len(lines) * 2).reshape(-1, 2))
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set
    else:
        return None
