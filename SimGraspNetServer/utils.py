"""
本脚本用于存放一些通用的工具函数
"""
import os
import cv2
import copy
import json
import time
import torch
import warnings
import numba as nb
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp


device = torch.device("cuda" if torch.cuda.is_available() 
                   else "mps" if torch.backends.mps.is_available()
                   else "cpu")


def Visualize_Masked_Image(img, mask):
    """
    可视化被掩码分割后的图像
    
    参数:
        img: RGB图像 [H, W, 3]
        mask: 二值掩码 [H, W]
    
    显示：
        - 左上：原始RGB图像
        - 右上：掩码可视化（白色=前景，黑色=背景）
        - 左下：分割后的前景图像（仅掩码区域）
        - 右下：原始图像叠加掩码边界
    """
    
    # 确保图像是 uint8 格式
    if img.dtype != np.uint8:
        img_display = (img * 255).astype(np.uint8) if img.max() <= 1 else img.astype(np.uint8)
    else:
        img_display = img
    
    # 确保是 RGB 格式（如果是 BGR 则转换）
    if len(img_display.shape) == 3 and img_display.shape[2] == 3:
        # 如果需要，从 BGR 转换到 RGB
        img_rgb = cv2.cvtColor(img_display, cv2.COLOR_BGR2RGB)
    else:
        img_rgb = img_display
    
    # 创建掩码的彩色版本
    mask_uint8 = (mask.astype(np.uint8) * 255)
    mask_color = cv2.cvtColor(mask_uint8, cv2.COLOR_GRAY2RGB)
    
    # 分割后的前景图像
    foreground_img = img_rgb.copy()
    foreground_img[~mask] = 0  # 将背景置为黑色
    
    # 获取掩码边界
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_with_contour = img_rgb.copy()
    cv2.drawContours(img_with_contour, contours, -1, (0, 255, 0), 2)  # 绿色边界
    
    # 创建可视化窗口
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('掩码分割结果可视化', fontsize=16, fontweight='bold')
    
    # 左上：原始图像
    axes[0, 0].imshow(img_rgb)
    axes[0, 0].set_title('原始 RGB 图像')
    axes[0, 0].axis('off')
    
    # 右上：掩码
    axes[0, 1].imshow(mask_color)
    axes[0, 1].set_title(f'掩码 (前景点数: {np.sum(mask)})')
    axes[0, 1].axis('off')
    
    # 左下：分割后的前景
    axes[1, 0].imshow(foreground_img)
    axes[1, 0].set_title('分割后的前景图像')
    axes[1, 0].axis('off')
    
    # 右下：原始图像 + 掩码边界
    axes[1, 1].imshow(img_with_contour)
    axes[1, 1].set_title('掩码边界叠加')
    axes[1, 1].axis('off')
    
    # 显示图像
    plt.tight_layout()
    plt.show()


def visualize_pcd(pcd, show_normals = False, normal_length = 0.01, normal_color = [1, 0, 0]):
    """
    可视化点云
    
    Args:
        pcd: open3d点云对象
        show_normals: 是否显示法线，默认False
        normal_length: 法线长度，默认0.01米
        normal_color: 法线颜色 [R, G, B]，默认红色 [1, 0, 0]
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name = "Point Cloud Visualization", width = 1280, height = 720)

    vis.add_geometry(pcd)

    # 添加法线可视化
    if show_normals:
        # 手动创建法线线段集合
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        # 创建线段端点：起点是点云点，终点是点+法线*长度
        line_points = []
        line_indices = []
        for i in range(len(points)):
            start_point = points[i]
            end_point = points[i] + normals[i] * normal_length
            
            line_points.append(start_point)
            line_points.append(end_point)
            line_indices.append([2*i, 2*i+1])
        
        # 创建LineSet对象
        normals_lineset = o3d.geometry.LineSet()
        normals_lineset.points = o3d.utility.Vector3dVector(line_points)
        normals_lineset.lines = o3d.utility.Vector2iVector(line_indices)
        normals_lineset.paint_uniform_color(normal_color)
        
        vis.add_geometry(normals_lineset)

    # 添加坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1, origin = [0, 0, 0])
    vis.add_geometry(coord_frame)

    # 渲染选项
    render_option = vis.get_render_option()
    render_option.background_color = np.array([0.1, 0.1, 0.1])
    render_option.point_size = 2.0

    vis.run()
    vis.destroy_window()


def collision_checker(pcd, 
                      gg_array = None, 
                      gripper = None, 
                      width = 0.0075, 
                      FINGER_WIDTH = 0.002, 
                      FINGER_HEIGHT = 0.020, 
                      FINGER_LENGTH = 0.200, 
                      MIDDLE_WIDTH = 0.004, 
                      MIDDLE_HEIGHT = 0.020, 
                      MIDDLE_LENGTH = 0.008, 
                      MIDDLE_OFFSET = 0.191, 
                      BASE_WIDTH = 0.060, 
                      BASE_HEIGHT = 0.040, 
                      BASE_LENGTH = 0.060,
                      collision_point_threshold = 3,    
                      min_points_per_region = 15,       
                      voxel_size = 0.005, 
                      depth_scale = -0.002, 
                      batch_size = 1024, 
                      visualize = False, 
                      show_num = 1, 
                      device = 'cuda' if torch.cuda.is_available() else 'cpu'
                      ):
    
    # --- 1. 数据准备 ---
    if isinstance(gg_array, np.ndarray):
        gg_array_torch = torch.from_numpy(gg_array).float().to(device)
    else:
        gg_array_torch = gg_array
    
    num_grasps = gg_array_torch.shape[0]
    pcd_np = np.asarray(pcd.points)
    points = torch.from_numpy(pcd_np).float().to(device)
    
    world_min = points.min(dim=0)[0]
    world_max = points.max(dim=0)[0]

    # --- 2. 构造夹爪控制点 (用于越界检测) ---
    def get_box_points(x_range, y_range, z_range):
        pts = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    pts.append([x, y, z])
        return pts

    left_finger_pts = get_box_points([-(width/2) - FINGER_WIDTH, -(width/2)], [-FINGER_HEIGHT/2, FINGER_HEIGHT/2], [0, FINGER_LENGTH])
    right_finger_pts = get_box_points([width/2, width/2 + FINGER_WIDTH], [-FINGER_HEIGHT/2, FINGER_HEIGHT/2], [0, FINGER_LENGTH])
    base_pts = get_box_points([-BASE_WIDTH/2, BASE_WIDTH/2], [-BASE_HEIGHT/2, BASE_HEIGHT/2], [-BASE_LENGTH, 0])

    all_control_pts = torch.tensor(left_finger_pts + right_finger_pts + base_pts, device=device, dtype=torch.float)

    # 初始化结果存储
    valid_mask = torch.zeros(num_grasps, dtype=torch.bool, device=device)
    grasp_contact_counts = torch.zeros(num_grasps, dtype=torch.float, device=device)

    all_R_corr, all_T_base, all_center_fixed = [], [], []

    # --- 3. 批量计算逻辑 ---
    for i in range(0, num_grasps, batch_size):
        end = min(i + batch_size, num_grasps)
        cur_gg = gg_array_torch[i:end]
        
        # 位姿解析
        raw_rot = cur_gg[:, 4:13].reshape(-1, 3, 3)
        x_axis = raw_rot[:, :, 0]
        z_axis = -raw_rot[:, :, 2] 
        y_axis_new = torch.cross(z_axis, x_axis, dim=1)
        y_axis_new = y_axis_new / (torch.norm(y_axis_new, dim=1, keepdim=True) + 1e-8)
        R_corr = torch.stack((x_axis, y_axis_new, z_axis), dim=2) 

        center_raw = cur_gg[:, 13:16]
        depth = cur_gg[:, 3:4] * depth_scale
        center_fixed = center_raw - z_axis * depth
        T_base = center_fixed - z_axis * FINGER_LENGTH 

        # A. 越界检测
        global_pts = torch.matmul(all_control_pts.unsqueeze(0), R_corr.transpose(1, 2)) + T_base.unsqueeze(1)
        is_in_view = ((global_pts[:, :, 0] >= world_min[0]) & (global_pts[:, :, 0] <= world_max[0]) &
                      (global_pts[:, :, 1] >= world_min[1]) & (global_pts[:, :, 1] <= world_max[1])).all(dim=1)

        # B. 局部坐标系变换
        targets = torch.matmul(points.unsqueeze(0) - T_base.unsqueeze(1), R_corr)

        # C. 碰撞检测区域定义
        mask_z_fingers = (targets[:, :, 2] > 0) & (targets[:, :, 2] < FINGER_LENGTH)
        mask_y_fingers = (targets[:, :, 1] > -FINGER_HEIGHT / 2) & (targets[:, :, 1] < FINGER_HEIGHT / 2)
        half_w = width / 2

        left_mask = mask_z_fingers & mask_y_fingers & (targets[:, :, 0] > -half_w - FINGER_WIDTH) & (targets[:, :, 0] < -half_w)
        right_mask = mask_z_fingers & mask_y_fingers & (targets[:, :, 0] > half_w) & (targets[:, :, 0] < half_w + FINGER_WIDTH)
        base_mask = (targets[:, :, 2] > -BASE_LENGTH) & (targets[:, :, 2] <= 0) & \
                    (targets[:, :, 0] > -BASE_WIDTH/2) & (targets[:, :, 0] < BASE_WIDTH/2) & \
                    (targets[:, :, 1] > -BASE_HEIGHT/2) & (targets[:, :, 1] < BASE_HEIGHT/2)

        is_left_free = left_mask.sum(1) <= collision_point_threshold
        is_right_free = right_mask.sum(1) <= collision_point_threshold
        is_base_free = base_mask.sum(1) <= collision_point_threshold
        
        # D. 中间接触点与质量分布
        inner_mask = (targets[:, :, 2] > MIDDLE_OFFSET) & (targets[:, :, 2] < MIDDLE_OFFSET + MIDDLE_LENGTH) & \
                     (targets[:, :, 0] > -MIDDLE_WIDTH/2) & (targets[:, :, 0] < MIDDLE_WIDTH/2) & \
                     (targets[:, :, 1] > -MIDDLE_HEIGHT/2) & (targets[:, :, 1] < MIDDLE_HEIGHT/2)

        # 【核心新增】：记录接触点数
        grasp_contact_counts[i:end] = inner_mask.sum(dim=1).float()

        upper_y_mask = inner_mask & (targets[:, :, 1] > 0)
        lower_y_mask = inner_mask & (targets[:, :, 1] <= 0)
        left_x_mask = inner_mask & (targets[:, :, 0] < 0)
        right_x_mask = inner_mask & (targets[:, :, 0] >= 0)

        is_y_balanced = (upper_y_mask.sum(1) >= min_points_per_region) & (lower_y_mask.sum(1) >= min_points_per_region)
        is_x_balanced = (left_x_mask.sum(1) >= min_points_per_region) & (right_x_mask.sum(1) >= min_points_per_region)

        # 综合判定
        batch_valid = is_in_view & is_left_free & is_right_free & is_base_free & is_y_balanced & is_x_balanced
        valid_mask[i:end] = batch_valid

        # print("\n")
        # num_nonzero = torch.count_nonzero(is_in_view)
        # print("[INFO]: 碰撞检测，存在视野范围内的抓取个数：", num_nonzero)
        # num_nonzero = torch.count_nonzero(is_left_free)
        # print("[INFO]: 碰撞检测，左边夹爪无碰撞的抓取个数：", num_nonzero)
        # num_nonzero = torch.count_nonzero(is_right_free)
        # print("[INFO]: 碰撞检测，右边夹爪无碰撞的抓取个数：", num_nonzero)
        # num_nonzero = torch.count_nonzero(is_base_free)
        # print("[INFO]: 碰撞检测，夹爪基座无碰撞的抓取个数：", num_nonzero)
        # num_nonzero = torch.count_nonzero(is_y_balanced)
        # print("[INFO]: 碰撞检测，夹爪中部 y 方向接触平衡的抓取个数", num_nonzero)
        # num_nonzero = torch.count_nonzero(is_x_balanced)
        # print("[INFO]: 碰撞检测，夹爪中部 x 方向接触平衡的抓取个数", num_nonzero)
        # print("\n")

        if visualize:
            all_R_corr.append(R_corr.cpu().numpy())
            all_T_base.append(T_base.cpu().numpy())
            all_center_fixed.append(center_fixed.cpu().numpy())

    # --- 4. 排序逻辑 ---
    validity_np = valid_mask.cpu().numpy()
    counts_np = grasp_contact_counts.cpu().numpy()

    # 获取所有通过检测的索引
    valid_indices = np.where(validity_np)[0]
    
    if len(valid_indices) > 0:
        # 对有效索引按其对应的点数进行降序排列
        sorted_valid_indices = valid_indices[np.argsort(counts_np[valid_indices])[::-1]]
        
        # 构造一个新的返回掩码，只有排在最前面的有效抓取为 True
        # 或者按照你的逻辑，返回一个“排序后的有效列表”
        # 这里为了保持“返回掩码”的习惯，我们返回一个包含排序信息的索引数组
        # 但既然你要求“不修改返回值类型”，我会返回一个排序后的有效索引数组
        result = sorted_valid_indices
    else:
        result = np.array([], dtype=int)

    # --- 5. 可视化 (展示最佳有效抓取 vs 一个无效抓取) ---
    if visualize and gripper is not None:
        # 设置显示数量
        show_num_valid = 5  # 显示前3个最好的有效抓取
        show_num_invalid = 5 # 显示前2个无效抓取示例

        # 获取所有原始索引
        all_indices = np.arange(num_grasps)
        # 找出无效抓取的索引
        invalid_indices = np.setdiff1d(all_indices, result)

        # 准备基础几何体（场景点云和坐标轴）
        geometries = [pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)]
        
        # 提取变换数据
        R_all = np.concatenate(all_R_corr, axis=0)
        T_all = np.concatenate(all_T_base, axis=0)

        # 1. 可视化前 N 个最佳有效抓取 (绿色)
        num_to_show_v = min(len(result), show_num_valid)
        if num_to_show_v > 0:
            for i in range(num_to_show_v):
                best_idx = result[i] # result 已经是按质量排序好的
                transform_v = np.eye(4)
                transform_v[:3, :3] = R_all[best_idx]
                transform_v[:3, 3] = T_all[best_idx]
                for part in gripper:
                    g_part = copy.deepcopy(part)
                    g_part.transform(transform_v)
                    g_part.paint_uniform_color([0, 1, 0]) # 绿色
                    geometries.append(g_part)
            print(f"[VISUALIZE]: 绿色为前 {num_to_show_v} 个最佳有效抓取")

        # 2. 可视化前 M 个不可抓取姿态示例 (红色)
        num_to_show_inv = min(len(invalid_indices), show_num_invalid)
        if num_to_show_inv > 0:
            for i in range(num_to_show_inv):
                bad_idx = invalid_indices[i] 
                transform_inv = np.eye(4)
                transform_inv[:3, :3] = R_all[bad_idx]
                transform_inv[:3, 3] = T_all[bad_idx]
                for part in gripper:
                    g_part = copy.deepcopy(part)
                    g_part.transform(transform_inv)
                    g_part.paint_uniform_color([1, 0, 0]) # 红色
                    geometries.append(g_part)
            print(f"[VISUALIZE]: 红色为前 {num_to_show_inv} 个无效抓取示例")

        o3d.visualization.draw_geometries(geometries, window_name=f"Green: Top {num_to_show_v} Valid | Red: Top {num_to_show_inv} Invalid")

    print(f"[INFO]: 总抓取数: {num_grasps}, 合法数: {len(result)}")
    return result


def filter_vertical_grasps_simple(grasp_array, max_angle_degrees = 15):
    """
    简洁版本：过滤保留与z轴夹角小于指定角度的抓取
    """
    if grasp_array.shape[0] == 0:
        return np.array([]).reshape(0, 16)
    
    # 提取旋转矩阵的z轴向量
    rotations = grasp_array[:, 4:13].reshape(-1, 3, 3)
    z_axes = rotations[:, :, 2]
    
    # 归一化z轴向量
    z_norms = np.linalg.norm(z_axes, axis=1)
    z_axes_normalized = z_axes / z_norms[:, np.newaxis]
    
    # 计算与全局z轴的点积绝对值
    cos_angles = np.abs(z_axes_normalized[:, 2])  # 只取z分量，等价于点积
    
    # 判断是否满足角度要求
    cos_threshold = np.cos(np.deg2rad(max_angle_degrees))
    valid_mask = cos_angles >= cos_threshold
    
    return grasp_array[valid_mask]


def pose_6d_to_matrix(pose_6d):
    """
    将6D位姿转换为4x4齐次变换矩阵（外旋版本）
    
    参数:
        pose_6d: numpy数组，6D位姿 [x, y, z, roll, pitch, yaw]（单位：米，弧度）
                其中 roll, pitch, yaw 是外旋（Extrinsic）欧拉角
    
    返回:
        T: 4x4齐次变换矩阵
    
    注意：
        - 外旋XYZ顺序：旋转矩阵 R = R_x(roll) @ R_y(pitch) @ R_z(yaw)
        - 执行顺序（从右到左）：先应用yaw（绕固定Z轴），再应用pitch（绕固定Y轴），最后应用roll（绕固定X轴）
        - 每次旋转都是绕原始固定坐标系的轴
        - 这个版本使用外旋，与Grasp_Pose_to_6D的外旋欧拉角提取互为逆操作
    """
    x, y, z, roll, pitch, yaw = pose_6d

    # 外旋XYZ顺序：R = R_x(roll) @ R_y(pitch) @ R_z(yaw)
    # 参数顺序[roll, pitch, yaw]对应'xyz'中的[x, y, z]
    rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    R_matrix = rotation.as_matrix()

    T = np.eye(4)
    T[:3, :3] = R_matrix
    T[:3, 3] = np.array([x, y, z])
    
    return T


def depth_image2pcd(img, depth_img, camera_matrix, factor_depth, mask = None):
    """
    将深度图像反投影为点云（不对深度图进行平滑）
    
    参数:
        img: RGB图像 [H, W, 3]
        depth_img: 深度图像 [H, W]
        camera_matrix: 相机内参 [3, 3]
        factor_depth: 深度因子
        mask: 可选的mask [H, W]，如果提供则只生成mask为True的点云
    
    返回:
        pcd: Open3D点云对象（完整场景或mask区域）
    """
    # ========== 核心优化1：提前提取相机参数，减少重复索引 ==========
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    
    h, w = depth_img.shape
    
    # ========== 核心优化2：减少meshgrid内存占用 + 数据类型优化 ==========
    # 直接生成一维数组后广播，避免创建大的二维meshgrid数组
    u = np.arange(w, dtype=np.float32)  # [W,]
    v = np.arange(h, dtype=np.float32)  # [H,]
    
    # 计算Z（提前转换数据类型，避免重复astype）
    Z = depth_img.astype(np.float32) / factor_depth  # [H, W]
    
    # ========== 核心优化3：向量化广播计算，避免meshgrid ==========
    # 广播计算X = (u - cx) * Z / fx → [H, W]
    X = (u[np.newaxis, :] - cx) * Z / fx
    # 广播计算Y = (v - cy) * Z / fy → [H, W]
    Y = (v[:, np.newaxis] - cy) * Z / fy
    
    # ========== 核心优化4：合并过滤逻辑，减少内存拷贝 ==========
    # 第一步：过滤深度为0的点（保留Z>0.1）
    valid = Z > 0.1  # [H, W]
    
    # 第二步：如果有mask，合并过滤条件（原地操作，减少拷贝）
    if mask is not None:
        # 确保mask和depth_img形状一致，且转为bool类型
        valid &= mask.astype(bool)
    
    # ========== 核心优化5：一次性提取有效点，减少ravel次数 ==========
    # 直接从2D数组提取有效点，避免先展平再过滤（减少内存中间变量）
    points = np.empty((np.count_nonzero(valid), 3), dtype=np.float32)
    points[:, 0] = X[valid]
    points[:, 1] = Y[valid]
    points[:, 2] = Z[valid]
    
    # # ========== 核心优化6：颜色提取优化 ==========
    # # 直接提取有效区域的颜色，避免展平整个图像
    # colors = np.empty((len(points), 2), dtype=np.float32)
    # colors[:, 0] = img[valid, 0] / 255.0
    # colors[:, 1] = img[valid, 1] / 255.0
    # colors[:, 2] = img[valid, 2] / 255.0
    
    # ========== 保持原功能：转换为float64并创建点云 ==========
    # 仅在最后转换为float64（原代码要求），前面全程用float32
    points = np.ascontiguousarray(points, dtype = np.float64)
    # colors = np.ascontiguousarray(colors, dtype=np.float64)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    # pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def pcd_normalize(points):
    centroid = np.mean(points, axis = 0)
    points = points - centroid
    max_distance = np.max(np.sqrt(np.sum(points ** 2, axis = 1)))
    points = points / max_distance
    return points, centroid, max_distance


def flip_rotation_matrix(rotation_matrix):
    """
    翻转旋转矩阵（对Y和Z轴取负）（类似test.py的flip_rotation_matrix）
    
    功能：
        对旋转矩阵的Y和Z轴取负，相当于绕X轴旋转180度。
        用于获得与可视化一致的6D位姿（从俯视图看相同，但roll角度不同）。
    
    Args:
        rotation_matrix (np.ndarray): 旋转矩阵 (3x3)
        
    Returns:
        np.ndarray: 翻转后的旋转矩阵 (3x3)
    """
    return np.column_stack([
        rotation_matrix[:, 0],   # X轴保持不变
        -rotation_matrix[:, 1],  # Y轴取负
        -rotation_matrix[:, 2]   # Z轴取负
    ])


def convert_grasp_pose_to_6d(best_grasp, pose_name):
    """
    将抓取姿态转换为6自由度位姿（位置+角度），并打印
    
    参数:
        best_grasp: 抓取姿态数组
        pose_name: 位姿名称（用于打印）
    
    返回:
        6自由度位姿数组（位置单位：米，角度单位：弧度）
    """
    pose = GrasPose_to_6Degree_extrinsic(best_grasp)
    pose[3:6] = np.deg2rad(pose[3:6])  # 后三维（角度）转换为弧度
    return pose


def GrasPose_to_6Degree_extrinsic(grasp_pose):
    """
    将抓取姿态转换为6D位姿（使用外旋，固定轴旋转）
    
    参数:
        grasp_pose: numpy数组，抓取姿态，17维格式（网络直接输出）：
            [score, width, height, depth, rot_matrix_9, center_x, center_y, center_z, obj_id]
            - 索引：[0, 1, 2, 3, 4:13, 13:16, 16]
            - rot_matrix_9: 旋转矩阵（3x3，按行展开），必须是世界坐标系下的表示
    
    返回:
        grasp_6dpose: numpy数组，6D位姿 [x, y, z, roll, pitch, yaw]（单位：米，度）
                     其中 roll, pitch, yaw 是外旋角度（绕固定世界坐标系轴旋转）
    
    外旋 vs 内旋：
        - 内旋ZYX（GrasPose_to_6Degree使用）：
          * 先绕当前Z轴旋转yaw，再绕新的Y轴旋转pitch，最后绕新的X轴旋转roll
          * 每次旋转绕的是旋转后的坐标系轴
        - 外旋XYZ（本函数使用）：
          * 先绕固定世界坐标系X轴旋转roll，再绕固定Y轴旋转pitch，最后绕固定Z轴旋转yaw
          * 每次旋转绕的是固定的世界坐标系轴（X=[1,0,0], Y=[0,1,0], Z=[0,0,1]）
        
    重要：
        - 对于同一个旋转矩阵，外旋XYZ的角度值 ≠ 内旋ZYX的角度值（一般不同）
        - 例如：内旋ZYX得到 [rx=-179°, ry=22°, rz=-152°]
          - 外旋XYZ可能得到 [rx=169°, ry=19°, rz=150°]
        - 两者都能正确重建同一个旋转矩阵，只是角度值不同
    
    实现：
        - 使用 scipy.spatial.transform.Rotation.as_euler('xyz') 提取外旋角度
        - scipy的as_euler('xyz')返回的是外旋XYZ角度（固定轴旋转）
    
    注意：
        - 旋转矩阵必须是世界坐标系下的表示
        - 如果旋转矩阵是相对于其他坐标系（如相机坐标系），需要先转换到世界坐标系
    """
    # 17维格式：[score, width, height, depth, rot(9), center(3), obj_id]
    position = grasp_pose[13:16]  # [x, y, z]
    rot_matrix_flat = grasp_pose[4:13]  # rotation_matrix (9个元素)
    rot_matrix = rot_matrix_flat.reshape(3, 3)
    
    try:
        rotation = Rotation.from_matrix(rot_matrix)
        
        # 提取外旋XYZ角度（固定轴旋转）
        # scipy的as_euler('xyz')返回外旋角度：先绕固定X轴旋转roll，再绕固定Y轴旋转pitch，最后绕固定Z轴旋转yaw
        euler_xyz_extrinsic = rotation.as_euler('xyz', degrees=False)
        
        roll_extrinsic = euler_xyz_extrinsic[0]   # roll
        pitch_extrinsic = euler_xyz_extrinsic[1]  # pitch
        yaw_extrinsic = euler_xyz_extrinsic[2]    # yaw
        
    except ImportError:
        # 如果scipy不可用，手动提取外旋XYZ角度
        # 外旋XYZ：R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
        sy = np.sqrt(rot_matrix[0, 0] * rot_matrix[0, 0] + 
                    rot_matrix[1, 0] * rot_matrix[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            roll_extrinsic = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
            pitch_extrinsic = np.arctan2(-rot_matrix[2, 0], sy)
            yaw_extrinsic = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
        else:
            roll_extrinsic = np.arctan2(-rot_matrix[1, 2], rot_matrix[1, 1])
            pitch_extrinsic = np.arctan2(-rot_matrix[2, 0], sy)
            yaw_extrinsic = 0
    
    # 构建6D位姿（度）
    grasp_6dpose = np.array([
        position[0],
        position[1],
        position[2],
        np.rad2deg(roll_extrinsic),
        np.rad2deg(pitch_extrinsic),
        np.rad2deg(yaw_extrinsic)
    ], dtype=np.float64)
    
    return grasp_6dpose


def translate_grasp_point_along_direction(grasp_pose, distance):
    """
    将抓取点沿着抓取方向（Z轴方向）移动一定距离
    
    参数:
        grasp_pose: numpy数组，抓取姿态，17维格式（网络直接输出）：
            [score, width, height, depth, rot_matrix_9, center_x, center_y, center_z, obj_id]
            - rot_matrix_9: 旋转矩阵的9个元素（按行展开）
            - center_x, center_y, center_z: 抓取中心位置（米）
        distance: float，移动距离（米）
            - 正数：沿着Z轴正方向移动（远离物体）
            - 负数：沿着Z轴负方向移动（接近物体）
    
    返回:
        grasp_pose_translated: numpy数组，移动后的抓取姿态
    
    注意:
        - 只更新抓取点位置，旋转矩阵保持不变
        - 距离单位：米
    """
    grasp_pose_translated = grasp_pose.copy()
    
    # 17维格式：[score, width, height, depth, rot(9), center(3), obj_id]
    position = grasp_pose[13:16]  # [x, y, z]
    rot_matrix_flat = grasp_pose[4:13]  # rotation_matrix (9个元素)
    rot_matrix = rot_matrix_flat.reshape(3, 3)
    
    # 获取Z轴方向（接近方向，第3列）
    z_axis = rot_matrix[:, 2]
    
    # 沿着Z轴方向移动抓取点
    position_translated = position + distance * z_axis
    
    # 更新抓取姿态中的位置
    grasp_pose_translated[13:16] = position_translated
    
    return grasp_pose_translated


def adjust_gripper_orientation(Rx, Ry, Rz):
    """
    针对 Ry 处于 [-20, 20] 稳定区间优化的姿态调整
    """
    # 1. 转换并应用 180度翻转
    # 既然 Ry 很小，直接绕局部 Z 轴旋转是非常安全的
    if -90 < Rz <= 90:
        r_orig = Rotation.from_euler('xyz', [Rx, Ry, Rz], degrees=True)
        # 右乘 'z' 轴 180 度
        r_final = r_orig * Rotation.from_euler('z', 180, degrees=True)
        Rx_n, Ry_n, Rz_n = r_final.as_euler('xyz', degrees=True)
    else:
        Rx_n, Ry_n, Rz_n = Rx, Ry, Rz

    # 2. 强力区间规范化
    # 即使 as_euler 返回了等价解，我们也通过简单的数值平移将其拉回目标区间
    # 因为 Ry 在 0 附近，Rx 和 Rz 的等价变换公式非常稳定
    
    # 规范化到 [-180, 180]
    Rz_n = (Rz_n + 180) % 360 - 180
    
    # 如果此时 Rz_n 不幸落在了 (-90, 90]，利用对称性手动平移
    # 物理意义：夹爪是对称的，直接数值加减 180 度而不改变物理指向
    if -90 < Rz_n <= 90:
        if Rz_n > 0:
            Rz_n -= 180
        else:
            Rz_n += 180
        # 相应地补偿 Rx 才能保持姿态完全一致 (Rx' = Rx ± 180, Ry' = 180 - Ry)
        # 但注意：如果强行平移 Rz 而不改 Ry，Ry 可能会跳出 [-20, 20]
        # 所以最稳妥的做法是：如果 Ry 很小，直接让 Rz = Rz ± 180, Rx = Rx ± 180
        Rx_n = (Rx_n + 180) % 360 - 180

    return float(Rx_n), float(Ry_n), float(Rz_n)


def rotate_grasp_matrix_90_deg(rotation_matrix):
    """将抓取姿态的旋转矩阵沿着抓取法线旋转90度"""
    grasp_normal = rotation_matrix[:, 2]
    
    cos_theta = np.cos(np.pi / 2)
    sin_theta = np.sin(np.pi / 2)
    ux, uy, uz = grasp_normal
    
    rotation_90_deg = np.array([
        [cos_theta + ux**2 * (1 - cos_theta), ux * uy * (1 - cos_theta) - uz * sin_theta, ux * uz * (1 - cos_theta) + uy * sin_theta],
        [uy * ux * (1 - cos_theta) + uz * sin_theta, cos_theta + uy**2 * (1 - cos_theta), uy * uz * (1 - cos_theta) - ux * sin_theta],
        [uz * ux * (1 - cos_theta) - uy * sin_theta, uz * uy * (1 - cos_theta) + ux * sin_theta, cos_theta + uz**2 * (1 - cos_theta)]
    ])
    
    rotated_matrix = np.dot(rotation_90_deg, rotation_matrix)
    return rotated_matrix


def filter_pointcloud_by_xy(pcd, x_range = (-1, 1), y_range = (-1, 1), z_range = (-1, 1)):
    points = np.asarray(pcd.points)

    x_condition = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    y_condition = (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    z_condition = (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    mask = x_condition & y_condition & z_condition

    filtered_points = points[mask]
    pcd.points = o3d.utility.Vector3dVector(filtered_points)

    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_colors = colors[mask]
        pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        filtered_normals = normals[mask]
        pcd.normals = o3d.utility.Vector3dVector(filtered_normals)


def filter_grasp_by_z(best_grasp_array, z_min = 0.001):
    """
    根据z值过滤N*16的二维数组（每条数据的14号坐标为z值）
    
    参数:
        best_grasp_array: 二维数组(N*16)，输入可以是Python列表或numpy数组
        z_min: 可选，z值最小值（包含），过滤出z >= z_min的数据
        z_max: 可选，z值最大值（包含），过滤出z <= z_max的数据
        target_z: 可选，目标z值，精确匹配（带容差）
        tolerance: 可选，精确匹配时的容差，默认1e-6
    
    返回:
        numpy.ndarray: 过滤后的N*16二维数组
    """
    # 转换为numpy数组并验证维度
    grasp_arr = np.asarray(best_grasp_array)
    # 提取14号坐标的z值（索引13，Python从0开始）
    z_vals = grasp_arr[:, 15]
    
    # 构建过滤条件
    filter_mask = np.ones(len(z_vals), dtype=bool)
    if z_min is not None:
        filter_mask &= (z_vals >= z_min)
    
    # 执行过滤并返回结果
    return grasp_arr[filter_mask]


def draw_gripper(width = None, 
                 FINGER_WIDTH = None, FINGER_HEIGHT = None, FINGER_LENGTH = None,
                 MIDDLE_WIDTH = None, MIDDLE_HEIGHT = None, MIDDLE_LENGTH = None, MIDDLE_BEGIN_z = None,
                 BASE_WIDTH = None, BASE_HEIGHT = None, BASE_LENGTH = None,
                 color = [0.1, 0.5, 0.6]):
    
    gripper = []                                                # 创建几何体列表
    unified_y_center = 0.0                                      # 统一的Y轴中心坐标
    
                                                                # 1. 左手指 ========================================
    left_finger = o3d.geometry.TriangleMesh.create_box(
        width = FINGER_WIDTH,                                   # X方向：14mm
        height = FINGER_HEIGHT,                                 # Y方向：25mm
        depth = FINGER_LENGTH                                   # Z方向：60mm
    )

    left_finger.translate([
        -(width / 2) - FINGER_WIDTH / 2,                        # X: 左侧位置
        unified_y_center - FINGER_HEIGHT / 2,                   # Y: 中心在0，所以偏移-HEIGHT/2
        0                                                       # Z: 从原点开始
    ])

    left_finger.paint_uniform_color(color)
    left_finger.compute_vertex_normals()
    gripper.append(left_finger)
    
                                                                # 2. 右手指 ========================================
    right_finger = o3d.geometry.TriangleMesh.create_box(
        width = FINGER_WIDTH,                                   # X方向：14mm
        height = FINGER_HEIGHT,                                 # Y方向：25mm
        depth = FINGER_LENGTH                                   # Z方向：60mm
    )
    right_finger.translate([
        (width / 2) - FINGER_WIDTH / 2,                         # X: 右侧位置
        unified_y_center - FINGER_HEIGHT / 2,                   # Y: 中心在0
        0                                                       # Z: 从原点开始
    ])
    right_finger.paint_uniform_color(color)
    right_finger.compute_vertex_normals()
    gripper.append(right_finger)
    
                                                                # 3. 中间连接部分（用于保证抓取深度）===================
    middle_connection = o3d.geometry.TriangleMesh.create_box(
        width = MIDDLE_WIDTH,                                   # X方向：16mm (0.8cm*2)
        height = MIDDLE_HEIGHT,                                 # Y方向：16mm (0.8cm*2)
        depth = MIDDLE_LENGTH                                   # Z方向：40mm (2.0cm*2)
    )
    middle_connection.translate([
        -MIDDLE_WIDTH / 2,                                      # X: 居中于两手指之间
        unified_y_center - MIDDLE_HEIGHT / 2,                   # Y: 中心在0，与手指中心对齐
        MIDDLE_BEGIN_z                                          # Z: 从原点开始，与手指起点对齐
    ])
    middle_color = [0.6, 0.4, 0.2]  # 黄色
    middle_connection.paint_uniform_color(middle_color)
    middle_connection.compute_vertex_normals()
    gripper.append(middle_connection)
    
                                                                # 4. 基座 =========================================
    base = o3d.geometry.TriangleMesh.create_box(
        width = BASE_WIDTH,                                     # X方向：130mm
        height = BASE_HEIGHT,                                   # Y方向：74mm
        depth = BASE_LENGTH                                     # Z方向：120mm
    )
    base.translate([
        -BASE_WIDTH / 2,                                        # x: 居中
        -BASE_HEIGHT / 2,                                       # y: 居中
        -BASE_LENGTH                                            # Z: 从 -120mm 到 0
    ])
    base_color = [c * 0.6 for c in color]
    base.paint_uniform_color(base_color)
    base.compute_vertex_normals()
    gripper.append(base)
    
    
    return gripper


def remove_floor_points(pcd, plane_params, threshold=0.001, device='cuda' if torch.cuda.is_available() else 'cpu'):
    """
    直接处理 Open3D 点云对象，剔除属于地板的点。
    
    :param pcd: open3d.geometry.PointCloud 对象
    :param plane_params: 平面方程参数 (A, B, C, D)
    :param threshold: 距离阈值 (单位: m)
    :param device: 运算设备 ('cuda' 或 'cpu')
    """
    if not pcd.has_points():
        return pcd

    # 1. 提取坐标并转换为 Tensor
    points_np = np.asarray(pcd.points)
    points = torch.from_numpy(points_np).float().to(device)
    
    # 2. 准备平面参数
    A, B, C, D = plane_params
    normal = torch.tensor([A, B, C], device=device).float()
    
    # 3. 计算点到平面的距离: d = |P · N + D|
    # 由于是单位向量，不需要除以范数
    dist = torch.abs(torch.matmul(points, normal) + D)
    
    # 4. 生成掩码：保留距离大于阈值的点（非地板点）
    mask = dist > threshold
    
    # 5. 构建过滤后的点云
    # 将掩码转回 CPU 以便 Open3D 使用索引
    mask_cpu = mask.cpu().numpy()
    
    # 使用 Open3D 内置的 select_by_index 效率最高，因为它会自动处理颜色和法线
    indices = np.where(mask_cpu)[0]
    filtered_pcd = pcd.select_by_index(indices)
    
    # 清理显存 (如果是 GPU 运行)
    if device == 'cuda':
        del points, dist, mask
        torch.cuda.empty_cache()
        
    return filtered_pcd