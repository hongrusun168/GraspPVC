import copy
import time
import torch
import numpy as np
import open3d as o3d
from models.SimGraspNet_cluster import Sim_Grasp_Net

device = "cuda" if torch.cuda.is_available() else "cpu"

def sim_grasp_net_model(SimGrasp_checkpoint_path):
    """
    加载 SimGrasp 模型
    参数:
        SimGrasp_checkpoint_path: SimGrasp 模型路径
    返回:
        SimGraspNet: SimGrasp 模型
    """
    # 加载 simgraspnet 模型
    SimGraspNet = Sim_Grasp_Net(seed_feat_dim = 256, is_training = False)
    SimGraspNet.to(device)
    checkpoint = torch.load(SimGrasp_checkpoint_path, map_location = device)
    state_dict = checkpoint["model_state_dict"]
    state_dict = {k.replace("module.", ""): v for k, v in state_dict.items()}
    SimGraspNet.load_state_dict(state_dict)
    SimGraspNet.eval()
    return SimGraspNet


def pcd_normalize(pcd):
    """
    对点云进行归一化
    参数:
        pcd: 点云
    返回:
        pcd: 归一化后的点云
        centroid: 中心点
        m: 最大距离
    """
    centroid = np.mean(pcd, axis = 0)
    pcd = pcd - centroid
    m = np.max(np.sqrt(np.sum(pcd ** 2, axis = 1)))
    pcd = pcd / m
    return pcd, centroid, m


def get_and_process_SimData(pipes_pcd, visualize = False, camera_location = None):
    """
    对点云进行预处理
    参数:
        pipes_pcd: 点云
        visualize: 是否可视化点云，默认为 False
        camera_location: 相机位置 (3,) numpy数组，用于法线朝向一致性。如果为None，使用切平面一致性方法
    返回:
        end_points: 预处理后的点云
    """

    downsampled_pcd = pipes_pcd              
    
    if len(downsampled_pcd.points) > 100_000:
        downsampled_pcd = downsampled_pcd.uniform_down_sample(int(len(downsampled_pcd.points) / 100_000))

    # 估计法线
    downsampled_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))

    if len(downsampled_pcd.points) > 40000:
        downsampled_pcd = downsampled_pcd.uniform_down_sample(int(len(downsampled_pcd.points) / 40000))
    
    
    # 确保法线方向一致性，并朝向Z轴正方向
    z_axis_positive = np.array([0.0, 0.0, 1.0])
    
    # 获取所有法线
    normals = np.asarray(downsampled_pcd.normals)
    
    # 确保所有法线都朝向Z轴正方向
    # 如果法线与Z轴正方向的点积为负，则翻转该法线
    for i in range(len(normals)):
        dot_product = np.dot(normals[i], z_axis_positive)
        if dot_product < 0:
            normals[i] = -normals[i]
    
    # 更新点云的法线
    downsampled_pcd.normals = o3d.utility.Vector3dVector(normals)
    
    # 可视化预处理后的点云（可选）
    if visualize:
        print(f"点云预处理完成: 原始点数={len(pipes_pcd.points)}, 下采样后点数={len(downsampled_pcd.points)}")
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name = "点云预处理可视化", width = 1600, height = 1200)
        vis.add_geometry(downsampled_pcd)
        
        # 添加法线可视化（如果有法线数据）
        if downsampled_pcd.has_normals() and len(downsampled_pcd.normals) == len(downsampled_pcd.points):
            points = np.asarray(downsampled_pcd.points)
            normals = np.asarray(downsampled_pcd.normals)
            
            # 创建法线线段（只显示部分法线，避免过于拥挤）
            sample_rate = max(1, len(points) // 40000)  # 最多显示1000条法线
            sampled_points = points[::sample_rate]
            sampled_normals = normals[::sample_rate]
            
            # 法线长度
            normal_length = 0.02  # 2厘米
            
            # 创建法线线段集合
            normal_lines = []
            normal_points = []
            for i in range(len(sampled_points)):
                start = sampled_points[i]
                end = start + sampled_normals[i] * normal_length
                normal_points.append(start)
                normal_points.append(end)
                normal_lines.append([i * 2, i * 2 + 1])
            
            if len(normal_lines) > 0:
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(normal_points)
                line_set.lines = o3d.utility.Vector2iVector(normal_lines)
                # 法线颜色：绿色
                line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in range(len(normal_lines))])
                vis.add_geometry(line_set)
                print(f"显示 {len(sampled_points)} 条法线（共 {len(points)} 个点）")
        
        # 设置点云颜色和渲染选项
        render_option = vis.get_render_option()
        render_option.point_size = 2.0
        render_option.background_color = np.array([0.1, 0.1, 0.1])
        render_option.show_coordinate_frame = True
        
        # 添加坐标系
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1, origin = np.array([0, 0, 0]))
        vis.add_geometry(coord_frame)
        
        # 设置相机视角
        ctr = vis.get_view_control()
        ctr.set_zoom(0.8)
        
        print("点云可视化窗口已打开（绿色线段=法线），关闭窗口继续处理...")
        vis.run()  # 阻塞，直到窗口关闭
        vis.destroy_window()
    
    # 获取归一化点云，中心点，最大距离
    normalized_pcd, centroid, m = pcd_normalize(np.asarray(downsampled_pcd.points))

    end_points = dict()
    cloud_sampled = np.zeros((normalized_pcd.shape[0], 6))
    cloud_sampled[:, 0:3] = normalized_pcd
    cloud_sampled[:, 3:6] = np.asarray(downsampled_pcd.normals)
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    cloud_sampled = cloud_sampled.to(device)
    end_points["point_clouds"] = cloud_sampled
    end_points["centroid"] = centroid
    end_points["m"] = m

    return end_points


def create_tuning_fork_gripper(center, rotation_matrix, score, scale = 20.0):
    """
    创建音叉形状的夹爪模型（一个手柄连着两个手指，使用长方体条）
    参数:
        center: 抓取中心点
        rotation_matrix: 旋转矩阵 (3x3)
        score: 抓取质量得分
        scale: 缩放因子
    返回:
        gripper_geometries: 包含音叉所有部件的几何体列表
    """
    gripper_geometries = []
    
    if score > 0.8:
        color = [0, 1, 0]                                               # 绿色
    elif score > 0.6:
        color = [0.5, 1, 0]                                             # 黄绿色
    elif score > 0.4:
        color = [1, 1, 0]                                               # 黄色
    elif score > 0.2:
        color = [1, 0.5, 0]                                             # 橙色
    else:
        color = [1, 0, 0]                                               # 红色
    
    handle_length = 0.02 * scale                                        # 手柄长度
    handle_width = 0.0015 * scale                                       # 手柄宽度
    finger_length = 0.03 * scale                                        # 手指长度
    finger_width = 0.0012 * scale                                       # 手指宽度
    finger_spacing = 0.030 * scale                                      # 两个手指之间的距离（扩大）
    connector_width = 0.001 * scale                                     # 连接条宽度
    
                                                                        # 创建左手指（长方体条）- 手指从 z=0 向后延伸（z正方向）
    left_finger = o3d.geometry.TriangleMesh.create_box(
        width = finger_width,
        height = finger_width,
        depth = finger_length
    )
                                                                        # 左手指从末端 (z = 0) 向后延伸到 z = finger_length
    left_finger.translate([
        -finger_spacing / 2 - finger_width / 2,
        -finger_width / 2,
        0
    ])
    gripper_geometries.append(left_finger)

                                                                        # 创建右手指（长方体条）- 手指从 z = 0 向后延伸（z正方向）
    right_finger = o3d.geometry.TriangleMesh.create_box(
        width = finger_width,
        height = finger_width,
        depth = finger_length
    )

                                                                        # 右手指从末端 (z = 0) 向后延伸到 z = finger_length
    right_finger.translate([
        finger_spacing / 2 - finger_width / 2,
        -finger_width / 2,
        0
    ])
    gripper_geometries.append(right_finger)
    
                                                                        # 创建分叉点的横向连接条（长方体条）- 连接条在手指后端
    connector_length = finger_spacing + finger_width                    # 连接条长度
    connector = o3d.geometry.TriangleMesh.create_box(
        width = connector_length,
        height = connector_width,
        depth = connector_width
    )
                                                                        # 连接条在 z = finger_length 位置，连接两个手指的后端
    connector.translate([
        -connector_length / 2,
        -connector_width / 2,
        finger_length
    ])
    gripper_geometries.append(connector)
    
                                                                        # 创建手柄（长方体条）- 手柄从连接条继续向后延伸
    handle = o3d.geometry.TriangleMesh.create_box(
        width=handle_width,
        height=handle_width,
        depth=handle_length
    )
                                                                        # 手柄从 z = finger_length 向后延伸到 z = finger_length + handle_length
    handle.translate([-handle_width / 2, -handle_width / 2, finger_length])
    gripper_geometries.append(handle)
    
    for geometry in gripper_geometries:
        geometry.rotate(rotation_matrix, center = [0, 0, 0])
        geometry.translate(center)
        geometry.paint_uniform_color(color)
        geometry.compute_vertex_normals()
    
    return gripper_geometries


def visualize_grasps(gg_array, pcd, scale_factor = 20.0, top_percent = 4, top1_only = True, sort_or_not = True, show_grasp_points = True, show_grasp_directions = True, show_coordinate_frames = False):
    """
    可视化抓取姿态（使用音叉夹爪模型）
    参数:
        gg_array: 抓取数组
        pcd: 点云
        scale_factor: 音叉模型缩放因子，默认为 20.0
        top_percent: 显示的抓取姿态比例
        top1_only: 是否只显示一个抓取姿态
        sort_or_not: 是否排序
        show_grasp_points: 是否显示抓取点（红色球体），默认为 True
        show_grasp_directions: 是否显示抓取方向（蓝色箭头），默认为 True
        show_coordinate_frames: 是否显示抓取姿态的坐标轴，默认为 False
    """
    display_top_grasps(gg_array, pcd, top_percent = top_percent, scale_factor = scale_factor, 
                    top1_only = top1_only, show_grasp_points = show_grasp_points, 
                    show_grasp_directions = show_grasp_directions, show_coordinate_frames = show_coordinate_frames)


def _rotation_matrix_from_vectors(vec1, vec2):
    """
    计算从vec1旋转到vec2的旋转矩阵
    参数:
        vec1: 源向量
        vec2: 目标向量
    返回:
        rotation_matrix: 3x3旋转矩阵
    """
    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    
    if (s < 1e-6):  # 向量平行
        if (c > 0):
            return np.eye(3)
        else:
            return -np.eye(3)
    
    kmat = np.array([[0, -v[2], v[1]], 
                     [v[2], 0, -v[0]], 
                     [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat @ kmat * ((1 - c) / (s ** 2))
    return rotation_matrix


def display_top_grasps(sorted_grasps, pcd, top_percent, scale_factor = 20.0, top1_only = False, 
                       show_grasp_points = False, show_grasp_directions = False, show_coordinate_frames = False):
    """
    显示top_percent的抓取姿态（使用音叉夹爪模型）
    参数:
        sorted_grasps: 排序后的抓取数组
        pcd: 点云
        top_percent: 显示的抓取姿态比例
        scale_factor: 音叉模型缩放因子
        top1_only: 是否只显示一个抓取姿态
        show_grasp_points: 是否显示抓取点（红色球体）
        show_grasp_directions: 是否显示抓取方向（蓝色箭头）
    """
    # 计算要显示的抓取数量
    num_grasps = len(sorted_grasps)
    if top1_only:
        num_display_grasps = 5
    else:
        num_display_grasps = int(num_grasps * 100 / 100)

    print(f"显示 {num_display_grasps} 个抓取姿态（音叉夹爪模型）")

    # 准备可视化元素
    all_geometries = []      # 所有几何体
    
    for idx, grasp in enumerate(sorted_grasps[:num_display_grasps]):
        print (grasp.shape)
        score = grasp[0]
        depth = grasp[1] * 0.01
        # depth = 0
        rot = grasp[4:13].reshape(3, 3)
        center = grasp[13:16]
        
        # 沿着接近方向移动depth距离
        # rot[:, 2] 是接近方向（z轴）
        approach_direction = rot[:, 2]
        adjusted_center = center - approach_direction * depth

        print(f"  抓取 {idx+1}: score={score:.3f}, depth={depth:.3f}, center={center}, adjusted={adjusted_center}")
        
        # 创建音叉夹爪模型（使用调整后的center）
        tuning_fork_parts = create_tuning_fork_gripper(adjusted_center, rot, score, scale = scale_factor)
        all_geometries.extend(tuning_fork_parts)
        
        # 可选：添加抓取点标记（红色球体，更明显）
        if show_grasp_points:
            # 创建球体来标记抓取点（固定大小，不受scale_factor影响，便于观察）
            # 抓取点球体（红色）- 在调整后的位置（实际抓取位置）
            sphere_radius = 0.001  # 固定2厘米半径，清晰可见
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius = sphere_radius, resolution = 20)
            sphere.translate(adjusted_center)  # 使用调整后的位置
            sphere.paint_uniform_color([1, 0, 0])  # 红色
            sphere.compute_vertex_normals()
            all_geometries.append(sphere)
            
            # 在原始中心位置也添加一个标记（黄色），表示抓取中心
            center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius = sphere_radius, resolution = 20)
            center_sphere.translate(center)  # 使用原始中心位置
            center_sphere.paint_uniform_color([1, 1, 0])  # 黄色
            center_sphere.compute_vertex_normals()
            all_geometries.append(center_sphere)
        
        # 添加抓取方向箭头（蓝色）
        if show_grasp_directions:
            # 获取接近方向（旋转矩阵的z轴）- 已在上面定义
            # 箭头长度：至少10厘米，或根据scale_factor缩放（但不小于10厘米）
            arrow_length = max(0.10, 0.08 * scale_factor)  # 至少10厘米，清晰可见
            arrow_end = adjusted_center + approach_direction * arrow_length
            
            # 创建线段表示方向
            points = [adjusted_center, arrow_end]
            lines = [[0, 1]]
            colors = [[0, 0.5, 1]]  # 明亮的蓝色
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            all_geometries.append(line_set)
            
            # 创建箭头头部（圆锥）- 固定尺寸，清晰可见
            arrow_head_radius = max(0.008, 0.002 * scale_factor)  # 至少8毫米
            arrow_head_height = max(0.025, 0.015 * scale_factor)  # 至少2.5厘米
            arrow_head = o3d.geometry.TriangleMesh.create_cone(
                radius = arrow_head_radius, 
                height = arrow_head_height
            )
            # 计算旋转矩阵使箭头指向正确方向
            z_axis = np.array([0, 0, 1])
            rotation_matrix = _rotation_matrix_from_vectors(z_axis, approach_direction)
            arrow_head.rotate(rotation_matrix, center = np.array([0, 0, 0]))
            arrow_head.translate(arrow_end)
            arrow_head.paint_uniform_color([0, 0.5, 1])  # 明亮的蓝色
            all_geometries.append(arrow_head)

    # 创建可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name = "抓取姿态可视化 (音叉夹爪模型)")
    
    # 添加点云
    vis.add_geometry(pcd)
    
    # 添加世界坐标系（原点在(0,0,0)，单位：米）
    # 坐标轴长度：0.1米（10厘米），便于观察
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    vis.add_geometry(coord_frame)
    
    # 添加所有音叉夹爪几何体
    for geometry in all_geometries:
        vis.add_geometry(geometry)

    # 设置渲染选项
    render_option = vis.get_render_option()
    render_option.background_color = np.array([0.1, 0.1, 0.1])  # 深灰色背景

    # 运行可视化
    vis.run()
    vis.destroy_window()

