import open3d as o3d
import numpy as np

def fit_plane_base_frame(ply_path):
    """
    在机械臂 Base 坐标系下拟合平面，并确保法向量朝上 (Z+)
    """
    # 1. 加载点云
    pcd = o3d.io.read_point_cloud(ply_path)
    original_pcd = copy.deepcopy(pcd) # 保留一份原始点云用于可视化
    
    # 采样以减轻计算压力
    if len(pcd.points) > 2000:
        pcd = pcd.uniform_down_sample(int(len(pcd.points) / 2000))
    
    points = np.asarray(pcd.points)
    if len(points) < 3:
        return None

    # 2. 最小二乘法拟合 (SVD)
    centroid = np.mean(points, axis=0)
    u, s, vh = np.linalg.svd(points - centroid)
    
    normal = vh[2, :]
    A, B, C = normal
    D = -np.dot(normal, centroid)

    # 3. 核心步骤：强制法向量朝上
    if C < 0:
        A, B, C, D = -A, -B, -C, -D
        normal = -normal

    dist = np.abs(np.dot(points, [A, B, C]) + D)
    rmse = np.sqrt(np.mean(dist**2))

    return {
        "equation": (A, B, C, D),
        "normal": normal,
        "centroid": centroid,
        "rmse": rmse,
        "pcd": original_pcd,  # 返回点云对象用于可视化
        "points": points      # 返回采样后的点
    }

def visualize_plane(res):
    if not res:
        return

    A, B, C, D = res["equation"]
    centroid = res["centroid"]
    normal = res["normal"]
    pcd = res["pcd"]

    # --- 创建平面模型 (可视化用) ---
    # 根据点云的包围盒计算平面的尺寸
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    size = max(extent[0], extent[1]) * 1.2 # 比点云范围稍微大一点

    # 创建一个平面网格
    mesh_plane = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=0.001)
    mesh_plane.paint_uniform_color([0.8, 0.8, 0.8]) # 灰色平面
    mesh_plane.translate([-size/2, -size/2, 0]) # 居中

    # 计算将 [0,0,1] 旋转到 normal 的旋转矩阵
    z_axis = np.array([0, 0, 1])
    # 计算旋转轴
    rot_axis = np.cross(z_axis, normal)
    rot_axis_norm = np.linalg.norm(rot_axis)
    
    if rot_axis_norm > 1e-6:
        rot_axis /= rot_axis_norm
        theta = np.arccos(np.dot(z_axis, normal))
        # 使用 Rodrigues 公式获取旋转矩阵
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(rot_axis * theta)
        mesh_plane.rotate(R, center=[0, 0, 0])

    # 移动到质心位置（由于 D 的存在，需微调 Z 确保在平面上）
    mesh_plane.translate(centroid)

    # --- 创建法向量箭头 ---
    arrow = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=centroid)
    # 或者创建一个专门指向法向的箭头
    # arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.002, cone_radius=0.004, 
    #                                               cylinder_height=0.04, cone_height=0.01)
    # ... 旋转逻辑同上 ...

    # 设置点云颜色 (内点绿色)
    pcd.paint_uniform_color([0, 0.6, 0]) 

    # 坐标系参考
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    print("[INFO]: 正在打开可视化窗口...")
    o3d.visualization.draw_geometries([pcd, mesh_plane, coord], 
                                      window_name="Plane Fit Visualization",
                                      mesh_show_back_face=True)

import copy # 引入 copy 库

# --- 执行 ---
res = fit_plane_base_frame("PVC_plane.ply")

if res:
    A, B, C, D = res["equation"]
    print(f"拟合结果:")
    print(f"方程: {A:.6f}x + {B:.6f}y + {C:.6f}z + {D:.6f} = 0")
    print(f"RMSE: {res['rmse'] * 1000:.3f} mm")
    
    # 开启可视化
    visualize_plane(res)