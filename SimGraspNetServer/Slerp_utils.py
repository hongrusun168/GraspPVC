import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import CubicSpline

def generate_robust_trajectory(start_point, target_pose, num_points=100):
    """
    输入：
    start_point: [x, y, z] 起始位置
    target_pose: [x, y, z, rx, ry, rz] 目标位姿
    
    约束：目标点的 Z 轴朝向为圆弧在终点处的切线方向
    """
    P1 = np.array(start_point)
    P2 = np.array(target_pose[:3])
    ori2 = np.array(target_pose[3:])

    # 1. 提取旋转矩阵，并取 Z 轴（第三列）作为切线 T2
    rot2_obj = Rotation.from_euler('xyz', ori2)
    rot2_mat = rot2_obj.as_matrix()
    T2 = rot2_mat[:, 2] # 目标点的 Z 轴
    T2 /= np.linalg.norm(T2)

    V = P1 - P2
    dist = np.linalg.norm(V)

    # 2. 奇异性检测（直线情况）
    cross_vt = np.cross(V, T2)
    if np.linalg.norm(cross_vt) / dist < 1e-4:
        print("检测到共线：路径退化为直线")
        trajectory = np.array([P1 + (P2 - P1) * t for t in np.linspace(0, 1, num_points)])
        mid_pos = (P1 + P2) / 2.0
        return trajectory, np.concatenate([mid_pos, ori2]), {"type": "line"}

    # 3. 圆弧几何计算
    # 得到圆平面法向（单位化）
    plane_normal = cross_vt / np.linalg.norm(cross_vt)
    
    # 径向向量 N：在圆平面内且垂直于切线 T2
    radial_N = np.cross(plane_normal, T2)
    radial_N /= np.linalg.norm(radial_N)
    
    # 计算半径 R (根据等腰三角形 P1-C-P2)
    radius = np.dot(V, V) / (2 * np.dot(V, radial_N))
    C = P2 + radius * radial_N
    abs_R = abs(radius)

    # 4. 建立圆平面局部坐标系进行角度计算
    u = (P2 - C) / abs_R # 径向作为局部 X
    v = np.cross(plane_normal, u) # 切向作为局部 Y

    vec_cp1 = (P1 - C) / abs_R
    angle_start = np.arctan2(np.dot(vec_cp1, v), np.dot(vec_cp1, u))

    # 5. 计算中点的 6-DoF 位姿
    angle_mid = angle_start / 2.0
    # 中点位置
    mid_pos = C + abs_R * (np.cos(angle_mid) * u + np.sin(angle_mid) * v)
    
    # 中点姿态：将终点旋转矩阵绕着【圆平面法向量】旋转 angle_mid
    # 注意：angle_mid 是从终点向起点回溯的角度
    rot_mid_offset = Rotation.from_rotvec(plane_normal * angle_mid)
    mid_rot_mat = rot_mid_offset.as_matrix() @ rot2_mat
    mid_ori = Rotation.from_matrix(mid_rot_mat).as_euler('xyz')
    
    mid_6dof = np.concatenate([mid_pos, mid_ori])

    # 6. 生成完整轨迹点
    theta = np.linspace(angle_start, 0, num_points)
    trajectory = np.array([C + abs_R * (np.cos(t) * u + np.sin(t) * v) for t in theta])
    
    return trajectory, mid_6dof, {"center": C, "radius": abs_R, "plane_normal": plane_normal}


def plot_trajectory(p1, p2_pose, trajectory, info):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    p2 = np.array(p2_pose[:3])

    # 绘制路径
    color = 'purple' if info['type'] == 'arc' else 'orange'
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color=color, linewidth=3, label=f"Path ({info['type']})")

    # 绘制关键点
    ax.scatter(*p1, color='blue', s=100, label='P1 (Start)')
    ax.scatter(*p2, color='red', s=100, label='P2 (Target)')
    
    if info['center'] is not None:
        ax.scatter(*info['center'], color='green', marker='X', s=100, label='Center')
        # 绘制半径虚线
        ax.plot([info['center'][0], p1[0]], [info['center'][1], p1[1]], [info['center'][2], p1[2]], 'k--', alpha=0.2)
        ax.plot([info['center'][0], p2[0]], [info['center'][1], p2[1]], [info['center'][2], p2[2]], 'k--', alpha=0.2)

    # 绘制切线
    t_len = np.linalg.norm(p2-p1) * 0.3 if info['type'] == 'line' else info['radius'] * 0.5
    t_vec = info['tangent'] * t_len
    ax.quiver(p2[0], p2[1], p2[2], t_vec[0], t_vec[1], t_vec[2], color='red', label='Tangent at P2')

    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.legend()
    
    # 统一比例尺
    all_pts = np.vstack([p1, p2, info['center']]) if info['center'] is not None else np.vstack([p1, p2])
    max_range = np.ptp(all_pts, axis=0).max() / 2
    mid = (all_pts.max(axis=0) + all_pts.min(axis=0)) / 2
    ax.set_xlim(mid[0]-max_range, mid[0]+max_range)
    ax.set_ylim(mid[1]-max_range, mid[1]+max_range)
    ax.set_zlim(mid[2]-max_range, mid[2]+max_range)
    
    plt.show()


def get_middle_pose(pose1, pose2):
    """
    pose1, pose2 格式为 [x, y, z, Rx, Ry, Rz] (单位：弧度)
    """
    # 1. 提取位置并线性插值
    pos1 = np.array(pose1[:3])
    pos2 = np.array(pose2[:3])
    mid_pos = (pos1 + pos2) / 2

    # 2. 提取姿态并进行 SLERP 插值
    # 注意：需根据你机器人的欧拉角顺序设置（AUBO 通常为 ZYX 或 XYZ，此处以 XYZ 为例）
    rot1 = Rotation.from_euler('xyz', pose1[3:])
    rot2 = Rotation.from_euler('xyz', pose2[3:])
    
    # 使用 Slerp 插件，t=0.5 表示中点
    from scipy.spatial.transform import Slerp
    key_times = [0, 1]
    key_rots = Rotation.from_quat([rot1.as_quat(), rot2.as_quat()])
    slerp = Slerp(key_times, key_rots)
    mid_rot = slerp(0.5).as_euler('xyz')

    # 3. 合并结果
    return np.concatenate([mid_pos, mid_rot]).tolist()




def smooth_6dof_coupled_planner(waypoints, num_points=400):
    """
    耦合路径规划（全弧度制输入）
    
    :param waypoints: 形状为 (N, 6) 的数组，姿态部分 [rx, ry, rz] 必须是弧度
    :param num_points: 插值总点数
    """
    waypoints = np.array(waypoints)
    n_waypoints = len(waypoints)
    
    if n_waypoints < 2:
        raise ValueError("至少需要两个点才能进行插值")

    # 1. 位置插值 (CubicSpline)
    pos = waypoints[:, 0:3]
    t_nodes = np.linspace(0, 1.0, n_waypoints)
    t_interp = np.linspace(0, 1.0, num_points)
    
    # 使用 'clamped' 边界条件确保起点和终点速度为 0
    cs_pos = CubicSpline(t_nodes, pos, bc_type='clamped')
    interp_pos = cs_pos(t_interp)

    # 2. 计算路径切线（一阶导数）来决定 Z 轴指向
    derivatives = cs_pos.derivative()(t_interp)
    
    interp_euler = []
    for i in range(num_points):
        tangent = derivatives[i]
        norm = np.linalg.norm(tangent)
        
        # 处理静止点（如起点/终点速度为0时）
        if norm < 1e-8:
            # 尝试使用邻近点的切线，或者直接沿用输入点的原始姿态
            if i < n_waypoints:
                 # 初始点使用输入位姿的弧度值
                 interp_euler.append(waypoints[0, 3:6])
            else:
                 interp_euler.append(interp_euler[-1])
            continue

        # 归一化切线作为 Z 轴 (工具朝向)
        z_axis = tangent / norm
        
        # 3. 构建正交坐标系（姿态耦合逻辑）
        # 选择一个参考向量来构建 X 和 Y 轴
        ref_vec = np.array([0, 1, 0])
        if abs(np.dot(z_axis, ref_vec)) > 0.98:
            ref_vec = np.array([1, 0, 0])
            
        x_axis = np.cross(ref_vec, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        # 组装旋转矩阵 (注意：这里直接生成弧度欧拉角)
        rot_matrix = np.column_stack((x_axis, y_axis, z_axis))
        # 'xyz' 对应 AUBO 的欧拉角约定，输出默认是弧度
        euler = Rotation.from_matrix(rot_matrix).as_euler('xyz')
        interp_euler.append(euler)

    # 合并 [x, y, z, rx, ry, rz] (全部为米和弧度)
    trajectory = np.hstack((interp_pos, np.array(interp_euler)))
    return trajectory

def visualize_trajectory(trajectory, stride=20):
    """
    可视化 6DOF 轨迹（弧度制数据验证）
    """
    traj = np.array(trajectory)
    pos = traj[:, 0:3]
    orient = traj[:, 3:6] # 这里已经是弧度

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制轨迹线
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'k--', alpha=0.4, label='Path')

    axis_length = 0.04 
    for i in range(0, len(traj), stride):
        # Rotation 直接接收弧度
        r = Rotation.from_euler('xyz', orient[i])
        rot_matrix = r.as_matrix()

        # 提取轴向量
        x_axis = rot_matrix[:, 0] * axis_length
        y_axis = rot_matrix[:, 1] * axis_length
        z_axis = rot_matrix[:, 2] * axis_length # 重点观察轴

        cp = pos[i]
        # 绘制末端坐标系
        ax.quiver(cp[0], cp[1], cp[2], x_axis[0], x_axis[1], x_axis[2], color='r', alpha=0.3)
        ax.quiver(cp[0], cp[1], cp[2], y_axis[0], y_axis[1], y_axis[2], color='g', alpha=0.3)
        # 加粗 Z 轴（蓝色），验证其是否始终沿着轨迹的切线方向
        ax.quiver(cp[0], cp[1], cp[2], z_axis[0], z_axis[1], z_axis[2], color='b', linewidth=2, alpha=0.8)

    # 坐标轴比例自适应
    max_range = np.array([pos[:,0].max()-pos[:,0].min(), 
                          pos[:,1].max()-pos[:,1].min(), 
                          pos[:,2].max()-pos[:,2].min()]).max() / 2.0
    mid = np.mean(pos, axis=0)
    ax.set_xlim(mid[0]-max_range, mid[0]+max_range)
    ax.set_ylim(mid[1]-max_range, mid[1]+max_range)
    ax.set_zlim(mid[2]-max_range, mid[2]+max_range)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("Coupled Planner (Radians): Z-axis aligns with motion direction")
    plt.show()