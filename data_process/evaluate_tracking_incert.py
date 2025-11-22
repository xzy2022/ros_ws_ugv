import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# ==========================================
# 新增: 路径插值函数
# ==========================================
def interpolate_path(df, step_size=0.01):
    """
    对路径进行线性插值，使其更加密集
    :param df: 原始路径 DataFrame
    :param step_size: 插值间隔 (米), 默认 1cm
    :return: 加密后的 DataFrame
    """
    # 1. 计算原始路径的累积距离 (s)
    x = df['x'].values
    y = df['y'].values
    # 计算相邻点距离
    diffs = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    # 累积距离 (第一个点是0)
    dist_cum = np.cumsum(np.append(0, diffs))
    
    # 2. 生成新的密集距离序列
    total_dist = dist_cum[-1]
    # 生成从0到总长度，步长为 step_size 的数组
    dist_dense = np.arange(0, total_dist, step_size)
    
    # 3. 插值
    df_dense = pd.DataFrame()
    
    # 线性插值 x, y, z, linear_v
    df_dense['x'] = np.interp(dist_dense, dist_cum, df['x'])
    df_dense['y'] = np.interp(dist_dense, dist_cum, df['y'])
    df_dense['z'] = np.interp(dist_dense, dist_cum, df['z'])
    df_dense['target_linear_v'] = np.interp(dist_dense, dist_cum, df['target_linear_v'])
    
    # 角度插值需要特殊处理 (解缠绕 -> 插值 -> 缠绕)
    yaw_unwrapped = np.unwrap(df['yaw'].values)
    yaw_dense = np.interp(dist_dense, dist_cum, yaw_unwrapped)
    # 归一化回 -pi 到 pi
    df_dense['yaw'] = (yaw_dense + np.pi) % (2 * np.pi) - np.pi
    
    return df_dense

def evaluate_path_tracking(gt_file, wp_file, save_dir=None):
    # ---------------------------------------------------------
    # 1. 读取数据
    # ---------------------------------------------------------
    print(f"正在读取文件:\n - GT (真实轨迹): {gt_file}\n - WP (参考路径): {wp_file}")
    
    try:
        df_gt = pd.read_csv(gt_file, sep=r'\s+', comment='#', 
                            names=['stamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'angular_v'])
        # 计算合成速度
        df_gt['linear_v'] = np.sqrt(df_gt['vx']**2 + df_gt['vy']**2 + df_gt['vz']**2)
    except Exception as e:
        print(f"读取真实路径文件出错: {e}")
        return

    try:
        df_wp_raw = pd.read_csv(wp_file, sep=r'\s+', comment='#', 
                            names=['id', 'x', 'y', 'z', 'yaw', 'target_linear_v'])
        
        # === 核心修改: 对参考路径进行插值 ===
        print(f"原始参考路径点数: {len(df_wp_raw)} (平均间距约1m)")
        print("正在进行路径插值加密 (step=0.01m)...")
        df_wp = interpolate_path(df_wp_raw, step_size=0.01)
        print(f"加密后参考路径点数: {len(df_wp)}")
        
    except Exception as e:
        print(f"读取规划路径文件出错: {e}")
        return

    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)

    # ---------------------------------------------------------
    # 2. 计算误差 (现在使用的是加密后的 df_wp)
    # ---------------------------------------------------------
    wp_xy = df_wp[['x', 'y']].values
    tree = KDTree(wp_xy)

    lateral_errors = []
    heading_errors = []
    velocity_errors = []
    matched_ref_vels = []
    matched_wp_indices = []

    for index, row in df_gt.iterrows():
        gt_x = row['x']
        gt_y = row['y']
        gt_yaw = row['yaw']
        gt_v = row['linear_v']

        # 查询最近点 (因为现在点很密，点到点的距离 近似等于 点到线的距离)
        dist, idx = tree.query([gt_x, gt_y])

        lateral_errors.append(dist)
        matched_wp_indices.append(idx)

        ref_yaw = df_wp.iloc[idx]['yaw']
        yaw_err = gt_yaw - ref_yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi  
        heading_errors.append(yaw_err)

        ref_v = df_wp.iloc[idx]['target_linear_v']
        vel_err = gt_v - ref_v
        
        velocity_errors.append(vel_err)
        matched_ref_vels.append(ref_v)

    df_gt['lateral_error'] = lateral_errors
    df_gt['heading_error'] = heading_errors
    df_gt['velocity_error'] = velocity_errors
    df_gt['ref_velocity'] = matched_ref_vels
    df_gt['matched_idx'] = matched_wp_indices

    lateral_errors = np.array(lateral_errors)
    heading_errors = np.array(heading_errors)
    velocity_errors = np.array(velocity_errors)
    matched_wp_indices = np.array(matched_wp_indices)

    # 统计指标
    rmse_lat = np.sqrt(np.mean(np.square(lateral_errors)))
    max_lat = np.max(np.abs(lateral_errors))
    max_error_idx = np.argmax(np.abs(lateral_errors))
    bad_gt_point = df_gt.iloc[max_error_idx]

    # ---------------------------------------------------------
    # 5. 绘图
    # ---------------------------------------------------------
    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    
    # --- 图1: 轨迹对比 ---
    plt.figure(figsize=(10, 8))
    # 绘制加密后的路径作为参考线
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path (Interp)', linewidth=2, alpha=0.4)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1.5, alpha=0.6)
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=100, marker='*', label='Max Lat Error', zorder=10)
    
    # 这里的箭头绘制逻辑依然有效，但 idx 现在对应的是加密后的数组
    target_timestamps = [5.0, 8.0, 13.0]
    marker_colors = ['purple', 'cyan', 'magenta']
    arrow_len = 1.5 
    
    for t_target, color in zip(target_timestamps, marker_colors):
        if len(gt_stamp) > 0:
            idx_closest = np.argmin(np.abs(gt_stamp - t_target))
            if np.abs(gt_stamp[idx_closest] - t_target) > 2.0: continue

            pt_gt = df_gt.iloc[idx_closest]
            idx_wp = matched_wp_indices[idx_closest] # 这是在 dense 数组中的索引
            pt_wp = df_wp.iloc[idx_wp]
            
            gt_yaw_rad = pt_gt['yaw']
            gt_dx, gt_dy = arrow_len * np.cos(gt_yaw_rad), arrow_len * np.sin(gt_yaw_rad)
            wp_yaw_rad = pt_wp['yaw']
            wp_dx, wp_dy = arrow_len * np.cos(wp_yaw_rad), arrow_len * np.sin(wp_yaw_rad)
            
            plt.arrow(pt_gt['x'], pt_gt['y'], gt_dx, gt_dy, head_width=0.4, fc=color, ec=color, linewidth=2, zorder=15)
            plt.arrow(pt_wp['x'], pt_wp['y'], wp_dx, wp_dy, head_width=0.3, fc='none', ec=color, linestyle=':', linewidth=1.5, zorder=15)
            plt.plot([pt_gt['x'], pt_wp['x']], [pt_gt['y'], pt_wp['y']], color=color, linestyle='--', linewidth=1, zorder=14)
            plt.text(pt_gt['x']+0.5, pt_gt['y'], f"{t_target}s", color=color, fontsize=10, fontweight='bold')

    plt.title("Trajectory & Heading Visualization")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "1_trajectory.png"), dpi=150)
    plt.close()

    # --- 图2: 横向误差 ---
    plt.figure(figsize=(8, 4))
    plt.plot(gt_stamp, np.abs(lateral_errors), 'g-', label='Lateral Error')
    plt.axhline(y=rmse_lat, color='r', linestyle=':', label=f'RMSE: {rmse_lat:.3f}m')
    plt.title(f"Lateral Error (Max: {max_lat:.3f}m)")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "2_lateral_error.png"), dpi=150)
    plt.close()

    # --- 图3: 航向误差 ---
    plt.figure(figsize=(8, 4))
    plt.plot(gt_stamp, np.degrees(heading_errors), 'm-')
    plt.title("Heading Error over Time [deg]")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [deg]")
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "3_heading_error.png"), dpi=150)
    plt.close()

    # --- 图4: 速度追踪 ---
    plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    plt.plot(gt_stamp, df_gt['ref_velocity'].to_numpy(), 'r--', label='Target Vel', linewidth=2)
    plt.plot(gt_stamp, df_gt['linear_v'].to_numpy(), 'b-', label='Actual Vel', linewidth=1.5, alpha=0.8)
    plt.title("Velocity Tracking Performance")
    plt.ylabel("Velocity [m/s]")
    plt.legend()
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(gt_stamp, velocity_errors, 'k-', label='Vel Error')
    plt.axhline(0, color='gray', linestyle='--')
    plt.ylabel("Error [m/s]")
    plt.xlabel("Timestamp [s]")
    plt.grid(True)
    plt.tight_layout()
    if save_dir: plt.savefig(os.path.join(save_dir, "4_velocity_tracking.png"), dpi=150)
    plt.close()

    # --- 图5: 直方图 ---
    plt.figure(figsize=(8, 4))
    plt.hist(lateral_errors, bins=20, color='gray', alpha=0.7)
    plt.title("Lateral Error Distribution")
    plt.xlabel("Error [m]")
    plt.ylabel("Count")
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "5_lateral_error_hist.png"), dpi=150)
    plt.close()

    # ---------------------------------------------------------
    # 6. 新增: 0-5s 路径密度分析 (为了对比，我们这里同时显示 原始稀疏点 和 真实轨迹)
    # ---------------------------------------------------------
    print("\n--- 正在生成 0-5s 路径密度分析图 ---")
    
    mask_time = (gt_stamp >= 0.0) & (gt_stamp <= 5.0)
    df_gt_subset = df_gt[mask_time].copy()
    
    if len(df_gt_subset) == 0:
        print("警告: 0-5s 内没有找到真实轨迹数据，跳过图6绘制。")
    else:
        # 注意: 这里的 matched_idx 已经是 dense 的索引了，不能直接用来切片 df_wp_raw
        # 为了图6的可视化，我们通过坐标范围来找 df_wp_raw 中的点，而不是通过索引
        
        gt_subset_x = df_gt_subset['x'].values
        gt_subset_y = df_gt_subset['y'].values
        
        min_x, max_x = np.min(gt_subset_x), np.max(gt_subset_x)
        min_y, max_y = np.min(gt_subset_y), np.max(gt_subset_y)
        
        # 扩大一点搜索范围
        margin = 2.0
        wp_raw_subset = df_wp_raw[
            (df_wp_raw['x'] > min_x - margin) & (df_wp_raw['x'] < max_x + margin) &
            (df_wp_raw['y'] > min_y - margin) & (df_wp_raw['y'] < max_y + margin)
        ].copy()

        # 绘图
        wp_sub_x = wp_raw_subset['x'].to_numpy()
        wp_sub_y = wp_raw_subset['y'].to_numpy()
        gt_sub_x = df_gt_subset['x'].to_numpy()
        gt_sub_y = df_gt_subset['y'].to_numpy()

        plt.figure(figsize=(12, 10))
        
        # 画原始稀疏点 (红色 X)
        plt.scatter(wp_sub_x, wp_sub_y, c='red', marker='x', s=150, label='Original Sparse WP (1m gap)', zorder=5)
        
        # 画插值后的密集路径 (灰色细线)
        # 同样为了性能，只画这一段区域的插值线
        df_dense_subset = df_wp[
             (df_wp['x'] > min_x - margin) & (df_wp['x'] < max_x + margin)
        ]
        plt.plot(df_dense_subset['x'].to_numpy(), df_dense_subset['y'].to_numpy(), 'k-', alpha=0.3, label='Interpolated Path (Used for Error Calc)')

        # 画真实轨迹
        plt.scatter(gt_sub_x, gt_sub_y, c='blue', marker='.', s=20, label='GT Path', zorder=6)
        
        plt.title(f"Impact of Upsampling on Error Calculation (0-5s)")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        
        if save_dir: plt.savefig(os.path.join(save_dir, "6_density_check_improved.png"), dpi=150)
        plt.close()

    print(f"所有图片已保存至: {save_dir}")

if __name__ == "__main__":
    gt_file = "data_process/v_20/pure_v.txt" 
    wp_file = "data_process/global_path_v_20.txt"
    save_dir = "result/v_20/pure_v"
    evaluate_path_tracking(gt_file, wp_file, save_dir)