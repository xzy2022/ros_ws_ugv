import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

def evaluate_path_tracking(gt_file, wp_file, save_dir=None):
    # ---------------------------------------------------------
    # 1. 读取数据
    # ---------------------------------------------------------
    print(f"正在读取文件:\n - GT (真实轨迹): {gt_file}\n - WP (参考路径): {wp_file}")
    
    try:
        # 读取包含 vx, vy, vz 的数据
        df_gt = pd.read_csv(gt_file, sep=r'\s+', comment='#', 
                            names=['stamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'angular_v'])
        
        # 计算合成速度 linear_v = sqrt(vx^2 + vy^2 + vz^2)
        df_gt['linear_v'] = np.sqrt(df_gt['vx']**2 + df_gt['vy']**2 + df_gt['vz']**2)
        
        print(f"GT 数据加载成功，已计算合成速度。数据行数: {len(df_gt)}")

    except Exception as e:
        print(f"读取真实路径文件出错: {e}")
        return

    try:
        df_wp = pd.read_csv(wp_file, sep=r'\s+', comment='#', 
                            names=['id', 'x', 'y', 'z', 'yaw', 'target_linear_v'])
    except Exception as e:
        print(f"读取规划路径文件出错: {e}")
        return

    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)

    # ---------------------------------------------------------
    # 2. 计算误差
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
    
    # 将 matched_wp_indices 存入 dataframe 方便后续切片调用
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
    # 5. 绘图 (常规图 1-5)
    # ---------------------------------------------------------
    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    
    # --- 图1: 轨迹对比 ---
    plt.figure(figsize=(10, 8))
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path', linewidth=2, alpha=0.4)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1.5, alpha=0.6)
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=100, marker='*', label='Max Lat Error', zorder=10)
    
    target_timestamps = [5.0, 8.0, 13.0]
    marker_colors = ['purple', 'cyan', 'magenta']
    arrow_len = 1.5 
    
    for t_target, color in zip(target_timestamps, marker_colors):
        if len(gt_stamp) > 0:
            idx_closest = np.argmin(np.abs(gt_stamp - t_target))
            if np.abs(gt_stamp[idx_closest] - t_target) > 2.0: continue

            pt_gt = df_gt.iloc[idx_closest]
            idx_wp = matched_wp_indices[idx_closest]
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
    plt.plot(gt_stamp, df_gt['linear_v'].to_numpy(), 'b-', label='Actual Vel (Sqrt Sum)', linewidth=1.5, alpha=0.8)
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
    # 6. 新增: 0-5s 路径密度详细分析
    # ---------------------------------------------------------
    print("\n--- 正在生成 0-5s 路径密度分析图 ---")
    
    # 1. 筛选 0-5s 的真实数据
    mask_time = (gt_stamp >= 0.0) & (gt_stamp <= 5.0)
    df_gt_subset = df_gt[mask_time].copy()
    
    if len(df_gt_subset) == 0:
        print("警告: 0-5s 内没有找到真实轨迹数据，跳过图6绘制。")
    else:
        # 2. 获取这段时间内匹配到的目标点索引范围
        matched_indices_subset = df_gt_subset['matched_idx'].values
        min_wp_idx = np.min(matched_indices_subset)
        max_wp_idx = np.max(matched_indices_subset)
        
        # 为了画图好看，多取前后各 2 个点
        start_wp = max(0, min_wp_idx - 2)
        end_wp = min(len(df_wp) - 1, max_wp_idx + 2)
        
        df_wp_subset = df_wp.iloc[start_wp : end_wp + 1].copy()
        
        # 3. 计算平均距离 (密度)
        gt_xy_sub = df_gt_subset[['x', 'y']].values
        if len(gt_xy_sub) > 1:
            gt_diffs = np.linalg.norm(np.diff(gt_xy_sub, axis=0), axis=1)
            avg_gt_dist = np.mean(gt_diffs)
        else:
            avg_gt_dist = 0.0

        wp_core_subset = df_wp.iloc[min_wp_idx : max_wp_idx + 1][['x', 'y']].values
        if len(wp_core_subset) > 1:
            wp_diffs = np.linalg.norm(np.diff(wp_core_subset, axis=0), axis=1)
            avg_wp_dist = np.mean(wp_diffs)
        else:
            avg_wp_dist = 0.0

        print(f"0-5s区间分析:")
        print(f" - 目标路径(WP) 平均相邻点距: {avg_wp_dist:.4f} m")
        print(f" - 真实路径(GT) 平均相邻点距: {avg_gt_dist:.4f} m")
        print(f" - 密度比 (WP/GT): {avg_wp_dist/avg_gt_dist if avg_gt_dist>0 else 0:.2f}倍")

        # 4. 准备绘图颜色
        colors_cycle = ['red', 'green', 'blue']
        
        wp_indices_local = np.arange(len(df_wp_subset))
        wp_colors = [colors_cycle[i % 3] for i in wp_indices_local]
        
        gt_colors = []
        for global_idx in df_gt_subset['matched_idx']:
            local_idx = global_idx - start_wp
            if 0 <= local_idx < len(df_wp_subset):
                gt_colors.append(colors_cycle[local_idx % 3])
            else:
                gt_colors.append('black')

        # === 关键修改: 转换为 NumPy 数组以避免报错 ===
        wp_sub_x = df_wp_subset['x'].to_numpy()
        wp_sub_y = df_wp_subset['y'].to_numpy()
        gt_sub_x = df_gt_subset['x'].to_numpy()
        gt_sub_y = df_gt_subset['y'].to_numpy()

        # 5. 绘图
        plt.figure(figsize=(12, 10))
        
        # 绘制目标点 (x号)
        plt.scatter(wp_sub_x, wp_sub_y, c=wp_colors, marker='x', s=100, label='Target WP (Cycle R-G-B)', zorder=5)
        
        # 绘制真实点 (原点)
        plt.scatter(gt_sub_x, gt_sub_y, c=gt_colors, marker='o', s=40, label='Actual GT (Matched Color)', alpha=0.8, zorder=6)
        
        # 连线 (使用转换后的 numpy 数组)
        plt.plot(wp_sub_x, wp_sub_y, 'k:', alpha=0.3)
        plt.plot(gt_sub_x, gt_sub_y, 'k-', alpha=0.2)

        plt.title(f"Path Density Analysis (0-5s)\nAvg WP Dist: {avg_wp_dist:.3f}m | Avg GT Dist: {avg_gt_dist:.3f}m")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        
        if save_dir: plt.savefig(os.path.join(save_dir, "6_density_check_0_5s.png"), dpi=150)
        plt.close()

    print(f"所有图片已保存至: {save_dir}")

if __name__ == "__main__":
    # 示例文件路径
    gt_file = "data_process/v_10/pure_v.txt" 
    wp_file = "data_process/global_path_v_10.txt"
    
    save_dir = "result/v_10/pure_v"

    evaluate_path_tracking(gt_file, wp_file, save_dir)