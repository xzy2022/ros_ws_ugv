import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

def evaluate_path_tracking(gt_file, wp_file, save_dir=None):
    # ---------------------------------------------------------
    # 1. 读取数据 (适配新的 TXT 格式)
    # ---------------------------------------------------------
    print(f"正在读取文件:\n - GT (真实轨迹): {gt_file}\n - WP (参考路径): {wp_file}")
    
    try:
        # === 修改 1: 更新列名以包含 vx, vy, vz ===
        df_gt = pd.read_csv(gt_file, sep=r'\s+', comment='#', 
                            names=['stamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'angular_v'])
        
        # === 修改 2: 计算合成速度 (Scalar Speed) ===
        # linear_v = sqrt(vx^2 + vy^2 + vz^2)
        df_gt['linear_v'] = np.sqrt(df_gt['vx']**2 + df_gt['vy']**2 + df_gt['vz']**2)
        
        print(f"GT 数据加载成功，已计算合成速度。数据行数: {len(df_gt)}")

    except Exception as e:
        print(f"读取真实路径文件出错: {e}")
        return

    try:
        # WP 文件格式保持不变
        df_wp = pd.read_csv(wp_file, sep=r'\s+', comment='#', 
                            names=['id', 'x', 'y', 'z', 'yaw', 'target_linear_v'])
    except Exception as e:
        print(f"读取规划路径文件出错: {e}")
        return

    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)

    # ---------------------------------------------------------
    # 2. 计算误差 (逻辑保持不变，此时 df_gt 中已有 linear_v)
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
        gt_v = row['linear_v'] # 这里使用的是刚才计算出的合成速度

        dist, idx = tree.query([gt_x, gt_y])

        lateral_errors.append(dist)
        matched_wp_indices.append(idx)

        # 航向误差计算
        ref_yaw = df_wp.iloc[idx]['yaw']
        yaw_err = gt_yaw - ref_yaw
        # 归一化到 [-pi, pi]
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

    lateral_errors = np.array(lateral_errors)
    heading_errors = np.array(heading_errors)
    velocity_errors = np.array(velocity_errors)
    matched_wp_indices = np.array(matched_wp_indices)

    # 统计指标
    rmse_lat = np.sqrt(np.mean(np.square(lateral_errors)))
    max_lat = np.max(np.abs(lateral_errors))
    # rmse_head_deg = np.degrees(np.sqrt(np.mean(np.square(heading_errors))))
    
    # 寻找最大误差
    max_error_idx = np.argmax(np.abs(lateral_errors))
    bad_gt_point = df_gt.iloc[max_error_idx]
    # bad_wp_idx = matched_wp_indices[max_error_idx]
    # bad_wp_point = df_wp.iloc[bad_wp_idx]

    # ---------------------------------------------------------
    # 5. 绘图
    # ---------------------------------------------------------
    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    
    # --- 图1: 轨迹对比 (带朝向箭头) ---
    plt.figure(figsize=(10, 8))
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path', linewidth=2, alpha=0.4)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1.5, alpha=0.6)
    
    # 标记最大误差
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=100, marker='*', label='Max Lat Error', zorder=10)
    
    # 定义要检查的时间点
    target_timestamps = [5.0, 8.0, 13.0]
    marker_colors = ['purple', 'cyan', 'magenta']
    arrow_len = 1.5 
    
    print("\n--- 关键时间点朝向验证 ---")
    for t_target, color in zip(target_timestamps, marker_colors):
        # 寻找最接近的时间戳
        if len(gt_stamp) > 0:
            idx_closest = np.argmin(np.abs(gt_stamp - t_target))
            
            # 防止时间差过大（例如数据只有1秒，却要找13秒的点）
            if np.abs(gt_stamp[idx_closest] - t_target) > 2.0:
                continue

            pt_gt = df_gt.iloc[idx_closest]
            idx_wp = matched_wp_indices[idx_closest]
            pt_wp = df_wp.iloc[idx_wp]
            
            # 真实车辆朝向
            gt_yaw_rad = pt_gt['yaw']
            gt_dx = arrow_len * np.cos(gt_yaw_rad)
            gt_dy = arrow_len * np.sin(gt_yaw_rad)
            
            # 参考点朝向
            wp_yaw_rad = pt_wp['yaw']
            wp_dx = arrow_len * np.cos(wp_yaw_rad)
            wp_dy = arrow_len * np.sin(wp_yaw_rad)
            
            print(f"Time {t_target}s (Actual {pt_gt['stamp']:.2f}s): GT Yaw={np.degrees(gt_yaw_rad):.1f}°, Ref Yaw={np.degrees(wp_yaw_rad):.1f}°")

            # 画真实位置点
            plt.scatter(pt_gt['x'], pt_gt['y'], c=color, s=80, edgecolors='black', marker='o', zorder=15)
            
            # 画真实朝向箭头 (实线箭头)
            plt.arrow(pt_gt['x'], pt_gt['y'], gt_dx, gt_dy, 
                    head_width=0.4, head_length=0.5, fc=color, ec=color, linewidth=2, zorder=15,
                    label=f'GT Heading @ {t_target}s' if t_target==target_timestamps[0] else "")
            
            # 画参考朝向箭头 (虚线箭头)
            plt.arrow(pt_wp['x'], pt_wp['y'], wp_dx, wp_dy, 
                    head_width=0.3, head_length=0.4, fc='none', ec=color, linestyle=':', linewidth=1.5, zorder=15)

            # 连接误差线
            plt.plot([pt_gt['x'], pt_wp['x']], [pt_gt['y'], pt_wp['y']], 
                    color=color, linestyle='--', linewidth=1, zorder=14)
            
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
    gt_head_err_deg = np.degrees(heading_errors)
    plt.plot(gt_stamp, gt_head_err_deg, 'm-')
    plt.title("Heading Error over Time [deg]")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [deg]")
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "3_heading_error.png"), dpi=150)
    plt.close()

    # --- 图4: 速度追踪情况 (合成速度) ---
    plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    plt.plot(gt_stamp, df_gt['ref_velocity'].to_numpy(), 'r--', label='Target Vel', linewidth=2)
    # 这里绘制的是我们计算出来的合成速度
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
    plt.legend()
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

    print(f"所有图片已保存至: {save_dir}")

if __name__ == "__main__":
    # 示例文件路径
    gt_file = "data_process/v_10/pure_v.txt" 
    wp_file = "data_process/global_path_v_10.txt"
    
    save_dir = "result/v_10/pure_v"

    evaluate_path_tracking(gt_file, wp_file, save_dir)