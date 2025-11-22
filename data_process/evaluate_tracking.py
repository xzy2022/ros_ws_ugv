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
        # 读取真实路径 (包含线速度 linear_v)
        # 格式: stamp x y z yaw linear_v angular_v
        df_gt = pd.read_csv(gt_file, sep=r'\s+', comment='#', 
                            names=['stamp', 'x', 'y', 'z', 'yaw', 'linear_v', 'angular_v'])
    except Exception as e:
        print(f"读取真实路径文件出错: {e}")
        return

    try:
        # 读取规划路径 (包含目标速度 target_linear_v)
        # 格式: id x y z yaw target_linear_v
        df_wp = pd.read_csv(wp_file, sep=r'\s+', comment='#', 
                            names=['id', 'x', 'y', 'z', 'yaw', 'target_linear_v'])
    except Exception as e:
        print(f"读取规划路径文件出错: {e}")
        return

    # 如果需要保存图片，创建目录
    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
        print(f"图片将保存到目录: {save_dir}")

    # ---------------------------------------------------------
    # 2. 计算误差 (位置、航向、速度)
    # ---------------------------------------------------------
    wp_xy = df_wp[['x', 'y']].values
    tree = KDTree(wp_xy)

    lateral_errors = []
    heading_errors = []
    velocity_errors = []     # 新增：速度误差
    matched_ref_vels = []    # 新增：匹配到的参考速度 (用于绘图)
    matched_wp_indices = []  # 记录索引

    for index, row in df_gt.iterrows():
        gt_x = row['x']
        gt_y = row['y']
        gt_yaw = row['yaw']
        gt_v = row['linear_v'] # 真实线速度

        # 寻找最近的规划点
        dist, idx = tree.query([gt_x, gt_y])

        lateral_errors.append(dist)
        matched_wp_indices.append(idx)

        # A. 计算航向误差
        ref_yaw = df_wp.iloc[idx]['yaw']
        yaw_err = gt_yaw - ref_yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
        heading_errors.append(yaw_err)

        # B. 计算速度误差 (新增)
        # 逻辑：当前时刻应该达到的速度 = 空间上最近的参考点的目标速度
        ref_v = df_wp.iloc[idx]['target_linear_v']
        vel_err = gt_v - ref_v # 实际 - 目标
        
        velocity_errors.append(vel_err)
        matched_ref_vels.append(ref_v)

    # 将列表添加回 DataFrame 或转为 numpy 数组
    df_gt['lateral_error'] = lateral_errors
    df_gt['heading_error'] = heading_errors
    df_gt['velocity_error'] = velocity_errors
    df_gt['ref_velocity'] = matched_ref_vels

    lateral_errors = np.array(lateral_errors)
    heading_errors = np.array(heading_errors)
    velocity_errors = np.array(velocity_errors)
    matched_wp_indices = np.array(matched_wp_indices)

    # ---------------------------------------------------------
    # 3. 计算统计指标
    # ---------------------------------------------------------
    # 横向误差 (m)
    rmse_lat = np.sqrt(np.mean(np.square(lateral_errors)))
    max_lat = np.max(np.abs(lateral_errors))
    mean_lat = np.mean(np.abs(lateral_errors))

    # 航向误差 (deg)
    rmse_head_rad = np.sqrt(np.mean(np.square(heading_errors)))
    max_head_rad = np.max(np.abs(heading_errors))
    mean_head_rad = np.mean(np.abs(heading_errors))

    rmse_head_deg = np.degrees(rmse_head_rad)
    max_head_deg = np.degrees(max_head_rad)
    mean_head_deg = np.degrees(mean_head_rad)

    # 速度误差 (m/s)
    rmse_vel = np.sqrt(np.mean(np.square(velocity_errors)))
    max_vel = np.max(np.abs(velocity_errors))
    mean_vel = np.mean(np.abs(velocity_errors))

    # ---------------------------------------------------------
    # 4. 打印最大误差诊断信息
    # ---------------------------------------------------------
    max_error_idx = np.argmax(np.abs(lateral_errors))
    bad_gt_point = df_gt.iloc[max_error_idx]
    bad_wp_idx = matched_wp_indices[max_error_idx]
    bad_wp_point = df_wp.iloc[bad_wp_idx]

    print("=" * 60)
    print("!!! 最大横向误差诊断报告 !!!")
    print("=" * 60)
    print(f"最大横向误差: {max_lat:.6f} m")
    print(f"发生时间: {bad_gt_point['stamp']:.6f}")
    print(f"当前速度: {bad_gt_point['linear_v']:.3f} m/s | 目标速度: {bad_gt_point['ref_velocity']:.3f} m/s")
    print("-" * 30)
    print(f"【GT 位置】 X: {bad_gt_point['x']:.4f}, Y: {bad_gt_point['y']:.4f}")
    print(f"【Ref 位置】 X: {bad_wp_point['x']:.4f}, Y: {bad_wp_point['y']:.4f} (Index: {bad_wp_idx})")
    print("=" * 60)

    print(f"\n--- 综合统计指标 ---")
    print(f"样本总数: {len(df_gt)}")
    print(f"1. 横向误差 (Lateral) [m]")
    print(f"   Max: {max_lat:.4f} | Mean: {mean_lat:.4f} | RMSE: {rmse_lat:.4f}")
    print(f"2. 航向误差 (Heading) [deg]")
    print(f"   Max: {max_head_deg:.4f} | Mean: {mean_head_deg:.4f} | RMSE: {rmse_head_deg:.4f}")
    print(f"3. 速度误差 (Velocity) [m/s]")
    print(f"   Max: {max_vel:.4f} | Mean: {mean_vel:.4f} | RMSE: {rmse_vel:.4f}")

    # ---------------------------------------------------------
    # 5. 绘图
    # ---------------------------------------------------------
    # 统一转换为 numpy 数组，避免 Pandas 索引报错
    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    
    # --- 图1: 轨迹对比 ---
    plt.figure(figsize=(8, 7))
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path', linewidth=2)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1)
    # 高亮最大误差
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=150, marker='*', label='Max Error (GT)', zorder=10)
    plt.scatter(bad_wp_point['x'], bad_wp_point['y'], c='orange', s=100, marker='x', label='Max Error (Ref)', zorder=10)
    plt.plot([bad_gt_point['x'], bad_wp_point['x']], [bad_gt_point['y'], bad_wp_point['y']], 'k-', linewidth=1)
    
    plt.title("Trajectory Comparison")
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
    plt.title(f"Lateral Error over Time (Max: {max_lat:.3f}m)")
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

    # --- 图4: 速度追踪情况 ---
    plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    # 显式使用 .to_numpy() 修复 matplotlib/pandas 兼容性问题
    plt.plot(gt_stamp, df_gt['ref_velocity'].to_numpy(), 'r--', label='Target Vel', linewidth=2)
    plt.plot(gt_stamp, df_gt['linear_v'].to_numpy(), 'b-', label='Actual Vel', linewidth=1.5, alpha=0.8)
    plt.title("Velocity Tracking Performance")
    plt.ylabel("Velocity [m/s]")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(gt_stamp, velocity_errors, 'k-', label='Vel Error (Actual - Target)')
    plt.axhline(0, color='gray', linestyle='--')
    plt.ylabel("Error [m/s]")
    plt.xlabel("Timestamp [s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    if save_dir: plt.savefig(os.path.join(save_dir, "4_velocity_tracking.png"), dpi=150)
    plt.close()

    # --- 图5 (恢复): 横向误差直方图 ---
    plt.figure(figsize=(8, 4))
    # 使用 numpy 数组直接绘图
    plt.hist(lateral_errors, bins=20, color='gray', alpha=0.7)
    plt.title("Lateral Error Distribution")
    plt.xlabel("Error [m]")
    plt.ylabel("Count")
    plt.grid(True)
    if save_dir: plt.savefig(os.path.join(save_dir, "5_lateral_error_hist.png"), dpi=150)
    plt.close()

    print(f"所有 5 张图片已保存至: {save_dir}")

if __name__ == "__main__":
    # 示例文件路径 (请修改为你实际的 txt 路径)
    # 注意: wp_file 现在指向 global_path_xxx.txt
    gt_file = "data_process/v_20/s-k=2.txt" 
    wp_file = "data_process/global_path_v_20.txt"
    
    save_dir = "result/v_20/s-k=2"

    evaluate_path_tracking(gt_file, wp_file, save_dir)