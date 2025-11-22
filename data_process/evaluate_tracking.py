import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

def evaluate_path_tracking(gt_file, wp_file, save_dir=None):
    # 1. 读取数据
    print(f"正在读取文件:\n - GT: {gt_file}\n - WP: {wp_file}")
    try:
        # 读取真实路径
        df_gt = pd.read_csv(gt_file, sep=r'\s+', comment='#', names=['stamp', 'x', 'y', 'z', 'yaw'])
    except Exception as e:
        print(f"读取真实路径文件出错: {e}")
        return

    try:
        # 读取规划路径
        df_wp = pd.read_csv(wp_file)
    except Exception as e:
        print(f"读取规划路径文件出错: {e}")
        return

    # 如果需要保存图片，创建目录
    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
        print(f"图片将保存到目录: {save_dir}")

    # 2. 计算误差
    wp_xy = df_wp[['x', 'y']].values
    tree = KDTree(wp_xy)

    lateral_errors = []
    heading_errors = []
    matched_wp_indices = []  # 用于存储匹配到的规划点索引

    for index, row in df_gt.iterrows():
        gt_x = row['x']
        gt_y = row['y']
        gt_yaw = row['yaw']

        # 寻找最近点
        dist, idx = tree.query([gt_x, gt_y])

        lateral_errors.append(dist)
        matched_wp_indices.append(idx)  # 记录索引

        # 计算航向误差（弧度）
        ref_yaw = df_wp.iloc[idx]['yaw']
        yaw_err = gt_yaw - ref_yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi  # wrap 到 [-pi, pi]
        heading_errors.append(yaw_err)

    df_gt['lateral_error'] = lateral_errors
    df_gt['heading_error'] = heading_errors

    # 转换列表为 numpy 数组以便后续计算
    lateral_errors = np.array(lateral_errors)
    heading_errors = np.array(heading_errors)  # 航向误差 (rad)
    matched_wp_indices = np.array(matched_wp_indices)

    # 3. 计算统计指标
    # 横向误差：最大值 / 均值 / RMSE （单位：m）
    rmse_lat = np.sqrt(np.mean(np.square(lateral_errors)))
    max_lat = np.max(np.abs(lateral_errors))
    mean_lat = np.mean(np.abs(lateral_errors))

    # 航向误差：最大值 / 均值 / RMSE （先在弧度下算，再转成角度打印）
    rmse_head_rad = np.sqrt(np.mean(np.square(heading_errors)))
    max_head_rad = np.max(np.abs(heading_errors))
    mean_head_rad = np.mean(np.abs(heading_errors))

    rmse_head_deg = np.degrees(rmse_head_rad)
    max_head_deg = np.degrees(max_head_rad)
    mean_head_deg = np.degrees(mean_head_rad)

    # ---------------------------------------------------------
    # 寻找并打印最大横向误差对应的两个点
    # ---------------------------------------------------------
    max_error_idx = np.argmax(np.abs(lateral_errors))  # 找到最大误差在 df_gt 中的行号

    # 获取对应的真实点数据
    bad_gt_point = df_gt.iloc[max_error_idx]

    # 获取对应的规划点数据 (使用之前保存的索引)
    bad_wp_idx = matched_wp_indices[max_error_idx]
    bad_wp_point = df_wp.iloc[bad_wp_idx]

    print("=" * 50)
    print("!!! 最大横向误差诊断报告 !!!")
    print("=" * 50)
    print(f"最大横向误差值: {max_lat:.6f} m")
    print(f"发生时间 (Timestamp): {bad_gt_point['stamp']:.6f}")
    print("-" * 30)
    print(f"【真实点位置 (Ground Truth)】:")
    print(f"  X: {bad_gt_point['x']:.6f}")
    print(f"  Y: {bad_gt_point['y']:.6f}")
    print("-" * 30)
    print(f"【匹配到的规划点位置 (Waypoint)】 (Index: {bad_wp_idx}):")
    print(f"  X: {bad_wp_point['x']:.6f}")
    print(f"  Y: {bad_wp_point['y']:.6f}")
    print("-" * 30)
    print(f"手动验证距离计算: {np.sqrt((bad_gt_point['x']-bad_wp_point['x'])**2 + (bad_gt_point['y']-bad_wp_point['y'])**2):.6f}")
    print("=" * 50)

    print(f"\n路径跟踪评估统计:")
    print(f"数据点数量: {len(df_gt)}")
    print("- 横向误差 (Lateral Error) - 单位: m")
    print(f"  Max : {max_lat:.6f} m")
    print(f"  Mean: {mean_lat:.6f} m")
    print(f"  RMSE: {rmse_lat:.6f} m")
    print("- 航向误差 (Heading Error) - 单位: degree")
    print(f"  Max : {max_head_deg:.6f} deg")
    print(f"  Mean: {mean_head_deg:.6f} deg")
    print(f"  RMSE: {rmse_head_deg:.6f} deg")

    # 4. 绘图并分别保存为单独图片
    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    gt_lat_err = df_gt['lateral_error'].to_numpy()
    gt_head_err_deg = np.degrees(df_gt['heading_error'].to_numpy())

    # 图1: 路径对比
    plt.figure(figsize=(7, 6))
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path', linewidth=2)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1)
    # 高亮最大误差的点
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=150, marker='*',
                label='Max Error Point (GT)', zorder=10)
    plt.scatter(bad_wp_point['x'], bad_wp_point['y'], c='orange', s=100, marker='x',
                label='Max Error Ref (WP)', zorder=10)
    # 画一条线连接这两个点
    plt.plot([bad_gt_point['x'], bad_wp_point['x']],
             [bad_gt_point['y'], bad_wp_point['y']], 'k-', linewidth=1)

    plt.title("Trajectory Comparison (X-Y)")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    if save_dir is not None:
        fig_path = os.path.join(save_dir, "trajectory_comparison.png")
        plt.savefig(fig_path, dpi=200)
        print(f"已保存: {fig_path}")
    plt.close()

    # 图2: 横向误差随时间
    plt.figure(figsize=(7, 4))
    plt.plot(gt_stamp, gt_lat_err, 'g-')
    plt.scatter(bad_gt_point['stamp'], max_lat, c='red', marker='*', s=100, zorder=10)
    plt.title(f"Lateral Error (Max: {max_lat:.4f} m)")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [m]")
    plt.grid(True)

    if save_dir is not None:
        fig_path = os.path.join(save_dir, "lateral_error_time.png")
        plt.savefig(fig_path, dpi=200)
        print(f"已保存: {fig_path}")
    plt.close()

    # 图3: 航向误差随时间
    plt.figure(figsize=(7, 4))
    plt.plot(gt_stamp, gt_head_err_deg, 'm-')
    plt.title("Heading Error vs Time")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [deg]")
    plt.grid(True)

    if save_dir is not None:
        fig_path = os.path.join(save_dir, "heading_error_time.png")
        plt.savefig(fig_path, dpi=200)
        print(f"已保存: {fig_path}")
    plt.close()

    # 图4: 横向误差直方图
    plt.figure(figsize=(7, 4))
    plt.hist(df_gt['lateral_error'], bins=20, color='gray', alpha=0.7)
    plt.title("Lateral Error Distribution")
    plt.xlabel("Error [m]")
    plt.ylabel("Count")
    plt.grid(True)

    if save_dir is not None:
        fig_path = os.path.join(save_dir, "lateral_error_hist.png")
        plt.savefig(fig_path, dpi=200)
        print(f"已保存: {fig_path}")
    plt.close()


if __name__ == "__main__":
    # 请确保文件名与实际路径一致
    gt_file = "data_process/stanley跟踪1.txt"
    wp_file = "data_process/waypoints.csv"

    # 新增：图片输出文件夹路径（可以改成你想要的路径）
    save_dir = "result/stanley跟踪1"

    evaluate_path_tracking(gt_file, wp_file, save_dir)
