import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

def evaluate_path_tracking(gt_file, wp_file):
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

    # 2. 计算误差
    wp_xy = df_wp[['x', 'y']].values
    tree = KDTree(wp_xy)

    lateral_errors = []
    heading_errors = []
    matched_wp_indices = [] # 新增：用于存储匹配到的规划点索引

    for index, row in df_gt.iterrows():
        gt_x = row['x']
        gt_y = row['y']
        gt_yaw = row['yaw']
        
        # 寻找最近点
        dist, idx = tree.query([gt_x, gt_y])
        
        lateral_errors.append(dist)
        matched_wp_indices.append(idx) # 记录索引
        
        # 计算航向误差
        ref_yaw = df_wp.iloc[idx]['yaw']
        yaw_err = gt_yaw - ref_yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
        heading_errors.append(yaw_err)

    df_gt['lateral_error'] = lateral_errors
    df_gt['heading_error'] = heading_errors
    
    # 转换列表为numpy数组以便操作
    lateral_errors = np.array(lateral_errors)
    matched_wp_indices = np.array(matched_wp_indices)

    # 3. 计算统计指标
    rmse_lat = np.sqrt(np.mean(np.square(lateral_errors)))
    max_lat = np.max(np.abs(lateral_errors))
    mean_lat = np.mean(np.abs(lateral_errors))
    rmse_head = np.sqrt(np.mean(np.square(heading_errors)))

    # ---------------------------------------------------------
    # 新增：寻找并打印最大误差对应的两个点
    # ---------------------------------------------------------
    max_error_idx = np.argmax(np.abs(lateral_errors)) # 找到最大误差在 df_gt 中的行号
    
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
    print(f"横向误差 RMSE: {rmse_lat:.6f} m")
    print(f"横向误差 Mean: {mean_lat:.6f} m")
    print(f"航向误差 RMSE: {np.degrees(rmse_head):.6f} degrees")

    # 4. 绘图
    plt.figure(figsize=(14, 8))

    wp_x = df_wp['x'].to_numpy()
    wp_y = df_wp['y'].to_numpy()
    gt_x = df_gt['x'].to_numpy()
    gt_y = df_gt['y'].to_numpy()
    gt_stamp = df_gt['stamp'].to_numpy()
    gt_lat_err = df_gt['lateral_error'].to_numpy()
    gt_head_err_deg = np.degrees(df_gt['heading_error'].to_numpy())

    # 子图1: 路径对比
    plt.subplot(2, 2, 1)
    plt.plot(wp_x, wp_y, 'r--', label='Reference Path', linewidth=2)
    plt.plot(gt_x, gt_y, 'b-', label='Actual Path', linewidth=1)
    
    # 高亮最大误差的点
    plt.scatter(bad_gt_point['x'], bad_gt_point['y'], c='red', s=150, marker='*', label='Max Error Point (GT)', zorder=10)
    plt.scatter(bad_wp_point['x'], bad_wp_point['y'], c='orange', s=100, marker='x', label='Max Error Ref (WP)', zorder=10)
    # 画一条线连接这两个点
    plt.plot([bad_gt_point['x'], bad_wp_point['x']], [bad_gt_point['y'], bad_wp_point['y']], 'k-', linewidth=1)
    
    plt.title("Trajectory Comparison (X-Y)")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    # 子图2: 横向误差
    plt.subplot(2, 2, 2)
    plt.plot(gt_stamp, gt_lat_err, 'g-')
    # 在时间轴上也标出来
    plt.scatter(bad_gt_point['stamp'], max_lat, c='red', marker='*', s=100, zorder=10)
    plt.title(f"Lateral Error (Max: {max_lat:.4f}m)")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [m]")
    plt.grid(True)

    # 子图3: 航向误差
    plt.subplot(2, 2, 3)
    plt.plot(gt_stamp, gt_head_err_deg, 'm-')
    plt.title("Heading Error vs Time")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Error [deg]")
    plt.grid(True)

    # 子图4: 直方图
    plt.subplot(2, 2, 4)
    plt.hist(df_gt['lateral_error'], bins=20, color='gray', alpha=0.7)
    plt.title("Lateral Error Distribution")
    plt.xlabel("Error [m]")
    plt.ylabel("Count")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 请确保文件名与实际路径一致
    gt_file = "data_process/纯点跟踪效果.txt"
    wp_file = "data_process/waypoints.csv"
    
    evaluate_path_tracking(gt_file, wp_file)