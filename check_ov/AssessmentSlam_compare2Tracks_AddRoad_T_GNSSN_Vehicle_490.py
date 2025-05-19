import dealLioSamResult
import config
import numpy as np
import re
import math
from scipy.spatial.transform import Rotation as R, Slerp
import geoFunc.trans as trans
import plotResult
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt

def loadIE(path : str, all_data : dict, Ti0i1 : np.ndarray, ref_xyz : np.ndarray = None):
    fp = open(path,'rt')

    if ref_xyz is None:
        is_ref_set  = False
    else:
        is_ref_set = True
        Ten0 = np.eye(4,4)
        # n frame  to e frame
        Ten0[0:3,0:3] = trans.Cen(ref_xyz)
        Ten0[0:3,3] = ref_xyz

    while True:
        line = fp.readline().strip()
        if line == '':break
        if line[0] == '#' :continue
        line = re.sub('\s\s+',' ',line)
        elem = line.split(' ')
        sod = float(elem[1])
        if sod not in all_data.keys():
            all_data[sod] ={}
        all_data[sod]['X0']   = float(elem[2])
        all_data[sod]['Y0']   = float(elem[3])
        all_data[sod]['Z0']   = float(elem[4])
        all_data[sod]['VX0']  = float(elem[15])
        all_data[sod]['VY0']  = float(elem[16])
        all_data[sod]['VZ0']  = float(elem[17])
        all_data[sod]['ATTX0']= float(elem[25])
        all_data[sod]['ATTY0']= float(elem[26])
        all_data[sod]['ATTZ0']= -float(elem[24])
        # 每次都重新计算了Ren，也就是认为n系也在动，但是小范围内认为n系不动，距离近的Ren的结果变化很小
        Ren = trans.Cen([all_data[sod]['X0'],all_data[sod]['Y0'],all_data[sod]['Z0']])
        ani0 = [all_data[sod]['ATTX0']/180*math.pi,\
                all_data[sod]['ATTY0']/180*math.pi,\
                all_data[sod]['ATTZ0']/180*math.pi]
        Rni0 = trans.att2m(ani0) # 旋转矩阵 ,大惯导 frame to n frame

        # 接下来将得到的真值(i0)转换到i1系下，因为本身的真值结果是在大惯导坐标系下(i0为大惯导坐标系)。
        Rni1 = np.matmul(Rni0,Ti0i1[0:3,0:3]) # 得到lidar frame to n frame的旋转矩阵
        Rei1 = np.matmul(Ren,Rni1) # 得到i1传感器在e系下的姿态，也就是lidar frame to e frame的旋转矩阵

        tei0 = np.array([all_data[sod]['X0'],all_data[sod]['Y0'],all_data[sod]['Z0']]) # 大惯导在e系下的位置
        tei1 = tei0 + Ren @ Rni0 @ Ti0i1[0:3,3] # 得到lidar在e系下的位置
        # 最终得到lidar frame to e frame的变换矩阵
        Tei1 = np.eye(4,4)
        Tei1[0:3,0:3] = Rei1
        Tei1[0:3,3] = tei1

        # 如果参考坐标系没有设置，则设置第一个坐标点为n系的原点
        if not is_ref_set:
            is_ref_set = True
            Ten0 = np.eye(4,4)
            Ten0[0:3,0:3] = trans.Cen(tei1)
            Ten0[0:3,3] = tei1
        # 计算i1传感器在n系下的位姿，Ten0是n系到e系的变换矩阵，因此需要求逆
        Tni1 = np.matmul(np.linalg.inv(Ten0),Tei1)
        all_data[sod]['T'] = Tni1 # lidar frame to n frame的变换矩阵

    fp.close()
    return all_data

def interpolate_SE3(timeGroudTruth, SE3GroudTruth, timeCalc):
    # 将输入转换为numpy数组
    time_groudTruth = np.array(timeGroudTruth)
    SE3_groudTruth = np.array(SE3GroudTruth)
    time_calc = np.array(timeCalc)  # 先将timeCalc转换为numpy数组

    # 检查输入的有效性
    assert time_groudTruth.ndim == 1 and len(time_groudTruth) >= 2, "Invalid time_groudTruth."
    assert np.all(np.diff(time_groudTruth) > 0), "time_groudTruth must be strictly increasing."
    assert SE3_groudTruth.shape == (len(time_groudTruth), 4, 4), "SE3_groudTruth shape mismatch."

    # 初始化插值结果列表
    interp_results = []

    # 提取旋转矩阵并创建Slerp对象
    rotations = R.from_matrix(SE3_groudTruth[:, :3, :3])
    slerp = Slerp(time_groudTruth, rotations)

    for idx, t in enumerate(time_calc):
        # 处理边界
        t1_idx = np.searchsorted(time_groudTruth, t, side='right') - 1
        t1_idx = np.clip(t1_idx, 0, len(time_groudTruth)-2)  # 确保t2_idx有效
        t2_idx = t1_idx + 1

        t1, t2 = time_groudTruth[t1_idx], time_groudTruth[t2_idx]
        T1, T2 = SE3_groudTruth[t1_idx], SE3_groudTruth[t2_idx]

        # 平移插值
        if np.isscalar(t) and np.isscalar(t1) and np.isscalar(t2):
            alpha = (t - t1) / (t2 - t1) if t2 != t1 else 0.0
        else:
            raise ValueError("t, t1, 或 t2 不是标量值。")
        interp_translation = (1 - alpha) * T1[:3, 3] + alpha * T2[:3, 3]

        # 旋转插值
        interp_rotation = slerp(t).as_matrix()

        # 将插值结果填入SE(3)矩阵
        interp_SE3 = np.eye(4)
        interp_SE3[:3, :3] = interp_rotation
        interp_SE3[:3, 3] = interp_translation
        interp_results.append((t, interp_SE3))

    return interp_results

def calculate_errors(slam_data, ground_truth_data):
    position_errors = []
    orientation_errors = []
    for (slam_time, slam_pose), (gt_time, gt_pose) in zip(slam_data, ground_truth_data):
        position_error = np.linalg.norm(slam_pose[:3, 3] - gt_pose[:3, 3])
        orientation_error = R.from_matrix(slam_pose[:3, :3]).inv() * R.from_matrix(gt_pose[:3, :3])
        euler_error = orientation_error.as_euler('xyz', degrees=True)
        position_errors.append((slam_time, position_error))
        orientation_errors.append((slam_time, euler_error))
    return position_errors, orientation_errors

def statistical_enu_and_euler_errors(slam_data, ground_truth_data):
    results = {
        "enu_errors": [],
        "euler_errors": [],
        "position_stats": {"east": {}, "north": {}, "up": {}},
        "orientation_stats": {"roll": {}, "pitch": {}, "yaw": {}}
    }
    east_errors, north_errors, up_errors = [], [], []
    roll_errors, pitch_errors, yaw_errors = [], [], []

    for (slam_time, slam_pose), (gt_time, gt_pose) in zip(slam_data, ground_truth_data):
        # 计算ENU误差
        enu_error = slam_pose[:3, 3] - gt_pose[:3, 3]
        results["enu_errors"].append((slam_time, enu_error))
        east_errors.append(enu_error[0])
        north_errors.append(enu_error[1])
        up_errors.append(enu_error[2])

        # 计算欧拉角误差
        orientation_error = R.from_matrix(slam_pose[:3, :3]).inv() * R.from_matrix(gt_pose[:3, :3])
        euler_error = orientation_error.as_euler('xyz', degrees=True)
        results["euler_errors"].append((slam_time, euler_error))
        roll_errors.append(euler_error[0])
        pitch_errors.append(euler_error[1])
        yaw_errors.append(euler_error[2])

    # 统计每个方向的ENU误差
    results["position_stats"]["east"] = {
        "rmse": np.sqrt(np.mean(np.square(east_errors))),
        "std": np.std(east_errors),
        "max": np.max(east_errors),
        "min": np.min(east_errors),
        "median": np.median(east_errors)
    }
    results["position_stats"]["north"] = {
        "rmse": np.sqrt(np.mean(np.square(north_errors))),
        "std": np.std(north_errors),
        "max": np.max(north_errors),
        "min": np.min(north_errors),
        "median": np.median(north_errors)
    }
    results["position_stats"]["up"] = {
        "rmse": np.sqrt(np.mean(np.square(up_errors))),
        "std": np.std(up_errors),
        "max": np.max(up_errors),
        "min": np.min(up_errors),
        "median": np.median(up_errors)
    }

    # 统计每个方向的欧拉角误差
    results["orientation_stats"]["roll"] = {
        "rmse": np.sqrt(np.mean(np.square(roll_errors))),
        "std": np.std(roll_errors),
        "max": np.max(roll_errors),
        "min": np.min(roll_errors),
        "median": np.median(roll_errors)
    }
    results["orientation_stats"]["pitch"] = {
        "rmse": np.sqrt(np.mean(np.square(pitch_errors))),
        "std": np.std(pitch_errors),
        "max": np.max(pitch_errors),
        "min": np.min(pitch_errors),
        "median": np.median(pitch_errors)
    }
    results["orientation_stats"]["yaw"] = {
        "rmse": np.sqrt(np.mean(np.square(yaw_errors))),
        "std": np.std(yaw_errors),
        "max": np.max(yaw_errors),
        "min": np.min(yaw_errors),
        "median": np.median(yaw_errors)
    }

    return results


def plot_combined_statistical_errors(all_statistical_results):
    # 提取每种 SLAM 类型的统计量
    slam_types = list(all_statistical_results.keys())
    position_directions = ["east", "north", "up"]
    orientation_directions = ["roll", "pitch", "yaw"]
    metrics = ["rmse", "std", "max", "min", "median"]

    # 绘制位置误差统计量
    fig, axs = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    for i, direction in enumerate(position_directions):
        metric_values = {metric: [] for metric in metrics}
        for metric in metrics:
            for slam_type in slam_types:
                metric_values[metric].append(all_statistical_results[slam_type]["position_stats"][direction][metric])

        x = np.arange(len(metrics))  # 横轴位置
        width = 0.2  # 柱状图宽度
        for j, slam_type in enumerate(slam_types):
            axs[i].bar(x + j * width, [metric_values[metric][j] for metric in metrics], width, label=slam_type)

        axs[i].set_title(f'{direction.capitalize()} Position Error Statistics')
        axs[i].set_xticks(x + width, metrics)
        axs[i].set_xlabel('Metrics')

        if i == 0:  # 仅在第一个子图设置纵轴标签
            axs[i].set_ylabel('Error (m)')
        axs[i].legend()

    plt.tight_layout()
    plt.show()

    # 绘制姿态误差统计量
    fig, axs = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    for i, direction in enumerate(orientation_directions):
        metric_values = {metric: [] for metric in metrics}
        for metric in metrics:
            for slam_type in slam_types:
                metric_values[metric].append(all_statistical_results[slam_type]["orientation_stats"][direction][metric])

        x = np.arange(len(metrics))  # 横轴位置
        width = 0.2  # 柱状图宽度
        for j, slam_type in enumerate(slam_types):
            axs[i].bar(x + j * width, [metric_values[metric][j] for metric in metrics], width, label=slam_type)

        axs[i].set_title(f'{direction.capitalize()} Orientation Error Statistics')
        axs[i].set_xticks(x + width, metrics)
        axs[i].set_xlabel('Metrics')
        axs[i].set_ylabel('Error (°)')
        axs[i].legend()

    plt.tight_layout()
    plt.show()


def calculate_enu_and_euler_errors(slam_data, ground_truth_data):
    enu_errors = []
    euler_errors = []
    for (slam_time, slam_pose), (gt_time, gt_pose) in zip(slam_data, ground_truth_data):
        enu_error = slam_pose[:3, 3] - gt_pose[:3, 3]
        enu_errors.append((slam_time, enu_error))

        orientation_error = R.from_matrix(slam_pose[:3, :3]).inv() * R.from_matrix(gt_pose[:3, :3])
        euler_error = orientation_error.as_euler('xyz', degrees=True)
        euler_errors.append((slam_time, euler_error))
    return enu_errors, euler_errors

def plot_enu_and_euler_errors(all_enu_errors, all_euler_errors):
    # 设置全局参数
    plt.rcdefaults()  # 避免历史配置干扰
    plt.rcParams.update({
        'legend.fontsize': 14,  # 图例字体大小
        'axes.labelsize': 16,   # 坐标轴标签字体大小
        'axes.titlesize': 18,   # 标题字体大小
        'xtick.labelsize': 14,  # x轴刻度字体大小
        'ytick.labelsize': 14,  # y轴刻度字体大小
        'lines.linewidth': 2,   # 线条宽度
        'font.family': 'DejaVu Sans',          # 改用通用字体
        'font.sans-serif': ['DejaVu Sans'],    # 明确指定备选字体
    })

    # Plot ENU errors
    fig, axs = plt.subplots(3, 1, figsize=(15, 15))
    for slam_type, enu_errors in all_enu_errors.items():
        timestamps = [item[0] for item in enu_errors]
        errors = np.array([item[1] for item in enu_errors])

        axs[0].plot(timestamps, errors[:, 0], label=f'{slam_type} - East Error')
        axs[1].plot(timestamps, errors[:, 1], label=f'{slam_type} - North Error')
        axs[2].plot(timestamps, errors[:, 2], label=f'{slam_type} - Up Error')

    for ax in axs:
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.legend()
        ax.tick_params(axis='both', which='major')
        ax.tick_params(axis='both', which='minor')

    axs[0].set_title('East Error')
    axs[1].set_title('North Error')
    axs[2].set_title('Up Error')

    plt.tight_layout()
    plt.show()

    # Plot Euler angle errors
    fig, axs = plt.subplots(3, 1, figsize=(15, 15))
    for slam_type, euler_errors in all_euler_errors.items():
        timestamps = [item[0] for item in euler_errors]
        errors = np.array([item[1] for item in euler_errors])

        axs[0].plot(timestamps, errors[:, 0], label=f'{slam_type} - Roll Error')
        axs[1].plot(timestamps, errors[:, 1], label=f'{slam_type} - Pitch Error')
        axs[2].plot(timestamps, errors[:, 2], label=f'{slam_type} - Yaw Error')

    for ax in axs:
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (°)')
        ax.legend()
        ax.tick_params(axis='both', which='major')
        ax.tick_params(axis='both', which='minor')

    axs[0].set_title('Roll Error')
    axs[1].set_title('Pitch Error')
    axs[2].set_title('Yaw Error')

    plt.tight_layout()
    plt.show()

def main():
    # 原始变换矩阵 lidar frame to 大惯导 frame
    Ti0i1 = np.array([
        [0.9997326730750637, -0.02272959144087399, -0.004274982985153601, 0.03],
        [0.022705545174019435, 0.9997263893640309, -0.0055991346006956004, 0.48],
        [0.00440108045844036, 0.0055005630583402, 0.9999748924543965, 0.33],
        [0.0, 0.0, 0.0, 1.0]
    ])
    rotation_matrix = Ti0i1[:3, :3]
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
    print(f"Roll: {euler_angles[0]}, Pitch: {euler_angles[1]}, Yaw: {euler_angles[2]}\n{'-' * 50}")
    euler_angles[2] += -2.8  # 调整yaw
    euler_angles[1] += -2.0  # 调整pitch
    euler_angles[0] += -0.0  # 调整roll

    new_rotation_matrix = R.from_euler('xyz', euler_angles, degrees=True).as_matrix()
    Ti0i1[:3, :3] = new_rotation_matrix
    print(f"Ti0i1:\n{Ti0i1}\n{'-' * 50}")
    # 加入路侧前后的对比结果
    # slam_types = ['LIO-SAM Adding RoadSide Lidar', 'LIO-SAM without RoadSide Lidar']
    # slam_paths = [
    #     '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/GNSS_vehicle_road_0327_2237/optimized_odom_tum.txt',
    #     '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/gps_vehicle_0327/optimized_odom_tum.txt'
    # ]

    # 最终因子图融合后的结果，加入路侧融合前后最终的因子图优化后的对比结果 下面三个变量都包含'LiDAR-IMU-Roadside_'
    slam_types = ['SmallGicp_GN','GN_Probability_StaticCov', 'GN_Probability_DynamicCov']
    slam_paths = [
        # '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/gps_vehicle_0407/optimized_odom_tum.txt',
        '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/490/GNSS_Vehicle_Road_0515_GN/optimized_odom_tum.txt',
        '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/490/GNSS_Vehicle_Road_0515_GNPro_StaticCov_490/optimized_odom_tum.txt',
        '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/490/GNSS_Vehicle_Road_0515_GNPro_DynamicVar_0.9YDirection_490/optimized_odom_tum.txt'
        # '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Vehicle_IMU_LIOSAM/490/GNSS_Vehicle_Road_0515_GNPro_DynamicCov_490/optimized_odom_tum.txt'
    ]

    all_data = {}
    # slam的原点为：30.3965858919968 114.146383022026 11.1162951593667
    # Slam原点转换为wgs84坐标为： -2252398.030 5024382.629 3208376.785
    ref_xyz = np.array([-2252398.030, 5024382.629, 3208376.785])  # slam轨迹的原点对应的位置坐标,已经将纬度经度高程转换为WGS84的三维坐标
    iePath = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Reference/IE.txt'  # 2.20日采集的第一次观测结果
    groundTruth = loadIE(iePath, all_data, Ti0i1, ref_xyz)

    groundTruthTimestamps = []
    groundTruthTnl = []
    for timestamp, data in groundTruth.items():
        groundTruthTimestamps.append(timestamp)
        groundTruthTnl.append(data['T'])

    all_enu_errors = {}
    all_euler_errors = {}
    all_statistical_results = {}
    for slam_type, slam_path in zip(slam_types, slam_paths):
        SlamResult = dealLioSamResult.main(slam_path, config.slam_in_absolute_n_frame)
        slamTimestamps = []
        slamTransforMatrix = []
        max_ground_truth_time = max(groundTruthTimestamps)
        min_ground_truth_time = min(groundTruthTimestamps)
        for timestamp, slamTll in SlamResult:
            if min_ground_truth_time <= timestamp <= max_ground_truth_time and timestamp >= 377613.4:
                slamTimestamps.append(timestamp)
                if config.slam_in_absolute_n_frame:
                    slamTransforMatrix.append((timestamp, slamTll))

        interpolatedGroundTruth = interpolate_SE3(groundTruthTimestamps, groundTruthTnl, slamTimestamps)
        errorResults = statistical_enu_and_euler_errors(slamTransforMatrix, interpolatedGroundTruth)
        # enu_errors, euler_errors = calculate_enu_and_euler_errors(slamTransforMatrix, interpolatedGroundTruth)
        all_enu_errors[slam_type] = errorResults['enu_errors']
        all_euler_errors[slam_type] = errorResults['euler_errors']
        all_statistical_results[slam_type] = errorResults

    plot_enu_and_euler_errors(all_enu_errors, all_euler_errors)
    print("当前字体:", plt.rcParams['font.family'])
    plot_combined_statistical_errors(all_statistical_results)

if __name__ == "__main__":
    main()