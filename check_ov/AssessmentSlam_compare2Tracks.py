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

    slam_types = ['Initiate Transform', 'registration Transform']
    slam_paths = [
        '/home/zhao/Data/roadSide_poses_1743237468.514390.txt',
        '/home/zhao/Data/init_vehicle_pose_1743237468.514460.txt'
    ]

    all_data = {}
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
    for slam_type, slam_path in zip(slam_types, slam_paths):
        SlamResult = dealLioSamResult.main(slam_path, config.slam_in_absolute_n_frame)
        slamTimestamps = []
        slamTransforMatrix = []
        max_ground_truth_time = max(groundTruthTimestamps)
        min_ground_truth_time = min(groundTruthTimestamps)
        for timestamp, slamTll in SlamResult:
            if min_ground_truth_time <= timestamp <= max_ground_truth_time:
                slamTimestamps.append(timestamp)
                if config.slam_in_absolute_n_frame:
                    slamTransforMatrix.append((timestamp, slamTll))

        interpolatedGroundTruth = interpolate_SE3(groundTruthTimestamps, groundTruthTnl, slamTimestamps)
        enu_errors, euler_errors = calculate_enu_and_euler_errors(slamTransforMatrix, interpolatedGroundTruth)
        all_enu_errors[slam_type] = enu_errors
        all_euler_errors[slam_type] = euler_errors

    plot_enu_and_euler_errors(all_enu_errors, all_euler_errors)
    print("当前字体:", plt.rcParams['font.family'])


if __name__ == "__main__":
    main()