import pandas as pd
import numpy as np
import subprocess
from scipy.spatial.transform import Rotation as R

# 读取 groundtruth.txt 文件
groundtruth_data = pd.read_csv('/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/groundtruth.txt',\
                               delim_whitespace=True, header=None, skiprows=2,
                               names=['Week', 'GPSTime', 'E', 'N', 'U', 'VX', 'VY', 'VZ', 'Heading', 'Pitch', 'Roll'])

# 提取位置和姿态信息
groundtruth_data['x'] = groundtruth_data['E']
groundtruth_data['y'] = groundtruth_data['N']
groundtruth_data['z'] = groundtruth_data['U']

# 将欧拉角转换为四元数
def euler_to_quaternion(heading, pitch, roll):
    r = R.from_euler('zyx', [heading, pitch, roll], degrees=True)
    return r.as_quat()

groundtruth_data[['qx', 'qy', 'qz', 'qw']] = groundtruth_data.apply(
    lambda row: pd.Series(euler_to_quaternion(row['Heading'], row['Pitch'], row['Roll'])), axis=1)

# 选择需要的列并保存为新的文件
groundtruth_data[['GPSTime', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']].to_csv('/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/groundtruth_converted.txt', \
                                                                         sep=' ', index=False, header=False)

print("Groundtruth data has been converted and saved to groundtruth_converted.txt")

# 定义 groundtruth 和 optimized 轨迹文件路径
groundtruth_file = '/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/groundtruth_converted.txt'
optimized_file = '/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/optimized_odom_tum.txt'

# 过滤时间范围
groundtruth_data_filtered = groundtruth_data[(groundtruth_data['GPSTime'] >= 23043.9968) & (groundtruth_data['GPSTime'] <= 23200.704)]
groundtruth_data_filtered[['GPSTime', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']].to_csv('/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/groundtruth_filtered.txt', \
                                                                                     sep=' ', index=False, header=False)

# 更新 groundtruth_file 路径
groundtruth_file_filtered = '/media/zhao/ZhaoZhibo1/AllData/GREAT/urban-02/result/groundtruth_filtered.txt'

# 分别计算 x 和 y 方向的 APE 误差
result_xy = subprocess.run(['evo_ape', 'tum', groundtruth_file_filtered, optimized_file, '-r', 'full', '-as', '--save_results', 'ape_results_x.zip', '--plot', '--plot_mode', 'xy'], capture_output=True, text=True)

# 打印 x 和 y 方向的 APE 误差结果
print("XY方向的APE误差结果:")
print(result_xy.stdout)
print(result_xy.stderr)
