import pandas as pd
import matplotlib.pyplot as plt

# 读取文件并跳过第一行注释
mems_data = pd.read_csv('/media/zhao/ZhaoZhibo/AllData/CalibrationData/2025_0103/IMU/MEMS_imu_data.txt', delim_whitespace=True, skiprows=1, header=None, names=['time', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'])
tactical_data = pd.read_csv('/media/zhao/ZhaoZhibo/AllData/CalibrationData/2025_0103/IMU/Tactical_imu_data.txt', delim_whitespace=True, skiprows=1, header=None, names=['time', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'])

# Convert pandas Series to numpy arrays
mems_time = mems_data['time'].values
mems_gx = mems_data['gx'].values
mems_gy = mems_data['gy'].values
mems_gz = mems_data['gz'].values
mems_ax = mems_data['ax'].values
mems_ay = mems_data['ay'].values
mems_az = mems_data['az'].values

tactical_time = tactical_data['time'].values
tactical_gx = tactical_data['gx'].values
tactical_gy = tactical_data['gy'].values
tactical_gz = tactical_data['gz'].values
tactical_ax = tactical_data['ax'].values
tactical_ay = tactical_data['ay'].values
tactical_az = tactical_data['az'].values

# 创建一个图形和子图
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# 绘制 MEMS 数据
axs[0].plot(mems_time, mems_gx, label='MEMS gx')
axs[0].plot(mems_time, mems_gy, label='MEMS gy')
axs[0].plot(mems_time, mems_gz, label='MEMS gz')
axs[0].set_ylabel('MEMS Gyroscope (rad/s)')
axs[0].legend(loc='upper right')

# 绘制 Tactical 数据
axs[1].plot(tactical_time, tactical_gx, label='Tactical gx')
axs[1].plot(tactical_time, tactical_gy, label='Tactical gy')
axs[1].plot(tactical_time, tactical_gz, label='Tactical gz')
axs[1].set_ylabel('Tactical Gyroscope (rad/s)')
axs[1].legend(loc='upper right')

# 绘制两个文件的加速度数据
axs[2].plot(mems_time, mems_ax, label='MEMS ax')
axs[2].plot(mems_time, mems_ay, label='MEMS ay')
axs[2].plot(mems_time, mems_az, label='MEMS az')
axs[2].plot(tactical_time, tactical_ax, label='Tactical ax', linestyle='dashed')
axs[2].plot(tactical_time, tactical_ay, label='Tactical ay', linestyle='dashed')
axs[2].plot(tactical_time, tactical_az, label='Tactical az', linestyle='dashed')
axs[2].set_ylabel('Acceleration (m/s^2)')
axs[2].set_xlabel('Time (s)')
axs[2].legend(loc='upper right')

# 显示图形
plt.tight_layout()
plt.show()