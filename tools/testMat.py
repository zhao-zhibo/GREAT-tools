import numpy as np

# 定义矩阵 A 和 B
# A 是相机到 MEMS IMU 的变换矩阵
T_camera_to_mems_imu = np.array([[0.99818125, 0.00621723, -0.05996276, 0.26695413],
                                 [0.05996458, -0.00010771, 0.99820050, 0.37697590],
                                 [0.00619958, -0.99998067, -0.00048033, 0.15036302],
                                 [0.00000000, 0.00000000, 0.00000000, 1.00000000]])

# B 是LiDAR到相机的变换矩阵
T_lidar_to_camera = np.array([[0.999123, 0.0418755, -5.483e-5, 0.285651],
                              [0.000164466, -0.00523338, -0.999986, -0.662855],
                              [-0.0418752, 0.999109, -0.00523568, 0.786325],
                              [0, 0, 0, 1]])

# 计算矩阵 A 和 B 的乘积
T_lidar_to_mems_imu = np.dot(T_camera_to_mems_imu, T_lidar_to_camera)

# 计算矩阵 C 的逆矩阵
T_mems_imu_to_lidar = np.linalg.inv(T_lidar_to_mems_imu)

# 打印结果
# LiDAR到 MEMS IMU 的变换矩阵
print("LiDAR到 MEMS IMU 的变换矩阵 A * B 是:")
for row in T_lidar_to_mems_imu:
    print(",".join(map(str, row)))

# MEMS IMU 到LiDAR的变换矩阵
print("\nMEMS IMU 到LiDAR的变换矩阵 C 的逆矩阵是:")
for row in T_mems_imu_to_lidar:
    print(",".join(map(str, row)))

# 定义相机到 Tactical IMU 的变换矩阵
T_camera_to_tactical_imu = np.array([[0.99790400, 0.00455830, -0.06455098, 0.24990092],
                                     [0.06454996, 0.00037084, 0.99791441, 0.61430253],
                                     [0.00457273, -0.99998954, 0.00007582, 0.14513545],
                                     [0.00000000, 0.00000000, 0.00000000, 1.00000000]])

# 计算矩阵 A 和 B 的乘积
T_lidar_to_tactical_imu = np.dot(T_camera_to_tactical_imu, T_lidar_to_camera)

# 计算矩阵 C 的逆矩阵
T_tactical_imu_to_lidar = np.linalg.inv(T_lidar_to_tactical_imu)

# 打印结果
# LiDAR 到 Tactical IMU 的变换矩阵
print("\nLiDAR 到 Tactical IMU 的变换矩阵 A * B 是:")
for row in T_lidar_to_tactical_imu:
    print(",".join(map(str, row)))

# Tactical IMU 到 LiDAR的变换矩阵
print("\nTactical IMU 到 LiDAR的变换矩阵 C 的逆矩阵是:")
for row in T_tactical_imu_to_lidar:
    print(",".join(map(str, row)))