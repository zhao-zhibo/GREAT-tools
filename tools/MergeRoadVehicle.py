#!/usr/bin/env python3
import rosbag
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import rospy
import re
from datetime import datetime, timezone, timedelta  # Added timedelta import
from gps_time import GPSTime
import geoFunc.trans as trans
import os
import pytz
import numpy as np
import math
from pyproj import Transformer  # 新增坐标转换库


# 配置参数
roadside90m_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/Yuan_90m/2_2025-02-20-16-52-37.bag'
roadside180m_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/Xu_180m/2_2025-02-20-16-52-45.bag'
vehicle_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/AfterPreProcess_mems.bag'
output_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/mergeVehicleRoad_withGNSS.bag'

def ecef_to_wgs84(x, y, z):
    # WGS84椭球参数[5,8](@ref)
    a = 6378137.0  # 长半轴 (m)
    b = 6356752.31424518  # 短半轴 (m)
    f = 1 / 298.257223563  # 扁率

    # 计算辅助参数[2,8](@ref)
    e_sq = (a ** 2 - b ** 2) / a ** 2  # 第一偏心率平方
    ep_sq = (a ** 2 - b ** 2) / b ** 2  # 第二偏心率平方

    p = np.sqrt(x ** 2 + y ** 2)
    theta = np.arctan2(z * a, p * b)  # 初始辅助角

    # Bowring迭代法（3次迭代可达毫米级精度）[5,8](@ref)
    for _ in range(3):
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        numerator = z + ep_sq * b * sin_theta ** 3
        denominator = p - e_sq * a * cos_theta ** 3
        theta = np.arctan2(numerator, denominator)

    # 最终纬度计算
    lat = np.arctan2(z + ep_sq * b * np.sin(theta) ** 3,
                     p - e_sq * a * np.cos(theta) ** 3)

    # 经度计算
    lon = np.arctan2(y, x)

    # 高程计算[8](@ref)
    N = a / np.sqrt(1 - e_sq * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N

    # 转换为角度
    lat_deg = np.degrees(lat)
    lon_deg = np.degrees(lon)

    return np.round(lat_deg, 9), np.round(lon_deg, 9), np.round(alt, 4)

def loadIE(path : str, all_data : dict, Ti0i1 : np.ndarray):
    fp = open(path,'rt')

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

        all_data[sod]['Tei1'] = Tei1 # lidar frame to e frame的变换矩阵，里面对应的平移就是lidar系在e系下的位置
    fp.close()
    return all_data

class GPSTimeConverter:
    def __init__(self):
        self.current_leap_seconds = 18
        self.gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    def convertUnix2Gps(self, unix_sec):
        """方法1：手动闰秒计算"""
        dt_aware = datetime.fromtimestamp(unix_sec, tz=timezone.utc)
        delta = dt_aware - self.gps_epoch
        total_seconds = delta.total_seconds() + self.current_leap_seconds
        return total_seconds % (7 * 86400)


def convert_unix_to_gps(unix_sec):
    """方法2：使用gpstime库计算"""
    dt_aware = datetime.fromtimestamp(unix_sec + 18, tz=timezone.utc)
    dt_naive_utc = dt_aware.replace(tzinfo=None)
    return GPSTime.from_datetime(dt_naive_utc).time_of_week

def process_roadside_bag(bag_path, topic_suffix):
    """处理路侧数据（带时间对比）"""
    converter = GPSTimeConverter()
    max_diff = 0.0
    total_diff = 0.0
    count = 0

    with rosbag.Bag(bag_path, 'r') as bag:
        pc_topics = get_roadside_pointcloud_topics(bag, topic_suffix)
        print(f"正在处理路侧点云话题: {pc_topics}")
        newTopic = pc_topics[0]

        for topic, msg, t in bag.read_messages():
            # 打印一下topic和pc_topics
            original_sec = msg.header.stamp.to_sec()

            # 两种方法计算结果
            gps_method1 = converter.convertUnix2Gps(original_sec)
            gps_method2 = convert_unix_to_gps(original_sec)

            # 计算差异
            diff = abs(gps_method1 - gps_method2)
            max_diff = max(max_diff, diff)
            total_diff += diff
            count += 1

            # # 打印对比结果
            # print(f"\n时间戳: {original_sec:.9f}")
            # print(f"方法1结果: {gps_method1:.6f}")
            # print(f"方法2结果: {gps_method2:.6f}")
            # print(f"差异: {diff:.6f}秒")

            # 使用其中一种方法更新消息（保持原逻辑）
            msg.header.stamp = rospy.Time.from_sec(gps_method1)
            yield newTopic, msg, msg.header.stamp

        # 打印统计信息
        if count > 0:
            print("\n=== 时间转换差异统计 ===")
            print(f"最大差异: {max_diff:.6f}秒")
            print(f"平均差异: {total_diff / count:.6f}秒")
            print(f"总数据点: {count}")

def get_roadside_pointcloud_topics(bag, topic_suffix):
    """获取所有PointCloud2类型的话题名称"""
    topic_info = bag.get_type_and_topic_info()
    return [
        topic.replace('/velodyne_points', f'/{topic_suffix}_velodyne_points')  # 修改话题名
        for topic, info in topic_info.topics.items()
        if info.msg_type == 'sensor_msgs/PointCloud2'
    ]

def merge_bags():
    all_data = {}
    iePath = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/Result/Reference/IE.txt'  # 2.20日采集的第一次观测结果

    Ti0i1 = np.array([
        [0.99888562, 0.02594205, -0.0394274, 0.03],
        [-0.02613833, 0.99964834, -0.00447079, 0.48],
        [0.03929756, 0.00549637, 0.99921244, 0.33],
        [0.0, 0.0, 0.0, 1.0]
    ])
    # Ti0i1为i1到i0的变换矩阵，也就是lidar frame to 大惯导 frame，最终返回的是lidar frame to n frame的变换矩阵
    groundTruth = loadIE(iePath, all_data, Ti0i1)

    """合并所有数据（关键修改点）"""
    with rosbag.Bag(output_bag_path, 'w') as outbag:

        # 处理GPS真值数据
        gps_count = 0
        epsilon = 1e-9
        for sod, data in groundTruth.items():
            if (abs((sod *10) % 1) < epsilon) and (sod < 377613.4):
                msg = NavSatFix()
                msg.header = Header()

                msg.header.stamp = rospy.Time.from_sec(sod)
                msg.header.frame_id = "gps"

                # 坐标转换核心逻辑 [1](@ref)
                tei1 = data['Tei1']
                x, y, z = tei1[0:3, 3]  # 提取平移分量
                # 使用 ecef_to_wgs84 函数进行转换
                lat1, lon1, alt1 = ecef_to_wgs84(x, y, z)  # ECEF转WGS84

                # 使用 pyproj 库进行转换
                ecef2lla = Transformer.from_crs(
                    "EPSG:4978",  # ECEF坐标系（X/Y/Z in meters）
                    "EPSG:4326",  # WGS84地理坐标系（lat/lon in degrees）
                    always_xy=True  # 确保输出顺序为(longitude, latitude)
                )
                lon2, lat2, alt2 = ecef2lla.transform(x, y, z)

                # 计算差异
                diff_lon = lon1 - lon2
                diff_lat = lat1 - lat2
                diff_alt = alt1 - alt2

                # # 打印结果
                # print(f"使用 ecef_to_wgs84 函数计算结果: 经度 = {lon1}, 纬度 = {lat1}, 高程 = {alt1}")
                # print(f"使用 pyproj 库计算结果: 经度 = {lon2}, 纬度 = {lat2}, 高程 = {alt2}")
                # print(f"差异: 经度 = {diff_lon}, 纬度 = {diff_lat}, 高程 = {diff_alt}")
                # 精度优化处理
                msg.latitude = np.round(lat1, 9)  # 保留9位小数 (~1mm精度)
                msg.longitude = np.round(lon1, 9)
                msg.altitude = np.round(alt1, 4)  # 高程保留4位小数

                # 协方差矩阵优化 [3](@ref)
                cov_matrix = [
                    0.0001, 0.0, 0.0,
                    0.0, 0.0001, 0.0,
                    0.0, 0.0, 0.001
                ]
                msg.position_covariance = cov_matrix
                msg.position_covariance_type = 1

                # 状态位设置优化
                msg.status.status = 0  # STATUS_FIX
                msg.status.service = 1  # SERVICE_GPS

                outbag.write('/gps/fix', msg, msg.header.stamp)
                gps_count += 1

        print(f"已写入 {gps_count} 条GPS真值消息")

        # 处理90m路侧数据
        roadside_count_90m = 0
        for topic, msg, t in process_roadside_bag(roadside90m_bag_path, '90m'):
            outbag.write(topic, msg, t)
            roadside_count_90m += 1
        print(f"已写入 {roadside_count_90m} 条90m路侧消息")

        # 处理180m路侧数据
        roadside_count_180m = 0
        for topic, msg, t in process_roadside_bag(roadside180m_bag_path, '180m'):
            outbag.write(topic, msg, t)
            roadside_count_180m += 1
        print(f"已写入 {roadside_count_180m} 条180m路侧消息")

        # 处理车载数据
        vehicle_count = 0
        with rosbag.Bag(vehicle_bag_path, 'r') as vbag:
            for topic, msg, t in vbag.read_messages():
                outbag.write(topic, msg, t)
                vehicle_count += 1
        print(f"已写入 {vehicle_count} 条车载消息")


if __name__ == '__main__':
    merge_bags()