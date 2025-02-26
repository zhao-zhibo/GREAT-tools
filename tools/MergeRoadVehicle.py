#!/usr/bin/env python3
import rosbag
import rospy
from datetime import datetime, timezone, timedelta  # Added timedelta import
from gps_time import GPSTime
import os
import pytz

# 配置参数
roadside90m_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/Yuan_90m/2_2025-02-20-16-52-37.bag'
roadside180m_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/Xu_180m/2_2025-02-20-16-52-45.bag'
vehicle_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/AfterPreProcess_mems.bag'
output_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/mergeVehicleRoad.bag'

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
    """合并所有数据（关键修改点）"""
    with rosbag.Bag(output_bag_path, 'w') as outbag:
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