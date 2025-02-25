#!/usr/bin/env python3
import rosbag
import rospy
from datetime import datetime, timedelta
import pytz
from gps_time import GPSTime
import os

# 配置参数
roadside_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/Yuan_90m/2_2025-02-20-16-52-37.bag'
vehicle_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/AfterPreProcess_mems.bag'
output_bag_path = '/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/data_2025220163953/AfterPreProcess/mergeVehicleRoad.bag'


def get_roadside_pointcloud_topics(bag):
    """获取所有PointCloud2类型的话题名称"""
    topic_info = bag.get_type_and_topic_info()
    return [
        topic for topic, info in topic_info.topics.items()
        if info.msg_type == 'sensor_msgs/PointCloud2'
    ]


def detect_timezone(roadside_bag):
    """修正版时区检测（基于消息类型而非话题名称）"""
    # 获取所有PointCloud2类型的话题
    pc_topics = get_roadside_pointcloud_topics(roadside_bag)
    if not pc_topics:
        raise ValueError("路侧bag中未找到PointCloud2类型的话题")

    # 从文件名解析预期北京时间
    filename_str = os.path.basename(roadside_bag_path)[2:21]
    filename_dt = datetime.strptime(filename_str, "%Y-%m-%d-%H-%M-%S")
    filename_beijing = pytz.timezone('Asia/Shanghai').localize(filename_dt)

    # 查找首帧点云消息
    first_cloud_time = None
    for topic, msg, _ in roadside_bag.read_messages(topics=pc_topics):
        first_cloud_time = msg.header.stamp.to_sec()
        break

    if first_cloud_time is None:
        raise ValueError("路侧bag中没有PointCloud2数据")

    # 计算两种假设的时间差
    original_dt = datetime.utcfromtimestamp(first_cloud_time)

    # 假设1：时间戳是UTC时间，对应北京时间 = UTC + 8h
    beijing_from_utc = original_dt + timedelta(hours=8)
    error_utc = abs(beijing_from_utc - filename_beijing.replace(tzinfo=None))

    # 假设2：时间戳是北京时间，对应UTC时间 = 时间戳 - 8h
    utc_from_local = original_dt - timedelta(hours=8)
    error_local = abs(utc_from_local - filename_beijing.astimezone(pytz.utc).replace(tzinfo=None))

    print(f"[Debug] UTC假设误差: {error_utc.total_seconds()}秒")
    print(f"       本地时间假设误差: {error_local.total_seconds()}秒")

    return 'UTC' if error_utc < error_local else 'Asia/Shanghai'


def convert_to_gps_seconds(dt, timezone):
    """时间转换（保持原逻辑）"""
    if timezone == 'Asia/Shanghai':
        dt = dt - timedelta(hours=8)
    gps_time = GPSTime.from_datetime(dt)
    return gps_time.time_of_week


def process_roadside_bag():
    """处理路侧数据（基于消息类型）"""
    with rosbag.Bag(roadside_bag_path, 'r') as bag:
        timezone = detect_timezone(bag)
        pc_topics = get_roadside_pointcloud_topics(bag)

        print(f"正在处理路侧点云话题: {pc_topics}")

        for topic, msg, t in bag.read_messages():
            if topic in pc_topics:
                # 时间转换逻辑
                original_sec = msg.header.stamp.to_sec()
                original_dt = datetime.utcfromtimestamp(original_sec)
                gps_seconds = convert_to_gps_seconds(original_dt, timezone)

                # 更新时间戳
                msg.header.stamp = rospy.Time.from_sec(gps_seconds)  # 使用rospy.Time
                yield topic, msg, msg.header.stamp
            else:
                yield topic, msg, t

def merge_bags():
    """合并所有数据（关键修改点）"""
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        # 先写入路侧数据（调整顺序）
        roadside_count = 0
        for topic, msg, t in process_roadside_bag():
            outbag.write(topic, msg, t)
            roadside_count += 1
        print(f"已写入 {roadside_count} 条路侧消息")

        # 再写入车载数据（顺序调整）
        vehicle_count = 0
        with rosbag.Bag(vehicle_bag_path, 'r') as vbag:
            for topic, msg, t in vbag.read_messages():
                outbag.write(topic, msg, t)
                vehicle_count += 1
        print(f"已写入 {vehicle_count} 条车载消息")


if __name__ == '__main__':
    merge_bags()