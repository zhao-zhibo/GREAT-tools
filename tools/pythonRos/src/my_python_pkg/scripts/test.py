#!/usr/bin/env python3
import rospy
from datetime import datetime, timezone
from gps_time import GPSTime


class GPSTimeConverter:
    def __init__(self):
        # 手动设置当前闰秒数（截至2023年12月为18秒）
        self.current_leap_seconds = 18

        # GPS起始时间（1980-01-06 00:00:00 UTC）
        self.gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    def convert_unix_to_gps(self, unix_sec):
        """手动处理闰秒的时间转换"""
        # 创建带时区的UTC时间对象
        dt_aware = datetime.fromtimestamp(unix_sec, tz=timezone.utc)

        # 计算总时间差
        delta = dt_aware - self.gps_epoch

        # 添加当前闰秒数
        total_seconds = delta.total_seconds() + self.current_leap_seconds

        # 计算周内秒
        week = int(total_seconds // (7 * 86400))
        tow = total_seconds % (7 * 86400)

        return tow


def main():
    rospy.init_node('gps_time_converter', anonymous=True)
    converter = GPSTimeConverter()
    rate = rospy.Rate(1)  # 1Hz

    try:
        while not rospy.is_shutdown():
            # 获取ROS时间并转换
            ros_sec = rospy.Time.now().to_sec()
            utc_from_ros = datetime.fromtimestamp(ros_sec, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            gps_tow_ros = converter.convert_unix_to_gps(ros_sec)

            # 获取系统UTC时间并转换
            sys_utc = datetime.now(timezone.utc)
            sys_utc_sec = sys_utc.timestamp()
            utc_from_sys = sys_utc.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            gps_tow_sys = converter.convert_unix_to_gps(sys_utc_sec)

            # 打印结果
            print("\n=== GPS时间实时转换 ===")
            print("[基于ROS时间戳]")
            print(f"ROS时间戳: {ros_sec:.9f}")
            print(f"转换得到UTC时间: {utc_from_ros}")
            print(f"GPS周内秒: {gps_tow_ros:.3f}")

            print("\n[基于系统UTC时间]")
            print(f"直接获取UTC时间: {utc_from_sys}")
            print(f"对应GPS周内秒: {gps_tow_sys:.3f}")
            print("=" * 40)

            rate.sleep()

    except Exception as e:
        rospy.logerr(f"转换错误: {str(e)}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass