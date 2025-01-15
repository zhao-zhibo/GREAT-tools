# import numpy as np
# import open3d as o3d
# import os
# import bisect
# import quaternion
# import pickle
# import tqdm
# from scipy.spatial.transform import Rotation
#
# def create_point_actor(points, colors=None):
#     """ open3d point cloud from numpy array """
#     point_cloud = o3d.geometry.PointCloud()
#     point_cloud.points = o3d.utility.Vector3dVector(points)
#     if colors is not None:
#         point_cloud.colors = o3d.utility.Vector3dVector(colors)
#     return point_cloud
#
#
# binary = open('/home/yuan/ysr/data/lidar2camer/Lidar/data/1735872315727048704.bin','rb')
# scan = np.fromfile(binary, dtype=np.float32).reshape((-1,4))
# points = scan[:, 0:3] # lidar xyz (front, left, up)
# remissions = scan[:, 3]
# remissions_normalized = (remissions - np.min(remissions)) / (np.max(remissions) - np.min(remissions))
# colors = np.stack([remissions_normalized] * 3, axis=-1)
# pcd = create_point_actor(points,colors)
# o3d.io.write_point_cloud("/home/yuan/ysr/data/lidar2camer/Lidar/pcd/%d.pcd"%441947800000000, pcd)


import os
import struct
import numpy as np
import open3d as o3d


# Example usage
input_file = '/home/zhao/Codes/Calibration/SensorsCalibration/src/SensorsCalibration/lidar2camera/manual_calib/data/GREAT_01114_02/1736846217147320064.bin'
output_file = '/home/zhao/Codes/Calibration/SensorsCalibration/src/SensorsCalibration/lidar2camera/manual_calib/data/GREAT_01114_02/206253600000000.pcd'

def load_bin(file_path):
    pointcloud = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    return pointcloud


# def save_pointcloud(pointcloud_np, file_name):
#     point_cloud_o3d = o3d.geometry.PointCloud()
#     point_cloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_np[:, 0:3])
#     o3d.io.write_point_cloud(file_name, point_cloud_o3d, write_ascii=False, compressed=True)


def save_pointcloud(pointcloud_np, method, file_name):
    """
    1) save pointcloud as npy/bin format using numpy API
    2) save pointclodu as pcd format using python API
    """
    if method == "npy":
        np.save(file_name, pointcloud_np)
    elif method == "bin":
        pointcloud_np.tofile(file_name)
    elif method == "pcd":
        with open(file_name, 'w') as f:
            f.write("# .PCD v.7 - Point Cloud Data file format\n")
            f.write("VERSION .7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write("WIDTH {}\n".format(pointcloud_np.shape[0]))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS {}\n".format(pointcloud_np.shape[0]))
            f.write("DATA binary\n")
            pointcloud_np.tofile(f)

            # ASCII
            # f.write("DATA ascii\n")
            # for i in range(pointcloud.shape[0]):
            #     f.write(
            #         str(pointcloud[i][0]) + " " + str(pointcloud[i][1]) + " " + str(pointcloud[i][2]) + " " + str(
            #             pointcloud[i][3]) + "\n")
points=load_bin(input_file)
save_pointcloud(points,"pcd",output_file)
print("转换完成！")