import rosbag
import sensor_msgs.point_cloud2
import numpy as np
import struct
import tqdm
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
import open3d as o3d

lineModel = RANSACRegressor()
dd = np.loadtxt('stamp.log',delimiter=',')
print(dd.shape)
plt.plot(dd[:,0],dd[:,1]/1e9 - dd[:,0])
lineModel.fit(dd[:,0].reshape(-1, 1), (dd[:,1]/1e9 - dd[:,0]).reshape(-1, 1))
a1 = lineModel.estimator_.coef_[0][0]
b = lineModel.estimator_.intercept_[0]
plt.plot(dd[:,0],a1*dd[:,0] + b)

plt.show()


#!   uint8 INT8=1
#!   uint8 UINT8=2
#!   uint8 INT16=3
#!   uint8 UINT16=4
#!   uint8 INT32=5
#!   uint8 UINT32=6
#!   uint8 FLOAT32=7
#!   uint8 FLOAT64=8

dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32), 
              ('intensity', np.float32), ('ring', np.uint16), ('time', np.float32)]

def pointcloud2_to_array_optimized(cloud_msg):
    # 使用 NumPy 的 frombuffer 批量读取 PointCloud2 数据
    return np.frombuffer(cloud_msg.data, dtype=np.dtype(dtype_list))

fp_stamp = open('stamp_lidar.txt','wt')
bag = rosbag.Bag('lidar/lidar-VLP.bag')
for topic, msg, t in tqdm.tqdm(bag.read_messages()):
    gpst = (t.to_sec()-b)/(a1 + 1)
    # if gpst<200100: continue
    # if gpst>201900: break
    # if gpst<196416: continue
    # if gpst>197990: break
    cloud_arr = pointcloud2_to_array_optimized(msg)

    # packed_data = np.empty(len(cloud_arr), dtype=[('x', 'f4'), ('y', 'f4'), 
    #                                               ('z', 'f4'), ('intensity', 'f4'),
    #                                               ('ring', 'u2'), ('time', 'f4')])
    # # 将 cloud_arr 数据按顺序填充到 packed_data
    # packed_data['x'] = cloud_arr['x']
    # packed_data['y'] = cloud_arr['y']
    # packed_data['z'] = cloud_arr['z']
    # packed_data['intensity'] = cloud_arr['intensity']
    # packed_data['ring'] = cloud_arr['ring']
    # packed_data['time'] = cloud_arr['time']

    packed_data = np.empty(len(cloud_arr), dtype=[('x', 'f4'), ('y', 'f4'), 
                                                  ('z', 'f4'), ('intensity', 'f4')])
    # 将 cloud_arr 数据按顺序填充到 packed_data
    packed_data['x'] = cloud_arr['x']
    packed_data['y'] = cloud_arr['y']
    packed_data['z'] = cloud_arr['z']
    packed_data['intensity'] = cloud_arr['intensity']

    # 直接将 packed_data 写入文件
    with open('lidar_out/%d.bin' % int(round(t.to_sec()*1e9)), 'wb') as fp:
        fp.write(packed_data.tobytes())
    # fp = open('lidar_out/%d.bin' % int(round(t.to_sec()*1e9)), 'rb')
    # fp = open('/mnt/e/2021_12_22/laser/data/1640186156876742656.bin','rb')
    # while True:
    #     dd = fp.read(16)
    #     ss0 = struct.unpack('ffff',dd)
    #     dd = fp.read(2)
    #     ss1 = struct.unpack('H',dd)
    #     dd = fp.read(4)
    #     ss2 = struct.unpack('f',dd)
    #     print(ss0+ss1+ss2)
    #     input()


    # # 保存为 PCD 文件
    # points_array = np.column_stack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z']))
    # cloud = o3d.geometry.PointCloud()
    # cloud.points = o3d.utility.Vector3dVector(points_array)
    # o3d.io.write_point_cloud('lidar_pcd/%d.pcd' % int(round(t.to_sec()*1e9)), cloud)

    fp_stamp.writelines('%.4f,%d.bin\n' % ( (t.to_sec()-b)/(a1 + 1) ,int(round(t.to_sec()*1e9))))