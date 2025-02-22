import math
import tqdm
import os
import sys

# 检查是否提供了路径入参
if len(sys.argv) < 2:
    print("Error: No path argument provided!", file=sys.stderr)
    sys.exit(1)

# 获取路径入参
base_path = sys.argv[1]

# 检查并创建目录
imu_path = os.path.join(base_path, 'AfterPreProcess/IMU/')
if not os.path.exists(imu_path):
    os.makedirs(imu_path)

# 打开文件
fp = open(os.path.join(base_path, 'imu_adis.txt'), 'rt')
fp_out = open(os.path.join(imu_path, 'MEMS_imu_data.txt'), 'wt')
fp_csv_out = open(os.path.join(imu_path, 'imu0.csv'), 'wt')

lines = fp.readlines()
for iline in tqdm.tqdm(range(1,len(lines)-1)):
    line = lines[iline].strip()
    elem = line.split(' ')
    if len(elem) != 7: continue
    t = float(elem[0])
    gx = -float(elem[1])
    gy = -float(elem[2])
    gz = float(elem[3])
    ax = -float(elem[4])
    ay = -float(elem[5])
    az = float(elem[6]) 
    fp_out.writelines('%.5f %.10f %.10f %.10f %.10f %.10f %.10f\n'%(t,gx,gy,gz,ax,ay,az))
    fp_csv_out.writelines('%d,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n'%(int(round(t*1e3)*1e6),gx*math.pi/180,gy*math.pi/180,gz*math.pi/180,ax,ay,az))
fp.close()
fp_out.close()
fp_csv_out.close()