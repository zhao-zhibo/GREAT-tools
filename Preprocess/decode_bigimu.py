import tqdm
import os
import sys

# 检查是否提供了路径入参
if len(sys.argv) < 2:
    print("Error: No path argument provided!", file=sys.stderr)
    sys.exit(1)
# 获取路径入参
base_path = sys.argv[1]

# 检查并创建目录，根据操作系统选择路径分隔符
if os.name == 'nt':  # Windows
    imu_path = os.path.join(base_path, 'AfterPreProcess\\IMU\\')
else:  # Ubuntu
    imu_path = os.path.join(base_path, 'AfterPreProcess/IMU/')
if not os.path.exists(imu_path):
    os.makedirs(imu_path)

fp = open(os.path.join(base_path, 'bigimu.txt'), 'rt')
fp_out = open(os.path.join(imu_path, 'Tactical_imu_data.txt'), 'wt')
lines = fp.readlines()
for iline in tqdm.tqdm(range(1,len(lines)-1)):
    line =lines[iline].strip().strip()
    elem = line.split(',')
    if len(elem) != 10: continue
    week = int(elem[1])
    t = float(elem[2])
    gx = -float(elem[3])
    gy = -float(elem[4])
    gz = float(elem[5])
    ax = -float(elem[6])* 9.80144145
    ay = -float(elem[7])* 9.80144145
    az = float(elem[8])* 9.80144145
    fp_out.writelines('%.5f %.10f %.10f %.10f %.10f %.10f %.10f\n'%(t,gx,gy,gz,ax,ay,az))
fp.close()
fp_out.close()