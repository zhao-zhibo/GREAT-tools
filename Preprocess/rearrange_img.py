import os
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor
import math
import shutil
import tqdm
import sys

# 检查是否提供了路径入参
if len(sys.argv) < 2:
    print("Error: No path argument provided!", file=sys.stderr)
    sys.exit(1)

# 获取路径入参
base_path = sys.argv[1]

# 检查并创建目录
image_path = os.path.join(base_path, 'AfterPreProcess/Image/')
if not os.path.exists(image_path):
    os.makedirs(image_path)

x_series = [] 
y_series = [] 
fp = open(os.path.join(base_path, 'stamp.log'), 'rt')
lines = fp.readlines()
for iline in tqdm.tqdm(range(1,len(lines)-1)):
    line = lines[iline].strip()
    elem = line.split(',')
    y_series.append(float(elem[0]))
    x_series.append(float(elem[1])/1e9)
fp.close()
x_series = np.array(x_series)
y_series = np.array(y_series)
lineModel = RANSACRegressor()
lineModel.fit(x_series.reshape(-1, 1), y_series.reshape(-1, 1))
a1 = lineModel.estimator_.coef_[0][0]
b = lineModel.estimator_.intercept_[0]
a1 = 1
b = -(x_series[0]-y_series[0])
plt.plot(x_series,a1*x_series + b - y_series)
plt.show()

for camid in ['img0','img1']:
    # 检查并创建目录
    imageXStampPath = os.path.join(image_path, camid)
    if not os.path.exists(imageXStampPath):
        os.makedirs(imageXStampPath)
    img_list = os.listdir(os.path.join(imageXStampPath))

    imageXDataPath = os.path.join(imageXStampPath, 'data')
    if not os.path.exists(imageXDataPath):
        os.makedirs(imageXDataPath)

    x_series = []
    tt_series = [] 
    f_list = []
    fp = open(base_path + 'stamp_cam%s.txt' % camid[-1], 'rt')
    lines = fp.readlines()
    for iline in tqdm.tqdm(range(0,len(lines)-1)):
        line = lines[iline].strip()
        if 'DROPFRAME' in line:
            x_series.append(0.0)
            tt_series.append(tt_series[-1])
            f_list.append('DROPFRAME')
            continue
        elem = line.split(',')
        tt = float(elem[0])/1e9
        x_series.append(a1*tt+ b)
        tt_series.append(tt)
        f_list.append(elem[1])
    fp.close()
    x_series = np.array(x_series)
    y_series =0.1 * np.arange(x_series.shape[0])


    lineModel = RANSACRegressor()
    lineModel.fit(y_series.reshape(-1, 1), x_series.reshape(-1, 1))
    t0 = math.floor(lineModel.predict(np.array([[0]]))[0,0]*10.0)/10
    plt.plot(tt_series, x_series - (lineModel.predict(y_series.reshape(-1,1)).reshape(-1)-lineModel.predict(np.array([[0]]))[0,0] + t0)) # this is the "correct" system time
    # plt.xlim(y_series[0],y_series[-1])
    plt.ylim(-10,10)
    plt.figure()
    plt.boxplot(x_series - (lineModel.predict(y_series.reshape(-1,1)).reshape(-1)-lineModel.predict(np.array([[0]]))[0,0] + t0))
    plt.ylim(-0.1,0.1)
    plt.show()

    # if not os.path.isdir('image_rearrange'):
    #     os.mkdir('image_rearrange')
    # if not os.path.isdir('image_rearrange/%s'%camid):
    #     os.mkdir('image_rearrange/%s'%camid)

    fp = open(os.path.join(imageXStampPath, 'timestamps.txt'), 'wt')
    for i in tqdm.tqdm(range(len(f_list)), desc='Processing'):
        if 'DROPFRAME' in f_list[i]: continue
        tt_ns = int(round((i*0.1 + t0)*1e3)*1e6)
        # if i*0.1 + t0< 200100 or i*0.1 + t0>201900: continue
        # if i*0.1 + t0< 196416 or i*0.1 + t0>197990: continue

        fp.writelines('%.6f,%d.jpg\n'%(i*0.1 + t0,tt_ns))
        shutil.copy(os.path.join(base_path,'image/cam%s' % camid[-1],f_list[i]),os.path.join(imageXDataPath,'%d.jpg'%tt_ns))
    fp.close()

# plt.plot