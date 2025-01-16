import os
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor
import math
import shutil
import tqdm

x_series = [] 
y_series = [] 
fp = open('stamp.log','rt')
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

for camid in ['cam0','cam1']:
    img_list = os.listdir('image/%s'%camid)

    x_series = []
    tt_series = [] 
    f_list = []
    fp = open('stamp_%s.txt'%camid,'rt')
    lines = fp.readlines()
    for iline in tqdm.tqdm(range(1,len(lines)-1)):
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

    if not os.path.isdir('image_rearrange'):
        os.mkdir('image_rearrange')
    if not os.path.isdir('image_rearrange/%s'%camid):
        os.mkdir('image_rearrange/%s'%camid)

    fp = open('stamp_rearrange_campus_%s.txt'%camid,'wt')
    for i in tqdm.tqdm(range(len(f_list)), desc='Processing'):
        if 'DROPFRAME' in f_list[i]: continue
        tt_ns = int(round((i*0.1 + t0)*1e3)*1e6)
        # if i*0.1 + t0< 200100 or i*0.1 + t0>201900: continue
        # if i*0.1 + t0< 196416 or i*0.1 + t0>197990: continue

        fp.writelines('%.6f,%d.jpg\n'%(i*0.1 + t0,tt_ns))
        shutil.copy(os.path.join('image/%s'%camid,f_list[i]),os.path.join('image_rearrange/%s'%camid,'%d.jpg'%tt_ns))
    fp.close()

# plt.plot