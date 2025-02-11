import os
import pyquaternion
import numpy as np
import geoFunc.trans as trans
import matplotlib.pyplot as plt

def JPL2R(q):
    w0 = q[0]
    w1 = q[1]
    w2 = q[2]
    w = q[3]
    q_v = np.array([[w0],[w1],[w2]])
    q_x = np.array([[0,-w2,w1],[w2,0,-w0],[-w1,w0,0]])  
    R = (2* pow(w,2)-1)*np.eye(3,3) - 2 * w * q_x + 2* np.matmul(q_v,q_v.T)
    return R

all_data= {}
# fp = open(r'Z:\kitti-360\2013_05_28_drive_0000_sync\ov_estimate.txt','rt')
# fp = open(r'Z:\kitti-360\results\ov_estimate_urban27_calib.txt','rt')/
# fp = open(r'\\JIGE\1ground_vio_results\ov_estimate_urban28_calib.txt','rt')
fp = open(r'/media/zhao/ZhaoZhibo/AllData/CalibrationData/2025_0103/ov_estimate_cam2tracial.txt','rt')
# fp = open(r'./ov_estimate_hit.txt','rt')
# fp = open(r'./ov_estimate.txt','rt')
# fp = open(r'./ov_estimate_0403_small.txt','rt')
# fp = open(r'./ov_estimate_0403_big.txt','rt')
# fp = open(r'./ov_estimate_1023.txt','rt')
# fp = open(r'./ov_estimate_0711.txt','rt')
# fp = open(r'./ov_estimate_urban28.txt','rt')
while True:
    line = fp.readline().strip()
    if line == '': break
    if line[0] == '#' : continue
    elem = line.split(' ')
    dt = elem[17]

    Ric = pyquaternion.Quaternion([float(elem[30]),float(elem[27]),float(elem[28]),float(elem[29])]).rotation_matrix
    tci = np.array([float(elem[31]),float(elem[32]),float(elem[33])])
    Tci = np.eye(4,4)
    Tci[0:3,0:3] = Ric.T
    Tci[0:3,3] = tci
    Tic = np.linalg.inv(Tci)
    Tic0 = Tic
    Ric = pyquaternion.Quaternion([float(elem[30+15]),float(elem[27+15]),float(elem[28+15]),float(elem[29+15])]).rotation_matrix
    tci = np.array([float(elem[31+15]),float(elem[32+15]),float(elem[33+15])])
    Tci = np.eye(4,4)
    Tci[0:3,0:3] = Ric.T
    Tci[0:3,3] = tci
    Tic = np.linalg.inv(Tci)
    Tic1 = Tic
    all_data[float(elem[0])] = {'Tic0':Tic0,'Tic1':Tic1}
fp.close() 

is_ref_set = False
t_series=[]
ric0x_series=[]
ric0y_series=[]
ric0z_series=[]
tic0x_series=[]
tic0y_series=[]
tic0z_series=[]
rc0c1x_series=[]
rc0c1y_series=[]
rc0c1z_series=[]
tc0c1x_series=[]
tc0c1y_series=[]
tc0c1z_series=[]
for sow in sorted(all_data.keys()):
    _Tic0 = all_data[sow]['Tic0']
    _Tic1 = all_data[sow]['Tic1']
    _Tc0c1 = np.matmul(np.linalg.inv(_Tic0),_Tic1)
    if is_ref_set == False:
        _Tic0_ref = _Tic0
        _Tic1_ref = _Tic1
        _Tc0c1_ref = _Tc0c1
        is_ref_set = True
    eTic0 = _Tic0_ref @ np.linalg.inv(_Tic0)
    eTc0c1 = _Tc0c1_ref @ np.linalg.inv(_Tc0c1)
    eaic0 = trans.m2att(eTic0[0:3,0:3])
    etic0 = _Tic0[0:3,3] # attention!
    eac0c1 = trans.m2att(eTc0c1[0:3,0:3])
    etc0c1 = _Tc0c1[0:3,3] # attention!
    ric0x_series.append(eaic0[0]*57.3)
    ric0y_series.append(eaic0[1]*57.3)
    ric0z_series.append(eaic0[2]*57.3)
    tic0x_series.append(etic0[0])
    tic0y_series.append(etic0[1])
    tic0z_series.append(etic0[2])
    rc0c1x_series.append(eac0c1[0]*57.3)
    rc0c1y_series.append(eac0c1[1]*57.3)
    rc0c1z_series.append(eac0c1[2]*57.3)
    tc0c1x_series.append(etc0c1[0])
    tc0c1y_series.append(etc0c1[1])
    tc0c1z_series.append(etc0c1[2])
    t_series.append(sow)
plt.figure()
plt.subplot(2,2,1)
plt.plot(t_series,ric0x_series,c='r')
plt.plot(t_series,ric0y_series,c='g')
plt.plot(t_series,ric0z_series,c='b')
plt.subplot(2,2,2)
plt.plot(t_series,tic0x_series,c='r')
plt.plot(t_series,tic0y_series,c='g')
plt.plot(t_series,tic0z_series,c='b')
plt.subplot(2,2,3)
plt.plot(t_series,rc0c1x_series,c='r')
plt.plot(t_series,rc0c1y_series,c='g')
plt.plot(t_series,rc0c1z_series,c='b')
plt.subplot(2,2,4)
plt.plot(t_series,tc0c1x_series,c='r')
plt.plot(t_series,tc0c1y_series,c='g')
plt.plot(t_series,tc0c1z_series,c='b')
plt.show()
    

Tc0c1 = np.matmul(np.linalg.inv(Tic0),Tic1)
print(dt,'%.6f,%.6f,%.6f,%.6f'%tuple(np.array(elem[19:23]).astype(np.float64)),\
      '%.6f,%.6f,%.6f,%.6f'%tuple(np.array(elem[23:27]).astype(np.float64)))
print(dt,'%.6f,%.6f,%.6f,%.6f'%tuple(np.array(elem[34:38]).astype(np.float64)),\
      '%.6f,%.6f,%.6f,%.6f'%tuple(np.array(elem[38:42]).astype(np.float64)))
print('Tic[0]:\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,' % tuple(Tic0.reshape(-1)))
print('Tic[1]:\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,' % tuple(Tic1.reshape(-1)))
print('Tcc[0]:\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,\n\
%.8f,%.8f,%.8f,%.8f,' % tuple(Tc0c1.reshape(-1)))