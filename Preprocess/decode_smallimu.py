import math
import tqdm


fp = open('imu_adis.txt','rt')
fp_out = open('smallimu_out.txt','wt')
fp_csv_out = open('imu0.csv','wt')
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