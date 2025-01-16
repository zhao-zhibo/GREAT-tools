import tqdm

fp = open('bigimu.txt','rt')
fp_out = open('bigimu_out.txt','wt')
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