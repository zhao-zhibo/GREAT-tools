import subprocess
import os

# 根据操作系统选择路径分隔符
if os.name == 'nt':  # Windows
    common_path = 'H:\\AllData\\tunnelRoadside\\data_2025220181111' + '\\'
    python_cmd = 'python'
else:  # Ubuntu
    common_path = '/media/zhao/ZhaoZhibo1T/AllData/Calibration_0301/data_20253117310' + '/'
    python_cmd = 'python3'

# 检查并创建目录
after_preprocess_path = os.path.join(common_path, 'AfterPreProcess')
if not os.path.exists(after_preprocess_path):
    os.makedirs(after_preprocess_path)

# 定义要执行的脚本
scripts = [
    'decode_bigimu.py',
    # 'decode_smallimu.py',
    # 'decode_lidar.py',
    # 'rearrange_img.py'
]

# 依次执行每个脚本
for script in scripts:
    subprocess.run([python_cmd, script, common_path])
    print(f"{script} 执行完成了")
    print("-" * 80)  # 打印一条长的横线

# 打印所有脚本执行完成
print("所有脚本执行完成了")