# config.py
# 选择参考时间戳,也就是slam的起始时刻的时间戳,而不是slam的第一个时刻的时间戳 这样是为了将所有位置和姿态转换到这个时刻下
# reference_timestamp = 382122.5201  # slam时刻的时间戳，这个位置的真值是固定节
reference_timestamp = 377301.1218 # slam起始位置的时间戳，这个位置的真值是浮点解
slam_in_absolute_n_frame = True
