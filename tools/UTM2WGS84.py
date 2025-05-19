import numpy as np
from pyproj import Transformer, Proj
import utm

def decimal_to_dms(deg):
    """将十进制度转换为度分秒（DMS）格式"""
    d = int(deg)
    m = int((deg - d) * 60)
    s = (deg - d - m / 60) * 3600
    return d, m, s

# 输入UTM坐标（武汉市Zone 49N）
easting, northing, altitude = 513500, 3364400, 0  # UTM坐标及椭球高（假设z=0）
# easting, northing, altitude = 513474.8480, 3364479.3410, 0  # UTM坐标及椭球高（假设z=0）

# -------------------------- 步骤1：UTM转WGS84经纬度（双重验证）--------------------------
# 方法1：pyproj转换
transformer_utm_to_wgs84 = Transformer.from_crs("epsg:4547", "epsg:4326")
lat_pyproj,lon_pyproj = transformer_utm_to_wgs84.transform(easting, northing)

# 方法2：utm库转换
lat_utm, lon_utm = utm.to_latlon(easting, northing, zone_number=50, zone_letter='N')

# 验证经纬度一致性
# assert np.allclose([lon_pyproj, lat_pyproj], [lon_utm, lat_utm], atol=1e-9), "经纬度转换不一致！"
print(f"WGS84经纬度（已验证一致）:\n 经度={lon_pyproj:.10f}°, 纬度={lat_pyproj:.10f}°, 高程={altitude}米")

# 转换为度分秒格式
lat_d, lat_m, lat_s = decimal_to_dms(lat_pyproj)
lon_d, lon_m, lon_s = decimal_to_dms(lon_pyproj)

print(f"纬度: {lat_d}°{lat_m}'{lat_s:.2f}\"")
print(f"经度: {lon_d}°{lon_m}'{lon_s:.2f}\"")

# -------------------------- 步骤2：WGS84经纬度转ECEF XYZ（双重方法）--------------------------
# 方法1：pyproj直接转换（推荐）
wgs84_geodetic = Proj(proj='latlong', ellps='WGS84', datum='WGS84')  # 定义地理坐标系
wgs84_geocentric = Proj(proj='geocent', ellps='WGS84', datum='WGS84')  # 定义地心坐标系

# 使用pyproj将经纬度（单位：度）转为ECEF XYZ（单位：米）
x_pyproj, y_pyproj, z_pyproj = wgs84_geocentric.transform(
    114.1404822, 30.3994217, 0,
    radians=False  # 输入为度数，非弧度
)
print(f"转换后的ECEF XYZ坐标:\n  X = {x_pyproj:.3f}米\n  Y = {y_pyproj:.3f}米\n  Z = {z_pyproj:.3f}米")


# 使用pyproj将经纬度（单位：度）转为ECEF XYZ（单位：米）
x_pyproj, y_pyproj, z_pyproj = wgs84_geocentric.transform(
    # 114.146383022026, 30.3965858919968, 11.1162952403245, # 490秒开始的bag包起点对应的原点经纬度
    114.14521846596, 30.3974618860168, 10.5882026274912, # 690秒开始的bag包起点对应的原点经纬度
    radians=False  # 输入为度数，非弧度
)
print(f"转换后的原点ECEF XYZ坐标:\n  X = {x_pyproj:.3f}米\n  Y = {y_pyproj:.3f}米\n  Z = {z_pyproj:.3f}米")
#
# # 方法2：手动公式计算（双重验证）
# # WGS84椭球参数
# a = 6378137.0  # 长半轴（米）
# f = 1 / 298.257223563  # 扁率
# e2 = 2*f - f**2  # 第一偏心率平方
#
# # 将经纬度转为弧度
# lat_rad = np.deg2rad(lat_pyproj)
# lon_rad = np.deg2rad(lon_pyproj)
#
# # 计算卯酉圈曲率半径
# N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
#
# # ECEF坐标计算公式
# x_manual = (N + altitude) * np.cos(lat_rad) * np.cos(lon_rad)
# y_manual = (N + altitude) * np.cos(lat_rad) * np.sin(lon_rad)
# z_manual = (N * (1 - e2) + altitude) * np.sin(lat_rad)
#
# # 输出结果并对比
# print("\nECEF XYZ坐标（pyproj）:")
# print(f"  X = {x_pyproj:.3f}米\n  Y = {y_pyproj:.3f}米\n  Z = {z_pyproj:.3f}米")
#
# print("\nECEF XYZ坐标（手动公式）:")
# print(f"  X = {x_manual:.3f}米\n  Y = {y_manual:.3f}米\n  Z = {z_manual:.3f}米")
#
# # 计算两种方法的差值
# diff_x = abs(x_pyproj - x_manual)
# diff_y = abs(y_pyproj - y_manual)
# diff_z = abs(z_pyproj - z_manual)
# print(f"\n坐标差值（验证精度）:\n  ΔX = {diff_x:.6f}米\n  ΔY = {diff_y:.6f}米\n  ΔZ = {diff_z:.6f}米")
#
# # -------------------------- 步骤3：反向验证（ECEF XYZ → WGS84经纬度 → UTM）--------------------------
# # 使用pyproj将ECEF转回地理坐标
# lon_rev, lat_rev, alt_rev = wgs84_geodetic.transform(
#     x_pyproj, y_pyproj, z_pyproj,
#     radians=False
# )
#
# # 将经纬度转回UTM
# transformer_wgs84_to_utm = Transformer.from_crs("epsg:4326", "epsg:32649")
# easting_rev, northing_rev = transformer_wgs84_to_utm.transform(lon_rev, lat_rev)
#
# # 计算反向误差
# error_easting = easting - easting_rev
# error_northing = northing - northing_rev
# print(f"\n反向转换UTM误差:\n  Δ东向 = {error_easting:.6f}米\n  Δ北向 = {error_northing:.6f}米")