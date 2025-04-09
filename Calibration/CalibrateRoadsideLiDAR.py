import os
import pandas as pd
import numpy as np

# 根据操作系统设置基础路径
if os.name == 'nt':  # Windows
    base_path = r"H:\AllData\tunnelRoadside\HDMap"
else:  # Ubuntu
    base_path = "/media/zhao/ZhaoZhibo1T/AllData/tunnelRoadside/HDMap"


# 配置参数
CONFIG = {
    "base_path": base_path,
    "enu_file": "90m_180m_UTM.txt",
    "lidar_file": "90m_180m_LiDAR.txt",
    "origin_subtract": (513500, 3364400, 0),  # ENU坐标系平移量
    "target_prefixes": ["180_", "90_"],  # 待标定的LiDAR前缀
    "columns": ['Name', 'X', 'Y', 'Z', 'AGL', 'INFO', 'SUBFOLDER']  # 数据列定义
}


def clean_numeric(s):
    """清洗数值数据，处理尾部空格和空值"""
    try:
        return float(str(s).replace('m', '').strip())
    except:
        return 0.0


def load_enu_data():
    """加载并处理ENU数据"""
    file_path = os.path.join(CONFIG["base_path"], CONFIG["enu_file"])

    # 读取原始数据
    df = pd.read_csv(
        file_path,
        header=None,
        skiprows=0,
        names=CONFIG["columns"],
        converters={
            'X': clean_numeric,
            'Y': clean_numeric,
            'Z': clean_numeric
        }
    )

    # 执行坐标系平移
    df['X'] = df['X'] - CONFIG["origin_subtract"][0]
    df['Y'] = df['Y'] - CONFIG["origin_subtract"][1]
    df['Z'] = df['Z'] - CONFIG["origin_subtract"][2]

    return df


def load_lidar_data():
    """加载并处理LiDAR数据"""
    file_path = os.path.join(CONFIG["base_path"], CONFIG["lidar_file"])

    df = pd.read_csv(
        file_path,
        header=None,
        skiprows=0,
        names=CONFIG["columns"],
        converters={
            'X': clean_numeric,
            'Y': clean_numeric,
            'Z': clean_numeric
        }
    )

    return df


def align_points(enu_df, lidar_df, prefix):
    """对齐特征点并获取原点"""
    try:
        # # 修改点：添加对Large的过滤
        # filter_condition = lambda x: (~x.str.contains('Origin')) & (~x.str.contains('Large'))

        # 修改为（仅排除Origin）:
        filter_condition = lambda x: ~x.str.contains('Origin')  # 移除了对Large的过滤

        # 过滤有效点（排除Origin和Large）
        enu_sub = enu_df[
            enu_df['Name'].str.startswith(prefix) &
            filter_condition(enu_df['Name'])
            ].copy()

        lidar_sub = lidar_df[
            lidar_df['Name'].str.startswith(prefix) &
            filter_condition(lidar_df['Name'])
            ].copy()

        # 合并同名点
        merged = pd.merge(enu_sub, lidar_sub, on='Name', suffixes=('_enu', '_lidar'))
        if merged.empty:
            raise ValueError("没有匹配的同名特征点")

        # 提取坐标点
        enu_points = merged[['X_enu', 'Y_enu', 'Z_enu']].values.astype(float)
        lidar_points = merged[['X_lidar', 'Y_lidar', 'Z_lidar']].values.astype(float)

        # 获取原点坐标
        enu_origin = enu_df[enu_df['Name'] == f'{prefix}Origin'][['X', 'Y', 'Z']].values[0]
        lidar_origin = lidar_df[lidar_df['Name'] == f'{prefix}Origin'][['X', 'Y', 'Z']].values[0]

        return enu_points, lidar_points, enu_origin, lidar_origin

    except Exception as e:
        print(f"对齐{prefix}LiDAR数据时发生错误: {str(e)}")
        return None, None, None, None
def compute_transformation(src_points, tgt_points):
    """计算LiDAR到ENU的刚体变换矩阵"""
    # 计算质心
    src_centroid = np.mean(src_points, axis=0)
    tgt_centroid = np.mean(tgt_points, axis=0)

    # 中心化坐标
    src_centered = src_points - src_centroid
    tgt_centered = tgt_points - tgt_centroid

    # SVD分解求旋转矩阵
    H = src_centered.T @ tgt_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 确保右手坐标系
    if np.linalg.det(R) < 0:
        Vt[2] *= -1
        R = Vt.T @ U.T

    # 计算平移向量
    t = tgt_centroid - R @ src_centroid

    # 构建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def evaluate_origin_error(T_nl, enu_origin):
    """评估原点标定误差"""
    # 提取平移量
    translation = T_nl[:3, 3]

    # 计算误差
    error_vector = translation - enu_origin
    error_norm = np.linalg.norm(error_vector)

    return error_norm, error_vector


def main():
    """主执行函数"""
    try:
        # 加载数据
        print("正在加载数据...")
        enu_df = load_enu_data()
        lidar_df = load_lidar_data()
        print("数据加载完成\n")

        # 标定每个LiDAR
        results = {}
        for prefix in CONFIG["target_prefixes"]:
            print(f"正在处理{prefix}LiDAR...")

            # 对齐特征点
            enu_pts, lidar_pts, enu_origin, lidar_origin = align_points(enu_df, lidar_df, prefix)
            if enu_pts is None:
                continue

            # 计算变换矩阵
            T_nl = compute_transformation(lidar_pts, enu_pts)

            # 评估误差
            error, error_vec = evaluate_origin_error(T_nl, enu_origin)

            # 保存结果
            results[prefix] = {
                "matrix": T_nl,
                "error": error,
                "error_components": error_vec,
                "points_used": len(enu_pts)
            }

        # 打印结果
        print("\n标定结果:")
        print("=" * 65)
        for prefix, res in results.items():
            print(f"\n{prefix}LiDAR标定结果（使用{res['points_used']}个特征点）")
            print("变换矩阵 T_nl:")
            print(np.round(res['matrix'], 6))
            print(f"原点标定误差: {res['error']:.6f} 米")
            print(f"误差分量 [X,Y,Z]: {np.round(res['error_components'], 6)}")
            print("-" * 65)

    except Exception as e:
        print(f"\n发生未预期的错误: {str(e)}")
        print("建议检查：")
        print("1. 文件路径是否正确")
        print("2. 数据文件格式是否与示例一致")
        print("3. 特征点名称是否匹配")


if __name__ == "__main__":
    main()