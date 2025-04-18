#!/usr/bin/python3
import numpy  # 数学计算库
import gtsam  # 因子图优化库，提供位姿和几何运算

def main():
    # 设置numpy打印格式：保留3位小数，不使用科学计数法
    numpy.set_printoptions(precision=3, suppress=True)

    # 随机生成一个SE(3)位姿（通过指数映射将6维李代数向量转为4x4位姿矩阵）
    # 前3个元素为平移向量，后3个为旋转向量（李代数）
    pose = gtsam.Pose3.Expmap(numpy.random.rand(6))

    # 随机生成一个三维点坐标（局部坐标系中的点）
    pt = gtsam.Point3(numpy.random.rand(3))

    # 打印原始位姿矩阵和点坐标
    print('--- pose ---')
    print(pose.matrix())  # 4x4变换矩阵
    print('--- pt ---')
    print(pt)  # 3D点坐标

    # 雅可比矩阵初始化：3行（点坐标x/y/z）x 6列（位姿6自由度）
    eps = 1e-6  # 数值微分微小增量
    J = numpy.zeros((3, 6))

    # 计算未扰动时的变换点坐标（基准值）
    pt0 = pose.transformFrom(pt)  # 将点pt从局部坐标系变换到世界坐标系

    # 遍历位姿的6个自由度（前3平移，后3旋转）
    for i in range(6):
        # 在第i个自由度上施加微小扰动
        delta = numpy.zeros(6)  # 初始化扰动向量
        delta[i] = eps  # 仅在第i个位置设置eps

        # 右乘扰动：将扰动施加在位姿的局部坐标系（更常用）
        # 公式：T_perturbed = T_original * exp(Δξ)
        perturbed_pose = pose.compose(gtsam.Pose3.Expmap(delta))

        # 计算扰动后的点坐标
        pti = perturbed_pose.transformFrom(pt)

        # 数值微分计算导数（雅可比矩阵的第i列）
        J[:, i] = (pti - pt0) / eps

    # 打印雅可比矩阵
    print('--- J ---')
    print(J)


if __name__ == '__main__':
    main()