import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from evo.core.metrics import PoseRelation, APE, RPE
from evo.core import trajectory

def extract_timestamps_and_matrices(data):
    timestamps = [item[0] for item in data]  # 单位: s
    matrices = [item[1] for item in data]
    return timestamps, matrices

def extract_translation(matrices):
    return np.array([matrix[:3, 3] for matrix in matrices])  # 单位: m

def extract_euler_angles(matrices):
    rotations = R.from_matrix([matrix[:3, :3] for matrix in matrices])
    return rotations.as_euler('xyz', degrees=True)  # 单位: 度

def plot_translations_and_errors(slam_timestamps, slam_trans, ground_truth_timestamps, ground_truth_trans, slam_type):
    errors = abs(slam_trans - ground_truth_trans)  # 单位: m
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    axs[0].plot(slam_timestamps, slam_trans[:, 0], label='SLAM E (m)')
    axs[0].plot(slam_timestamps, slam_trans[:, 1], label='SLAM N (m)')
    axs[0].plot(slam_timestamps, slam_trans[:, 2], label='SLAM U (m)')
    axs[0].plot(ground_truth_timestamps, ground_truth_trans[:, 0], label='Ground Truth E (m)', linestyle='dashed')
    axs[0].plot(ground_truth_timestamps, ground_truth_trans[:, 1], label='Ground Truth N (m)', linestyle='dashed')
    axs[0].plot(ground_truth_timestamps, ground_truth_trans[:, 2], label='Ground Truth U (m)', linestyle='dashed')
    axs[0].legend()
    axs[0].set_title(f'Translations ({slam_type})')
    axs[0].set_xlabel('Time (s)')

    axs[1].plot(slam_timestamps, errors[:, 0], label='Error E (m)')
    axs[1].plot(slam_timestamps, errors[:, 1], label='Error N (m)')
    axs[1].plot(slam_timestamps, errors[:, 2], label='Error U (m)')
    axs[1].legend()
    axs[1].set_title(f'Translation Errors ({slam_type})')
    axs[1].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()

def plot_euler_angles_and_errors(slam_timestamps, slam_angles, ground_truth_timestamps, ground_truth_angles, slam_type):
    errors = np.abs(slam_angles - ground_truth_angles)  # 单位: 度
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    axs[0].plot(slam_timestamps, slam_angles[:, 0], label='SLAM Roll (°)')
    axs[0].plot(slam_timestamps, slam_angles[:, 1], label='SLAM Pitch (°)')
    axs[0].plot(slam_timestamps, slam_angles[:, 2], label='SLAM Yaw (°)')
    axs[0].plot(ground_truth_timestamps, ground_truth_angles[:, 0], label='Ground Truth Roll (°)', linestyle='dashed')
    axs[0].plot(ground_truth_timestamps, ground_truth_angles[:, 1], label='Ground Truth Pitch (°)', linestyle='dashed')
    axs[0].plot(ground_truth_timestamps, ground_truth_angles[:, 2], label='Ground Truth Yaw (°)', linestyle='dashed')
    axs[0].legend()
    axs[0].set_title(f'Euler Angles ({slam_type})')
    axs[0].set_xlabel('Time (s)')

    axs[1].plot(slam_timestamps, errors[:, 0], label='Error Roll (°)')
    axs[1].plot(slam_timestamps, errors[:, 1], label='Error Pitch (°)')
    axs[1].plot(slam_timestamps, errors[:, 2], label='Error Yaw (°)')
    axs[1].legend()
    axs[1].set_title(f'Euler Angle Errors ({slam_type})')
    axs[1].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()

def plot_2d_trajectory(slam_trans, ground_truth_trans, slam_type):
    plt.figure()
    plt.plot(slam_trans[:, 0], slam_trans[:, 1], label='SLAM Trajectory (m)')
    plt.plot(ground_truth_trans[:, 0], ground_truth_trans[:, 1], label='Ground Truth Trajectory (m)', linestyle='dashed')
    plt.legend()
    plt.title(f'2D Trajectory (E vs N) ({slam_type})')
    plt.xlabel('E (m)')
    plt.ylabel('N (m)')
    plt.axis('equal')  # 使横轴和纵轴的刻度一致
    plt.show()

def compare_slam_and_ground_truth(slam_data, ground_truth_data, slam_type):
    slam_timestamps, slam_matrices = extract_timestamps_and_matrices(slam_data)
    ground_truth_timestamps, ground_truth_matrices = extract_timestamps_and_matrices(ground_truth_data)

    if slam_timestamps != ground_truth_timestamps:
        print("时间戳不对应，无法进行对比。")
        return

    slam_trans = extract_translation(slam_matrices)
    ground_truth_trans = extract_translation(ground_truth_matrices)
    plot_2d_trajectory(slam_trans, ground_truth_trans, slam_type)
    plot_translations_and_errors(slam_timestamps, slam_trans, ground_truth_timestamps, ground_truth_trans, slam_type)
    compute_rmse_and_statistics(slam_trans, ground_truth_trans)

    slam_angles = extract_euler_angles(slam_matrices)
    ground_truth_angles = extract_euler_angles(ground_truth_matrices)
    plot_euler_angles_and_errors(slam_timestamps, slam_angles, ground_truth_timestamps, ground_truth_angles, slam_type)
def compute_rmse_and_statistics(slam_trans, ground_truth_trans):
    errors = slam_trans - ground_truth_trans
    squared_errors = errors ** 2

    # Per-direction statistics
    rmse = np.sqrt(np.mean(squared_errors, axis=0))
    mean_error = np.mean(errors, axis=0)
    median_error = np.median(errors, axis=0)
    std_error = np.std(errors, axis=0)

    # Combined statistics
    rmse_combined = np.sqrt(np.sum(rmse ** 2))
    mean_error_combined = np.sqrt(np.sum(mean_error ** 2))
    median_error_combined = np.sqrt(np.sum(median_error ** 2))
    std_error_combined = np.sqrt(np.sum(std_error ** 2))

    print(f"RMSE: {rmse}, Combined RMSE: {rmse_combined}")
    print(f"Mean Error: {mean_error}, Combined Mean Error: {mean_error_combined}")
    print(f"Median Error: {median_error}, Combined Median Error: {median_error_combined}")
    print(f"Standard Deviation of Error: {std_error}, Combined Standard Deviation of Error: {std_error_combined}")

def evo_trajectories(slamTransforMatrix, interpolatedGroundTruth):
    slamTimestamps = [timestamp for timestamp, _ in slamTransforMatrix]
    slamPoses_se3 = [slam_matrix for _, slam_matrix in slamTransforMatrix]
    slam_trajectory_evo = trajectory.Trajectory(poses_se3=slamPoses_se3, timestamps=slamTimestamps)
    gtTimestamps = [timestamp for timestamp, _ in interpolatedGroundTruth]
    gtPoses_se3 = [gt_matrix for _, gt_matrix in interpolatedGroundTruth]
    gt_trajectory_evo = trajectory.Trajectory(poses_se3=gtPoses_se3, timestamps=gtTimestamps)

    # 计算 APE
    ape = APE(pose_relation=PoseRelation.translation_part)
    ape.process_data((gt_trajectory_evo, slam_trajectory_evo))
    ape_errors = ape.get_all_statistics()
    print("APE Errors:", ape_errors)

    # 计算 RPE
    rpe = RPE(pose_relation=PoseRelation.translation_part)
    rpe.process_data((gt_trajectory_evo, slam_trajectory_evo))
    rpe_errors = rpe.get_all_statistics()
    print("RPE Errors:", rpe_errors)

