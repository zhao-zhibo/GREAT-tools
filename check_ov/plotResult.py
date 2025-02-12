import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def extract_timestamps_and_matrices(data):
    timestamps = [item[0] for item in data]
    matrices = [item[1] for item in data]
    return timestamps, matrices

def extract_translation(matrices):
    return np.array([matrix[:3, 3] for matrix in matrices])

def extract_euler_angles(matrices):
    rotations = R.from_matrix([matrix[:3, :3] for matrix in matrices])
    return rotations.as_euler('xyz', degrees=True)

def plot_translations(slam_trans, ground_truth_trans):
    plt.figure()
    plt.plot(slam_trans[:, 0], label='SLAM E')
    plt.plot(slam_trans[:, 1], label='SLAM N')
    plt.plot(slam_trans[:, 2], label='SLAM U')
    plt.plot(ground_truth_trans[:, 0], label='Ground Truth E', linestyle='dashed')
    plt.plot(ground_truth_trans[:, 1], label='Ground Truth N', linestyle='dashed')
    plt.plot(ground_truth_trans[:, 2], label='Ground Truth U', linestyle='dashed')
    plt.legend()
    plt.title('Translations')
    plt.show()

def plot_translation_errors(slam_trans, ground_truth_trans):
    errors = np.abs(slam_trans - ground_truth_trans)
    plt.figure()
    plt.plot(errors[:, 0], label='Error E')
    plt.plot(errors[:, 1], label='Error N')
    plt.plot(errors[:, 2], label='Error U')
    plt.legend()
    plt.title('Translation Errors')
    plt.show()

def plot_euler_angles(slam_angles, ground_truth_angles):
    plt.figure()
    plt.plot(slam_angles[:, 0], label='SLAM Roll')
    plt.plot(slam_angles[:, 1], label='SLAM Pitch')
    plt.plot(slam_angles[:, 2], label='SLAM Yaw')
    plt.plot(ground_truth_angles[:, 0], label='Ground Truth Roll', linestyle='dashed')
    plt.plot(ground_truth_angles[:, 1], label='Ground Truth Pitch', linestyle='dashed')
    plt.plot(ground_truth_angles[:, 2], label='Ground Truth Yaw', linestyle='dashed')
    plt.legend()
    plt.title('Euler Angles')
    plt.show()

def plot_euler_angle_errors(slam_angles, ground_truth_angles):
    errors = np.abs(slam_angles - ground_truth_angles)
    plt.figure()
    plt.plot(errors[:, 0], label='Error Roll')
    plt.plot(errors[:, 1], label='Error Pitch')
    plt.plot(errors[:, 2], label='Error Yaw')
    plt.legend()
    plt.title('Euler Angle Errors')
    plt.show()

def plot_2d_trajectory(slam_trans, ground_truth_trans):
    plt.figure()
    plt.plot(slam_trans[:, 0], slam_trans[:, 1], label='SLAM Trajectory')
    plt.plot(ground_truth_trans[:, 0], ground_truth_trans[:, 1], label='Ground Truth Trajectory', linestyle='dashed')
    plt.legend()
    plt.title('2D Trajectory (E vs N)')
    plt.xlabel('E')
    plt.ylabel('N')
    plt.show()

def compare_slam_and_ground_truth(slam_data, ground_truth_data):
    slam_timestamps, slam_matrices = extract_timestamps_and_matrices(slam_data)
    ground_truth_timestamps, ground_truth_matrices = extract_timestamps_and_matrices(ground_truth_data)

    if slam_timestamps != ground_truth_timestamps:
        print("时间戳不对应，无法进行对比。")
        return

    slam_trans = extract_translation(slam_matrices)
    ground_truth_trans = extract_translation(ground_truth_matrices)
    plot_2d_trajectory(slam_trans, ground_truth_trans)
    plot_translations(slam_trans, ground_truth_trans)
    plot_translation_errors(slam_trans, ground_truth_trans)

    slam_angles = extract_euler_angles(slam_matrices)
    ground_truth_angles = extract_euler_angles(ground_truth_matrices)
    plot_euler_angles(slam_angles, ground_truth_angles)
    plot_euler_angle_errors(slam_angles, ground_truth_angles)
