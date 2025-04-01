import numpy as np
import config
from scipy.spatial.transform import Rotation as R

def read_tum_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#'):
                continue
            elements = line.strip().split()
            timestamp = float(elements[0])
            position = np.array([float(elements[1]), float(elements[2]), float(elements[3])])
            quaternion = np.array([float(elements[4]), float(elements[5]), float(elements[6]), float(elements[7])])
            data.append((timestamp, position, quaternion))
    return data

def quaternion_to_matrix(q):
    r = R.from_quat(q)
    return r.as_matrix()

def create_transformation_matrix(position, quaternion):
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = quaternion_to_matrix(quaternion)
    transformation_matrix[:3, 3] = position
    return transformation_matrix

def processTumNframe(data):
    transformed_data = []
    for timestamp, position, quaternion in data:
        transformation_matrix = create_transformation_matrix(position, quaternion)
        transformed_data.append((timestamp, transformation_matrix))
    return transformed_data

def process_tum_data(data, reference_timestamp):
    reference_matrix = None
    for timestamp, position, quaternion in data:
        if timestamp == reference_timestamp:
            reference_matrix = create_transformation_matrix(position, quaternion)
            break

    if reference_matrix is None:
        raise ValueError("Reference timestamp not found in data")

    transformed_data = []
    for timestamp, position, quaternion in data:
        transformation_matrix = create_transformation_matrix(position, quaternion)
        new_transformation_matrix = np.linalg.inv(reference_matrix) @ transformation_matrix
        transformed_data.append((timestamp, new_transformation_matrix))

    return transformed_data

def main(file_path, isNframe = False):
    data = read_tum_file(file_path)
    # 选择一个时刻作为参考，然后将所有位姿转换到参考坐标系下，因为这个时刻对应的真值的位姿比较准确，因此选择这个时刻作为参考
    if isNframe:
        # 如果传入的是True，执行另���个函数
        return processTumNframe(data)
    else:
        # 如果传入的是False，执行当前的process函数
        return process_tum_data(data, config.reference_timestamp)

if __name__ == "__main__":
    main()