<<<<<<< HEAD
#!/usr/bin/env python3
"""
读取ROS2相机标定结果，输出可直接用于Go2电表检测代码的内参矩阵
"""
import yaml
import numpy as np
import argparse


def read_calibration_params(calib_file):
    """
    解析标定yaml文件，提取内参K和畸变系数
    :param calib_file: 标定结果yaml文件路径
    :return: K(内参矩阵), dist(畸变系数), calib_data(完整yaml字典，失败时为None)
    """
    try:
        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        # 提取相机内参（camera_matrix）
        cam_matrix = calib_data['camera_matrix']['data']
        K = np.array([
            [cam_matrix[0], cam_matrix[1], cam_matrix[2]],
            [cam_matrix[3], cam_matrix[4], cam_matrix[5]],
            [cam_matrix[6], cam_matrix[7], cam_matrix[8]]
        ], dtype=np.float32)

        # 提取畸变系数（distortion_coefficients）
        dist_coeffs = np.array(calib_data['distortion_coefficients']['data'], dtype=np.float32)

        return K, dist_coeffs, calib_data

    except FileNotFoundError:
        print(f"错误：未找到文件 {calib_file}")
        return None, None, None
    except KeyError as e:
        print(f"错误：文件格式异常，缺少字段 {e}")
        return None, None, None


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='读取ROS2相机标定参数')
    parser.add_argument('--file', '-f', required=True, help='标定结果yaml文件路径')
    args = parser.parse_args()

    # 读取内参
    K, dist, calib_data = read_calibration_params(args.file)
    if K is None or dist is None:
        return

    # 输出结果（可直接复制到机器狗代码中）
    w = calib_data.get('image_width', '未知') if calib_data else '未知'
    h = calib_data.get('image_height', '未知') if calib_data else '未知'
    print("\n==================== 相机内参结果 ====================")
    print(f"分辨率：{w}×{h}")
    print("\n1. 内参矩阵 K（复制到代码中的self.K）：")
    print("self.K = np.array([")
    print(f"    [{K[0, 0]}, {K[0, 1]}, {K[0, 2]}],")
    print(f"    [{K[1, 0]}, {K[1, 1]}, {K[1, 2]}],")
    print(f"    [{K[2, 0]}, {K[2, 1]}, {K[2, 2]}]")
    print("], dtype=np.float32)")

    print("\n2. 畸变系数 dist（复制到代码中的self.dist_coeffs）：")
    print(f"self.dist_coeffs = np.array({dist.tolist()}, dtype=np.float32)")
    print("=======================================================")


if __name__ == '__main__':
=======
#!/usr/bin/env python3
"""
读取ROS2相机标定结果，输出可直接用于Go2电表检测代码的内参矩阵
"""
import yaml
import numpy as np
import argparse


def read_calibration_params(calib_file):
    """
    解析标定yaml文件，提取内参K和畸变系数
    :param calib_file: 标定结果yaml文件路径
    :return: K(内参矩阵), dist(畸变系数), calib_data(完整yaml字典，失败时为None)
    """
    try:
        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        # 提取相机内参（camera_matrix）
        cam_matrix = calib_data['camera_matrix']['data']
        K = np.array([
            [cam_matrix[0], cam_matrix[1], cam_matrix[2]],
            [cam_matrix[3], cam_matrix[4], cam_matrix[5]],
            [cam_matrix[6], cam_matrix[7], cam_matrix[8]]
        ], dtype=np.float32)

        # 提取畸变系数（distortion_coefficients）
        dist_coeffs = np.array(calib_data['distortion_coefficients']['data'], dtype=np.float32)

        return K, dist_coeffs, calib_data

    except FileNotFoundError:
        print(f"错误：未找到文件 {calib_file}")
        return None, None, None
    except KeyError as e:
        print(f"错误：文件格式异常，缺少字段 {e}")
        return None, None, None


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='读取ROS2相机标定参数')
    parser.add_argument('--file', '-f', required=True, help='标定结果yaml文件路径')
    args = parser.parse_args()

    # 读取内参
    K, dist, calib_data = read_calibration_params(args.file)
    if K is None or dist is None:
        return

    # 输出结果（可直接复制到机器狗代码中）
    w = calib_data.get('image_width', '未知') if calib_data else '未知'
    h = calib_data.get('image_height', '未知') if calib_data else '未知'
    print("\n==================== 相机内参结果 ====================")
    print(f"分辨率：{w}×{h}")
    print("\n1. 内参矩阵 K（复制到代码中的self.K）：")
    print("self.K = np.array([")
    print(f"    [{K[0, 0]}, {K[0, 1]}, {K[0, 2]}],")
    print(f"    [{K[1, 0]}, {K[1, 1]}, {K[1, 2]}],")
    print(f"    [{K[2, 0]}, {K[2, 1]}, {K[2, 2]}]")
    print("], dtype=np.float32)")

    print("\n2. 畸变系数 dist（复制到代码中的self.dist_coeffs）：")
    print(f"self.dist_coeffs = np.array({dist.tolist()}, dtype=np.float32)")
    print("=======================================================")


if __name__ == '__main__':
>>>>>>> a5a410c (add default camera calib yaml)
    main()