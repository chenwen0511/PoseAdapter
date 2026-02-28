#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
读取相机标定参数
"""

import yaml
import numpy as np
import argparse


def read_calib_params(yaml_file):
    """
    从 YAML 文件读取相机标定参数
    
    Args:
        yaml_file: 标定文件路径
        
    Returns:
        dict: {
            'camera_matrix': 3x3 矩阵,
            'dist_coeffs': 畸变系数,
            'width': 图像宽度,
            'height': 图像高度
        }
    """
    with open(yaml_file, 'r') as f:
        calib_data = yaml.safe_load(f)
    
    # 提取相机矩阵
    K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    
    # 提取畸变系数
    D = np.array(calib_data['distortion_coefficients']['data'])
    
    # 提取图像尺寸
    width = calib_data['image_width']
    height = calib_data['image_height']
    
    return {
        'camera_matrix': K,
        'dist_coeffs': D,
        'width': width,
        'height': height
    }


def main():
    parser = argparse.ArgumentParser(description='读取相机标定参数')
    parser.add_argument('-f', '--file', required=True, help='标定 YAML 文件路径')
    args = parser.parse_args()
    
    params = read_calib_params(args.file)
    
    print("相机内参矩阵 K:")
    print(params['camera_matrix'])
    print("\n畸变系数:")
    print(params['dist_coeffs'])
    print(f"\n图像尺寸: {params['width']} x {params['height']}")
    
    # 输出可用于代码的格式
    print("\n" + "="*50)
    print("Python 代码格式:")
    print("="*50)
    K = params['camera_matrix']
    D = params['dist_coeffs']
    print(f"self.K = np.array([[{K[0,0]:.6f}, {K[0,1]:.6f}, {K[0,2]:.6f}],")
    print(f"                   [{K[1,0]:.6f}, {K[1,1]:.6f}, {K[1,2]:.6f}],")
    print(f"                   [{K[2,0]:.6f}, {K[2,1]:.6f}, {K[2,2]:.6f}]])")
    print(f"self.dist_coeffs = np.array([{D[0]:.6f}, {D[1]:.6f}, {D[2]:.6f}, {D[3]:.6f}, {D[4]:.6f}])")


if __name__ == '__main__':
    main()
