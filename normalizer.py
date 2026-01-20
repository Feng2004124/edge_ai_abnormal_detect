import numpy as np


class Normalizer:
    """数据归一化器，支持动态范围设置"""

    def __init__(self, norm_range=(-1, 1)):
        """
        初始化归一化器
        :param norm_range: 归一化目标范围 (default: (-1, 1))
        """
        self.norm_min, self.norm_max = norm_range

    def normalize(self, data):
        """归一化数据到指定范围"""
        # 检查数据是否为空
        if data.size == 0:
            print("警告: 输入数据为空，返回零数组")
            return np.zeros_like(data)

        min_val = np.min(data)
        max_val = np.max(data)
        if max_val == min_val:  # 防止除零错误
            return np.zeros_like(data)

        # 线性归一化公式
        normalized = (data - min_val) / (max_val - min_val)
        # 映射到目标范围
        return normalized * (self.norm_max - self.norm_min) + self.norm_min