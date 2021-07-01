import numpy as np
import open3d as o3d


class NormalDistributionTransformation:
    def __init__(self, grid_size: float, pointcloud: np.ndarray):
        self.grid_size = grid_size
        self.target_pointcloud = pointcloud

    def match(self, source_pointcloud: np.ndarray):
        source_pointcloud.reshape()
