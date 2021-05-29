import open3d as o3d


def read_pointcloud(path):
    """
    读取ModelNet 40中的点云文件
    :param path: 点云文件路径
    :return: 返回列表格式的点云数据
    """
    with open(path) as file_object:
        off_line = file_object.readline()

        if len(off_line) > 4:
            num_point = int(off_line[3: -1].split(' ')[0])
        else:
            num_point = int(file_object.readline().split(' ')[0])

        line_num = 0
        point_cloud = []
        for line in file_object.readlines():
            if line_num < num_point:
                point = [float(value) for value in line.strip().split(' ')]
                line_num += 1
                point_cloud.append(point)
            else:
                break
    return point_cloud


def display_pointcloud(point_cloud):
    """
    通过open3d进行点云显示
    :param point_cloud: 列表格式的点云数据
    :return: void
    """
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(point_cloud)
    o3d.visualization.draw_geometries([pointcloud_o3d])
