#!/usr/bin/env python3
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def ros2open3d(data: PointCloud2):
    tmp_pcd_name = '/tmp/tmp_cloud.pcd'
    header = '# .PCD v0.7 - Point Cloud Data file format\n' + \
             'VERSION 0.7\n' + \
             'FIELDS x y z rgb\n' + \
             'SIZE 4 4 4 4\n' + \
             'TYPE F F F F\n' + \
             'COUNT 1 1 1 1\n' + \
             'WIDTH %d\n' + \
             'HEIGHT %d\n' + \
             'VIEWPOINT 0 0 0 1 0 0 0\n' + \
             'POINTS %d\n' + \
             'DATA ascii\n'

    with open(tmp_pcd_name, 'w') as f:
        f.write(header % (data.width, data.height, data.width*data.height) + '\n')
        for p in pc2.read_points(data, skip_nans=True):
            f.write('%f %f %f %e' % (p[0], p[1], p[2], p[3]) + '\n')
        f.write('\n')

    return o3d.io.read_point_cloud(tmp_pcd_name)

def open3d2ros(data, frame_id: str):
    FIELDS = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]

    pcl_point = np.array(data.points)
    pcl_color = np.array(data.colors)

    # format color data
    tmp_color = np.c_[np.zeros(pcl_color.shape[1])]
    tmp_color = np.floor(pcl_color[:,0] * 255) * 2**16 + np.floor(pcl_color[:,1] * 255) * 2**8 + np.floor(pcl_color[:,2] * 255) # 16bit shift, 8bit shift, 0bit shift

    pcl = np.c_[pcl_point, tmp_color]

    return pc2.create_cloud(Header(frame_id=frame_id), FIELDS, pcl)
